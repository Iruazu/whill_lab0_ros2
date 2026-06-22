#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Convert a DUFOMap static PCD into a Nav2 occupancy grid (.pgm + .yaml).

This is the M5R-6 (Issue #50) tail end of the M5-R map-building
pipeline: GLIM (ADR-0003) -> DUFOMap (ADR-0004, dynamic-removal) ->
*this script* -> Nav2 map_server.

It is intentionally a sibling of (not a refactor of) the older
``scripts/pcd_to_occupancy_grid.py``.  The older script was written for
the M5-b FAST-LIO map: it hard-codes a +/-20 m crop around the world
origin and ray-casts from (0, 0) on the assumption that the sensor
started at the world origin and never strayed far from it.  Those
assumptions no longer hold under the M5-R pipeline -- the DUFOMap
output is already cleaned of pedestrian streaks and may cover an
arbitrary outdoor loop whose centroid sits nowhere near the GLIM map
origin.  Rather than retrofit the old script and risk breaking
``docs/maps/lab-legacy-m5b/`` (which ``nav_launch.py`` still consumes),
we leave it alone and start clean here.

Differences from the legacy script that matter:

* The XY bounding box is computed from the input, not hard-coded.  A
  configurable ``--padding`` adds breathing room so obstacles at the
  edge of the traverse do not get clipped.
* The ray-cast anchor defaults to the **occupied-cell centroid** rather
  than (0, 0).  This is still a single-anchor approximation -- it does
  not match per-scan ray-casting that ``octomap``/``UFO`` would do --
  but for offline conversion of an already-static cloud it is enough
  to give Nav2 a connected free region to plan through, and it does
  not require carrying the per-scan poses through this stage.  A
  proper per-scan implementation is a follow-up Issue if needed.
* The YAML output matches ``docs/maps/_template/occupancy.yaml``
  exactly (including ``free_thresh: 0.196`` -- the Nav2 map_server
  default -- not the legacy 0.25 the M5-b script writes).
* No outlier-cluster filter and no clear-radius hack.  Both existed
  in the legacy script to paper over FAST-LIO drift + chair
  self-returns; DUFOMap removes the former, and the latter is moot
  since we no longer assume the chair starts at world origin.

Z-slice defaults (0.1 m to 1.5 m) preserve the chair-relevant band
the legacy script also used: above floor noise / sloped ground, and
below most ceiling fixtures and door lintels.

PGM pixel convention follows ROS ``map_server``:
    0   = occupied (black)
    254 = free
    205 = unknown
The PGM is written as binary P5 with the 0..255 maxval so map_server
can ingest it without translation.

Usage:

    scripts/m5r_pcd_to_occupancy.py <input.pcd> <output-dir> [options]

Example (matches the M5R-4 bench run):

    scripts/m5r_pcd_to_occupancy.py /tmp/m5r49_dufomap/static.pcd \\
        docs/maps/lab-loop --force

Output (in <output-dir>):
    occupancy.pgm   binary P5, 1 byte / cell
    occupancy.yaml  Nav2 map_server metadata (resolution / origin / thresholds)
"""

import argparse
import sys
from pathlib import Path

import numpy as np


# ROS map_server pixel convention. Keep these as module constants so
# downstream readers (and any future per-cell debug dump) can refer to
# them symbolically instead of re-deriving the magic numbers.
PIX_OCCUPIED = 0
PIX_FREE = 254
PIX_UNKNOWN = 205


def read_pcd_xyz(path: Path) -> np.ndarray:
    """Return an (N, 3) float32 array of XYZ from a PCD (binary or ASCII).

    DUFOMap emits binary PCDs with FIELDS = ``x y z`` (no intensity, no
    RGB). GLIM's per-keyframe staging is ASCII with the same XYZ-only
    layout. We support both formats and ignore any extra fields so the
    converter remains usable on hand-edited PCDs without needing a
    re-encode step. PCL/Open3D are deliberately not pulled in -- the
    parser is small and the only mandatory dependency is numpy, which
    matches the rest of the M5-R script suite.
    """
    with path.open("rb") as f:
        header_lines = []
        while True:
            raw_line = f.readline()
            if not raw_line:
                raise ValueError(f"{path}: unexpected EOF inside PCD header")
            line = raw_line.decode("ascii", errors="ignore").strip()
            header_lines.append(line)
            if line.startswith("DATA"):
                data_fmt = line.split()[1]
                break
        # Capture binary payload position so we can re-read as bytes
        # later. Doing it now avoids juggling text/binary modes.
        binary_offset = f.tell()
        binary_payload = f.read()

    # Parse header values. Each PCD header field appears on exactly one
    # line; we tolerate them appearing in any order (the spec allows it).
    def header_field(name: str) -> list[str]:
        for line in header_lines:
            if line.startswith(name + " "):
                return line.split()[1:]
        raise ValueError(f"{path}: PCD header missing '{name}'")

    fields = header_field("FIELDS")
    sizes = [int(x) for x in header_field("SIZE")]
    counts = [int(x) for x in header_field("COUNT")]
    types = header_field("TYPE")
    n_points = int(header_field("POINTS")[0])

    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError(
            f"{path}: PCD FIELDS {fields!r} missing one of x/y/z"
        )

    # PCD allows SIZE=4 + TYPE=I/U/F. We only handle TYPE F for x/y/z
    # because reading int32 bits as float32 silently corrupts data.
    # DUFOMap, GLIM and our own m5r_glim_to_pcd.py all emit TYPE F F F,
    # so this guard only fires on hand-edited or third-party PCDs.
    for name in ("x", "y", "z"):
        i = fields.index(name)
        if types[i] != "F":
            raise ValueError(
                f"{path}: PCD field {name!r} has TYPE {types[i]!r}; "
                f"only TYPE F (float) is supported"
            )

    if data_fmt == "ascii":
        # Re-read the file as text for the ASCII branch -- doing it in a
        # second pass keeps the binary path zero-copy and the ASCII path
        # straightforward, at the cost of one extra read on ASCII files.
        text = path.read_text().splitlines()
        body = text[len(header_lines):]
        if len(body) < n_points:
            # Without this guard the missing rows would leak through as
            # zero-initialised XYZ from np.empty (which numpy does NOT
            # zero), producing phantom (0, 0, 0) hits near the world
            # origin. Bail early with a clear message instead.
            raise ValueError(
                f"{path}: ASCII PCD body has {len(body)} lines, "
                f"header POINTS={n_points}"
            )
        xyz = np.empty((n_points, 3), dtype=np.float32)
        xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
        for i, line in enumerate(body[:n_points]):
            tokens = line.split()
            xyz[i, 0] = float(tokens[xi])
            xyz[i, 1] = float(tokens[yi])
            xyz[i, 2] = float(tokens[zi])
        return xyz

    if data_fmt != "binary":
        # binary_compressed would need lzf decoding; not worth the
        # dependency until something in the pipeline actually emits it.
        raise ValueError(
            f"{path}: DATA {data_fmt!r} unsupported (only ascii/binary)"
        )

    step = sum(s * c for s, c in zip(sizes, counts))
    expected_bytes = step * n_points
    if len(binary_payload) < expected_bytes:
        raise ValueError(
            f"{path}: binary payload {len(binary_payload)} bytes < "
            f"expected {expected_bytes} (POINTS={n_points}, step={step})"
        )

    arr = np.frombuffer(
        binary_payload[:expected_bytes], dtype=np.uint8
    ).reshape(n_points, step)
    xyz = np.empty((n_points, 3), dtype=np.float32)
    offset = 0
    for name, s, c in zip(fields, sizes, counts):
        # Only consume x/y/z; skip everything else by advancing the
        # offset (intensity, rgb, normal_*, etc.).
        if name in ("x", "y", "z") and s == 4 and c == 1:
            idx = "xyz".index(name)
            xyz[:, idx] = np.frombuffer(
                arr[:, offset:offset + 4].tobytes(), dtype=np.float32
            )
        offset += s * c
    # binary_offset is unused after the read but we keep it captured
    # above for readability — if a future debug printout wants to show
    # "header ended at byte N", it is already in scope.
    del binary_offset
    return xyz


def bresenham_rays_to_free(
    grid: np.ndarray,
    anchor_px: tuple[int, int],
    hit_pixels: np.ndarray,
) -> None:
    """Mark cells along anchor->hit rays as free, in-place.

    For each unique occupied pixel we walk Bresenham from the anchor
    toward (but excluding) that pixel, flipping intervening UNKNOWN
    cells to FREE. We deliberately do not touch cells that are already
    OCCUPIED: occupancy wins over freeness if a later hit's ray passes
    through an earlier hit. That matches Nav2's costmap occupancy
    semantics (occupied is the safer assumption).

    The single-anchor approximation is documented at module level: a
    per-scan implementation would carry the keyframe poses through this
    stage, which is out of scope for Issue #50.
    """
    anchor_col, anchor_row = anchor_px
    h, w = grid.shape
    for hit_col, hit_row in hit_pixels:
        dx = abs(hit_col - anchor_col)
        dy = -abs(hit_row - anchor_row)
        sx = 1 if anchor_col < hit_col else -1
        sy = 1 if anchor_row < hit_row else -1
        err = dx + dy
        col, row = anchor_col, anchor_row
        while True:
            if col == hit_col and row == hit_row:
                # Stop just before stamping the occupied endpoint as
                # free (the occupied stamp happens outside this loop).
                break
            if 0 <= col < w and 0 <= row < h:
                if grid[row, col] == PIX_UNKNOWN:
                    grid[row, col] = PIX_FREE
                elif grid[row, col] == PIX_OCCUPIED:
                    # Occlusion: a real obstacle blocks this ray.
                    # Anything beyond it is not "definitely free" so
                    # stop walking.
                    break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                col += sx
            if e2 <= dx:
                err += dx
                row += sy


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a DUFOMap static PCD into a Nav2-compatible 2D "
            "occupancy grid (occupancy.pgm + occupancy.yaml). Part of "
            "the M5-R map pipeline (Issue #50)."
        ),
    )
    parser.add_argument(
        "input_pcd", type=Path,
        help="Input static PCD (binary or ASCII, XYZ fields required)",
    )
    parser.add_argument(
        "output_dir", type=Path,
        help="Output directory; occupancy.pgm and occupancy.yaml are "
             "written into it. Typically docs/maps/<site>/.",
    )
    parser.add_argument(
        "--resolution", type=float, default=0.05,
        help="Cell size in metres (default 0.05, matches "
             "_template/occupancy.yaml)",
    )
    # Z slice defaults preserve the chair-relevant obstacle band the
    # legacy M5-b script also used. Above this floor noise and sloped
    # ground get baked in; below the cap, door lintels and signage end
    # up as phantom obstacles even though the chair passes underneath.
    parser.add_argument(
        "--z-min", type=float, default=0.1,
        help="Lower Z slice [m] (default 0.1, excludes floor noise)",
    )
    parser.add_argument(
        "--z-max", type=float, default=1.5,
        help="Upper Z slice [m] (default 1.5, excludes lintels / signs)",
    )
    parser.add_argument(
        "--anchor-x", type=float, default=None,
        help="Ray-cast anchor X [m] (default: occupied-cell XY centroid)",
    )
    parser.add_argument(
        "--anchor-y", type=float, default=None,
        help="Ray-cast anchor Y [m] (default: occupied-cell XY centroid)",
    )
    parser.add_argument(
        "--no-raycast", action="store_true",
        help="Skip the Bresenham free-marking pass; cells are either "
             "occupied or unknown. Useful for debugging or when "
             "track_unknown_space=false in Nav2.",
    )
    # 0.196 matches Nav2 map_server's documented default and the
    # _template/occupancy.yaml committed under #47. The legacy
    # pcd_to_occupancy_grid.py wrote 0.25, which is a *Nav1*-era value;
    # we deliberately do not propagate that here.
    parser.add_argument(
        "--occupied-thresh", type=float, default=0.65,
        help="YAML occupied_thresh (default 0.65, Nav2 default)",
    )
    parser.add_argument(
        "--free-thresh", type=float, default=0.196,
        help="YAML free_thresh (default 0.196, Nav2 default)",
    )
    parser.add_argument(
        "--padding", type=float, default=2.0,
        help="Padding [m] added outside the XY bounding box so edge "
             "obstacles are not clipped (default 2.0)",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="Overwrite existing occupancy.pgm / occupancy.yaml. "
             "Default is to abort, matching the rest of the M5-R "
             "script suite.",
    )
    args = parser.parse_args()

    if not args.input_pcd.is_file():
        print(f"ERROR: input PCD {args.input_pcd} not found",
              file=sys.stderr)
        return 1

    out_dir: Path = args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = out_dir / "occupancy.pgm"
    yaml_path = out_dir / "occupancy.yaml"

    if not args.force:
        existing = [p for p in (pgm_path, yaml_path) if p.exists()]
        if existing:
            names = ", ".join(p.name for p in existing)
            print(
                f"ERROR: {names} already exists under {out_dir}. "
                f"Re-run with --force to overwrite.",
                file=sys.stderr,
            )
            return 1

    print(f"Reading {args.input_pcd} ...", file=sys.stderr)
    try:
        xyz = read_pcd_xyz(args.input_pcd)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"Loaded {len(xyz)} points", file=sys.stderr)

    z_mask = (xyz[:, 2] >= args.z_min) & (xyz[:, 2] <= args.z_max)
    sliced = xyz[z_mask]
    print(
        f"Z-sliced to [{args.z_min:.2f}, {args.z_max:.2f}]: "
        f"{len(sliced)} points retained",
        file=sys.stderr,
    )
    if len(sliced) == 0:
        print(
            "ERROR: no points survived Z-slice. Check --z-min / --z-max "
            "against the PCD bounding box.",
            file=sys.stderr,
        )
        return 1

    xmin = float(sliced[:, 0].min()) - args.padding
    xmax = float(sliced[:, 0].max()) + args.padding
    ymin = float(sliced[:, 1].min()) - args.padding
    ymax = float(sliced[:, 1].max()) + args.padding
    res = args.resolution
    width = int(np.ceil((xmax - xmin) / res))
    height = int(np.ceil((ymax - ymin) / res))
    print(
        f"Grid: {width} x {height} cells @ {res} m  "
        f"(bbox x:[{xmin + args.padding:.2f}, {xmax - args.padding:.2f}], "
        f"y:[{ymin + args.padding:.2f}, {ymax - args.padding:.2f}] "
        f"+ padding {args.padding} m)",
        file=sys.stderr,
    )

    # Guard against pathological inputs producing multi-GB grids.
    # 100 M cells = ~100 MB at 1 byte/cell. Pick that as a soft cap and
    # bail with a helpful message rather than OOM-killing the host.
    if width * height > 100_000_000:
        print(
            f"ERROR: grid would be {width * height} cells "
            f"(>{100_000_000} cap). Use a coarser --resolution or "
            f"narrower --z-min/--z-max (or pre-crop the PCD).",
            file=sys.stderr,
        )
        return 1

    grid = np.full((height, width), PIX_UNKNOWN, dtype=np.uint8)

    # World -> pixel mapping. PGM row 0 is the top of the image but our
    # YAML origin is the bottom-left corner (Nav2 map_server convention),
    # so we flip the row axis.
    cols = ((sliced[:, 0] - xmin) / res).astype(np.int64)
    rows = height - 1 - ((sliced[:, 1] - ymin) / res).astype(np.int64)
    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    cols, rows = cols[valid], rows[valid]
    # Unique occupied cells (used both for anchor centroid and for the
    # ray-cast pass).
    occupied = np.unique(np.stack([cols, rows], axis=1), axis=0)

    if args.anchor_x is not None and args.anchor_y is not None:
        anchor_world = (args.anchor_x, args.anchor_y)
        anchor_source = "user"
    elif args.anchor_x is not None or args.anchor_y is not None:
        print(
            "ERROR: --anchor-x and --anchor-y must be passed together",
            file=sys.stderr,
        )
        return 1
    else:
        # Centroid of occupied cells in world coords. This is a coarse
        # but pose-free proxy for "somewhere the sensor was likely able
        # to see from". A U-shaped traverse can put the centroid outside
        # the navigable region -- the user should pass an explicit
        # --anchor-{x,y} in that case (see "Known concerns" in the
        # m5r-pipeline doc).
        cx_px = float(occupied[:, 0].mean())
        cy_px = float(occupied[:, 1].mean())
        ax = xmin + (cx_px + 0.5) * res
        ay = ymin + (height - 1 - cy_px + 0.5) * res
        anchor_world = (ax, ay)
        anchor_source = "auto, occupied centroid"

    anchor_col = int(round((anchor_world[0] - xmin) / res))
    anchor_row = height - 1 - int(round((anchor_world[1] - ymin) / res))
    print(
        f"Anchor ({anchor_source}): "
        f"({anchor_world[0]:.2f}, {anchor_world[1]:.2f})",
        file=sys.stderr,
    )
    # User-supplied anchor outside the grid would leave every cell
    # UNKNOWN and (worse) make Bresenham walk through ~200k off-grid
    # pixels per ray before hitting the in-grid bounds-guard. Skip the
    # whole pass and warn instead. The auto-centroid path can never
    # trip this.
    anchor_in_grid = 0 <= anchor_col < width and 0 <= anchor_row < height
    if not anchor_in_grid:
        print(
            f"WARNING: anchor pixel ({anchor_col}, {anchor_row}) is "
            f"outside the grid (0..{width-1}, 0..{height-1}); "
            "skipping ray-cast — all cells will be OCCUPIED or UNKNOWN.",
            file=sys.stderr,
        )

    if not args.no_raycast and anchor_in_grid:
        print(
            f"Ray-casting from anchor to {len(occupied)} occupied cells...",
            file=sys.stderr,
        )
        bresenham_rays_to_free(grid, (anchor_col, anchor_row), occupied)

    # Stamp occupied last so free-marking from later rays cannot
    # overwrite a real obstacle (defence in depth -- the ray walker
    # already respects this, but the explicit stamp removes any doubt).
    grid[rows, cols] = PIX_OCCUPIED

    n_occ = int((grid == PIX_OCCUPIED).sum())
    n_free = int((grid == PIX_FREE).sum())
    n_unk = int((grid == PIX_UNKNOWN).sum())
    print(
        f"Occupied: {n_occ} cells   Free: {n_free} cells   "
        f"Unknown: {n_unk} cells",
        file=sys.stderr,
    )

    # PGM P5 binary. The comment line carries the producer name so a
    # future debug session can grep the file and find the script that
    # made it. We deliberately omit timestamps / input paths from the
    # header: keeping the output byte-identical for identical inputs
    # preserves the idempotency contract documented in the help text.
    with pgm_path.open("wb") as f:
        f.write(b"P5\n")
        f.write(b"# m5r_pcd_to_occupancy.py output\n")
        f.write(f"{width} {height}\n".encode("ascii"))
        f.write(b"255\n")
        f.write(grid.tobytes())
    print(
        f"Wrote {pgm_path} (1 byte/pixel, {pgm_path.stat().st_size} bytes)"
    )

    # YAML output. Field order mirrors docs/maps/_template/occupancy.yaml
    # so a side-by-side diff highlights only the values that differ
    # (origin + the image filename if a future caller wants to override).
    yaml_text = (
        f"# Generated by scripts/m5r_pcd_to_occupancy.py from "
        f"{args.input_pcd.name}\n"
        f"image: occupancy.pgm\n"
        f"resolution: {res}\n"
        f"origin: [{xmin:.3f}, {ymin:.3f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: {args.occupied_thresh}\n"
        f"free_thresh: {args.free_thresh}\n"
    )
    yaml_path.write_text(yaml_text)
    print(f"Wrote {yaml_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
