#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Convert GLIM keyframe dumps into per-keyframe PCD files for DUFOMap.

This is the input adapter for M5R-4 (Issue #49, dynamic-object removal).
DUFOMap [KTH-RPL] takes per-scan PCD files annotated with a PCL
``VIEWPOINT`` header that carries the sensor pose at scan time.  GLIM
[koide3/glim], which we accepted as the M5-R map-building SLAM in
ADR-0003, instead serialises each keyframe to its own directory with:

* ``points_compact.bin`` — Nx3 ``Eigen::Vector3f`` raw dump, **expressed
  in the keyframe-local frame** (i.e. relative to the keyframe origin,
  not the world frame).
* ``data.txt`` — a text dump that opens with ``T_world_origin:`` followed
  by a 4x4 row-major float matrix giving the keyframe origin pose in the
  world (= map) frame.

We map these into the PCD-with-VIEWPOINT contract DUFOMap expects.
Concretely:

* The point cloud is written out untouched (still in keyframe-local
  coordinates).  We do **not** pre-transform to world here because
  DUFOMap's run() entry point applies the supplied pose internally
  (``cloud_transform=False``), and pre-transforming would double-apply.
* The VIEWPOINT header is filled from T_world_origin's translation and
  rotation.  PCL's VIEWPOINT quaternion order is ``qw qx qy qz``, which
  is the opposite of scipy's (x, y, z, w); the conversion here is
  explicit so the next reader doesn't have to remember which way scipy
  goes.

We write PCD ASCII (not binary) for two reasons:

1. The volumes are small (50 k points per keyframe in the 2026-06-21
   bench run = ~600 KiB ASCII).  Binary saves disk and parse time, but
   not enough to justify a binary writer when ASCII keeps the files
   diff-able and debuggable.
2. ``open3d`` is intentionally not pulled in here — the visual-diff tool
   (``m5r_dufomap_diff.py``) needs it, but the converter does not, and
   keeping it light means CI / headless runs can use this script with
   only numpy + scipy.

Idempotency: refuses to overwrite an existing output directory unless
``--force`` is passed.  This mirrors ``scripts/m5r3_run_glim.sh``'s
``--force`` convention so an accidental re-run cannot silently destroy
the staging artefacts a later DUFOMap run depends on.

Usage:

    scripts/m5r_glim_to_pcd.py \\
        --glim-out docs/m5r-bench-data/<run>/glim-out \\
        --out-dir /tmp/m5r49_staging

Output layout (matches DUFOMap's expected ``<scene>/pcd/*.pcd`` shape):

    <out-dir>/
      pcd/
        000000.pcd
        000001.pcd
        ...
"""

import argparse
import re
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


# T_world_origin appears once near the top of every GLIM data.txt.  We
# anchor on the literal label, skip its (possibly empty) value on the
# same line, then read the next 4 lines as the 4x4 matrix.  Doing this
# with regex + line slicing instead of a streaming parser keeps the
# parser trivial and resilient to the per-row whitespace varying with
# GLIM's pretty-printer alignment (e.g. some rows have 8 spaces, some 10).
_LABEL = "T_world_origin:"


def parse_t_world_origin(data_txt: Path) -> np.ndarray:
    """Return the 4x4 T_world_origin matrix from a GLIM data.txt.

    GLIM serialises each keyframe's pose-in-world as a row-major float
    matrix immediately after the ``T_world_origin:`` label line. We
    require all four rows to parse cleanly as 4 floats each — partial
    matches would silently corrupt downstream geometry.
    """
    lines = data_txt.read_text().splitlines()
    label_idx = None
    for i, line in enumerate(lines):
        if line.strip() == _LABEL:
            label_idx = i
            break
    if label_idx is None:
        raise ValueError(f"{data_txt}: '{_LABEL}' label not found")

    matrix_lines = lines[label_idx + 1: label_idx + 5]
    if len(matrix_lines) < 4:
        raise ValueError(
            f"{data_txt}: only {len(matrix_lines)} lines after "
            f"'{_LABEL}', need 4"
        )

    rows = []
    for row_idx, raw in enumerate(matrix_lines):
        # GLIM aligns columns with leading spaces; split on whitespace.
        tokens = raw.strip().split()
        if len(tokens) != 4:
            raise ValueError(
                f"{data_txt}: row {row_idx} of T_world_origin had "
                f"{len(tokens)} tokens, expected 4: {raw!r}"
            )
        rows.append([float(tok) for tok in tokens])

    return np.asarray(rows, dtype=np.float64)


def t_world_origin_to_viewpoint(T: np.ndarray) -> tuple[float, float, float,
                                                        float, float, float,
                                                        float]:
    """Map a 4x4 pose into (tx, ty, tz, qw, qx, qy, qz) for PCL VIEWPOINT.

    PCL stores VIEWPOINT as translation followed by quaternion in
    (w, x, y, z) order.  scipy.spatial.transform.Rotation.as_quat
    returns (x, y, z, w), so we re-order explicitly.
    """
    if T.shape != (4, 4):
        raise ValueError(f"expected 4x4 matrix, got shape {T.shape}")
    tx, ty, tz = T[0, 3], T[1, 3], T[2, 3]
    qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
    return (float(tx), float(ty), float(tz),
            float(qw), float(qx), float(qy), float(qz))


def write_pcd_ascii(out_path: Path,
                    points: np.ndarray,
                    viewpoint: tuple[float, float, float,
                                     float, float, float, float]) -> None:
    """Write an XYZ ASCII PCD file with a VIEWPOINT header.

    The header layout matches PCL's format spec
    (https://pointclouds.org/documentation/tutorials/pcd_file_format.html).
    We emit only the XYZ channel — intensities are available in GLIM's
    ``intensities_compact.bin`` alongside the points, but DUFOMap's
    occupancy-based filter does not use intensity, so carrying it here
    would only inflate file size.
    """
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points must be Nx3, got shape {points.shape}")

    n = points.shape[0]
    tx, ty, tz, qw, qx, qy, qz = viewpoint

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        f"VIEWPOINT {tx} {ty} {tz} {qw} {qx} {qy} {qz}\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )

    with out_path.open("w") as f:
        f.write(header)
        # %g keeps the file readable while preserving float32 precision
        # within ~7 significant digits — adequate for VLP-16 range
        # accuracy (~3 cm at 100 m).
        for x, y, z in points:
            f.write(f"{x:.6g} {y:.6g} {z:.6g}\n")


# GLIM keyframe directories are six-digit zero-padded numerics
# (000000, 000001, ...).  Anchoring on the regex avoids accidentally
# matching auxiliary directories like config/ which sit at the same
# level inside glim-out/.
_KEYFRAME_DIR_RE = re.compile(r"^\d{6}$")


def find_keyframes(glim_out: Path) -> list[Path]:
    """Return GLIM keyframe directories in numeric order.

    GLIM emits one directory per keyframe with names matching
    ``[0-9]{6}``.  Other top-level entries (``config/``, ``traj_*.txt``,
    ``graph.bin``, etc.) are filtered out so accidental file additions
    next to the keyframes don't break the converter.
    """
    if not glim_out.is_dir():
        raise FileNotFoundError(f"GLIM output dir not found: {glim_out}")
    keyframes = sorted(
        p for p in glim_out.iterdir()
        if p.is_dir() and _KEYFRAME_DIR_RE.match(p.name)
    )
    if not keyframes:
        raise FileNotFoundError(
            f"No keyframe directories (NNNNNN) under {glim_out}"
        )
    return keyframes


def convert_keyframe(kf_dir: Path, out_pcd: Path) -> int:
    """Convert one GLIM keyframe directory into one PCD file.

    Returns the number of points written.  Raises FileNotFoundError if
    either required input file is missing — we treat partial keyframes
    as a hard error rather than silently skipping, because GLIM only
    dumps a directory once both files are written (so missing files
    indicate disk corruption or a truncated run).
    """
    points_bin = kf_dir / "points_compact.bin"
    data_txt = kf_dir / "data.txt"
    if not points_bin.is_file():
        raise FileNotFoundError(f"{points_bin} missing")
    if not data_txt.is_file():
        raise FileNotFoundError(f"{data_txt} missing")

    raw = np.fromfile(points_bin, dtype=np.float32)
    if raw.size % 3 != 0:
        raise ValueError(
            f"{points_bin}: size {raw.size} floats not divisible by 3"
        )
    points = raw.reshape(-1, 3)

    T_wo = parse_t_world_origin(data_txt)
    viewpoint = t_world_origin_to_viewpoint(T_wo)

    write_pcd_ascii(out_pcd, points, viewpoint)
    return points.shape[0]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Convert GLIM keyframe dumps (NNNNNN/points_compact.bin + "
            "data.txt) into per-keyframe PCD files with PCL VIEWPOINT "
            "headers, ready for DUFOMap (Issue #49)."
        ),
    )
    parser.add_argument(
        "--glim-out", required=True, type=Path,
        help="GLIM output directory (contains NNNNNN/ keyframe subdirs)",
    )
    parser.add_argument(
        "--out-dir", required=True, type=Path,
        help="Target staging directory; pcd/ subdir is created inside",
    )
    parser.add_argument(
        "--force", action="store_true",
        help=(
            "Overwrite an existing <out-dir>/pcd/ subtree. Default is to "
            "abort if pcd/ exists, mirroring scripts/m5r3_run_glim.sh."
        ),
    )
    args = parser.parse_args()

    try:
        keyframes = find_keyframes(args.glim_out)
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    pcd_dir = args.out_dir / "pcd"
    if pcd_dir.exists() and any(pcd_dir.iterdir()) and not args.force:
        print(
            f"ERROR: {pcd_dir} is not empty. Re-run with --force to "
            f"overwrite.", file=sys.stderr,
        )
        return 1
    pcd_dir.mkdir(parents=True, exist_ok=True)

    total_points = 0
    for kf in keyframes:
        out_pcd = pcd_dir / f"{kf.name}.pcd"
        try:
            n = convert_keyframe(kf, out_pcd)
        except (FileNotFoundError, ValueError) as exc:
            print(f"ERROR converting {kf}: {exc}", file=sys.stderr)
            return 1
        total_points += n
        print(f"{kf.name}: {n} points -> {out_pcd}")

    print(
        f"\nWrote {len(keyframes)} PCD files "
        f"({total_points} points total) to {pcd_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
