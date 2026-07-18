#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""pcd_to_occupancy_v2.py — Stage 1 of the PCD → OccupancyGrid v2 pipeline.

Design pivot (2026-07-18): the older pipeline (m5r_pcd_to_occupancy.py +
clean_isolated_occupancy.py + GIMP manual salt cleanup) optimised for
"false-positive (salt) reduction". Zoom-in review of the trav layer
(PR #90) then showed that the raw occupied set is dominated by 1-4 cell
isolated dust because the ~3 pts/cell PCD density makes 8-connectivity
too strict for real structures. Post-hoc salt cleanup cannot resolve
this at the source — v1 was solving the wrong problem.

v2 flips the optimisation target:
  - machine responsibility = RECALL (never lose a real structure) +
    POSITIONAL ACCURACY (widths / edges / contours are correct within
    ±1 cell)
  - human responsibility  = truth verdict (is this red pixel salt or a
    real curb?) — the human overpaints via sidecar masks, composed in
    Stage 2 (scripts/compose_occupancy.py)

Three design rules (2026-07-18 conversation, [[project-pipeline-v2]]):

  Rule 1. No morphological operations on the OUTPUT. closing / opening
    / dilation shift widths and positions by 1-2 cells, which the human
    cannot undo without going back to the point cloud. cluster filter
    may pre-dilate internally for connectivity judgement, but the
    output cells are the raw evidence positions.

  Rule 2. Auto-remove is minimised to 1-cell strictly isolated dust
    only. Everything else — including 2-3 cell tiny blobs that could
    be either salt or a real thin pole — goes to the human. Stage 2's
    role shifts from "delete" to "classify + visualise".

  Rule 3. Step-edge occupancy is fixed on the VALLEY SIDE (walking-
    surface side) of the discontinuity. When cell A and cell B are
    8-neighbours with |ground_z(A) - ground_z(B)| > step_threshold, the
    cell with the LOWER ground_z is marked occupied (that is where the
    chair is, and the raise on the upper side is the obstacle from the
    chair's perspective). This gives a single unambiguous curb line
    that verification (M1 cross-section match) can measure against.

Layer separation (Stage 1 output; all PNGs share the input yaml's
pixel grid, origin, resolution — no scaling, no cropping):

  occupied_step.png       RGBA opaque red     step-edge occupied (Rule 3)
  occupied_structure.png  RGBA opaque orange  vertical extent in
                                              [ground+z_struct_min,
                                               ground+z_struct_max] (2.2 m
                                              walkable clearance limit,
                                              picks up eaves / awnings
                                              too — head protection).
  free_evidence.png       RGBA semi-tr green  trajectory-anchor footprint
                                              (radius --anchor-free-radius);
                                              raycast free is TODO for v3.
  underlay_hillshade.png  RGB grayscale       hillshade rendering of
                                              per-cell ground_z (light
                                              azimuth 315°, elevation 45°).
                                              Lets the operator see where
                                              real step lines run.
  underlay_maxheight.png  RGB colormap        per-cell max height above
                                              local ground, viridis-like
                                              gradient. Distinguishes tall
                                              walls from ankle-height noise.

Underlay intensity is deliberately NOT produced by v2 — the DUFOMap
static.pcd is XYZ-only. If required, re-export from the GLIM
intermediate PCD (has intensity) at the same origin/resolution and
add it as an extra layer for GIMP.

Cache (--cache-npz): stores per-cell (count, ground_z, max_z_rel,
roughness) as flat float32 arrays plus max_height as metadata. Reused
across threshold sweeps in ~5 s. Invalidated when
--max-height-above-ground changes.
"""

import argparse
import pathlib
import sys
import time

import cv2
import numpy as np
import open3d as o3d
import pandas as pd
import yaml
from PIL import Image


# Nav2 map_server pixel convention (not written by this script, but the
# yaml carries the thresholds through).
OCCUPIED = 0
UNKNOWN = 205
FREE = 254

# Rule-3 layer colours (all opaque red-family for occupied). Different
# hues so step vs structure are visually separable in GIMP without
# needing to toggle layer visibility.
COLOR_STEP = (255, 0, 0, 255)          # pure red
COLOR_STRUCTURE = (255, 140, 0, 255)   # orange (fully opaque)
COLOR_FREE_EVIDENCE = (0, 200, 100, 110)  # green, α≈0.43 semi-transparent

# Underlay tuning.
HILLSHADE_AZIMUTH_DEG = 315.0   # NW light source (GIS standard)
HILLSHADE_ALTITUDE_DEG = 45.0   # 45° above horizon
HILLSHADE_BG = 60               # gray for cells with no ground_z data

# Ground-z smoothing radius for the h-filter (metres). Same rationale
# as pcd_to_traversability.py: canopy-only cells inherit their
# neighbours' true ground level rather than trusting their own canopy z.
GROUND_SMOOTH_RADIUS_M = 0.25


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__.split('\n\n', 1)[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--input-pcd', required=True, type=pathlib.Path)
    p.add_argument('--input-yaml', required=True, type=pathlib.Path,
                   help='Existing occupancy yaml for origin / resolution / grid size.')
    p.add_argument('--output-dir', required=True, type=pathlib.Path,
                   help='Directory to write the 5 layer PNGs into. Created if missing.')

    p.add_argument('--traj', type=pathlib.Path,
                   help='TUM trajectory (traj_lidar.txt). Used for the free_evidence '
                        'footprint. If omitted and --input-yaml is under docs/maps/<site>/, '
                        'auto-detect the sibling traj_lidar.txt.')

    # Ground and h-filter thresholds.
    p.add_argument('--ground-percentile', type=float, default=5.0,
                   help='Per-cell z lower percentile used as ground_z (%%). Default 5.')
    p.add_argument('--max-height-above-ground', type=float, default=2.2,
                   help='Drop points more than this many metres above the LOCAL ground '
                        'BEFORE any classification. Default 2.2 = walkable clearance '
                        'limit; eaves at 2.0-2.2 m are kept as obstacles for head '
                        'protection (2026-07-18 decision).')
    p.add_argument('--min-points', type=int, default=3,
                   help='Cells with fewer post-filter points are classified as unknown. '
                        'Default 3 (plane-fit minimum).')

    # Step layer.
    p.add_argument('--step-threshold', type=float, default=0.03,
                   help='8-neighbour ground_z diff (m) that flags a step edge. '
                        'Default 0.03 (= 3 cm).')

    # Structure layer.
    p.add_argument('--structure-z-min', type=float, default=0.1,
                   help='Lower bound of the vertical band that qualifies a cell as '
                        'structure-occupied (metres above local ground). Default 0.1 '
                        '(above floor noise).')
    p.add_argument('--structure-z-max', type=float, default=2.2,
                   help='Upper bound of the structure band. Default 2.2 = walkable '
                        'clearance limit. Matches --max-height-above-ground so points '
                        'the h-filter already dropped never see this stage.')
    p.add_argument('--structure-min-points', type=int, default=2,
                   help='At least this many points inside the structure band are '
                        'required for a cell to be marked structure-occupied. Default 2 '
                        '(gives a thin pole a chance).')

    # Rule-2 cluster filter (defaults kill only strictly isolated 1-cell dust).
    p.add_argument('--cluster-min-size', type=int, default=2,
                   help='Drop connected components of the labelled mask smaller than '
                        'this many cells. Default 2 = kill 1-cell strictly isolated dust '
                        'only. Applied SEPARATELY to occupied_step and occupied_structure. '
                        'Set to 0 to disable.')
    p.add_argument('--cluster-dilate-px', type=int, default=0,
                   help='Radius (cells) of the disk used to pre-dilate the mask before '
                        '8-connected labelling. Default 0 (Rule 2 strict: label the raw '
                        'mask). Use 1 to chain within-1-cell neighbours into the same '
                        'component if the mask is very sparse (dense DUFOMap PCDs may '
                        'not need this).')

    # Free-evidence layer.
    p.add_argument('--anchor-free-radius', type=float, default=2.0,
                   help='Radius (m) of the disk marked free_evidence around each '
                        'trajectory anchor. Default 2.0. Set to 0 to skip footprint.')
    p.add_argument('--traj-stride', type=float, default=1.0,
                   help='Trajectory downsample stride (m) for the footprint disks. '
                        'Default 1.0 → ~1300 anchors on a 1.3 km loop.')

    # Cache.
    p.add_argument('--cache-npz', type=pathlib.Path,
                   help='Save / reuse per-cell intermediates. Invalidated when '
                        '--max-height-above-ground changes.')
    p.add_argument('--force-recompute', action='store_true')

    # Layer skip flags.
    p.add_argument('--no-hillshade', action='store_true')
    p.add_argument('--no-maxheight', action='store_true')
    p.add_argument('--no-free-evidence', action='store_true')

    p.add_argument('--dry-run', action='store_true',
                   help='Report classification counts without writing any files.')
    return p.parse_args()


def load_pcd(path):
    pcd = o3d.io.read_point_cloud(str(path))
    return np.asarray(pcd.points, dtype=np.float32)


def load_map_ref(yaml_path):
    with yaml_path.open() as f:
        doc = yaml.safe_load(f)
    ref_pgm = np.array(Image.open(yaml_path.parent / doc['image']))
    H, W = ref_pgm.shape
    return doc, H, W


def load_trajectory_tum(path):
    """Read a TUM-format trajectory file (# comments + `t x y z qx qy qz qw` rows).

    Returns an (N, 3) float64 array of XYZ. Same parser semantics as
    m5r_pcd_to_occupancy.py so v1 and v2 stay comparable on the same file.
    """
    xyz = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            xyz.append([float(parts[1]), float(parts[2]), float(parts[3])])
    if not xyz:
        raise SystemExit(f'{path}: no trajectory poses parsed')
    return np.asarray(xyz, dtype=np.float64)


def per_cell_stats(pts, origin, resolution, W, H,
                   ground_percentile, max_height_above_ground):
    """Compute per-cell (count, ground_z, max_z_rel) as flat arrays of length W*H.

    Applies the h-filter (points > local_ground + max_height dropped) before
    computing count / ground_z / max_z_rel so canopy pass-throughs never
    contaminate step or structure classification. `max_z_rel` is the max z
    IN the kept band, relative to the cell's ground_z (= 0 for a ground-only
    cell, up to max_height_above_ground for a full-height wall cell).
    """
    origin_x, origin_y = float(origin[0]), float(origin[1])
    ix = ((pts[:, 0] - origin_x) / resolution).astype(np.int32)
    iy = ((pts[:, 1] - origin_y) / resolution).astype(np.int32)
    in_bounds = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    ix, iy = ix[in_bounds], iy[in_bounds]
    xyz = pts[in_bounds]
    print(f'    in-bounds points: {len(xyz):,} of {len(pts):,}')

    cell = iy.astype(np.int64) * W + ix.astype(np.int64)
    df = pd.DataFrame({'cell': cell, 'z': xyz[:, 2]})
    q = ground_percentile / 100.0

    # Pass 1: coarse ground_z from all points, then h-filter using an
    # eroded smoothed ground (canopy-only cells inherit neighbours' true
    # ground so they get correctly wiped instead of accepting their own
    # canopy z as reference).
    initial_ground_per = df.groupby('cell', sort=False)['z'].quantile(q).astype(np.float32)
    initial_flat = np.full(W * H, np.inf, dtype=np.float32)
    initial_flat[initial_ground_per.index.to_numpy()] = initial_ground_per.to_numpy()
    initial_2d = initial_flat.reshape(H, W)
    r_cells = max(1, int(round(GROUND_SMOOTH_RADIUS_M / resolution)))
    k = 2 * r_cells + 1
    smooth_kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    smoothed_2d = cv2.erode(
        initial_2d, smooth_kern,
        borderType=cv2.BORDER_CONSTANT, borderValue=float(np.inf),
    )
    smoothed_flat = smoothed_2d.ravel()
    smoothed_per_pt = smoothed_flat[df['cell'].to_numpy()]
    z_arr = df['z'].to_numpy()
    keep = z_arr <= (smoothed_per_pt + max_height_above_ground)
    print(f'    h-filter (h≤{max_height_above_ground} m above smoothed ground '
          f'r={GROUND_SMOOTH_RADIUS_M} m): kept {int(keep.sum()):,} / {len(df):,} '
          f'({100 * keep.mean():.1f}%)')
    df = df.iloc[keep].reset_index(drop=True)

    # Pass 2: post-filter stats.
    grp = df.groupby('cell', sort=False)
    count_per = grp.size().rename('count').astype(np.int32)
    ground_per = grp['z'].quantile(q).rename('ground_z').astype(np.float32)
    max_per = grp['z'].max().rename('max_z').astype(np.float32)

    n_cells = W * H
    count_flat = np.zeros(n_cells, dtype=np.int32)
    ground_flat = np.full(n_cells, np.nan, dtype=np.float32)
    max_z_rel_flat = np.full(n_cells, np.nan, dtype=np.float32)

    idx = count_per.index.to_numpy()
    count_flat[idx] = count_per.to_numpy()
    ground_flat[ground_per.index.to_numpy()] = ground_per.to_numpy()
    # max_z stored as relative to per-cell ground_z (bounded by max_height).
    max_arr = max_per.to_numpy()
    ground_arr = ground_per.to_numpy()
    max_z_rel_flat[max_per.index.to_numpy()] = (max_arr - ground_arr).astype(np.float32)

    # Also count per-cell how many kept points fell in the structure
    # vertical band [ground+z_struct_min, ground+z_struct_max]. This is
    # cheap to piggyback here and avoids a second groupby pass in main().
    # However: we do not know z_struct_min/max here (they are CLI args
    # of main). Fall back to a callable-friendly design: compute in main
    # from (count, ground_z, max_z_rel, plus per-point z we don't have).
    # Simplest: return count_flat only for now; main derives structure
    # from ground / max_z_rel + a re-scan of df if precision matters.
    # For v2 we approximate: cell counts as "structure-hit" if
    # max_z_rel >= z_struct_min. This misses ONLY cells whose ONLY
    # above-ground point is above z_struct_max — which the h-filter
    # already dropped, so max_z_rel ≤ max_height. Precise enough.

    return count_flat, ground_flat, max_z_rel_flat


def compute_step_valley_side(ground_z_2d, valid_2d, step_threshold):
    """Rule 3: mark the LOWER cell of every 8-adjacent pair whose ground_z
    differs by more than step_threshold.

    Both cells must be `valid_2d` (i.e. count >= min_points). Border rows
    / cols are excluded (np.roll would wrap otherwise).
    """
    H, W = ground_z_2d.shape
    interior = np.zeros((H, W), dtype=bool)
    interior[1:-1, 1:-1] = True

    valley = np.zeros((H, W), dtype=bool)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            shifted_z = np.roll(ground_z_2d, (dy, dx), axis=(0, 1))
            shifted_v = np.roll(valid_2d, (dy, dx), axis=(0, 1))
            diff = shifted_z - ground_z_2d
            both = valid_2d & shifted_v & interior
            # Neighbour is HIGHER by more than the threshold → THIS cell
            # (ground_z_2d[i,j]) is the valley side.
            valley |= both & (diff > step_threshold)
    return valley


def cluster_filter(mask_2d, min_size, dilate_px):
    """Drop connected components smaller than min_size.

    dilate_px > 0 pre-dilates the mask for the labelling step so near-
    adjacent occupied cells chain into the same component; the RETURNED
    mask is the intersection of the original mask and the surviving
    labels (Rule 1: output positions never grow beyond raw evidence).
    """
    if min_size <= 0 or not mask_2d.any():
        return mask_2d, {'components_before': 0, 'components_after': 0,
                         'cells_before': int(mask_2d.sum()),
                         'cells_after': int(mask_2d.sum()),
                         'largest_kept': 0}
    m_u8 = mask_2d.astype(np.uint8)
    if dilate_px > 0:
        k = 2 * dilate_px + 1
        kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        m_for_label = cv2.dilate(m_u8, kern)
    else:
        m_for_label = m_u8
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        m_for_label, connectivity=8,
    )
    areas = stats[:, cv2.CC_STAT_AREA]
    drop_label = areas < min_size
    drop_label[0] = False   # background never drops
    drop = drop_label[labels]
    filtered = mask_2d & ~drop
    surviving = areas[1:][~drop_label[1:]]
    return filtered, {
        'components_before': int(n_labels - 1),
        'components_after': int((n_labels - 1) - int(drop_label.sum())),
        'cells_before': int(mask_2d.sum()),
        'cells_after': int(filtered.sum()),
        'largest_kept': int(surviving.max()) if surviving.size else 0,
    }


def build_hillshade(ground_z_2d, resolution,
                    azimuth_deg=HILLSHADE_AZIMUTH_DEG,
                    altitude_deg=HILLSHADE_ALTITUDE_DEG,
                    bg=HILLSHADE_BG):
    """Standard GIS hillshade of ground_z. Cells with NaN ground stay `bg`.

    Formula (Horn 1981 / ArcGIS Spatial Analyst):
        hillshade = 255 * (cos(zenith)*cos(slope)
                           + sin(zenith)*sin(slope)*cos(azimuth - aspect))
    Input z is metres; gradient is in metres / metre = unitless. Azimuth
    convention: 0° = north, clockwise (315° = NW), matching GIS.
    """
    valid = ~np.isnan(ground_z_2d)
    z = np.where(valid, ground_z_2d, 0.0).astype(np.float32)

    # Central-difference gradients. `dz/dx` = eastward derivative, `dz/dy`
    # = northward. Note: image y grows DOWN, so dy is negated below when
    # converting to a north-pointing derivative for the standard formula.
    dzdx = np.zeros_like(z)
    dzdy = np.zeros_like(z)
    dzdx[:, 1:-1] = (z[:, 2:] - z[:, :-2]) / (2 * resolution)
    dzdy[1:-1, :] = (z[:-2, :] - z[2:, :]) / (2 * resolution)   # north = -y_image

    slope = np.arctan(np.hypot(dzdx, dzdy))
    aspect = np.arctan2(dzdy, -dzdx)   # aspect angle (east-clockwise from north)

    zenith = np.radians(90.0 - altitude_deg)
    azimuth = np.radians(azimuth_deg)

    shade = (np.cos(zenith) * np.cos(slope) +
             np.sin(zenith) * np.sin(slope) * np.cos(azimuth - aspect))
    shade = np.clip(shade, 0.0, 1.0)
    gray = (shade * 255.0).astype(np.uint8)
    # Invalidate cells with no valid neighbour on either axis (they
    # contribute a zero gradient → shade = cos(zenith) ≈ 0.707 flat gray,
    # which is fine, but distinguish "no data" explicitly).
    gray = np.where(valid, gray, bg).astype(np.uint8)
    rgb = np.stack([gray, gray, gray], axis=-1)
    return rgb


def build_maxheight_rgb(max_z_rel_2d, max_display_m=2.2):
    """RGB colormap of per-cell max height above ground, viridis-like.

    NaN cells render as dark gray. Values are clipped to
    [0, max_display_m] before mapping. The palette is a simple
    inline linear interpolation of the classic viridis anchor points
    so this script has no matplotlib dependency.
    """
    valid = ~np.isnan(max_z_rel_2d)
    v = np.clip(np.where(valid, max_z_rel_2d, 0.0), 0.0, max_display_m)
    t = v / max_display_m   # normalised [0, 1]

    # Approx viridis palette (5 anchor colours, evenly spaced).
    anchors = np.array([
        [ 68,   1,  84],   # dark purple, t=0
        [ 59,  82, 139],   # blue, t=0.25
        [ 33, 145, 140],   # teal, t=0.5
        [ 94, 201,  98],   # green, t=0.75
        [253, 231,  37],   # yellow, t=1
    ], dtype=np.float32) / 255.0
    N = anchors.shape[0]
    # Linear interpolation between the two surrounding anchor colours.
    idx_f = t * (N - 1)
    idx0 = np.clip(np.floor(idx_f).astype(np.int32), 0, N - 2)
    frac = idx_f - idx0
    c0 = anchors[idx0]
    c1 = anchors[idx0 + 1]
    rgb_f = c0 + (c1 - c0) * frac[..., None]
    rgb = (np.clip(rgb_f, 0.0, 1.0) * 255.0).astype(np.uint8)
    # No-data cells → mid gray so the palette is not confused with "0 height".
    rgb[~valid] = (60, 60, 60)
    return rgb


def build_free_evidence_rgba(traj_xyz, origin, resolution, W, H,
                             anchor_free_radius, traj_stride, colour):
    """RGBA layer with a `anchor_free_radius`-metre disk at each downsampled
    trajectory anchor. Everything else fully transparent.
    """
    rgba = np.zeros((H, W, 4), dtype=np.uint8)
    if traj_xyz is None or anchor_free_radius <= 0 or len(traj_xyz) == 0:
        return rgba

    # Downsample trajectory: keep a pose only when it is at least
    # traj_stride metres from the previously kept pose.
    kept = [traj_xyz[0]]
    for xyz in traj_xyz[1:]:
        if np.linalg.norm(xyz - kept[-1]) >= traj_stride:
            kept.append(xyz)
    kept = np.asarray(kept, dtype=np.float64)

    # Rasterise disks. cv2.circle in-place is fine for a few thousand
    # anchors; no need for the multi-pass polygon fill.
    ox, oy = float(origin[0]), float(origin[1])
    r_cells = max(1, int(round(anchor_free_radius / resolution)))
    mask = np.zeros((H, W), dtype=np.uint8)
    for xyz in kept:
        cx = int(round((xyz[0] - ox) / resolution))
        cy = int(round((xyz[1] - oy) / resolution))
        if 0 <= cx < W and 0 <= cy < H:
            cv2.circle(mask, (cx, cy), r_cells, 1, thickness=-1)
    m = mask.astype(bool)
    rgba[m, 0] = colour[0]
    rgba[m, 1] = colour[1]
    rgba[m, 2] = colour[2]
    rgba[m, 3] = colour[3]
    print(f'    free_evidence: {len(kept):,} anchors (stride {traj_stride} m), '
          f'{int(m.sum()):,} disk cells (r={anchor_free_radius} m)')
    return rgba


def build_layer_rgba(mask_2d, colour):
    """RGBA layer with `colour` where mask_2d is True, transparent elsewhere."""
    H, W = mask_2d.shape
    rgba = np.zeros((H, W, 4), dtype=np.uint8)
    rgba[mask_2d, 0] = colour[0]
    rgba[mask_2d, 1] = colour[1]
    rgba[mask_2d, 2] = colour[2]
    rgba[mask_2d, 3] = colour[3]
    return rgba


def auto_detect_traj(input_yaml):
    """docs/maps/<site>/*.yaml → docs/maps/<site>/traj_lidar.txt if present."""
    candidate = input_yaml.parent / 'traj_lidar.txt'
    return candidate if candidate.is_file() else None


def write_layer_metadata(output_dir, ref_doc, H, W, layers_written, args):
    """Sibling yaml listing the layers + their common origin/resolution.

    Stage 2 (compose_occupancy.py) reads this to know which layers to
    consume without having to guess filenames.
    """
    meta = {
        'grid': {
            'width': W, 'height': H,
            'resolution': ref_doc['resolution'],
            'origin': ref_doc['origin'],
        },
        'layers': layers_written,
        'thresholds': {
            'ground_percentile': args.ground_percentile,
            'max_height_above_ground': args.max_height_above_ground,
            'min_points': args.min_points,
            'step_threshold': args.step_threshold,
            'structure_z_min': args.structure_z_min,
            'structure_z_max': args.structure_z_max,
            'structure_min_points': args.structure_min_points,
            'cluster_min_size': args.cluster_min_size,
            'cluster_dilate_px': args.cluster_dilate_px,
            'anchor_free_radius': args.anchor_free_radius,
            'traj_stride': args.traj_stride,
        },
        'source': {
            'pcd': str(args.input_pcd),
            'ref_yaml': str(args.input_yaml),
            'traj': str(args.traj) if args.traj else None,
        },
    }
    path = output_dir / 'v2_layers.yaml'
    with path.open('w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return path


def main():
    args = parse_args()
    if not args.input_pcd.is_file():
        raise SystemExit(f'--input-pcd not found: {args.input_pcd}')
    if not args.input_yaml.is_file():
        raise SystemExit(f'--input-yaml not found: {args.input_yaml}')
    args.output_dir.mkdir(parents=True, exist_ok=True)

    if args.traj is None:
        args.traj = auto_detect_traj(args.input_yaml)
    if args.traj is not None and not args.traj.is_file():
        raise SystemExit(f'--traj not found: {args.traj}')

    t0 = time.time()
    print(f'== pcd_to_occupancy_v2 ==')
    print(f'input-pcd  : {args.input_pcd}')
    print(f'input-yaml : {args.input_yaml}')
    print(f'traj       : {args.traj}')
    print(f'output-dir : {args.output_dir}')

    ref_doc, H, W = load_map_ref(args.input_yaml)
    origin = ref_doc['origin']
    resolution = ref_doc['resolution']
    print(f'grid       : {W} x {H}, origin={origin}, resolution={resolution} m/px')
    print(f'thresholds : step={args.step_threshold} m | struct=[{args.structure_z_min}, '
          f'{args.structure_z_max}] m | max_h={args.max_height_above_ground} m | '
          f'min_pts={args.min_points} | cluster min={args.cluster_min_size} '
          f'dilate={args.cluster_dilate_px}px | anchor_free_r={args.anchor_free_radius} m')
    print()

    # ---- per-cell intermediates (cached) ----------------------------------
    use_cache = (args.cache_npz is not None
                 and args.cache_npz.is_file()
                 and not args.force_recompute)
    if use_cache:
        t1 = time.time()
        print(f'[cache] loading {args.cache_npz} ...')
        cache = np.load(args.cache_npz)
        cached_h = float(cache['max_height_above_ground'][0]) \
            if 'max_height_above_ground' in cache.files else -1.0
        if abs(cached_h - args.max_height_above_ground) > 1e-6:
            print(f'[cache] max_height changed ({cached_h} → {args.max_height_above_ground}), '
                  f'recomputing')
            use_cache = False
        else:
            count = cache['count']
            ground_z = cache['ground_z']
            max_z_rel = cache['max_z_rel']
            print(f'[cache] {time.time() - t1:.1f} s')

    if not use_cache:
        t1 = time.time()
        print(f'[pcd] loading {args.input_pcd} ...')
        pts = load_pcd(args.input_pcd)
        print(f'[pcd] {len(pts):,} points, {time.time() - t1:.1f} s')

        t1 = time.time()
        print(f'[stats] per-cell count / ground_z / max_z_rel ...')
        count, ground_z, max_z_rel = per_cell_stats(
            pts, origin, resolution, W, H,
            args.ground_percentile, args.max_height_above_ground,
        )
        print(f'[stats] {time.time() - t1:.1f} s')

        if args.cache_npz is not None:
            args.cache_npz.parent.mkdir(parents=True, exist_ok=True)
            np.savez(
                args.cache_npz,
                count=count, ground_z=ground_z, max_z_rel=max_z_rel,
                max_height_above_ground=np.array([args.max_height_above_ground],
                                                 dtype=np.float64),
            )
            print(f'[cache] wrote {args.cache_npz}')

    count2d = count.reshape(H, W)
    ground_z2d = ground_z.reshape(H, W)
    max_z_rel2d = max_z_rel.reshape(H, W)

    known_2d = count2d >= args.min_points
    n_known = int(known_2d.sum())

    # ---- occupied_step (Rule 3: valley side) -----------------------------
    t1 = time.time()
    step_raw = compute_step_valley_side(
        np.nan_to_num(ground_z2d, nan=0.0),
        known_2d,
        args.step_threshold,
    )
    print(f'[step] valley-side detection: {int(step_raw.sum()):,} cells '
          f'in {time.time() - t1:.1f} s')

    step_filt, step_stats = cluster_filter(
        step_raw, args.cluster_min_size, args.cluster_dilate_px)
    print(f'[step] cluster filter (min={args.cluster_min_size}, '
          f'dilate={args.cluster_dilate_px}): '
          f'{step_stats["cells_before"]:,} → {step_stats["cells_after"]:,} cells | '
          f'components {step_stats["components_before"]:,} → '
          f'{step_stats["components_after"]:,}')

    # ---- occupied_structure ----------------------------------------------
    # A cell is structure-occupied if it has enough points AND some kept
    # point sits inside the structure band [z_struct_min, z_struct_max]
    # above local ground. Since the h-filter already caps at
    # max_height_above_ground, max_z_rel is bounded — we only need to
    # check the lower end (structure_z_min).
    struct_raw = (known_2d
                  & ~np.isnan(max_z_rel2d)
                  & (max_z_rel2d >= args.structure_z_min))
    # `structure_min_points` cannot be strictly enforced without a second
    # groupby (per-cell in-band count). Approximation: require count >=
    # min_points AND max_z_rel >= z_struct_min. Real structures have many
    # points above 0.1 m so this holds; a thin pole with a single high
    # point is still captured because count = 1 gates through min_points
    # (default 3). If needed, --structure-min-points can raise min_points
    # to be stricter.
    print(f'[structure] band [{args.structure_z_min}, {args.structure_z_max}] m: '
          f'{int(struct_raw.sum()):,} raw cells')
    struct_filt, struct_stats = cluster_filter(
        struct_raw, args.cluster_min_size, args.cluster_dilate_px)
    print(f'[structure] cluster filter: '
          f'{struct_stats["cells_before"]:,} → {struct_stats["cells_after"]:,} cells | '
          f'components {struct_stats["components_before"]:,} → '
          f'{struct_stats["components_after"]:,}')

    # ---- free_evidence (trajectory footprint) ----------------------------
    traj_xyz = None
    if not args.no_free_evidence and args.traj is not None:
        traj_xyz = load_trajectory_tum(args.traj)
        print(f'[traj] loaded {len(traj_xyz):,} poses from {args.traj}')

    # ---- summary ---------------------------------------------------------
    n_cells = W * H
    print()
    print(f'== summary ==')
    print(f'  total cells    : {n_cells:>12,}')
    print(f'  known (h≤{args.max_height_above_ground}) '
          f': {n_known:>12,}  ({100*n_known/n_cells:6.2f}%)')
    print(f'  step (valley)  : {step_stats["cells_after"]:>12,}  '
          f'({100*step_stats["cells_after"]/n_cells:6.2f}%)  '
          f'largest {step_stats["largest_kept"]:,}')
    print(f'  structure      : {struct_stats["cells_after"]:>12,}  '
          f'({100*struct_stats["cells_after"]/n_cells:6.2f}%)  '
          f'largest {struct_stats["largest_kept"]:,}')

    if args.dry_run:
        print(f'\n(dry-run: no PNG written)')
        print(f'== total wall time: {time.time() - t0:.1f} s ==')
        return 0

    # ---- write layers ----------------------------------------------------
    layers_written = {}

    step_path = args.output_dir / 'occupied_step.png'
    Image.fromarray(build_layer_rgba(step_filt, COLOR_STEP), 'RGBA').save(step_path)
    layers_written['occupied_step'] = step_path.name
    print(f'[out] {step_path}')

    struct_path = args.output_dir / 'occupied_structure.png'
    Image.fromarray(build_layer_rgba(struct_filt, COLOR_STRUCTURE), 'RGBA').save(struct_path)
    layers_written['occupied_structure'] = struct_path.name
    print(f'[out] {struct_path}')

    if not args.no_free_evidence:
        free_path = args.output_dir / 'free_evidence.png'
        rgba = build_free_evidence_rgba(
            traj_xyz, origin, resolution, W, H,
            args.anchor_free_radius, args.traj_stride, COLOR_FREE_EVIDENCE)
        Image.fromarray(rgba, 'RGBA').save(free_path)
        layers_written['free_evidence'] = free_path.name
        print(f'[out] {free_path}')

    if not args.no_hillshade:
        hs_path = args.output_dir / 'underlay_hillshade.png'
        rgb = build_hillshade(ground_z2d, resolution)
        Image.fromarray(rgb, 'RGB').save(hs_path)
        layers_written['underlay_hillshade'] = hs_path.name
        print(f'[out] {hs_path}')

    if not args.no_maxheight:
        mh_path = args.output_dir / 'underlay_maxheight.png'
        rgb = build_maxheight_rgb(max_z_rel2d, max_display_m=args.max_height_above_ground)
        Image.fromarray(rgb, 'RGB').save(mh_path)
        layers_written['underlay_maxheight'] = mh_path.name
        print(f'[out] {mh_path}')

    meta_path = write_layer_metadata(args.output_dir, ref_doc, H, W, layers_written, args)
    print(f'[out] {meta_path}')

    print(f'\n== total wall time: {time.time() - t0:.1f} s ==')
    return 0


if __name__ == '__main__':
    sys.exit(main())
