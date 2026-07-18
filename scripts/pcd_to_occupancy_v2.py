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
    Stage 2 (scripts/compose_occupancy.py). The composer's priority is
    keepout > free_mask > machine_occ > machine_free, so the human
    CAN erase machine noise.

Three design rules (locked 2026-07-18):

  Rule 1. No morphological operations on the OUTPUT. closing / opening
    / dilation shift widths and positions by 1-2 cells, which the human
    cannot undo without going back to the point cloud. cluster filter
    may pre-dilate internally for connectivity judgement, but the
    output cells are the raw evidence positions.

  Rule 2. Auto-remove is minimised. Defaults now leave BOTH layers
    unfiltered (--step-cluster-min-size 1, --structure-cluster-min-size
    1) until M2 (curb continuity metric) can prove that a stricter
    setting does not silently break curb lines. Salt is exposed to the
    human via salt_candidates.png (see below) rather than removed
    automatically.

  Rule 3. Step-edge occupancy is placed on the CHAIR-ACCESSIBLE side
    of the discontinuity. For a curb, "accessible" is the lower side
    (chair is on the road, curb goes UP to sidewalk); for a ditch,
    "accessible" is the higher side (chair is on the road, ditch drops
    DOWN). We determine accessibility from proximity to the trajectory
    footprint (free_evidence disk) — whichever side of the step pair
    is closer to a traj anchor wins. Falls back to the valley side
    when no traj is available. This gives the verification cross-
    section (M1) an unambiguous curb line to measure against, and it
    handles negative obstacles (ditches, drops) correctly.

Layer separation (Stage 1 output; all PNGs share the input yaml's
pixel grid, origin, resolution — no scaling, no cropping):

  occupied_step.png       RGBA opaque red     Rule-3 chair-accessible
  occupied_structure.png  RGBA opaque orange  vertical extent in
                                              [ground+z_struct_min,
                                               ground+z_struct_max] (2.2 m
                                              walkable clearance limit).
  free_evidence.png       RGBA semi-tr green  trajectory-anchor footprint
                                              disks + optional Bresenham
                                              raycast (--free-raycast).
  salt_candidates.png     RGBA colour-coded   per-cell cluster area on the
                                              (step ∪ structure) union:
                                              1-3 cells magenta (very
                                              likely salt) → 4-16 red →
                                              17-64 orange → 65+ green
                                              (likely real). Exposes the
                                              size distribution to the
                                              human without removing any
                                              cells (per Rule 2).
  underlay_hillshade.png  RGB grayscale       hillshade rendering of
                                              per-cell ground_z (light
                                              azimuth 315°, elevation 45°).
  underlay_maxheight.png  RGB viridis         per-cell max height above
                                              local ground.

Underlay intensity is deliberately NOT produced by v2 — the DUFOMap
static.pcd is XYZ-only. If required, re-export from the GLIM
intermediate PCD at the same origin/resolution and add it as an
extra layer.

Free evidence:
  Default: trajectory-anchor 2 m disks only. This limits the machine-
  claimed planning space to ~4 m around the traj, and typically
  leaves 90+% of the map UNKNOWN. Fast (~1 s), no dependency on the
  occupancy mask being correct.

  With --free-raycast: also cast Bresenham rays from each anchor to
  every occupied cell within --max-range m (720-angular-bin dedup;
  same algorithm as v1 m5r_pcd_to_occupancy.py). Adds substantial
  FREE coverage between trajectory and observed walls, reducing
  UNKNOWN dramatically. Takes ~30-60 s on a 6640x6295 grid with
  1300 anchors. Recommended for A/B comparison against v1.

Cache (--cache-npz): per-cell (count, ground_z, max_z_rel) as flat
float32 arrays plus max_height as metadata. Invalidated when
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


OCCUPIED = 0
UNKNOWN = 205
FREE = 254

# Layer colours (RGBA).
COLOR_STEP = (255, 0, 0, 255)
COLOR_STRUCTURE = (255, 140, 0, 255)
COLOR_FREE_EVIDENCE = (0, 200, 100, 110)

# salt_candidates.png cluster-size colour bands (RGBA).
SALT_BAND_MAGENTA = (255,   0, 200, 255)   # 1-3 cells   — very likely salt
SALT_BAND_RED     = (255,  60,   0, 255)   # 4-16 cells  — likely salt
SALT_BAND_ORANGE  = (255, 160,   0, 255)   # 17-64 cells — small; could be either
SALT_BAND_GREEN   = ( 40, 200,  40, 255)   # 65+ cells   — likely real

HILLSHADE_AZIMUTH_DEG = 315.0
HILLSHADE_ALTITUDE_DEG = 45.0
HILLSHADE_BG = 60

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
                   help='Directory to write the layer PNGs into. Created if missing.')

    p.add_argument('--traj', type=pathlib.Path,
                   help='TUM trajectory (traj_lidar.txt). Used for the free_evidence '
                        'footprint AND for Rule-3 chair-accessible step placement. If '
                        'omitted and --input-yaml is under docs/maps/<site>/, auto-'
                        'detect the sibling traj_lidar.txt.')

    p.add_argument('--ground-percentile', type=float, default=5.0,
                   help='Per-cell z lower percentile used as ground_z (%%). Default 5.')
    p.add_argument('--max-height-above-ground', type=float, default=2.2,
                   help='Drop points more than this many metres above the LOCAL '
                        'ground BEFORE any classification. Default 2.2 = walkable '
                        'clearance limit; eaves 2.0-2.2 m kept as obstacles for '
                        'head protection.')
    p.add_argument('--min-points', type=int, default=3,
                   help='Cells with fewer post-filter points are classified as '
                        'unknown. Default 3 (plane-fit minimum).')

    p.add_argument('--step-threshold', type=float, default=0.03,
                   help='8-neighbour ground_z diff (m) that flags a step edge. '
                        'Default 0.03 (= 3 cm).')

    p.add_argument('--structure-z-min', type=float, default=0.1,
                   help='Lower bound of the structure vertical band (m above local '
                        'ground). Default 0.1 (above floor noise).')
    p.add_argument('--structure-z-max', type=float, default=2.2,
                   help='Upper bound of the structure band. Default 2.2 = walkable '
                        'clearance limit. Matches --max-height-above-ground.')

    # Per-layer cluster filter (Rule 2 — conservative defaults).
    p.add_argument('--step-cluster-min-size', type=int, default=1,
                   help='Drop step components smaller than N cells. Default 1 = '
                        'NO deletion. PR #91 review noted 80%% of raw step cells '
                        'were single-cell on the campus PCD, at least partly real '
                        'curb evidence broken into single pixels by point-cloud '
                        'sparsity. Do NOT default to a stricter setting until M2 '
                        '(curb continuity metric) can prove nothing breaks.')
    p.add_argument('--structure-cluster-min-size', type=int, default=1,
                   help='Drop structure components smaller than N cells. Default 1 '
                        '= NO deletion. Same conservative rationale as step.')
    p.add_argument('--cluster-dilate-px', type=int, default=0,
                   help='Radius (cells) of the disk used to pre-dilate the mask '
                        'before 8-connected labelling. Default 0 (Rule 1 strict: '
                        'label the raw mask, output cells are raw positions).')

    p.add_argument('--anchor-free-radius', type=float, default=2.0,
                   help='Radius (m) of the disk marked free_evidence around each '
                        'trajectory anchor. Default 2.0.')
    p.add_argument('--traj-stride', type=float, default=1.0,
                   help='Trajectory downsample stride (m). Default 1.0.')
    p.add_argument('--free-raycast', dest='free_raycast', action='store_true',
                   default=True,
                   help='Also cast Bresenham rays from each anchor to occupied '
                        'cells within --max-range m (720 angular bin dedup). '
                        'DEFAULT ON (2026-07-18 decision) — campus operation '
                        'needs the wider machine_free coverage (~14%% vs 4%% '
                        'footprint-only) so Nav2 has room to plan around, and '
                        'the composer priority (keepout > free_mask > '
                        'machine_occ > machine_free) means the v1 curb-'
                        'penetration risk cannot re-emerge. Use '
                        '--no-free-raycast to opt out (baseline / speed).')
    p.add_argument('--no-free-raycast', dest='free_raycast', action='store_false')
    p.add_argument('--max-range', type=float, default=20.0,
                   help='Bresenham ray max distance (m). Only used with '
                        '--free-raycast. Default 20 = Velodyne VLP-16 outdoor '
                        'effective range.')
    p.add_argument('--ray-unknown-stop-cells', type=int, default=3,
                   help='Stop a Bresenham ray when it enters this many CONSECUTIVE '
                        'cells that have zero PCD evidence in the DILATED data-'
                        'presence mask (see --data-presence-dilate-cells). Rolls '
                        'the free-marking back this many cells before stopping, '
                        'so the ray never claims cells inside a building interior '
                        '(point-cloud gap) as free. Default 3 (= 15 cm walk into '
                        'a hole before we call it "unknown terrain"). Only used '
                        'with --free-raycast.')
    p.add_argument('--data-presence-dilate-cells', type=int, default=3,
                   help='Dilate the raw data-presence mask (raw_count > 0) by '
                        'this radius before it is used as the raycast stopper '
                        'input. Prevents legitimate pass-through cells (LiDAR '
                        'ray traversed but no return) from being read as '
                        '"unknown terrain" — a dilated cell counts as data-'
                        'present if any DUFOMap point exists within this radius. '
                        'Default 3 cells (= 15 cm; DUFOMap voxelises at 20 cm '
                        'so one point per voxel covers a ~4x4 cell area at 5 cm '
                        'resolution, and this dilation ensures the pass-through '
                        'volume between points reads present). Only used with '
                        '--free-raycast.')

    p.add_argument('--no-hillshade', action='store_true')
    p.add_argument('--no-maxheight', action='store_true')
    p.add_argument('--no-free-evidence', action='store_true')
    p.add_argument('--no-salt-candidates', action='store_true')

    p.add_argument('--cache-npz', type=pathlib.Path,
                   help='Save / reuse per-cell intermediates. Invalidated when '
                        '--max-height-above-ground changes.')
    p.add_argument('--force-recompute', action='store_true')

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


def downsample_trajectory(xyz, min_stride_m):
    kept = [xyz[0]]
    for p in xyz[1:]:
        if np.linalg.norm(p - kept[-1]) >= min_stride_m:
            kept.append(p)
    return np.asarray(kept, dtype=np.float64)


def per_cell_stats(pts, origin, resolution, W, H,
                   ground_percentile, max_height_above_ground):
    """Return (count, ground_z, max_z_rel, raw_count) flat arrays of length W*H.

    `raw_count` = number of in-bounds points that landed in each cell BEFORE
    the h-filter. Used as `data_presence` in main() (a cell with raw_count>0
    has SOME PCD evidence, regardless of height). This is what the raycast
    "walked into unknown terrain" check needs to distinguish (a) empty
    corridor between anchor and wall from (b) point-cloud gap inside a
    building. Introduced 2026-07-18 to fix the raycast-leak-through-
    buildings bug reported in PR #91 review.
    """
    # Nav2 map_server convention: image row 0 is the TOP of the pgm which
    # corresponds to the HIGHEST map_y (origin_y + H*res). We must flip
    # y here so that arrays indexed [py, px] load into pgm in the correct
    # orientation. (Bug fixed 2026-07-18: earlier v2 draft skipped this
    # flip and produced PNGs that were upside-down against
    # occupancy_cleaned.pgm — verified 92.5% match with flip vs 20%
    # without.)
    origin_x, origin_y = float(origin[0]), float(origin[1])
    ix = ((pts[:, 0] - origin_x) / resolution).astype(np.int32)
    iy = (H - 1 - ((pts[:, 1] - origin_y) / resolution).astype(np.int32)).astype(np.int32)
    in_bounds = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    ix, iy = ix[in_bounds], iy[in_bounds]
    xyz = pts[in_bounds]
    print(f'    in-bounds points: {len(xyz):,} of {len(pts):,}')

    cell = iy.astype(np.int64) * W + ix.astype(np.int64)
    df = pd.DataFrame({'cell': cell, 'z': xyz[:, 2]})
    q = ground_percentile / 100.0

    # raw_count = pre-h-filter cell counts (any height). Feeds the
    # raycast data_presence check downstream.
    raw_count_per = df.groupby('cell', sort=False).size()
    raw_count_flat = np.zeros(W * H, dtype=np.int32)
    raw_count_flat[raw_count_per.index.to_numpy()] = raw_count_per.to_numpy()

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

    grp = df.groupby('cell', sort=False)
    count_per = grp.size().rename('count').astype(np.int32)
    ground_per = grp['z'].quantile(q).rename('ground_z').astype(np.float32)
    max_per = grp['z'].max().rename('max_z').astype(np.float32)

    n_cells = W * H
    count_flat = np.zeros(n_cells, dtype=np.int32)
    ground_flat = np.full(n_cells, np.nan, dtype=np.float32)
    max_z_rel_flat = np.full(n_cells, np.nan, dtype=np.float32)
    count_flat[count_per.index.to_numpy()] = count_per.to_numpy()
    ground_flat[ground_per.index.to_numpy()] = ground_per.to_numpy()
    max_arr = max_per.to_numpy()
    ground_arr = ground_per.to_numpy()
    max_z_rel_flat[max_per.index.to_numpy()] = (max_arr - ground_arr).astype(np.float32)
    return count_flat, ground_flat, max_z_rel_flat, raw_count_flat


def compute_step_accessible_side(ground_z_2d, valid_2d, accessibility_2d,
                                  step_threshold):
    """Rule 3: mark the CHAIR-ACCESSIBLE cell of each 8-adjacent step pair.

    For every 8-neighbour pair (self, neighbour) where |ground_z diff| >
    step_threshold and both are valid:
      * If accessibility_2d[self] > accessibility_2d[neighbour], self is
        the accessible side → mark self.
      * If tie (or both zero), fall back to the valley side (self has
        LOWER ground_z → mark self). Preserves the safe default for
        curbs when no trajectory information is available.

    `accessibility_2d`: per-cell scalar (higher = more accessible). Typical:
    negated distance transform from the free_evidence disk (smaller
    distance = higher score). When no trajectory is available, pass an
    all-zero array — everything ties and the valley rule wins.
    """
    H, W = ground_z_2d.shape
    interior = np.zeros((H, W), dtype=bool)
    interior[1:-1, 1:-1] = True

    occ = np.zeros((H, W), dtype=bool)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            shifted_z = np.roll(ground_z_2d, (dy, dx), axis=(0, 1))
            shifted_v = np.roll(valid_2d, (dy, dx), axis=(0, 1))
            shifted_a = np.roll(accessibility_2d, (dy, dx), axis=(0, 1))
            diff_z = shifted_z - ground_z_2d
            step_here = valid_2d & shifted_v & interior & (np.abs(diff_z) > step_threshold)
            self_more_accessible = accessibility_2d > shifted_a
            tie = accessibility_2d == shifted_a
            self_is_valley = diff_z > 0
            occ |= step_here & (self_more_accessible | (tie & self_is_valley))
    return occ


def cluster_filter(mask_2d, min_size, dilate_px):
    """Drop connected components smaller than min_size.

    Rule 1: never dilates the returned mask (dilate_px is used only for
    the internal labelling connectivity judgement). min_size <= 1 is a
    no-op: keeps every cell, but still reports component counts for the
    diagnostic log.
    """
    if not mask_2d.any():
        return mask_2d, {
            'components_before': 0, 'components_after': 0,
            'cells_before': 0, 'cells_after': 0, 'largest_kept': 0,
        }
    if min_size <= 1:
        n_labels, _, stats, _ = cv2.connectedComponentsWithStats(
            mask_2d.astype(np.uint8), connectivity=8)
        areas = stats[1:, cv2.CC_STAT_AREA]   # skip background
        return mask_2d, {
            'components_before': int(n_labels - 1),
            'components_after': int(n_labels - 1),
            'cells_before': int(mask_2d.sum()),
            'cells_after': int(mask_2d.sum()),
            'largest_kept': int(areas.max()) if areas.size else 0,
        }
    m_u8 = mask_2d.astype(np.uint8)
    if dilate_px > 0:
        k = 2 * dilate_px + 1
        kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        m_for_label = cv2.dilate(m_u8, kern)
    else:
        m_for_label = m_u8
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(m_for_label, connectivity=8)
    areas = stats[:, cv2.CC_STAT_AREA]
    drop_label = areas < min_size
    drop_label[0] = False
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


def build_salt_candidates_rgba(mask_2d):
    """Colour occupied cells by 8-connected component area (dilate=1 for label,
    original positions for output per Rule 1).

    Bands: 1-3 magenta, 4-16 red, 17-64 orange, 65+ green.
    """
    H, W = mask_2d.shape
    rgba = np.zeros((H, W, 4), dtype=np.uint8)
    stats_by_band = {'1-3': 0, '4-16': 0, '17-64': 0, '65+': 0}
    if not mask_2d.any():
        return rgba, stats_by_band
    m_u8 = mask_2d.astype(np.uint8)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    dilated = cv2.dilate(m_u8, kern)
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(dilated, connectivity=8)
    areas = stats[:, cv2.CC_STAT_AREA]

    per_cell_area = np.zeros_like(mask_2d, dtype=np.int32)
    per_cell_area[mask_2d] = areas[labels[mask_2d]]

    bands = [
        ((per_cell_area >= 1)  & (per_cell_area <= 3)  & mask_2d, SALT_BAND_MAGENTA, '1-3'),
        ((per_cell_area >= 4)  & (per_cell_area <= 16) & mask_2d, SALT_BAND_RED,     '4-16'),
        ((per_cell_area >= 17) & (per_cell_area <= 64) & mask_2d, SALT_BAND_ORANGE,  '17-64'),
        ((per_cell_area >= 65) & mask_2d,                          SALT_BAND_GREEN,   '65+'),
    ]
    for band_mask, colour, label in bands:
        n = int(band_mask.sum())
        stats_by_band[label] = n
        if n > 0:
            rgba[band_mask, 0] = colour[0]
            rgba[band_mask, 1] = colour[1]
            rgba[band_mask, 2] = colour[2]
            rgba[band_mask, 3] = colour[3]
    return rgba, stats_by_band


def build_hillshade(ground_z_2d, resolution,
                    azimuth_deg=HILLSHADE_AZIMUTH_DEG,
                    altitude_deg=HILLSHADE_ALTITUDE_DEG,
                    bg=HILLSHADE_BG):
    valid = ~np.isnan(ground_z_2d)
    z = np.where(valid, ground_z_2d, 0.0).astype(np.float32)
    dzdx = np.zeros_like(z)
    dzdy = np.zeros_like(z)
    dzdx[:, 1:-1] = (z[:, 2:] - z[:, :-2]) / (2 * resolution)
    dzdy[1:-1, :] = (z[:-2, :] - z[2:, :]) / (2 * resolution)
    slope = np.arctan(np.hypot(dzdx, dzdy))
    aspect = np.arctan2(dzdy, -dzdx)
    zenith = np.radians(90.0 - altitude_deg)
    azimuth = np.radians(azimuth_deg)
    shade = (np.cos(zenith) * np.cos(slope) +
             np.sin(zenith) * np.sin(slope) * np.cos(azimuth - aspect))
    shade = np.clip(shade, 0.0, 1.0)
    gray = (shade * 255.0).astype(np.uint8)
    gray = np.where(valid, gray, bg).astype(np.uint8)
    return np.stack([gray, gray, gray], axis=-1)


def build_maxheight_rgb(max_z_rel_2d, max_display_m=2.2):
    valid = ~np.isnan(max_z_rel_2d)
    v = np.clip(np.where(valid, max_z_rel_2d, 0.0), 0.0, max_display_m)
    t = v / max_display_m
    anchors = np.array([
        [ 68,   1,  84], [ 59,  82, 139], [ 33, 145, 140],
        [ 94, 201,  98], [253, 231,  37],
    ], dtype=np.float32) / 255.0
    N = anchors.shape[0]
    idx_f = t * (N - 1)
    idx0 = np.clip(np.floor(idx_f).astype(np.int32), 0, N - 2)
    frac = idx_f - idx0
    c0 = anchors[idx0]
    c1 = anchors[idx0 + 1]
    rgb_f = c0 + (c1 - c0) * frac[..., None]
    rgb = (np.clip(rgb_f, 0.0, 1.0) * 255.0).astype(np.uint8)
    rgb[~valid] = (60, 60, 60)
    return rgb


def rasterise_footprint_disk(anchors_xy_m, origin, resolution, W, H, radius_m):
    """Rasterise anchor disks. Nav2 convention: image row 0 = highest map_y."""
    mask = np.zeros((H, W), dtype=np.uint8)
    if radius_m <= 0 or len(anchors_xy_m) == 0:
        return mask.astype(bool)
    ox, oy = float(origin[0]), float(origin[1])
    r_cells = max(1, int(round(radius_m / resolution)))
    for xy in anchors_xy_m:
        cx = int(round((xy[0] - ox) / resolution))
        cy = H - 1 - int(round((xy[1] - oy) / resolution))
        if 0 <= cx < W and 0 <= cy < H:
            cv2.circle(mask, (cx, cy), r_cells, 1, thickness=-1)
    return mask.astype(bool)


def raycast_free_from_anchors(free_mask, occupied_mask, data_presence_mask,
                               anchors_xy_m, origin, resolution, W, H,
                               max_range_m, unknown_stop_cells=3,
                               n_angular_bins=720):
    """Bresenham raycast free-marking, extended with two stopping conditions
    (2026-07-18, PR #91 review P0):

      1. Ray stops on any `occupied_mask` cell (as before — walls, curbs).
      2. Ray stops after `unknown_stop_cells` CONSECUTIVE cells with
         `data_presence_mask` == False, and the free-marking is rolled
         back that many cells before stopping. Prevents the ray from
         claiming cells inside a building interior (point-cloud gap
         with no evidence at any height) as free — which was the
         v1-style leakage bug that motivated this rewrite.

    `data_presence_mask` should be True for any cell that has ANY PCD
    evidence at ANY height (pre-h-filter). A pure LiDAR pass-through
    that returned no point AND a building interior both read False;
    the run-length counter distinguishes them (a real corridor is
    surrounded by True cells, a building gap is a long True-False-
    True... run inside the gap).

    Modifies `free_mask` in place. Returns count of NEW free cells.
    """
    if len(anchors_xy_m) == 0:
        return 0
    ox, oy = float(origin[0]), float(origin[1])
    max_r_px = max_range_m / resolution
    max_r_sq = max_r_px ** 2
    bin_step = 2.0 * np.pi / n_angular_bins

    occ_rows, occ_cols = np.where(occupied_mask)
    if occ_cols.size == 0:
        return 0
    occ_col_f = occ_cols.astype(np.float64)
    occ_row_f = occ_rows.astype(np.float64)

    n_new_free = 0
    n_anchors = len(anchors_xy_m)
    for i, xy in enumerate(anchors_xy_m):
        a_col = int(round((xy[0] - ox) / resolution))
        a_row = H - 1 - int(round((xy[1] - oy) / resolution))
        if not (0 <= a_col < W and 0 <= a_row < H):
            continue
        if not free_mask[a_row, a_col] and not occupied_mask[a_row, a_col]:
            free_mask[a_row, a_col] = True
            n_new_free += 1

        dc = occ_col_f - a_col
        dr = occ_row_f - a_row
        d_sq = dc * dc + dr * dr
        near = d_sq <= max_r_sq
        if not near.any():
            continue
        near_col = occ_cols[near].astype(np.int64)
        near_row = occ_rows[near].astype(np.int64)
        near_d = np.sqrt(d_sq[near])
        angle = np.arctan2(dr[near], dc[near])
        bin_idx = np.clip(((angle + np.pi) / bin_step).astype(np.int64),
                          0, n_angular_bins - 1)
        order = np.lexsort((near_d, bin_idx))
        bin_sorted = bin_idx[order]
        _, first_idx = np.unique(bin_sorted, return_index=True)
        keep_col = near_col[order][first_idx]
        keep_row = near_row[order][first_idx]

        for hc, hr in zip(keep_col, keep_row):
            steps = int(max(abs(hc - a_col), abs(hr - a_row)))
            if steps < 2:
                continue
            xs = np.linspace(a_col, hc, steps + 1)[1:-1].round().astype(np.int64)
            ys = np.linspace(a_row, hr, steps + 1)[1:-1].round().astype(np.int64)
            valid = (xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)
            xs = xs[valid]; ys = ys[valid]
            if xs.size == 0:
                continue
            # Stop condition 1: occupied cell.
            occ_here = occupied_mask[ys, xs]
            if occ_here.any():
                first_occ = int(np.argmax(occ_here))
                xs = xs[:first_occ]; ys = ys[:first_occ]
                if xs.size == 0:
                    continue
            # Stop condition 2 (P0 fix): consecutive-unknown run.
            if unknown_stop_cells > 0 and xs.size > 0:
                # Walk the ray, count consecutive cells with
                # data_presence == False. When the counter reaches N,
                # cut the ray to the position just BEFORE the run
                # started (i.e. drop the last N cells to be safe).
                presence = data_presence_mask[ys, xs]
                # Find first index where a run of length `unknown_stop_cells`
                # of consecutive False starts. Vectorised: window sum of
                # (~presence).astype(int) over rolling length N; any window
                # with sum == N is a run start.
                absent = (~presence).astype(np.int8)
                n = xs.size
                if n >= unknown_stop_cells:
                    # cumulative sum trick for rolling window sum
                    csum = np.concatenate(([0], np.cumsum(absent)))
                    win = csum[unknown_stop_cells:] - csum[:-unknown_stop_cells]
                    run_starts = np.where(win == unknown_stop_cells)[0]
                    if run_starts.size > 0:
                        first_run_start = int(run_starts[0])
                        # Keep cells up to (but excluding) the run start.
                        xs = xs[:first_run_start]
                        ys = ys[:first_run_start]
                    if xs.size == 0:
                        continue
            new_free = ~free_mask[ys, xs] & ~occupied_mask[ys, xs]
            if new_free.any():
                sel_x = xs[new_free]; sel_y = ys[new_free]
                free_mask[sel_y, sel_x] = True
                n_new_free += int(sel_x.size)

        if (i + 1) % 200 == 0 or i + 1 == n_anchors:
            print(f'    raycast anchor {i + 1}/{n_anchors}  '
                  f'({n_new_free:,} new free cells)', file=sys.stderr)
    return n_new_free


def build_layer_rgba(mask_2d, colour):
    H, W = mask_2d.shape
    rgba = np.zeros((H, W, 4), dtype=np.uint8)
    rgba[mask_2d, 0] = colour[0]
    rgba[mask_2d, 1] = colour[1]
    rgba[mask_2d, 2] = colour[2]
    rgba[mask_2d, 3] = colour[3]
    return rgba


def auto_detect_traj(input_yaml):
    candidate = input_yaml.parent / 'traj_lidar.txt'
    return candidate if candidate.is_file() else None


def write_layer_metadata(output_dir, ref_doc, H, W, layers_written, args):
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
            'step_cluster_min_size': args.step_cluster_min_size,
            'structure_cluster_min_size': args.structure_cluster_min_size,
            'cluster_dilate_px': args.cluster_dilate_px,
            'anchor_free_radius': args.anchor_free_radius,
            'traj_stride': args.traj_stride,
            'free_raycast': args.free_raycast,
            'max_range': args.max_range,
            'ray_unknown_stop_cells': args.ray_unknown_stop_cells,
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
          f'min_pts={args.min_points}')
    print(f'cluster    : step_min={args.step_cluster_min_size} | '
          f'struct_min={args.structure_cluster_min_size} | '
          f'dilate={args.cluster_dilate_px}px')
    print(f'free_ev    : anchor_disk_r={args.anchor_free_radius} m | '
          f'raycast={"ON (max_range=" + str(args.max_range) + " m, unknown_stop=" + str(args.ray_unknown_stop_cells) + " cells)" if args.free_raycast else "OFF"}')
    print()

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
            print(f'[cache] max_height changed ({cached_h} → '
                  f'{args.max_height_above_ground}), recomputing')
            use_cache = False
        elif 'raw_count' not in cache.files:
            # Old cache without raw_count (pre-P0-raycast-leak-fix). Must
            # recompute — data_presence downstream needs raw_count.
            print(f'[cache] missing raw_count (pre-P0 fix), recomputing')
            use_cache = False
        else:
            count = cache['count']
            ground_z = cache['ground_z']
            max_z_rel = cache['max_z_rel']
            raw_count = cache['raw_count']
            print(f'[cache] {time.time() - t1:.1f} s')

    if not use_cache:
        t1 = time.time()
        print(f'[pcd] loading {args.input_pcd} ...')
        pts = load_pcd(args.input_pcd)
        print(f'[pcd] {len(pts):,} points, {time.time() - t1:.1f} s')

        t1 = time.time()
        print(f'[stats] per-cell count / ground_z / max_z_rel / raw_count ...')
        count, ground_z, max_z_rel, raw_count = per_cell_stats(
            pts, origin, resolution, W, H,
            args.ground_percentile, args.max_height_above_ground,
        )
        print(f'[stats] {time.time() - t1:.1f} s')

        if args.cache_npz is not None:
            args.cache_npz.parent.mkdir(parents=True, exist_ok=True)
            np.savez(
                args.cache_npz,
                count=count, ground_z=ground_z, max_z_rel=max_z_rel,
                raw_count=raw_count,
                max_height_above_ground=np.array([args.max_height_above_ground],
                                                 dtype=np.float64),
            )
            print(f'[cache] wrote {args.cache_npz}')

    count2d = count.reshape(H, W)
    ground_z2d = ground_z.reshape(H, W)
    max_z_rel2d = max_z_rel.reshape(H, W)
    # data_presence: cell has SOME PCD evidence at ANY height (pre-h-filter).
    # Distinguishes "outdoor with no obstacle in this cell" (data_presence
    # False, LiDAR ray passed through no return) from "point-cloud gap
    # inside a building" (also data_presence False). ← BOTH read as false
    # here; the raycast then relies on adjacent data_presence to tell them
    # apart via the --ray-unknown-stop-cells counter.
    data_presence_2d = raw_count.reshape(H, W) > 0

    known_2d = count2d >= args.min_points
    n_known = int(known_2d.sum())

    # --- Load + downsample trajectory (needed for Rule 3 accessibility) ---
    anchors_xy = None
    if args.traj is not None:
        traj_xyz = load_trajectory_tum(args.traj)
        anchors_xyz = downsample_trajectory(traj_xyz, args.traj_stride)
        anchors_xy = anchors_xyz[:, :2]
        print(f'[traj] {len(traj_xyz):,} poses → {len(anchors_xy):,} anchors '
              f'(stride {args.traj_stride} m)')

    # --- footprint disk (feeds free_evidence AND Rule 3 accessibility) ---
    t1 = time.time()
    footprint_mask = np.zeros((H, W), dtype=bool)
    if anchors_xy is not None:
        footprint_mask = rasterise_footprint_disk(
            anchors_xy, origin, resolution, W, H, args.anchor_free_radius)
    print(f'[footprint] disk r={args.anchor_free_radius} m: '
          f'{int(footprint_mask.sum()):,} cells in {time.time() - t1:.1f} s')

    # --- Rule 3 accessibility: negated DT from footprint ---
    if footprint_mask.any():
        inv = (~footprint_mask).astype(np.uint8)
        dt = cv2.distanceTransform(inv, cv2.DIST_L2, 5)
        accessibility_2d = (-dt).astype(np.float32)
    else:
        accessibility_2d = np.zeros((H, W), dtype=np.float32)

    # --- occupied_step (Rule 3 accessible side) ---
    t1 = time.time()
    step_raw = compute_step_accessible_side(
        np.nan_to_num(ground_z2d, nan=0.0),
        known_2d,
        accessibility_2d,
        args.step_threshold,
    )
    print(f'[step] accessible-side detection: {int(step_raw.sum()):,} cells '
          f'in {time.time() - t1:.1f} s')

    step_filt, step_stats = cluster_filter(
        step_raw, args.step_cluster_min_size, args.cluster_dilate_px)
    if args.step_cluster_min_size > 1:
        print(f'[step] cluster filter (min={args.step_cluster_min_size}, '
              f'dilate={args.cluster_dilate_px}): '
              f'{step_stats["cells_before"]:,} → {step_stats["cells_after"]:,} cells | '
              f'components {step_stats["components_before"]:,} → '
              f'{step_stats["components_after"]:,}')
    else:
        print(f'[step] cluster filter DISABLED (min=1): '
              f'{step_stats["cells_after"]:,} cells across '
              f'{step_stats["components_after"]:,} components')

    # --- occupied_structure ---
    struct_raw = (known_2d
                  & ~np.isnan(max_z_rel2d)
                  & (max_z_rel2d >= args.structure_z_min))
    print(f'[structure] band [{args.structure_z_min}, {args.structure_z_max}] m: '
          f'{int(struct_raw.sum()):,} raw cells')
    struct_filt, struct_stats = cluster_filter(
        struct_raw, args.structure_cluster_min_size, args.cluster_dilate_px)
    if args.structure_cluster_min_size > 1:
        print(f'[structure] cluster filter (min={args.structure_cluster_min_size}, '
              f'dilate={args.cluster_dilate_px}): '
              f'{struct_stats["cells_before"]:,} → {struct_stats["cells_after"]:,} cells')
    else:
        print(f'[structure] cluster filter DISABLED (min=1): '
              f'{struct_stats["cells_after"]:,} cells across '
              f'{struct_stats["components_after"]:,} components')

    # --- free_evidence: footprint + optional raycast ---
    free_evidence = footprint_mask.copy()
    if args.free_raycast and (step_filt | struct_filt).any() and anchors_xy is not None:
        t1 = time.time()
        # Ray stoppers: union of step + structure (P0 fix — v1 only used
        # structure, which let rays leak past low steps).
        occupied_union = step_filt | struct_filt
        # Dilate data_presence so pass-through cells (LiDAR ray traversed
        # but no return) are treated as present. Without this, an outdoor
        # corridor between DUFOMap points reads as data-void and the
        # unknown_stop counter fires within a metre of every anchor,
        # collapsing raycast coverage to ≈ footprint alone.
        if args.data_presence_dilate_cells > 0:
            k = 2 * args.data_presence_dilate_cells + 1
            dp_kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            data_presence_for_ray = cv2.dilate(
                data_presence_2d.astype(np.uint8), dp_kern).astype(bool)
        else:
            data_presence_for_ray = data_presence_2d
        print(f'[raycast] {len(anchors_xy):,} anchors × up to {args.max_range} m '
              f'against {int(occupied_union.sum()):,} occupied cells | '
              f'data_presence dilated r={args.data_presence_dilate_cells} '
              f'({int(data_presence_for_ray.sum()):,} cells) | '
              f'unknown_stop={args.ray_unknown_stop_cells}')
        n_new = raycast_free_from_anchors(
            free_evidence, occupied_union, data_presence_for_ray, anchors_xy,
            origin, resolution, W, H, args.max_range,
            unknown_stop_cells=args.ray_unknown_stop_cells)
        print(f'[raycast] added {n_new:,} free cells in {time.time() - t1:.1f} s')

    n_cells = W * H
    print()
    print(f'== summary ==')
    print(f'  total cells    : {n_cells:>12,}')
    print(f'  known (h≤{args.max_height_above_ground}) '
          f': {n_known:>12,}  ({100*n_known/n_cells:6.2f}%)')
    print(f'  step (Rule 3)  : {step_stats["cells_after"]:>12,}  '
          f'({100*step_stats["cells_after"]/n_cells:6.2f}%)  '
          f'largest {step_stats["largest_kept"]:,}')
    print(f'  structure      : {struct_stats["cells_after"]:>12,}  '
          f'({100*struct_stats["cells_after"]/n_cells:6.2f}%)  '
          f'largest {struct_stats["largest_kept"]:,}')
    print(f'  free_evidence  : {int(free_evidence.sum()):>12,}  '
          f'({100*int(free_evidence.sum())/n_cells:6.2f}%)')

    if args.dry_run:
        print(f'\n(dry-run: no PNG written)')
        print(f'== total wall time: {time.time() - t0:.1f} s ==')
        return 0

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
        Image.fromarray(build_layer_rgba(free_evidence, COLOR_FREE_EVIDENCE),
                        'RGBA').save(free_path)
        layers_written['free_evidence'] = free_path.name
        print(f'[out] {free_path}')

    if not args.no_salt_candidates:
        salt_path = args.output_dir / 'salt_candidates.png'
        occ_union = step_filt | struct_filt
        rgba, band_stats = build_salt_candidates_rgba(occ_union)
        Image.fromarray(rgba, 'RGBA').save(salt_path)
        layers_written['salt_candidates'] = salt_path.name
        print(f'[out] {salt_path}  bands: '
              f'1-3={band_stats["1-3"]:,} | 4-16={band_stats["4-16"]:,} | '
              f'17-64={band_stats["17-64"]:,} | 65+={band_stats["65+"]:,}')

    if not args.no_hillshade:
        hs_path = args.output_dir / 'underlay_hillshade.png'
        Image.fromarray(build_hillshade(ground_z2d, resolution), 'RGB').save(hs_path)
        layers_written['underlay_hillshade'] = hs_path.name
        print(f'[out] {hs_path}')

    if not args.no_maxheight:
        mh_path = args.output_dir / 'underlay_maxheight.png'
        Image.fromarray(build_maxheight_rgb(max_z_rel2d,
                                             max_display_m=args.max_height_above_ground),
                        'RGB').save(mh_path)
        layers_written['underlay_maxheight'] = mh_path.name
        print(f'[out] {mh_path}')

    meta_path = write_layer_metadata(args.output_dir, ref_doc, H, W, layers_written, args)
    print(f'[out] {meta_path}')

    print(f'\n== total wall time: {time.time() - t0:.1f} s ==')
    return 0


if __name__ == '__main__':
    sys.exit(main())
