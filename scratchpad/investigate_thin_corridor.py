#!/usr/bin/env python3
"""occupancy grid の「痩せた」領域を切り分ける。

生成物:
  1. scratchpad/occupancy_with_traj.png  — pgm に traj xy を赤で重畳
  2. scratchpad/occupancy_z_profile.png  — traj z の推移プロット (+ z-slice 帯)
  3. stdout: 領域別 traj z 統計 + アンカー別 hit count 統計

Usage:
  python3 scratchpad/investigate_thin_corridor.py
"""

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

REPO = Path(__file__).resolve().parents[1]
MAP_DIR = REPO / "docs/maps/campus"
OUT_DIR = REPO / "scratchpad"


def load_pgm(path):
    with open(path, "rb") as f:
        f.readline()  # magic
        f.readline()  # comment
        dims = f.readline().split()
        f.readline()  # maxval
        w, h = int(dims[0]), int(dims[1])
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    return data, w, h


def load_yaml(path):
    txt = path.read_text()
    origin = None
    resolution = None
    for line in txt.splitlines():
        line = line.strip()
        if line.startswith("origin:"):
            v = line[len("origin:"):].strip()
            v = v.strip("[]").split(",")
            origin = (float(v[0]), float(v[1]))
        elif line.startswith("resolution:"):
            resolution = float(line[len("resolution:"):].strip())
    return origin, resolution


def load_traj_tum(path):
    """Return (N, 4) array: t, x, y, z."""
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        toks = line.split()
        if len(toks) < 4:
            continue
        rows.append([float(toks[0]), float(toks[1]), float(toks[2]), float(toks[3])])
    return np.array(rows, dtype=np.float64)


def world_to_pixel(x, y, origin_x, origin_y, res, h):
    col = ((x - origin_x) / res).astype(np.int64)
    row = h - 1 - ((y - origin_y) / res).astype(np.int64)
    return col, row


def main():
    pgm_path = MAP_DIR / "occupancy.pgm"
    yaml_path = MAP_DIR / "occupancy.yaml"
    traj_path = MAP_DIR / "traj_lidar.txt"

    pgm, w, h = load_pgm(pgm_path)
    origin, res = load_yaml(yaml_path)
    ox, oy = origin
    traj = load_traj_tum(traj_path)
    print(f"pgm: {w}x{h} @ {res} m/px  origin=({ox:.2f}, {oy:.2f})")
    print(f"traj: {len(traj)} poses")
    print(f"  x range: [{traj[:,1].min():+.2f}, {traj[:,1].max():+.2f}] m")
    print(f"  y range: [{traj[:,2].min():+.2f}, {traj[:,2].max():+.2f}] m")
    print(f"  z range: [{traj[:,3].min():+.3f}, {traj[:,3].max():+.3f}] m")
    print(f"  z quartiles: "
          f"{np.percentile(traj[:,3], [10, 25, 50, 75, 90])}")

    # === 1. overlay traj on pgm ===
    # Colorize pgm: unknown = light blue-gray, free = white, occupied = black
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[pgm == 205] = [135, 145, 155]
    rgb[pgm == 254] = [255, 255, 255]
    rgb[pgm == 0] = [0, 0, 0]

    # Convert traj xy to pixels and stamp red
    col, row = world_to_pixel(traj[:, 1], traj[:, 2], ox, oy, res, h)
    valid = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    col, row = col[valid], row[valid]
    # Thicken by 1 px in each direction for visibility at downscale
    for dc in (-1, 0, 1):
        for dr in (-1, 0, 1):
            c = np.clip(col + dc, 0, w - 1)
            r = np.clip(row + dr, 0, h - 1)
            rgb[r, c] = [220, 30, 30]

    img = Image.fromarray(rgb, mode="RGB")
    W_OUT = 1800
    scale = W_OUT / w
    img_small = img.resize((W_OUT, int(h * scale)), Image.NEAREST)
    out1 = OUT_DIR / "occupancy_with_traj.png"
    img_small.save(out1)
    print(f"wrote {out1}")

    # === 2. z profile of traj + z-slice band ===
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8),
                                    gridspec_kw={"height_ratios": [2, 1]})
    t0 = traj[0, 0]
    t = traj[:, 0] - t0
    ax1.plot(t, traj[:, 3], "b-", linewidth=0.5, label="traj z (LiDAR frame)")
    ax1.axhspan(0.1, 1.5, alpha=0.25, color="green",
                label="z-slice [0.1, 1.5] (world z)")
    ax1.axhline(y=traj[0, 3], color="orange", linestyle="--",
                label=f"start z = {traj[0,3]:.3f}")
    ax1.axhline(y=traj[-1, 3], color="red", linestyle="--",
                label=f"end z = {traj[-1,3]:.3f}")
    ax1.set_ylabel("world z [m]")
    ax1.set_xlabel("bag time [s]")
    ax1.set_title("LiDAR trajectory z vs z-slice band [0.1, 1.5]")
    ax1.legend(loc="best")
    ax1.grid(True, alpha=0.3)

    # z histogram
    ax2.hist(traj[:, 3], bins=100, color="steelblue", edgecolor="black")
    ax2.axvspan(0.1, 1.5, alpha=0.25, color="green")
    ax2.set_xlabel("world z [m]")
    ax2.set_ylabel("count")
    ax2.set_title("z distribution over the loop")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out2 = OUT_DIR / "occupancy_z_profile.png"
    plt.savefig(out2, dpi=100)
    plt.close()
    print(f"wrote {out2}")

    # === 3. Region-of-interest analysis: right-upper "thin" area ===
    # Preview image reports the top / top-right as thin. In world coords
    # that's high y (top) and moderately high x (right). Let's split the
    # map into 4 quadrants around the traj centroid and report each.
    cx = float(traj[:, 1].mean())
    cy = float(traj[:, 2].mean())
    print()
    print(f"traj centroid: ({cx:.2f}, {cy:.2f})")
    print()
    print("Quadrant analysis (traj z & pose count):")
    print(f"{'quad':<15}{'n':>7}{'z_min':>9}{'z_max':>9}{'z_mean':>9}{'x_span':>15}{'y_span':>15}")
    for name, x_mask, y_mask in [
        ("upper-left",  traj[:, 1] < cx, traj[:, 2] >= cy),
        ("upper-right", traj[:, 1] >= cx, traj[:, 2] >= cy),
        ("lower-left",  traj[:, 1] < cx, traj[:, 2] < cy),
        ("lower-right", traj[:, 1] >= cx, traj[:, 2] < cy),
    ]:
        m = x_mask & y_mask
        if not m.any():
            print(f"  {name:<13}{0:>7}    (empty)")
            continue
        sub = traj[m]
        xs = f"[{sub[:,1].min():+6.1f},{sub[:,1].max():+6.1f}]"
        ys = f"[{sub[:,2].min():+6.1f},{sub[:,2].max():+6.1f}]"
        print(f"  {name:<13}{len(sub):>7}"
              f"{sub[:,3].min():>9.3f}{sub[:,3].max():>9.3f}{sub[:,3].mean():>9.3f}"
              f"{xs:>15}{ys:>15}")

    # === 4. Per-anchor hit count: which anchors have few near occupied cells? ===
    # Downsample traj to 1m stride to match the runtime anchors.
    def downsample(xy, stride):
        kept = [0]
        last = xy[0]
        for i in range(1, len(xy)):
            p = xy[i]
            if np.hypot(p[0] - last[0], p[1] - last[1]) >= stride:
                kept.append(i)
                last = p
        return np.array(kept)

    idx = downsample(traj[:, 1:3], 1.0)
    anchors_xy = traj[idx, 1:3]
    anchors_z = traj[idx, 3]

    # Load occupied cells from pgm (in pixel space)
    occ_rows, occ_cols = np.where(pgm == 0)
    # Convert to world coordinates
    occ_x = ox + (occ_cols + 0.5) * res
    occ_y = oy + (h - 1 - occ_rows + 0.5) * res

    # For each anchor, count occupied cells within max_range = 20m
    max_range = 20.0
    hit_counts = np.zeros(len(anchors_xy), dtype=np.int64)
    for i, (ax_x, ax_y) in enumerate(anchors_xy):
        d2 = (occ_x - ax_x) ** 2 + (occ_y - ax_y) ** 2
        hit_counts[i] = int((d2 <= max_range * max_range).sum())

    print()
    print("Anchor hit-count distribution (max_range = 20m):")
    print(f"  total anchors: {len(anchors_xy)}")
    print(f"  hit_count percentiles: "
          f"p10={np.percentile(hit_counts,10):.0f}  "
          f"p25={np.percentile(hit_counts,25):.0f}  "
          f"p50={np.percentile(hit_counts,50):.0f}  "
          f"p75={np.percentile(hit_counts,75):.0f}  "
          f"p90={np.percentile(hit_counts,90):.0f}")
    thin_thr = 100  # heuristic: fewer than 100 occupied cells within 20m = "starved"
    starved = hit_counts < thin_thr
    print(f"  starved anchors (hits < {thin_thr}): {starved.sum()} "
          f"({100*starved.mean():.1f}%)")
    if starved.any():
        s_xy = anchors_xy[starved]
        s_z = anchors_z[starved]
        print(f"    starved xy range: "
              f"x=[{s_xy[:,0].min():+.1f},{s_xy[:,0].max():+.1f}], "
              f"y=[{s_xy[:,1].min():+.1f},{s_xy[:,1].max():+.1f}]")
        print(f"    starved z range: [{s_z.min():+.3f},{s_z.max():+.3f}] "
              f"(mean {s_z.mean():+.3f})")

    # === 5. Draw starved anchors as yellow on the overlay ===
    if starved.any():
        col_s, row_s = world_to_pixel(
            anchors_xy[starved, 0], anchors_xy[starved, 1], ox, oy, res, h,
        )
        v = (col_s >= 0) & (col_s < w) & (row_s >= 0) & (row_s < h)
        col_s, row_s = col_s[v], row_s[v]
        for dc in range(-3, 4):
            for dr in range(-3, 4):
                c = np.clip(col_s + dc, 0, w - 1)
                r = np.clip(row_s + dr, 0, h - 1)
                rgb[r, c] = [255, 200, 0]  # amber
        img2 = Image.fromarray(rgb, mode="RGB")
        img2_small = img2.resize((W_OUT, int(h * scale)), Image.NEAREST)
        out3 = OUT_DIR / "occupancy_starved_anchors.png"
        img2_small.save(out3)
        print(f"wrote {out3}  (amber = starved anchors overlay)")


if __name__ == "__main__":
    main()
