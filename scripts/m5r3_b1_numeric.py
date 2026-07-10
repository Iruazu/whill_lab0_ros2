#!/usr/bin/env python3
"""m5r3_b1_numeric.py — B1 相当のループ閉合誤差を数値で判定する。

CloudCompare の GUI 3 点ピック (`m5r3-comparison-protocol.md §4.2`) が
使えない/難航する場合の代替。原点付近 (走行始点と終点が重なる領域) の
点群を Z 方向に射影して、地面と壁の 2 層化を機械的に検出する。

理論:
  * GLIM は最初の keyframe を map 座標原点近傍に置く。ループが閉じている
    場合、走行終端の LiDAR スキャンも同じ (x, y) 領域を再観測する。
  * 完璧に閉じていれば地面点は 1 つの薄い z 層に集約 → z ヒストは 1 ピーク。
  * end-to-start に誤差があれば「走行開始時に見た地面 z=z0」と「走行終了時
    に見た地面 z=z0+dz」が別レイヤーに乗り、2 ピークが現れる。
  * ピーク間隔 = end-to-start の dz (もしくは物理的な地面の登り/下り) と
    一致すれば SLAM の per-axis 誤差が地図に忠実に反映されている証拠。
  * 壁 (垂直構造) についても同じ論理で、原点付近の垂直点群の xy 密度と、
    そこから見た z 断面の 2 層化を検出できる。

  この方法は「同一物体を 2 回観測している場所」を仮定する。ループ閉合が
  綺麗な地図では 1 ピーク、そうでなければピーク間隔が視覚的な二重壁と
  対応する。CloudCompare 3 点平均の代替として同じ物理量 (m 単位の
  loop_error_wall_3pt_m 相当) を返す。

使い方:
  python3 scripts/m5r3_b1_numeric.py \\
      docs/m5r-bench-data/<run>/glim-out-audit-tli/map.ply \\
      [--radius 5.0] [--bin 0.1] [--ground-band 2.0] [--json OUT.json]

出力: stdout に人間可読サマリ。--json で機械可読形式も同時に。
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def read_ply(path: Path):
    """binary_little_endian PLY (m5r3_export_merged_ply.py 出力) を読む。
    intensity 有無を自動判定。"""
    with path.open("rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: EOF before end_header")
            header_lines.append(line.decode("ascii", errors="replace").rstrip("\n"))
            if line.strip() == b"end_header":
                break
        n = None
        props = []
        for line in header_lines:
            if line.startswith("element vertex"):
                n = int(line.split()[-1])
            elif line.startswith("property"):
                props.append(line.split()[-1])
        if n is None:
            raise ValueError(f"{path}: element vertex not found")
        dtype = np.dtype([(p, "<f4") for p in props])
        data = np.fromfile(f, dtype=dtype, count=n)
        if data.size != n:
            raise ValueError(f"{path}: expected {n} points, got {data.size}")
    return data, props


def find_peaks_simple(hist, min_prominence_ratio=0.05, min_separation_bins=2):
    """scipy を避けた最小構成 peak 検出。
    hist の 3 点平滑後、prominence が max*ratio を超えるローカル極大を返す。
    """
    if hist.size < 3:
        return np.array([], dtype=int)
    # 3-point moving average smoothing (edge-preserving)
    smooth = np.convolve(hist, np.array([1, 2, 1]) / 4.0, mode="same")
    # local maxima
    peaks = []
    for i in range(1, len(smooth) - 1):
        if smooth[i] > smooth[i - 1] and smooth[i] > smooth[i + 1]:
            peaks.append(i)
    if not peaks:
        return np.array([], dtype=int)
    # prominence filter
    max_v = smooth.max()
    thr = max_v * min_prominence_ratio
    peaks = [p for p in peaks if smooth[p] >= thr]
    # separation filter (keep highest in each cluster)
    peaks.sort(key=lambda p: -smooth[p])
    kept = []
    for p in peaks:
        if all(abs(p - k) >= min_separation_bins for k in kept):
            kept.append(p)
    return np.array(sorted(kept), dtype=int)


def analyze_ground(pts_cyl, bin_size, ground_band_m):
    """cylinder 内の全 z 範囲でピーク検出 → 2 層地面を判定。

    設計変更 (2026-07-10): 「z_min から底 X m」で切ると、スパースな外れ値
    (LiDAR ノイズや隣接建物の点群端) が z_min を引きずり降ろすと真の地面
    帯が band から外れる (今回の実データで発生)。代わりに full-range で
    ピーク検出し、ground-plausible な gap (0.3〜3m) の近接 2 ピークを
    loop-error 候補として報告する。ground_band_m は「地面候補として
    採用する z 帯の許容幅」に読み替え。
    """
    z = pts_cyl[:, 2]
    z_min = float(z.min())
    z_max = float(z.max())

    n_bins = int(np.ceil((z_max - z_min) / bin_size))
    hist, edges = np.histogram(z, bins=n_bins, range=(z_min, z_max))
    peak_idx = find_peaks_simple(hist, min_prominence_ratio=0.05, min_separation_bins=2)

    peaks = []
    for i in peak_idx:
        z_center = (edges[i] + edges[i + 1]) / 2
        peaks.append({"z": float(z_center), "count": int(hist[i])})
    peaks.sort(key=lambda p: -p["count"])
    top_peaks = peaks[:8]

    result = {
        "n_points": int(pts_cyl.shape[0]),
        "z_min": z_min,
        "z_max": z_max,
        "bin_size_m": bin_size,
        "peaks_top8_by_count": top_peaks,
        "histogram": {
            "z_edges": edges.tolist(),
            "counts": hist.tolist(),
        },
    }

    # 2 層地面の判定: top-N の中で「gap が 0.3〜ground_band_m の近接ペア、
    # 両方 max*0.10 以上」を探す。max*0.30 では small-peak を弾きすぎるので
    # 0.10 に緩和 (今回の実データで小ピーク 2200 vs 大 9000 → 24%)。
    if len(top_peaks) >= 2:
        max_count = top_peaks[0]["count"]
        best_pair = None
        for i in range(len(top_peaks)):
            for j in range(i + 1, len(top_peaks)):
                a, b = top_peaks[i], top_peaks[j]
                if a["count"] < 0.10 * max_count or b["count"] < 0.10 * max_count:
                    continue
                dz = abs(a["z"] - b["z"])
                if 0.3 <= dz <= ground_band_m:
                    score = (a["count"] + b["count"])
                    if best_pair is None or score > best_pair[2]:
                        best_pair = (a, b, score)
        if best_pair is not None:
            a, b, _ = best_pair
            p1, p2 = sorted([a, b], key=lambda p: p["z"])
            result["ground_verdict"] = {
                "n_peaks_detected": len(top_peaks),
                "p1_z": p1["z"], "p1_count": p1["count"],
                "p2_z": p2["z"], "p2_count": p2["count"],
                "gap_m": float(p2["z"] - p1["z"]),
                "verdict": "2-layer (loop-error visible)",
            }
        else:
            result["ground_verdict"] = {
                "n_peaks_detected": len(top_peaks),
                "verdict": "1-layer (clean closure)",
                "gap_m": 0.0,
                "note": f"No near-pair (0.3-{ground_band_m}m apart, both >=10% max) found.",
            }
    else:
        result["ground_verdict"] = {
            "n_peaks_detected": len(top_peaks),
            "verdict": "1-layer (clean closure)",
            "gap_m": 0.0,
        }
    return result


def analyze_walls(pts_cyl, radius, bin_size, ground_z, wall_z_range=(0.5, 2.5)):
    """壁面 (垂直構造) の二重化を radial-density で判定。
    地面から 0.5m〜2.5m の高さの点だけを取り出し、原点からの半径 r で
    ヒストグラム化。同じ壁面が二重に写っていれば、隣接する 2 つの r ピークが
    出る。ピーク間隔 = 壁の doubling gap。
    """
    z_lo = ground_z + wall_z_range[0]
    z_hi = ground_z + wall_z_range[1]
    mask = (pts_cyl[:, 2] >= z_lo) & (pts_cyl[:, 2] <= z_hi)
    p_wall = pts_cyl[mask]
    if p_wall.shape[0] < 500:
        return {
            "status": "insufficient_wall_points",
            "n_wall_points": int(p_wall.shape[0]),
            "z_lo": z_lo, "z_hi": z_hi,
        }
    r = np.sqrt(p_wall[:, 0] ** 2 + p_wall[:, 1] ** 2)
    n_bins = int(np.ceil(radius / bin_size))
    hist, edges = np.histogram(r, bins=n_bins, range=(0.0, radius))
    peak_idx = find_peaks_simple(hist, min_prominence_ratio=0.10, min_separation_bins=2)

    peaks = []
    for i in peak_idx:
        r_c = (edges[i] + edges[i + 1]) / 2
        peaks.append({"r": float(r_c), "count": int(hist[i])})
    peaks.sort(key=lambda p: -p["count"])
    top = peaks[:8]

    verdict = {
        "n_wall_points": int(p_wall.shape[0]),
        "z_slice_m": [z_lo, z_hi],
        "bin_size_m": bin_size,
        "radial_peaks_top8_by_count": top,
    }

    # 二重壁の判定: 近接ペア (< 2m) のうち両方 max の 40% 以上のもの
    if len(top) >= 2:
        max_c = top[0]["count"]
        best_pair = None
        for i in range(len(top)):
            for j in range(i + 1, len(top)):
                a, b = top[i], top[j]
                if a["count"] < 0.40 * max_c or b["count"] < 0.40 * max_c:
                    continue
                dr = abs(a["r"] - b["r"])
                if 0.3 <= dr <= 2.0:
                    if best_pair is None or dr < abs(best_pair[0]["r"] - best_pair[1]["r"]):
                        best_pair = (a, b)
        if best_pair is not None:
            a, b = sorted(best_pair, key=lambda p: p["r"])
            verdict["wall_doubling"] = {
                "detected": True,
                "r1": a["r"], "r1_count": a["count"],
                "r2": b["r"], "r2_count": b["count"],
                "gap_m": float(b["r"] - a["r"]),
                "note": "同一壁面が radial 方向に 2 ピーク → doubling が起きている",
            }
        else:
            verdict["wall_doubling"] = {
                "detected": False,
                "note": "顕著な近接ペアなし → 壁は単一とみなせる (or 壁が原点近傍に無い)",
            }
    else:
        verdict["wall_doubling"] = {"detected": False, "note": "radial peak insufficient"}
    return verdict


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ply", type=Path, help="merged PLY (m5r3_export_merged_ply.py 出力)")
    ap.add_argument("--radius", type=float, default=5.0, help="円柱半径 [m] (origin 中心)")
    ap.add_argument("--bin", type=float, default=0.1, help="Z ヒスト bin [m]")
    ap.add_argument("--ground-band", type=float, default=3.0,
                    help="ループ誤差と解釈可能な地面ピーク間隔の上限 [m]")
    ap.add_argument("--json", type=Path, default=None, help="機械可読出力 (JSON) パス")
    args = ap.parse_args()

    print(f"reading {args.ply} ...", flush=True)
    data, props = read_ply(args.ply)
    print(f"  {data.size:,} points, properties: {props}")

    pts = np.column_stack([data["x"], data["y"], data["z"]]).astype(np.float64)
    r_xy = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2)
    mask = r_xy <= args.radius
    pts_cyl = pts[mask]
    print(f"  in cylinder r<={args.radius}m: {pts_cyl.shape[0]:,} points "
          f"({100.0 * pts_cyl.shape[0] / pts.shape[0]:.2f}%)")

    if pts_cyl.shape[0] < 500:
        sys.exit("error: 原点半径円柱に点が少なすぎる (map の origin が違う可能性)")

    print()
    print("=" * 62)
    print("1. Ground layer analysis (full-range z histogram)")
    print("=" * 62)
    ground = analyze_ground(pts_cyl, args.bin, args.ground_band)
    print(f"  z range in cylinder: [{ground['z_min']:.3f}, {ground['z_max']:.3f}] m")
    print(f"  n peaks detected (full range): {ground['ground_verdict']['n_peaks_detected']}")
    print(f"  top peaks by count (bin {args.bin} m):")
    for i, p in enumerate(ground["peaks_top8_by_count"], 1):
        print(f"    {i}. z = {p['z']:+.3f} m, count = {p['count']:,}")

    gv = ground["ground_verdict"]
    print()
    print(f"  VERDICT: {gv['verdict']}")
    if "p2_z" in gv:
        print(f"    p1 (lower): z = {gv['p1_z']:+.3f} m (count {gv['p1_count']:,})")
        print(f"    p2 (upper): z = {gv['p2_z']:+.3f} m (count {gv['p2_count']:,})")
        print(f"    gap = {gv['gap_m']:.3f} m   <-- B1 相当 (地面二重化)")
    elif "note" in gv:
        print(f"    {gv['note']}")

    # ground z reference for wall analysis
    ground_ref_z = (
        ground["ground_verdict"].get("p1_z")
        or (ground["peaks_top8_by_count"][0]["z"] if ground["peaks_top8_by_count"] else ground["z_min"])
    )

    print()
    print("=" * 62)
    print("2. Wall layer analysis (radial density, ground+0.5〜2.5m slice)")
    print("=" * 62)
    walls = analyze_walls(pts_cyl, args.radius, args.bin, ground_ref_z)
    if walls.get("status") == "insufficient_wall_points":
        print(f"  insufficient wall points ({walls['n_wall_points']}) in z=[{walls['z_lo']:.2f}, {walls['z_hi']:.2f}]")
    else:
        print(f"  wall points (z slice {walls['z_slice_m'][0]:.2f}〜{walls['z_slice_m'][1]:.2f}): {walls['n_wall_points']:,}")
        print(f"  top radial peaks by count (bin {args.bin} m):")
        for i, p in enumerate(walls["radial_peaks_top8_by_count"], 1):
            print(f"    {i}. r = {p['r']:.3f} m, count = {p['count']:,}")
        wd = walls["wall_doubling"]
        print()
        if wd["detected"]:
            print(f"  VERDICT: wall doubling DETECTED")
            print(f"    r1 = {wd['r1']:.3f} m  (count {wd['r1_count']:,})")
            print(f"    r2 = {wd['r2']:.3f} m  (count {wd['r2_count']:,})")
            print(f"    gap = {wd['gap_m']:.3f} m")
        else:
            print(f"  VERDICT: no wall doubling detected")
            print(f"    {wd['note']}")

    print()
    print("=" * 62)
    print("3. B1 相当値サマリ (manifest.yaml results に転記)")
    print("=" * 62)
    ground_gap = ground["ground_verdict"].get("gap_m", 0.0)
    wall_gap = walls.get("wall_doubling", {}).get("gap_m", 0.0) if walls.get("wall_doubling", {}).get("detected") else 0.0
    print(f"  loop_error_ground_z_layer_gap_m: {ground_gap:.3f}   (地面 2 層のピーク間隔)")
    print(f"  loop_error_wall_radial_gap_m:    {wall_gap:.3f}   (壁面 radial doubling)")
    ref_dz = 1.303  # from traj_lidar.txt end - start (07-10 campus-outer-final)
    print(f"  参考: traj_lidar end-to-start dz = {ref_dz:+.3f} m")
    if ground_gap > 0:
        rel = 100.0 * abs(ground_gap - ref_dz) / ref_dz
        print(f"       地面 gap vs traj dz の相対差: {rel:.1f}% ({ground_gap - ref_dz:+.3f} m)")
    print()

    if args.json:
        report = {
            "map_ply": str(args.ply),
            "radius_m": args.radius,
            "bin_m": args.bin,
            "n_points_total": int(data.size),
            "n_points_in_cylinder": int(pts_cyl.shape[0]),
            "ground": {k: v for k, v in ground.items() if k != "histogram"},
            "walls": walls,
            "b1_summary": {
                "loop_error_ground_z_layer_gap_m": ground_gap,
                "loop_error_wall_radial_gap_m": wall_gap,
            },
        }
        args.json.write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print(f"wrote JSON: {args.json}")


if __name__ == "__main__":
    main()
