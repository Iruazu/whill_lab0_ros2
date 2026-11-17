#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_gsi_edge_snap.py — GSI の道路縁を、縁ごとに自分たちの証拠（段・構造物・反射強度）へ寄せる試作（#202、202d）。

#205 の剛体の位置合わせ（georef.yaml）は全体の平行移動と回転だけを直す。電子国土基本図の
道路縁は図化の誤差（地図情報レベル 2500、標準偏差 1.75 m 以内）を線ごとに持つので、左右の縁が
別々にずれ、道幅が変わって見える可能性がある。全体の剛体の誤差（原点 0.1〜1 m、yaw 0.1°）は
幅を変えないので、ここでは扱わない。

手順:
  A. 縁ごとの局所補正
     1. GSI の道路縁（ftCode // 100 == 22）の線を --seg-m 以下の区間に分ける（元の頂点は残す）。
     2. 区間ごとに、どちら側が道かを GSI の道路の面（#205 と同じ作り方）で決める。決まらなければ
        最も近い GSI の中心線の向きで決める。それでも決まらなければ「向き不明」で動かさない。
     3. 区間の中点から法線方向 ±--search-m の断面を、区間に沿った 5 本で取り、証拠を探す。
        優先は 段（縁石・高い段・v2 の段差層）＞ 構造物（v2 の occupied_structure）＞
        反射強度の段（舗装 → 芝・植栽、強度 --max-intensity 以上を芝とする 202c の壁）。
        段・構造物は帯の内側の境、強度は舗装 → 芝の境を縁の位置とする。元の位置より内側（道の側）に
        段・構造物の候補があれば、外側の候補は使わない（道の側から来て最初に当たる所が道の端）。
        強度は背の低い植栽の上でも舗装に見える・木の陰で誤る（ユーザーの目視）ので、強度だけの
        根拠は信頼度を下げ（既定 0.3）、寄せる量の上限も小さくする（--max-offset-intensity-m）。
     4. 区間の並びで、近傍（--smooth-window 区間）の中央値から --jump-m より飛ぶ区間を却下し、
        残りを中央値でならす。近傍に寄せた区間が少ない孤立した区間も却下する。
     5. 証拠が無い・曖昧・却下の区間は動かさず、理由を記録する（未補正）。
     6. 補正後の縁と GSI の中心線から、map_georef_gsi.gsi_road_area と同じ方法で道路の面を作る。
  B. 幅の検算：GSI の中心線に沿って --width-step-m ごとに、中心線の法線の両向きに最初に当たる
     道路縁までの距離の和を、補正前・補正後で測る。参考に、同じ断面で反射強度の舗装の続く幅も
     測る。差が --width-thr-m を超える所を一覧にする（軌跡から --near-m 以内）。
  C. 安全側の版（参考）：補正後の面から、信頼できる根拠（段・構造物）で寄せられなかった縁の
     --uncert-m 以内を削り、段・構造物のセルも除く。

既定の地図（occupancy_candidate.yaml、candidate_final.pgm）は読まず、変えない。candidates/ の手塗りは
比べる相手として読むだけで、変えない。
出力は --out-dir（小さい確認用の画像と数値、Git 管理）と --work-dir（原寸の面・全区間の表、Git 外）と
--gimp-dir（GIMP 用の原寸 RGBA の 2 枚、Git 外）。
"""

import argparse
import csv
import importlib.util
import json
import math
import pathlib
import sys
import time

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.spatial import cKDTree

REPO = pathlib.Path(__file__).resolve().parent.parent
D = REPO / 'docs/maps/campus-m8'
sys.path.insert(0, str(REPO / 'scripts'))

import map_roadway_from_curbs as mr  # noqa: E402

# 根拠の種類。番号は表と画像の凡例で使う。
KINDS = ('none', 'step', 'structure', 'intensity')
# 画像の色（RGB）。未補正は桃、補正前の GSI の縁は灰。
KIND_COLOR = {'step': (230, 0, 0), 'structure': (255, 140, 0), 'intensity': (0, 110, 255),
              'none': (230, 0, 200), 'far': (150, 150, 150)}
ORIG_COLOR = (150, 150, 150)
CENTER_COLOR = (230, 190, 0)
# 202c・#205 と同じ区切り（map 座標 [m]、x0, x1, y0, y1）。
BOXES = {'start_road': (-10, 70, -15, 15), 'seimon': (-100, -75, -20, 5),
         'north_road': (-125, 75, 145, 175)}

# 表の列。空の入力でも見出しだけの CSV を書けるように定数で持つ（rows[0] に頼らない）。
SEG_FIELDS = ['line', 'seg', 'x', 'y', 'len_m', 'dist_traj_m', 'side_by', 'out_nx', 'out_ny', 'status', 'kind',
              'raw_offset_m', 'offset_m', 'confidence', 'n_candidates', 'swept_dropped', 'grass_inside_dropped',
              'outward_blocked', 'dup']
WIDTH_FIELDS = ['center', 's_m', 'x', 'y', 'nx', 'ny', 'dist_traj_m', 'gsi_left_m', 'gsi_right_m', 'snap_left_m',
                'snap_right_m', 'paved_left_m', 'paved_right_m', 'w_gsi_m', 'w_snap_m', 'w_paved_m', 'd_snap_m',
                'd_paved_m', 'flag_snap', 'flag_paved']

# 証拠のラスタのビット。1 枚の uint8 にまとめて、断面の標本を 1 回の添字で引く。
B_STEP, B_STRUCT, B_GRASS, B_PAVED, B_GROUND = 1, 2, 4, 8, 16


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n', 1)[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    home = pathlib.Path.home()
    p.add_argument('--gsi-module', type=pathlib.Path, default=REPO / 'scripts/map_georef_gsi.py')
    p.add_argument('--georef-yaml', type=pathlib.Path, default=D / 'georef.yaml')
    p.add_argument('--raw-dir', type=pathlib.Path, default=home / 'bags/m8-experiments/205/raw',
                   help='#205 で取得済みのタイル（読むだけ。取り直さない）。')
    p.add_argument('--vector-zoom', type=int, default=16)
    p.add_argument('--ortho-zoom', type=int, default=18)
    p.add_argument('--road-cache', type=pathlib.Path,
                   default=home / 'bags/m8-experiments/202c/gsi_ref/gsi_road_area_k2.npz',
                   help='map_roadway_vs_gsi.py の GSI の道路の面（補正前）。作り直した面と一致するかを確かめる。')
    p.add_argument('--layers-yaml', type=pathlib.Path, default=D / 'v2/v2_layers.yaml')
    p.add_argument('--traj', type=pathlib.Path, default=D / 'traj_lidar.txt')
    p.add_argument('--ground-npz', type=pathlib.Path,
                   default=home / 'bags/m8-experiments/202c/ground_raster.npz')
    p.add_argument('--hand-mask', type=pathlib.Path,
                   default=D / 'v2/candidates/candidate_roadway_merged.png',
                   help='比べる相手の手塗り（正解ではない。人の塗りの規則で狭めに塗られている）。')
    p.add_argument('--hand-candidate-mask', type=pathlib.Path,
                   default=D / 'v2/candidates/candidate_roadway_mask.png',
                   help='#205 の縁の距離（中央値 1.2 m、p90 2.3 m）を再現する確認用。')
    p.add_argument('--auto-mask', type=pathlib.Path,
                   default=D / 'v2/experiments/202c/intensity_gate/mp10_tuned_i20/roadway_auto.png',
                   help='202c の強度 < 20 の自動の roadway。')
    p.add_argument('--out-dir', type=pathlib.Path, default=D / 'v2/experiments/202d')
    p.add_argument('--work-dir', type=pathlib.Path, default=home / 'bags/m8-experiments/202d')
    p.add_argument('--gimp-dir', type=pathlib.Path, default=home / 'bags/m8-experiments/gimp-layers',
                   help='17_gsi_edge_snapped.png と 18_width_diff.png を書く場所。')
    p.add_argument('--no-gimp', action='store_true',
                   help='GIMP 用の 2 枚を書かない（パラメータを試すとき、既存の 2 枚を上書きしないため）。')

    # --- 証拠（202c の「10 点・調整」と同じ縁石の抽出、強度 20 の壁） ---
    p.add_argument('--min-points', type=int, default=10)
    p.add_argument('--trend-dz-min', type=float, default=0.03)
    p.add_argument('--curb-dz-min', type=float, default=0.05)
    p.add_argument('--max-intensity', type=float, default=20.0,
                   help='芝・植栽とみなす強度の下限（202c の既定）。')

    # --- A. 縁ごとの局所補正 ---
    p.add_argument('--seg-m', type=float, default=1.0, help='道路縁を分ける区間の長さの上限 [m]。')
    p.add_argument('--search-m', type=float, default=1.5,
                   help='法線方向に探す範囲 [m]（±）。#205 の手塗りの縁→GSI の縁の p90 2.3 m より狭く、'
                        '中央値 1.2 m より広い。広げるほど隣の道・植栽の縁を拾う。')
    p.add_argument('--process-m', type=float, default=20.0,
                   help='軌跡からこの距離 [m] より遠い区間は探さない（LiDAR の届く 15 m の外）。')
    p.add_argument('--min-support', type=float, default=0.8,
                   help='区間に沿った 5 本の断面のうち、証拠がこの割合以上で同じ位置にあれば「線」とみなす。'
                        '縁に平行な線だけを拾い、断面を横切る点状の物（車止め、1 本の木）を落とす。'
                        '0.6（3 本）では、発進点の西（x −10〜0 の南）で駐輪・駐車の帯の中の段の点の並びに'
                        '1 m 外へ寄った（202d の目視）。0.8 ではそこは未補正になり、軌跡 8 m 以内の寄せた'
                        '区間は 74%% → 70%%、手塗りに対する IoU はほぼ同じ（タイルの重なりを除く前の値。'
                        'README 202d）。')
    p.add_argument('--min-inside-m', type=float, default=0.2,
                   help='段・構造物の手前（道の側）に、これだけ証拠の無い所が要る。窓の端から始まる帯は'
                        '「道の端」ではない（縁が帯の中にある）ので使わない。')
    p.add_argument('--trans-win-m', type=float, default=0.4,
                   help='強度の段：境の内側この幅の舗装、外側この幅の芝の割合をそれぞれ見る。')
    p.add_argument('--trans-frac', type=float, default=0.7,
                   help='強度の段で、内側の舗装・外側の芝に要る割合。')
    p.add_argument('--min-observed', type=float, default=0.3,
                   help='断面の標本のうち、地面か強度が分かる割合がこれ未満なら「見ていない」として動かさない。')
    p.add_argument('--max-offset-m', type=float, default=1.5, help='寄せる量の上限 [m]（段・構造物）。')
    p.add_argument('--max-offset-intensity-m', type=float, default=1.0,
                   help='強度だけを根拠にした寄せの上限 [m]。強度は補助なので段・構造物より小さくする。')
    p.add_argument('--swept-max-structure', type=float, default=0.03,
                   help='外へ寄せて道に足す帯の構造物の割合の上限。駐輪・駐車の帯の点々（数 %%）を止める。')
    p.add_argument('--swept-max-step', type=float, default=0.1,
                   help='外へ寄せて道に足す帯の段の割合の上限。')
    p.add_argument('--swept-margin-m', type=float, default=0.1,
                   help='道に足す帯から除く、候補の手前の幅 [m]（候補の段そのものを数えない）。')
    p.add_argument('--inside-grass-max', type=float, default=0.5,
                   help='段・構造物の候補のすぐ内側（--trans-win-m）の芝の割合がこれ以上なら、その段は道の端ではない'
                        '（植栽の島・生垣の縁）として使わない。')
    p.add_argument('--agree-m', type=float, default=0.3,
                   help='段・構造物の縁と強度の段がこの距離以内なら、信頼度を上げる。')
    p.add_argument('--dup-radius-m', type=float, default=0.5,
                   help='タイルの重なりで 2 回入った線の区間・幅の標本を同じとみなす距離 [m]。区間の長さ（1 m）'
                        'の半分。道の両側の縁（3 m 以上離れる）と、同じ線の隣の区間は混ざらない。')
    p.add_argument('--smooth-window', type=int, default=5, help='連続性を見る近傍の区間数（奇数）。')
    p.add_argument('--jump-m', type=float, default=0.5,
                   help='近傍の中央値からこれより飛ぶ区間を却下する [m]。')
    p.add_argument('--min-neighbors', type=int, default=2,
                   help='近傍（自分を含む）に寄せた区間がこれ未満なら孤立として却下する。')

    # --- 道路の面（#205 と同じ） ---
    p.add_argument('--metric-scale', type=int, default=2)
    p.add_argument('--gap-m', type=float, default=0.3)
    p.add_argument('--max-half-width-m', type=float, default=10.0)

    # --- B. 幅の検算 ---
    p.add_argument('--width-step-m', type=float, default=1.0)
    p.add_argument('--width-thr-m', type=float, default=0.5)
    p.add_argument('--near-m', type=float, default=8.0, help='#202・#205 と同じ「軌跡の近く」の幅。')
    p.add_argument('--paved-gap-m', type=float, default=0.3,
                   help='舗装の幅：舗装でない所がこの長さ続いたら止める（マンホール・継ぎ目を越える）。')

    # --- C. 安全側の版 ---
    p.add_argument('--uncert-m', type=float, default=0.5,
                   help='信頼できる根拠で寄せられなかった縁を内側へ削る幅 [m]。')

    p.add_argument('--crop', nargs=5, action='append', default=None, metavar=('NAME', 'X0', 'X1', 'Y0', 'Y1'),
                   help='原寸の確認画像の名前と範囲（map 座標）。何度でも。既定は発進点付近（start、#202・#205 と'
                        '同じ x −10〜70、y −15〜15）と北の道（north、x −10〜70、y 150〜172。GSI の縁が芝の島に'
                        '0.6 m 入っていた所）。')
    p.add_argument('--png-limit', type=int, default=3_000_000,
                   help='Git に置く図の上限 [byte]。超えたら 256 色に減色する（記録に残す）。')
    a = p.parse_args(argv)
    if a.crop is None:
        a.crop = [['start', -10, 70, -15, 15], ['north', -10, 70, 150, 172]]
    a.crop = [[c[0]] + [float(v) for v in c[1:]] for c in a.crop]
    return a


# ---------------------------------------------------------------- 幾何（合成データで試せる純粋な関数）

def densify(P, step):
    """折れ線 P (n, 2) の各辺を step 以下に分ける。元の頂点は残す（offset 0 なら形が変わらない）。"""
    out = [P[:1]]
    for a, b in zip(P[:-1], P[1:]):
        L = float(np.hypot(*(b - a)))
        m = max(1, int(math.ceil(L / step - 1e-9)))
        t = (np.arange(1, m + 1) / m)[:, None]
        out.append(a + t * (b - a))
    Q = np.concatenate(out)
    keep = np.r_[True, np.hypot(*np.diff(Q, axis=0).T) > 1e-9]
    return Q[keep]


def segment_frames(V):
    """頂点 V (n+1, 2) の各区間の中点・単位接線・左法線・長さ。"""
    d = np.diff(V, axis=0)
    L = np.hypot(d[:, 0], d[:, 1])
    t = d / np.maximum(L, 1e-12)[:, None]
    n = np.stack([-t[:, 1], t[:, 0]], 1)
    return (V[:-1] + V[1:]) / 2, t, n, L


def runs(mask):
    """1 次元の bool の連続した True の (始め, 終わり) の添字（終わりを含む）。"""
    m = np.r_[False, np.asarray(mask, bool), False].astype(np.int8)
    d = np.diff(m)
    return list(zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0] - 1))


def barrier_edges(support, s, min_support, min_inside):
    """段・構造物の断面の支持率 support（s の各位置）から、道の端の候補位置を返す。

    s は外向き（道の外が正）。帯の内側の境（道の側から来て最初に当たる所）を縁とする。
    窓の内側の端から min_inside 以内に始まる帯は使わない（縁が窓の外の帯の中にあり、
    帯の内側の境は分からない）。"""
    ds = s[1] - s[0]
    k = int(round(min_inside / ds))
    out = []
    for i0, _ in runs(support >= min_support):
        if i0 < k:
            continue
        out.append(float(s[i0] - ds / 2))
    return out


def intensity_edges(paved, grass, s, win, frac):
    """舗装 → 芝の段の候補位置。境の内側 win の舗装の割合と外側 win の芝の割合がともに frac 以上。"""
    ds = s[1] - s[0]
    w = max(1, int(round(win / ds)))
    n = len(s)
    ok = np.zeros(n, bool)
    cp = np.r_[0.0, np.cumsum(paved)]
    cg = np.r_[0.0, np.cumsum(grass)]
    for j in range(w, n - w + 1):
        pin = (cp[j] - cp[j - w]) / w
        gout = (cg[j + w] - cg[j]) / w
        ok[j] = pin >= frac and gout >= frac
    return [float((s[a] + s[b]) / 2 - ds / 2) for a, b in runs(ok)]


def filter_swept(cands, s, struct_sup, step_sup, max_struct, max_step, margin):
    """外へ寄せる候補のうち、寄せて道に足す帯（0 から候補の手前 margin まで）に構造物・段が
    多いものを落とす。返り値は (残った候補, 落とした数)。

    発進点の西（x −10〜6 の南の縁）では、道の外の駐輪・駐車の帯（構造物層が点々と出る）の
    奥にある段へ 1 m 外へ寄った（202d の 1 回目の目視）。道に足す帯は、道と同じく何も無い
    はずなので、そこに構造物・段がある候補は「道の端」ではないとみなす。内へ寄せる候補
    （道から削る）は、削る帯に何があってもよいので見ない。"""
    keep, drop = [], 0
    for c in cands:
        if c > 0:
            z = (s > 0) & (s < c - margin)
            if z.sum() >= 2 and (struct_sup[z].mean() > max_struct or step_sup[z].mean() > max_step):
                drop += 1
                continue
        keep.append(c)
    return keep, drop


def filter_inside_grass(cands, s, grass, win, max_frac):
    """段・構造物の候補のうち、すぐ内側（道の側の win）が芝・植栽（強度の壁）で占められるものを落とす。
    返り値は (残った候補, 落とした数)。

    道の端の段なら、段の内側は道（舗装）のはず。北へ延びる道（x 23〜27）の東の縁では、舗装の端の
    0.6 m 外にある生垣の段へ寄った（202d の目視。間は芝の帯）。強度は「暗い = 舗装」の側では
    背の低い植栽で誤るが、「明るい = 舗装でない」の側は誤りにくい（白線は 202c の opening で
    壁から外してある）ので、ここでは明るい側だけを使う。"""
    ds = s[1] - s[0]
    w = max(1, int(round(win / ds)))
    keep, drop = [], 0
    for c in cands:
        i = int(np.searchsorted(s, c))   # c は s[i-1] と s[i] の間の境
        z = grass[max(0, i - w):i]
        if z.size and z.mean() >= max_frac:
            drop += 1
            continue
        keep.append(c)
    return keep, drop


def block_outward(step_c, struct_c, int_c):
    """元の位置より内側（s < 0）に段・構造物の候補があれば、外側（s > 0）の候補をすべて落とす。
    返り値は (段, 構造物, 強度の候補, 落とした数)。

    choose_evidence は元の位置に最も近い候補を取るので、内 −0.8 m に縁石、外 +0.5 m に別の段が
    あると +0.5 を選び、縁石を道に含めてしまう（202d のレビュー SF-1）。道の端は「道の側から来て
    最初に当たる段」なので、内側に段があれば、その外の候補は道の端になりえない。
    「内外の両側に候補があれば曖昧として未補正」にする案もあるが、内側の段は道の端の根拠として
    そのまま使えるので、未補正にして根拠を捨てるより内側を取る方を選んだ。
    ここで見るのは filter_swept・filter_inside_grass を通った後の候補。内側が芝の生垣の段のように
    道の端でないと分かった段は、外の候補を止めない。強度の段は内側にあっても外を止めない
    （暗い背の低い植栽で誤る側なので、段・構造物を覆す根拠にしない）。"""
    if not any(c < 0 for c in list(step_c) + list(struct_c)):
        return list(step_c), list(struct_c), list(int_c), 0
    out = [[c for c in cs if c <= 0] for cs in (step_c, struct_c, int_c)]
    n = sum(len(a) - len(b) for a, b in zip((step_c, struct_c, int_c), out))
    return out[0], out[1], out[2], n


def decide_side(road_plus, road_minus, vc, dc, nrm, max_half_width):
    """区間ごとに、道が法線 nrm の正の側か負の側かを決める。返り値は (符号 ±1, 決め方, 決まったか)。

    road_plus / road_minus : 法線の正・負の側で GSI の道路の面に入った標本の割合（0〜1）
    vc, dc                 : 最も近い GSI の中心線のセルへのベクトルと距離 [m]
    道路の面の割合の差が 0.5 以上なら、それで決める（'road_area'）。両側とも道（交差点の中など）か
    両側とも道でないときは、中心線が max_half_width 以内にあり、その向きが法線とはっきり合う
    （|vc・nrm| > 0.3 |vc|）ときだけ中心線で決める（'centerline'）。それでも決まらなければ
    ''（向き不明）で、動かさない。"""
    road_plus = np.asarray(road_plus, float)
    road_minus = np.asarray(road_minus, float)
    side = road_plus - road_minus
    by_area = np.abs(side) >= 0.5
    sign = np.where(side > 0, 1.0, -1.0)
    dot = (np.asarray(vc, float) * np.asarray(nrm, float)).sum(1)
    dc = np.asarray(dc, float)
    by_cent = ~by_area & (dc <= max_half_width) & (np.abs(dot) > 0.3 * np.maximum(dc, 1e-6))
    sign = np.where(by_cent, np.sign(dot), sign)
    how = np.where(by_area, 'road_area', np.where(by_cent, 'centerline', '')).astype(object)
    return sign, how, by_area | by_cent


def find_duplicates(xy, nv, line, radius):
    """重なった線の区間・標本を探す。返り値は各要素について、先に残した重なり相手の添字（無ければ −1）。

    別の線（line が違う）で、位置が radius 以内、向き nv がほぼ同じ（内積 > 0.9）なら重なりとみなす。
    同じ線の隣の区間は除く。前から順に見て、先に残したものを正とする。"""
    out = np.full(len(xy), -1)
    if len(xy) == 0:
        return out
    tree = cKDTree(xy)
    kept = np.zeros(len(xy), bool)
    for i in range(len(xy)):
        for j in sorted(tree.query_ball_point(xy[i], radius)):
            if j < i and kept[j] and line[j] != line[i] and float(np.dot(nv[i], nv[j])) > 0.9:
                out[i] = j
                break
        kept[i] = out[i] < 0
    return out


def nearest(cands):
    return min(cands, key=abs) if cands else None


def choose_evidence(step_c, struct_c, int_c, max_off, max_off_int, agree_m):
    """候補から 1 つ選ぶ。返り値は (offset, kind, confidence, status, n_candidates)。

    優先は 段 > 構造物 > 強度。同じ種類の中では元の位置（0）に最も近い候補を選ぶ
    （GSI はユーザーの目視で概ね良好なので、寄せる量の小さい解を取る）。
    信頼度：段 0.8、構造物 0.6、強度だけ 0.3。強度の段が agree_m 以内で一致すれば +0.15。
    同じ種類の候補が複数あれば曖昧として −0.1 ずつ下げる。"""
    for kind, cands, base, cap in (('step', step_c, 0.8, max_off), ('structure', struct_c, 0.6, max_off),
                                   ('intensity', int_c, 0.3, max_off_int)):
        if not cands:
            continue
        off = nearest(cands)
        if abs(off) > cap:
            return 0.0, kind, 0.0, 'rejected_cap', len(cands)
        conf = base - 0.1 * (len(cands) - 1)
        if kind != 'intensity' and int_c and abs(nearest([c - off for c in int_c])) <= agree_m:
            conf += 0.15
        return off, kind, round(float(min(1.0, max(0.1, conf))), 3), 'snapped', len(cands)
    return 0.0, 'none', 0.0, 'no_evidence', 0


def smooth_offsets(off, snapped, window, jump, min_neighbors, cap=None):
    """1 本の線の区間の並びで連続性を見る。返り値は (ならした offset, 状態の配列)。

    状態：'snapped'、'rejected_jump'（近傍の中央値から jump より飛ぶ）、
    'rejected_isolated'（近傍に寄せた区間が min_neighbors 未満）、
    'rejected_short_line'（線の区間の数が min_neighbors 未満で、近傍を数えられない）、それ以外は元のまま。
    寄せた区間は、近傍の残った区間の中央値に置き換える（1 区間の揺れをならす）。
    cap（区間ごとの上限、省略可）を与えると、ならした値をその区間の根拠の上限で切る。強度だけの
    区間が段の区間の大きな値に引かれて、強度の上限を超えないようにする（レビュー SF-4）。

    短い線（タイルの境で切れた端など）は、以前は min(min_neighbors, n) で孤立の判定を素通りしていた
    （レビュー SF-3）。1 区間だけの寄せを連続性で確かめる手段が無いので、救わずに別の状態で記録する。"""
    off = np.asarray(off, float)
    ok = np.asarray(snapped, bool).copy()
    n = off.size
    h = window // 2
    status = np.where(ok, 'snapped', '').astype(object)
    for i in np.nonzero(ok)[0]:
        lo, hi = max(0, i - h), min(n, i + h + 1)
        nb = off[lo:hi][ok[lo:hi]]
        if abs(off[i] - np.median(nb)) > jump:
            status[i] = 'rejected_jump'
    ok2 = status == 'snapped'
    out = np.zeros(n)
    for i in np.nonzero(ok2)[0]:
        lo, hi = max(0, i - h), min(n, i + h + 1)
        nb = off[lo:hi][ok2[lo:hi]]
        if n < min_neighbors:
            status[i] = 'rejected_short_line'
            continue
        if nb.size < min_neighbors:
            status[i] = 'rejected_isolated'
            continue
        v = float(np.median(nb))
        if cap is not None:
            v = float(np.clip(v, -cap[i], cap[i]))
        out[i] = v
    return out, status


def shift_vertices(V, u, off):
    """区間ごとの外向きの法線 u (n, 2) と寄せる量 off (n,) で頂点 V (n+1, 2) を動かす。
    頂点は両隣の区間の移動の平均だけ動く（未補正の区間との境は半分ずつで滑らかにつなぐ）。"""
    vec = u * np.asarray(off, float)[:, None]
    acc = np.zeros_like(V)
    cnt = np.zeros(len(V))
    acc[:-1] += vec
    acc[1:] += vec
    cnt[:-1] += 1
    cnt[1:] += 1
    return V + acc / cnt[:, None]


def ray_hits(P, Dv, A, B, tmax):
    """半直線 P + t Dv（t > 0）が線分 AB に最初に当たる t（無ければ NaN）。点を塊にして全組を解く。"""
    out = np.full(len(P), np.nan)
    E = B - A
    for s0 in range(0, len(P), 256):
        p, d = P[s0:s0 + 256, None, :], Dv[s0:s0 + 256, None, :]
        den = d[..., 0] * E[None, :, 1] - d[..., 1] * E[None, :, 0]
        w = A[None] - p
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[..., 0] * E[None, :, 1] - w[..., 1] * E[None, :, 0]) / den
            v = (w[..., 0] * d[..., 1] - w[..., 1] * d[..., 0]) / den
        ok = (np.abs(den) > 1e-12) & (t > 1e-6) & (t <= tmax) & (v >= 0) & (v <= 1)
        t = np.where(ok, t, np.inf).min(axis=1)
        out[s0:s0 + 256] = np.where(np.isfinite(t), t, np.nan)
    return out


def paved_run(codes, step, gap_cells):
    """断面に沿った証拠の並び codes（中心から外へ）で、舗装が続く長さ [m]。
    舗装でない所が gap_cells 続いたら止める。止まった所が芝・段・構造物なら長さを、
    分からない所（地面も強度も無い）や端まで続いたなら NaN を返す。"""
    paved = (codes & B_PAVED) > 0
    stop_kind = (codes & (B_GRASS | B_STEP | B_STRUCT)) > 0
    last, miss = -1, 0
    for i in range(len(codes)):
        if paved[i]:
            last, miss = i, 0
            continue
        miss += 1
        if miss >= gap_cells:
            j = last + 1
            seg = stop_kind[j:i + 1]
            return (last + 1) * step if (last >= 0 and seg.any()) else float('nan')
    return float('nan')


# ---------------------------------------------------------------- 入出力

def load_module(path):
    spec = importlib.util.spec_from_file_location('map_georef_gsi', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def build_evidence(args, H, W, res, ox, oy, layers_dir, doc):
    """5 cm の証拠のラスタ（ビットの uint8）を作る。202c の「10 点・調整」と強度 20 の壁と同じ引数。"""
    a = mr.parse_args(['--ground-npz', str(args.ground_npz), '--min-points', str(args.min_points),
                       '--trend-dz-min', str(args.trend_dz_min), '--curb-dz-min', str(args.curb_dz_min),
                       '--max-intensity', str(args.max_intensity)])
    ground, count, _ = mr.ground_raster_from_npz(args.ground_npz, H, W, res, ox, oy, a)
    smooth, _, known = mr.fill_and_smooth(ground, count, res, a)
    del ground, count
    structure = mr.load_mask_alpha(layers_dir / doc['layers']['occupied_structure'], (H, W))
    step_layer = mr.load_mask_alpha(layers_dir / doc['layers']['occupied_step'], (H, W))
    curb, high = mr.detect_curbs(smooth, known, structure, res, a.window, a.curb_dz_min, a.curb_dz_max,
                                 a.curb_slope_min, a.trend_window, a.trend_dz_min)
    del smooth
    gate, inten = mr.intensity_gate(args.ground_npz, H, W, res, a)
    ikn = np.isfinite(inten)
    del inten
    # 段の線は 1 セル幅で斜めに走ると 5 cm の標本の間をすり抜けるので、1 セル太らせる。
    step = ndimage.binary_dilation(curb | high | step_layer)
    ev = np.zeros((H, W), np.uint8)
    ev[step] |= B_STEP
    ev[structure] |= B_STRUCT
    ev[gate] |= B_GRASS
    ev[ikn & ~gate] |= B_PAVED
    ev[known | ikn] |= B_GROUND
    stats = {'curb_cells': int(curb.sum()), 'high_step_cells': int(high.sum()),
             'step_layer_cells': int(step_layer.sum()), 'structure_cells': int(structure.sum()),
             'grass_cells': int(gate.sum()), 'paved_cells': int((ikn & ~gate).sum())}
    return ev, stats


def sample(ev, xy, H, res, ox, oy):
    r, c = mr.xy_to_rc(xy.reshape(-1, 2), H, res, ox, oy)
    ok = (r >= 0) & (r < ev.shape[0]) & (c >= 0) & (c < ev.shape[1])
    out = np.zeros(r.shape, np.uint8)
    out[ok] = ev[r[ok], c[ok]]
    return out.reshape(xy.shape[:-1])


def main(argv=None):
    args = parse_args(argv)
    t0 = time.time()
    g = load_module(args.gsi_module)
    doc, W, H, res, ox, oy = mr.load_grid(args.layers_yaml)
    layers_dir = args.layers_yaml.parent
    k = args.metric_scale
    rm = res * k
    top = oy + H * res
    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.work_dir.mkdir(parents=True, exist_ok=True)
    T = g.load_georef(args.georef_yaml)
    Tinv = np.linalg.inv(T)
    summary = {'timing_s': {}}

    # ---- GSI の線（map 座標の折れ線）
    roads = g.load_vector(args.raw_dir.expanduser(), args.vector_zoom)['road']
    edges_f = [f for f in roads if int(f['props'].get('ftCode', 0)) // 100 == 22]
    cent_f = [f for f in roads if int(f['props'].get('ftCode', 0)) // 100 == 27]

    def to_xy(f):
        return np.stack(g.apply(Tinv, f['EN'][:, 0], f['EN'][:, 1]), 1)

    def feats(polys):
        return [{'type': 2, 'EN': np.stack(g.apply(T, P[:, 0], P[:, 1]), 1)} for P in polys]

    edge_xy = [to_xy(f) for f in edges_f]
    cent_xy = [to_xy(f) for f in cent_f]
    c_m = g.rasterize_EN(cent_f, Tinv, W, H, res, ox, oy, width=1, scale=k)

    # ---- 証拠
    t1 = time.time()
    ev, ev_stats = build_evidence(args, H, W, res, ox, oy, layers_dir, doc)
    summary['evidence_cells'] = ev_stats
    summary['timing_s']['evidence'] = round(time.time() - t1, 1)
    print(f'[証拠] {ev_stats}（{summary["timing_s"]["evidence"]} s）')

    traj = mr.load_traj_xy(args.traj)
    ttree = cKDTree(traj[::5])

    # ---- 補正前の道路の面（同じ作り方で作り、キャッシュと一致するか確かめる）
    dense = [densify(P, args.seg_m) for P in edge_xy]
    e0 = g.rasterize_EN(feats(dense), Tinv, W, H, res, ox, oy, width=1, scale=k)
    road0, _ = g.gsi_road_area(e0, c_m, rm, args.gap_m, args.max_half_width_m)
    if args.road_cache.is_file():
        with np.load(args.road_cache) as f:
            cr = f['road']
        summary['cache_check'] = {'cells_cache': int(cr.sum()), 'cells_rebuilt': int(road0.sum()),
                                  'xor_cells': int((cr ^ road0).sum())}
        print(f'[確認] 補正前の面をキャッシュと比べる：{summary["cache_check"]}')
    # 向きを決める補助：最も近い中心線のセル
    dcen, (icr, icc) = ndimage.distance_transform_edt(~c_m, return_indices=True)

    # ---- A. 区間ごとの探索
    t1 = time.time()
    ds = res
    s = np.arange(-args.search_m, args.search_m + ds / 2, ds)
    along = np.array([0.1, 0.3, 0.5, 0.7, 0.9])
    seg_rows = []
    shifted = []
    per_line = []
    for li, V in enumerate(dense):
        mid, tng, nrm, L = segment_frames(V)
        n = len(mid)
        dtraj = ttree.query(mid)[0]
        # どちら側が道か：道路の面を法線の ±0.5〜1.5 m で見る（判定は decide_side）
        acc = {}
        for sgn in (1, -1):
            a_ = np.zeros(n)
            for d in (0.5, 1.0, 1.5):
                P = mid + sgn * d * nrm
                r = np.floor((top - P[:, 1]) / rm).astype(int)
                c = np.floor((P[:, 0] - ox) / rm).astype(int)
                ok = (r >= 0) & (r < road0.shape[0]) & (c >= 0) & (c < road0.shape[1])
                v = np.zeros(n)
                v[ok] = road0[r[ok], c[ok]]
                a_ += v / 3
            acc[sgn] = a_
        r = np.clip(np.floor((top - mid[:, 1]) / rm).astype(int), 0, c_m.shape[0] - 1)
        c = np.clip(np.floor((mid[:, 0] - ox) / rm).astype(int), 0, c_m.shape[1] - 1)
        cx = ox + (icc[r, c] + 0.5) * rm
        cy = top - (icr[r, c] + 0.5) * rm
        vc = np.stack([cx - mid[:, 0], cy - mid[:, 1]], 1)
        sign, how, known_side = decide_side(acc[1], acc[-1], vc, dcen[r, c] * rm, nrm, args.max_half_width_m)
        # 外向きの法線（道の外が正）
        u = -sign[:, None] * nrm
        off = np.zeros(n)
        kind = np.array(['none'] * n, dtype=object)
        conf = np.zeros(n)
        status = np.array(['far'] * n, dtype=object)
        ncand = np.zeros(n, int)
        swept_drop = np.zeros(n, int)
        grass_drop = np.zeros(n, int)
        out_block = np.zeros(n, int)
        work = np.nonzero(dtraj <= args.process_m)[0]
        status[(dtraj <= args.process_m) & ~known_side] = 'side_unknown'
        work = work[known_side[work]]
        if work.size:
            Pm = (mid[work, None, None, :] + ((along - 0.5)[None, :, None, None] * L[work, None, None, None])
                  * tng[work, None, None, :] + s[None, None, :, None] * u[work, None, None, :])
            code = sample(ev, Pm, H, res, ox, oy)                      # (m, 5, Ns)
            sup = {b: ((code & b) > 0).mean(axis=1) for b in (B_STEP, B_STRUCT, B_GRASS, B_PAVED, B_GROUND)}
            obs = ((code & B_GROUND) > 0).mean(axis=(1, 2))
            for j, i in enumerate(work):
                if obs[j] < args.min_observed:
                    status[i] = 'not_observed'
                    continue
                sc = barrier_edges(sup[B_STEP][j], s, args.min_support, args.min_inside_m)
                tc = barrier_edges(sup[B_STRUCT][j], s, args.min_support, args.min_inside_m)
                ic = intensity_edges(sup[B_PAVED][j], sup[B_GRASS][j], s, args.trans_win_m, args.trans_frac)
                nd = 0
                fs = []
                for cs in (sc, tc, ic):
                    kept, dr = filter_swept(cs, s, sup[B_STRUCT][j], sup[B_STEP][j], args.swept_max_structure,
                                            args.swept_max_step, args.swept_margin_m)
                    fs.append(kept)
                    nd += dr
                sc, tc, ic = fs
                sc, d1 = filter_inside_grass(sc, s, sup[B_GRASS][j], args.trans_win_m, args.inside_grass_max)
                tc, d2 = filter_inside_grass(tc, s, sup[B_GRASS][j], args.trans_win_m, args.inside_grass_max)
                grass_drop[i] = d1 + d2
                swept_drop[i] = nd
                sc, tc, ic, out_block[i] = block_outward(sc, tc, ic)
                o, kd, cf, st, nc = choose_evidence(sc, tc, ic, args.max_offset_m,
                                                    args.max_offset_intensity_m, args.agree_m)
                off[i], kind[i], conf[i], status[i], ncand[i] = o, kd, cf, st, nc
        raw_off = off.copy()
        cap = np.where(kind == 'intensity', args.max_offset_intensity_m, args.max_offset_m)
        sm, st2 = smooth_offsets(off, status == 'snapped', args.smooth_window, args.jump_m, args.min_neighbors,
                                 cap)
        rej = (status == 'snapped') & (st2 != 'snapped')
        status[rej] = st2[rej]
        conf[rej] = 0.0
        final = np.where(status == 'snapped', sm, 0.0)
        V2 = shift_vertices(V, u, final)
        shifted.append(V2)
        per_line.append({'V0': V, 'V1': V2, 'status': status, 'kind': kind, 'near': dtraj <= args.near_m})
        for i in range(n):
            seg_rows.append({'line': li, 'seg': i, 'x': round(float(mid[i, 0]), 3), 'y': round(float(mid[i, 1]), 3),
                             'len_m': round(float(L[i]), 3), 'dist_traj_m': round(float(dtraj[i]), 2),
                             'side_by': how[i] or 'unknown', 'out_nx': round(float(u[i, 0]), 4),
                             'out_ny': round(float(u[i, 1]), 4), 'status': status[i],
                             # 却下・上限超えでも、見つかった根拠の種類は残す（状態で区別する）
                             'kind': kind[i],
                             'raw_offset_m': round(float(raw_off[i]), 3), 'offset_m': round(float(final[i]), 3),
                             'confidence': round(float(conf[i]), 3), 'n_candidates': int(ncand[i]),
                             'swept_dropped': int(swept_drop[i]), 'grass_inside_dropped': int(grass_drop[i]),
                             'outward_blocked': int(out_block[i])})
    summary['timing_s']['snap'] = round(time.time() - t1, 1)
    del dcen, icr, icc

    # ---- 補正後の道路の面
    e1 = g.rasterize_EN(feats(shifted), Tinv, W, H, res, ox, oy, width=1, scale=k)
    road1, _ = g.gsi_road_area(e1, c_m, rm, args.gap_m, args.max_half_width_m)

    # ---- C. 安全側の版：段・構造物で寄せられなかった縁の uncert 以内を削り、段・構造物を除く
    untrusted = []
    for pl in per_line:
        trusted = (pl['status'] == 'snapped') & np.isin(pl['kind'], ['step', 'structure'])
        for a, b in runs(~trusted):
            untrusted.append(pl['V1'][a:b + 2])
    eu = g.rasterize_EN(feats(untrusted), Tinv, W, H, res, ox, oy, width=1, scale=k)
    du = ndimage.distance_transform_edt(~eu) * rm
    barrier_k = g.downsample_any((ev & (B_STEP | B_STRUCT)) > 0, k)
    road_safe = road1 & (du > args.uncert_m)
    road_safe[:barrier_k.shape[0], :barrier_k.shape[1]] &= ~barrier_k
    del du, eu

    # ---- 区間の集計
    # 地理院ベクトルのタイルは隣のタイルと縁を重ねて持つので、同じ線が 2 回入る。区間は線の始まりから
    # 切るので、重なった 2 本の区間の中点はずれることがある。別の線の区間で、中点が --dup-radius-m
    # 以内、外向きの法線がほぼ同じものを重なりとみなし、数えるときは後の方を除く。重なった 2 本は
    # 同じ証拠を見るが、区間の切り方が違うので状態・寄せる量が食い違うことがある。食い違いの数を
    # 実測して summary.json に残す（レビュー SF-2）。補正後の面には両方の線が入る（道路の面は、
    # どちらの線も越えない所になる）。
    dup_of = find_duplicates(np.array([[r['x'], r['y']] for r in seg_rows]).reshape(-1, 2),
                             np.array([[r['out_nx'], r['out_ny']] for r in seg_rows]).reshape(-1, 2),
                             np.array([r['line'] for r in seg_rows]), args.dup_radius_m)
    pairs = {'all': [0, 0, 0], 'near_traj': [0, 0, 0]}   # 組の数、状態の食い違い、寄せる量の差 > 0.1 m
    for r, j in zip(seg_rows, dup_of):
        r['dup'] = bool(j >= 0)
        if j < 0:
            continue
        q = seg_rows[j]
        for key in (('all', 'near_traj') if r['dist_traj_m'] <= args.near_m else ('all',)):
            pairs[key][0] += 1
            pairs[key][1] += int(r['status'] != q['status'])
            pairs[key][2] += int(abs(r['offset_m'] - q['offset_m']) > 0.1)
    rows = [r for r in seg_rows if not r['dup']]
    near_rows = [r for r in rows if r['dist_traj_m'] <= args.near_m]
    summary['duplicates'] = {
        'radius_m': args.dup_radius_m,
        'segments_dropped': len(seg_rows) - len(rows),
        'segments_dropped_near_traj': sum(1 for r in seg_rows if r['dup'] and r['dist_traj_m'] <= args.near_m),
        'pairs': {kk: {'pairs': v[0], 'status_mismatch': v[1], 'offset_diff_gt_0p1m': v[2]}
                  for kk, v in pairs.items()}}
    print('[重なり]', json.dumps(summary['duplicates'], ensure_ascii=False))

    def tally(rs):
        n = len(rs)
        stc = {}
        for r in rs:
            stc[r['status']] = stc.get(r['status'], 0) + 1
        sn = [r for r in rs if r['status'] == 'snapped']
        byk = {}
        for kd in ('step', 'structure', 'intensity'):
            o = np.array([abs(r['offset_m']) for r in sn if r['kind'] == kd])
            so = np.array([r['offset_m'] for r in sn if r['kind'] == kd])
            byk[kd] = {'n': int(o.size), 'abs_median_m': round(float(np.median(o)), 3) if o.size else None,
                       'abs_p90_m': round(float(np.percentile(o, 90)), 3) if o.size else None,
                       'signed_median_m': round(float(np.median(so)), 3) if so.size else None,
                       'outward_share': round(float((so > 0).mean()), 3) if so.size else None}
        o = np.array([abs(r['offset_m']) for r in sn])
        Ls = sum(r['len_m'] for r in rs)
        return {'segments': n, 'length_m': round(Ls, 1), 'status': stc,
                'snapped_share': round(len(sn) / n, 4) if n else None,
                'snapped_length_share': round(sum(r['len_m'] for r in sn) / Ls, 4) if Ls else None,
                'abs_offset_median_m': round(float(np.median(o)), 3) if o.size else None,
                'abs_offset_p90_m': round(float(np.percentile(o, 90)), 3) if o.size else None,
                'by_kind': byk}

    summary['segments_near_traj'] = tally(near_rows)
    summary['segments_processed'] = tally([r for r in rows if r['status'] != 'far'])
    sb = BOXES['start_road']
    summary['segments_start_road'] = tally([r for r in near_rows if sb[0] <= r['x'] <= sb[1]
                                            and sb[2] <= r['y'] <= sb[3]])
    print('[A] 軌跡 8 m 以内', json.dumps(summary['segments_near_traj'], ensure_ascii=False))
    fields = SEG_FIELDS
    with open(args.work_dir / 'edge_segments.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fields)
        w.writeheader()
        w.writerows(rows)
    with open(args.out_dir / 'edge_segments_near.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fields)
        w.writeheader()
        w.writerows(near_rows)

    # ---- B. 幅の検算
    t1 = time.time()
    A0 = np.concatenate([V[:-1] for V in dense])
    B0 = np.concatenate([V[1:] for V in dense])
    A1 = np.concatenate([V[:-1] for V in shifted])
    B1 = np.concatenate([V[1:] for V in shifted])
    wrows = []
    gap_cells = max(1, int(round(args.paved_gap_m / res)))
    sr = np.arange(0, args.max_half_width_m + res / 2, res)
    for ci, P in enumerate(cent_xy):
        seglen = np.hypot(*np.diff(P, axis=0).T)
        cum = np.r_[0, np.cumsum(seglen)]
        if cum[-1] < 1e-6:
            continue
        ss = np.arange(args.width_step_m / 2, cum[-1], args.width_step_m)
        if ss.size == 0:
            continue
        X = np.stack([np.interp(ss, cum, P[:, 0]), np.interp(ss, cum, P[:, 1])], 1)
        ds_ = 0.25
        Xa = np.stack([np.interp(np.clip(ss - ds_, 0, cum[-1]), cum, P[:, 0]),
                       np.interp(np.clip(ss - ds_, 0, cum[-1]), cum, P[:, 1])], 1)
        Xb = np.stack([np.interp(np.clip(ss + ds_, 0, cum[-1]), cum, P[:, 0]),
                       np.interp(np.clip(ss + ds_, 0, cum[-1]), cum, P[:, 1])], 1)
        tg = Xb - Xa
        tg /= np.maximum(np.hypot(tg[:, 0], tg[:, 1]), 1e-9)[:, None]
        nr = np.stack([-tg[:, 1], tg[:, 0]], 1)
        dt = ttree.query(X)[0]
        sel = dt <= args.near_m
        if not sel.any():
            continue
        X, nr, ss, dt = X[sel], nr[sel], ss[sel], dt[sel]
        res_lr = {}
        for nm, (A, B) in (('gsi', (A0, B0)), ('snap', (A1, B1))):
            l_ = ray_hits(X, nr, A, B, args.max_half_width_m)
            r_ = ray_hits(X, -nr, A, B, args.max_half_width_m)
            res_lr[nm] = (l_, r_)
        Pl = X[:, None, :] + sr[None, :, None] * nr[:, None, :]
        Pr = X[:, None, :] - sr[None, :, None] * nr[:, None, :]
        cl = sample(ev, Pl, H, res, ox, oy)
        crr = sample(ev, Pr, H, res, ox, oy)
        for j in range(len(X)):
            pl_ = paved_run(cl[j], res, gap_cells)
            pr_ = paved_run(crr[j], res, gap_cells)
            gl, gr = res_lr['gsi'][0][j], res_lr['gsi'][1][j]
            sl, srr = res_lr['snap'][0][j], res_lr['snap'][1][j]
            wrows.append({'center': ci, 's_m': round(float(ss[j]), 2), 'x': round(float(X[j, 0]), 3),
                          'y': round(float(X[j, 1]), 3), 'nx': round(float(nr[j, 0]), 4),
                          'ny': round(float(nr[j, 1]), 4), 'dist_traj_m': round(float(dt[j]), 2),
                          'gsi_left_m': gl, 'gsi_right_m': gr, 'snap_left_m': sl, 'snap_right_m': srr,
                          'paved_left_m': pl_, 'paved_right_m': pr_})

    def wsum(r, a, b):
        return None if r[a] is None or r[b] is None else round(r[a] + r[b], 3)

    def wdiff(r, a, b):
        return None if r[a] is None or r[b] is None else round(r[a] - r[b], 3)

    # 中心線もタイルの重なりで 2 回入る。重なった 2 本は標本の位置が 1 m 刻みの中でずれることが
    # あるので、位置を丸めた一致ではなく、区間と同じ find_duplicates（別の中心線、--dup-radius-m 以内、
    # 法線がほぼ平行）で 1 つにする。半径より大きくずれた重なりは二重に数えうる（README 202d）。
    nv = np.array([[r['nx'], r['ny']] for r in wrows]).reshape(-1, 2)
    nv = nv * np.where(nv[:, :1] < 0, -1.0, 1.0)                   # 重なった中心線は向きが逆のこともある
    dupw = find_duplicates(np.array([[r['x'], r['y']] for r in wrows]).reshape(-1, 2), nv,
                           np.array([r['center'] for r in wrows]), args.dup_radius_m)
    summary['duplicates']['width_samples_near_traj_before'] = len(wrows)
    summary['duplicates']['width_samples_dropped'] = int((dupw >= 0).sum())
    wrows = [r for r, j in zip(wrows, dupw) if j < 0]
    for r in wrows:
        for kk in ('gsi_left_m', 'gsi_right_m', 'snap_left_m', 'snap_right_m', 'paved_left_m', 'paved_right_m'):
            r[kk] = None if not np.isfinite(r[kk]) else round(float(r[kk]), 3)
        r['w_gsi_m'] = wsum(r, 'gsi_left_m', 'gsi_right_m')
        r['w_snap_m'] = wsum(r, 'snap_left_m', 'snap_right_m')
        r['w_paved_m'] = wsum(r, 'paved_left_m', 'paved_right_m')
        r['d_snap_m'] = wdiff(r, 'w_snap_m', 'w_gsi_m')
        r['d_paved_m'] = wdiff(r, 'w_paved_m', 'w_gsi_m')
        r['flag_snap'] = r['d_snap_m'] is not None and abs(r['d_snap_m']) > args.width_thr_m
        r['flag_paved'] = r['d_paved_m'] is not None and abs(r['d_paved_m']) > args.width_thr_m
    summary['timing_s']['width'] = round(time.time() - t1, 1)
    wf = WIDTH_FIELDS
    with open(args.work_dir / 'width_samples.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, wf)
        w.writeheader()
        w.writerows(wrows)
    flagged = [r for r in wrows if r['flag_snap'] or r['flag_paved']]
    with open(args.out_dir / 'width_diff.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, wf)
        w.writeheader()
        w.writerows(flagged)
    # 連続した旗を区間にまとめる（同じ中心線で width_step の隣同士）
    sections = []
    for key in ('flag_snap', 'flag_paved'):
        cur = None
        for r in wrows:
            if r[key] and cur and cur['center'] == r['center'] and \
                    abs(r['s_m'] - cur['s1']) <= args.width_step_m * 1.01:
                cur['s1'] = r['s_m']
                cur['x1'], cur['y1'] = r['x'], r['y']
                cur['d'].append(r['d_snap_m' if key == 'flag_snap' else 'd_paved_m'])
                continue
            if cur:
                sections.append(cur)
                cur = None
            if r[key]:
                cur = {'kind': key[5:], 'center': r['center'], 's0': r['s_m'], 's1': r['s_m'],
                       'x0': r['x'], 'y0': r['y'], 'x1': r['x'], 'y1': r['y'],
                       'd': [r['d_snap_m' if key == 'flag_snap' else 'd_paved_m']]}
        if cur:
            sections.append(cur)
    for sct in sections:
        d = np.array(sct.pop('d'))
        sct['n_samples'] = int(d.size)
        sct['length_m'] = round(sct['s1'] - sct['s0'] + args.width_step_m, 2)
        sct['diff_median_m'] = round(float(np.median(d)), 3)
        sct['diff_max_abs_m'] = round(float(np.abs(d).max()), 3)
    (args.out_dir / 'width_diff_sections.json').write_text(json.dumps(
        {'threshold_m': args.width_thr_m, 'note': 'diff = 補正後（snap）または舗装（paved）の幅 − GSI の幅。'
         '舗装の幅は反射強度の参考で、背の低い植栽・木の陰で誤る', 'sections': sections},
        ensure_ascii=False, indent=1))

    def wstats(key, rs):
        v = np.array([r[key] for r in rs if r[key] is not None])
        return {'n': int(v.size), 'median_m': round(float(np.median(v)), 3) if v.size else None,
                'p10_m': round(float(np.percentile(v, 10)), 3) if v.size else None,
                'p90_m': round(float(np.percentile(v, 90)), 3) if v.size else None}
    summary['width'] = {
        'samples_near_traj': len(wrows),
        'w_gsi': wstats('w_gsi_m', wrows), 'w_snap': wstats('w_snap_m', wrows), 'w_paved': wstats('w_paved_m', wrows),
        'd_snap': wstats('d_snap_m', wrows), 'd_paved': wstats('d_paved_m', wrows),
        'flag_snap_samples': int(sum(r['flag_snap'] for r in wrows)),
        'flag_paved_samples': int(sum(r['flag_paved'] for r in wrows)),
        'sections_snap': sum(1 for x in sections if x['kind'] == 'snap'),
        'sections_paved': sum(1 for x in sections if x['kind'] == 'paved')}
    print('[B] 幅', json.dumps(summary['width'], ensure_ascii=False))

    # ---- 評価（0.1 m。GSI の面を「評価するマスク」、手塗り・自動を「参照」にする）
    tm = g.traj_mask(args.traj, H, W, res, ox, oy, k)
    near_k = ndimage.distance_transform_edt(~tm) * rm <= args.near_m
    refs = {'hand_merged': mr.load_mask(args.hand_mask, (H, W)),
            'hand_candidate': mr.load_mask(args.hand_candidate_mask, (H, W)),
            'auto_i20': mr.load_mask(args.auto_mask, (H, W))}
    refs_k = {n_: g.downsample_mean(m, k) >= 0.5 for n_, m in refs.items()}
    versions = {'gsi_before': road0, 'gsi_snapped': road1, 'gsi_safe': road_safe}
    ev_out = {}
    for rn, ref in refs_k.items():
        hh, ww = min(ref.shape[0], road0.shape[0]), min(ref.shape[1], road0.shape[1])
        ref = ref[:hh, :ww]
        regs = {'whole': None, f'near_traj_{args.near_m:g}m': near_k[:hh, :ww]}
        for bn, bx in BOXES.items():
            regs[bn] = g.box_mask((hh, ww), H, res, ox, oy, k, bx)
        ev_out[rn] = {}
        for vn, m in versions.items():
            m = m[:hh, :ww]
            ev_out[rn][vn] = {r_: g.overlap(m, ref, reg) for r_, reg in regs.items()}
        # 参照の縁から最も近い GSI の道路縁までの距離（#205 と同じ、5 m 以内、軌跡 8 m 以内）
        b = ref & ~ndimage.binary_erosion(ref)
        for vn, em in (('gsi_before', e0), ('gsi_snapped', e1)):
            de = ndimage.distance_transform_edt(~em[:hh, :ww]) * rm
            dd = de[b & near_k[:hh, :ww]]
            dd = dd[dd <= 5.0]
            ev_out[rn][vn]['edge_to_gsi_edge_m'] = {
                'n': int(dd.size), 'median': round(float(np.median(dd)), 3) if dd.size else None,
                'p90': round(float(np.percentile(dd, 90)), 3) if dd.size else None}
        print(f'[評価] {rn}：' + '  '.join(
            f'{vn} 8m IoU {ev_out[rn][vn][f"near_traj_{args.near_m:g}m"]["iou"]} '
            f'P {ev_out[rn][vn][f"near_traj_{args.near_m:g}m"]["precision"]} '
            f'R {ev_out[rn][vn][f"near_traj_{args.near_m:g}m"]["recall"]}' for vn in versions))
    summary['overlap'] = ev_out
    summary['road_cells'] = {vn: int(m.sum()) for vn, m in versions.items()}
    summary['road_cells_near_traj'] = {vn: int((m & near_k).sum()) for vn, m in versions.items()}
    np.savez_compressed(args.work_dir / 'gsi_road_versions_k2.npz', before=road0, snapped=road1, safe=road_safe,
                        edges_before=e0, edges_snapped=e1, centers=c_m)
    for vn, m in (('gsi_snapped', road1), ('gsi_safe', road_safe)):
        full = np.pad(m.repeat(k, 0).repeat(k, 1), ((0, H - k * m.shape[0]), (0, W - k * m.shape[1])))
        mr.save_mask_rgba(args.work_dir / f'{vn}_road.png', full)
    del refs, refs_k

    # ---- 画像
    png = {}
    lines_by_kind = []   # (頂点, 色) 補正後の縁を状態・根拠ごとの連続区間に分けて描く
    for pl in per_line:
        # 探していない区間（軌跡から --process-m より遠い）は補正前と同じ灰、探して動かさなかった区間は桃
        lab = np.where(pl['status'] == 'snapped', pl['kind'], np.where(pl['status'] == 'far', 'far', 'none'))
        for kd in ('far', 'none', 'intensity', 'structure', 'step'):
            for a, b in runs(lab == kd):
                lines_by_kind.append((pl['V1'][a:b + 2], KIND_COLOR[kd]))

    def draw_lines(d, polys, col, width, to_px):
        for P in polys:
            if len(P) < 2:
                continue
            cc, rr = to_px(P)
            d.line(list(zip(cc.tolist(), rr.tolist())), fill=col, width=width)

    # 1/4 の重ね絵：補正後の道路の面（薄緑）、補正前の面だけ（薄赤）、補正前の縁（灰）、
    # 補正後の縁（根拠で色分け）、中心線（黄）、軌跡（細い黒）
    q = 4
    rq0 = g.downsample_mean(road0, q // k) >= 0.5
    rq1 = g.downsample_mean(road1, q // k) >= 0.5
    hq, wq = H // q, W // q
    rq0, rq1 = rq0[:hq, :wq], rq1[:hq, :wq]
    rgb = np.full((hq, wq, 3), 255, np.uint8)
    rgb[rq1] = (205, 235, 205)
    rgb[rq0 & ~rq1] = (250, 190, 190)
    rgb[rq1 & ~rq0] = (150, 215, 150)
    pil = Image.fromarray(rgb)
    d = ImageDraw.Draw(pil)

    def px_q(P):
        return g.map_to_px(P[:, 0], P[:, 1], H, res, ox, oy, q)
    draw_lines(d, [traj[::20]], (60, 60, 60), 1, px_q)
    draw_lines(d, cent_xy, CENTER_COLOR, 1, px_q)
    draw_lines(d, dense, ORIG_COLOR, 1, px_q)
    for P, col in lines_by_kind:
        draw_lines(d, [P], col, 2, px_q)
    png['overview_q.png'] = g.save_png(pil, args.out_dir / 'overview_q.png', args.png_limit)

    # 幅の差（1/4）：補正後が GSI より狭い = 赤、広い = 青の横線。舗装の幅の差 = 橙の細線。
    pil = Image.fromarray(rgb.copy())
    d = ImageDraw.Draw(pil)
    draw_lines(d, cent_xy, CENTER_COLOR, 1, px_q)
    draw_lines(d, dense, ORIG_COLOR, 1, px_q)

    def cross(r, which):
        x, y, nx, ny = r['x'], r['y'], r['nx'], r['ny']
        lft, rgt = r[f'{which}_left_m'], r[f'{which}_right_m']
        return np.array([[x + nx * lft, y + ny * lft], [x - nx * rgt, y - ny * rgt]])
    for r in wrows:
        if r['flag_paved']:
            draw_lines(d, [cross(r, 'paved')], (255, 150, 0), 1, px_q)
    for r in wrows:
        if r['flag_snap']:
            draw_lines(d, [cross(r, 'snap')], (220, 0, 0) if r['d_snap_m'] < 0 else (0, 80, 255), 3, px_q)
    png['width_diff_q.png'] = g.save_png(pil, args.out_dir / 'width_diff_q.png', args.png_limit)
    del rgb

    # 原寸の確認画像：写真と強度の 2 枚ずつ。補正前の GSI の縁（強度の版は白、写真の版は薄い灰の細線）、
    # 補正後の縁（根拠で色分け）、中心線（黄）、軌跡（黄の細線）。強度の版には段（赤）・構造物（橙）も重ねる。
    mosaic, gx0, gy0 = g.load_ortho_mosaic(args.raw_dir.expanduser(), args.ortho_zoom)
    with np.load(args.ground_npz) as f:
        inten_all = f['intensity_mean']
    for cname, x0, x1, y0, y1 in args.crop:
        # 地図の外を指定したら止める（切り出しが空・負の大きさになり、画像が壊れるため）
        if not (ox <= x0 < x1 <= ox + W * res and oy <= y0 < y1 <= top):
            raise SystemExit(f'--crop {cname}: x {x0}〜{x1}、y {y0}〜{y1} が地図（x {ox:g}〜{ox + W * res:g}、'
                             f'y {oy:g}〜{top:g}）の外')
        c0, r0 = (int(round(float(v))) for v in g.map_to_px(x0, y1, H, res, ox, oy, 1))
        c1, r1 = (int(round(float(v))) for v in g.map_to_px(x1, y0, H, res, ox, oy, 1))
        xs = ox + (np.arange(c0, c1 + 1) + 0.5) * res
        ys = top - (np.arange(r0, r1 + 1) + 0.5) * res
        Xc, Yc = np.meshgrid(xs, ys)
        photo = g.warp_ortho(T, Xc, Yc, mosaic, gx0, gy0, args.ortho_zoom)
        inten_c = inten_all[r0:r1 + 1, c0:c1 + 1]
        v = np.clip((np.nan_to_num(inten_c) - 2.0) / (72.8 - 2.0), 0, 1) * 255
        inten = np.dstack([v, v, v]).astype(np.uint8)
        inten[~np.isfinite(inten_c)] = (40, 40, 60)
        evc = ev[r0:r1 + 1, c0:c1 + 1]
        inten[(evc & B_STRUCT) > 0] = (255, 170, 60)
        inten[(evc & B_STEP) > 0] = (200, 40, 40)

        def px_c(P, c0=c0, r0=r0):
            cc, rr = g.map_to_px(P[:, 0], P[:, 1], H, res, ox, oy, 1)
            return cc - c0, rr - r0
        for kind_, base in (('photo', photo), ('intensity', inten)):
            pil = Image.fromarray(base)
            d = ImageDraw.Draw(pil)
            draw_lines(d, [traj], (255, 255, 0), 1, px_c)
            draw_lines(d, cent_xy, CENTER_COLOR, 2, px_c)
            draw_lines(d, dense, (255, 255, 255) if kind_ == 'intensity' else (200, 200, 200), 2, px_c)
            for P, col in lines_by_kind:
                draw_lines(d, [P], col, 3, px_c)
            name = f'{cname}_crop_{kind_}_1to1.png'
            png[name] = g.save_png(pil, args.out_dir / name, args.png_limit)
    del mosaic, inten_all

    # ---- GIMP 用の原寸 RGBA（2 枚）
    if not args.no_gimp:
        args.gimp_dir.mkdir(parents=True, exist_ok=True)

        def px_1(P):
            return g.map_to_px(P[:, 0], P[:, 1], H, res, ox, oy, 1)
        im = Image.new('RGBA', (W, H), (0, 0, 0, 0))
        d = ImageDraw.Draw(im)
        for P, col in lines_by_kind:
            draw_lines(d, [P], col + (255,), 5, px_1)
        im.save(args.gimp_dir / '17_gsi_edge_snapped.png', compress_level=3)
        im = Image.new('RGBA', (W, H), (0, 0, 0, 0))
        d = ImageDraw.Draw(im)
        for r in wrows:
            if r['flag_paved']:
                draw_lines(d, [cross(r, 'paved')], (255, 150, 0, 200), 3, px_1)
        for r in wrows:
            if r['flag_snap']:
                draw_lines(d, [cross(r, 'snap')], ((255, 0, 0, 255) if r['d_snap_m'] < 0 else (0, 90, 255, 255)),
                           9, px_1)
        im.save(args.gimp_dir / '18_width_diff.png', compress_level=3)
        print(f'[GIMP] {args.gimp_dir}/17_gsi_edge_snapped.png, 18_width_diff.png')

    summary['png'] = png
    summary['png_bytes'] = {n_: (args.out_dir / n_).stat().st_size for n_ in png}
    summary['params'] = {kk: (mr.short(vv) if isinstance(vv, pathlib.Path) else vv) for kk, vv in vars(args).items()}
    summary['elapsed_s'] = round(time.time() - t0, 1)
    (args.out_dir / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1))
    print(f'所要 {summary["elapsed_s"]} s。{args.out_dir}/summary.json')
    return 0


if __name__ == '__main__':
    sys.exit(main())
