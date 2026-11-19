#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""map_widen_hand_roadway.py — 手塗りの roadway を、補正後の地理院の縁（202d）まで外へ広げた候補を作る（#154、154w）。

ユーザーの判断（2026-11-17、判断票の回答）に従う：
  Q1  広げるのは、202d の区間のうち status=snapped かつ kind が step / structure（赤・橙）に面する所だけ。
  Q2  左上の広場、斜めの舗装路の入口、発進点西の駐輪・駐車の帯、北の道の駐車場の入口は含めない
      （--exclude-box の既定値。補正後の面・赤橙の区間に入っていないことを実測で確かめ、入っていれば箱で落とす）。
  Q3  北の道の東の路肩の帯は、補正後の縁のまま（補正後の面の外なので、何もしなくても入らない）。
  Q4  強度だけで寄せた区間（kind=intensity）は採らない（未補正と同じ扱い）。
  Q5  補正後の地理院の面は、手塗りの確定版の候補づくりに使う。
  前提 202c の強度 < 20 の自動抽出は、背の低い植栽を含むので根拠に使わない（強度は確認の数字だけに使う）。

2 回目の判断（2026-11-19、154w の判断票の回答 R1〜R6）。既定の振る舞いにした：
  R1  発進点から正門前へ向かう道の西（map の −y 側）の駐輪・駐車の帯は、x −77〜+6 の全体を含めない
      （Q2 の発進点西の箱 x −10〜0 を延ばした。--exclude-box の既定値）。
  R2  北の道の南（x 44〜60、y 150〜160.5）の出っ張りは広げたまま。
  R3  上限（--max-widen-m）まで広がった区間は、その区間に割り当てた帯を全部広げない
      （--capped-policy drop、既定。上限で切った所の縁は補正後の縁に届かず、手塗りに平行な直線になる。
      clip で前の振る舞い＝上限で切って広げる）。
  R4  北へ延びる道の両側（x 21〜27、y 5〜16）は広げたまま。
  R5  点の密な帯（x ≈ −95、y 73〜94.5）は含めない（--exclude-box の既定値）。
  R6  帯の中の小さい障害物（段 ∪ 構造物の 8 連結で 3 セル以下）は、手塗りの内側の塩と同じ規則で消す
      （--band-salt-max-cells、既定 3。0 で前の振る舞い＝帯から 1 セル太らせて除き OCC のまま）。
      2026-11-19 のレビュー MF-1 で、1 セル以下の隙間で並ぶ点の格子は塊とみなして残す版を既定にした
      （--band-salt-gap-cells 1。0 で手塗りの内側と同じ生の 8 連結。ユーザーが決め直す。下の R6 の説明）。

向きの注意：georef.yaml では map の x 軸は北から 185°（ほぼ南）、+y はほぼ東を向く。このスクリプトの
コメント・箱の名前で「北の道」「南の縁」と書いた物は、画像の上下（上 = map の +y = 実際はほぼ東）の
呼び名で、方位ではない。R1・R5 から後に書いた所は、方位か「map の ±y 側」で書く。

広げてよい帯（widen_band）：
  1. 補正後の地理院の道路の面（gsi_snapped_road.png）の中で、手塗りの roadway の外。
  2. 手塗りからの距離が --max-widen-m 以内（202d の寄せる量の上限 1.5 m と同じ既定）。
  3. そのセルに最も近い補正後の縁の区間が赤・橙で、最も近い「それ以外」（青・桃・灰）の区間より
     --tie-m 以上近い。境目で迷うセルは広げない側に倒す（安全側）。
  4. その最も近い赤・橙の区間の外向きの法線で見て、縁の外側にあるセルは落とす。補正後の面は 10 cm の
     格子で描いてあり縁が階段になるので、縁のすぐ外のセルが面に入ることがある（レビュー SF-5）。
  5. 段・構造物（v2 の occupied_step ∪ occupied_structure。compose_occupancy の machine_occ と同じ集合）を
     --barrier-dilate-m だけ正方形の要素で太らせて除く。帯を free の塗りにも入れるので、除かないと機械の
     障害物を消してしまう。太らせるのは、static.pcd が 10 cm のボクセルなので、植栽・生垣の構造物が 2 セル
     ごと（間に 1 セル）の点の並びになり、太らせないと点の間のセルが帯に残って植え込みの中へ入り込むため
     （初回の目視、x 104〜106、y 18〜19）。正方形にするのは、楕円の 3×3 は十字で、斜めに 2 セルごと（間に
     1 セル）に並ぶ点の間を 8 連結の帯がすり抜けるため（レビュー MF-1）。v2 の salt_candidates は段 ∪ 構造物と同じ集合
     （2026-11-17 に実測で一致）なので、別には足さない（一致しなければ止める）。
  6. keepout の塗りを除く（今の候補は空だが、keepout は free より優先なので明示する）。
  7. 除外の箱（--exclude-box）の中を除く。
  8. 残りの 8 連結の成分のうち、手塗りに接するものだけを残す（離れた飛び地は足さない）。
  内側へ（手塗りを削る方向）は変えない。

  R6 の小さい障害物は、keepout（6.）と R3 の除外を先に引いた後、5. の前に分ける。段 ∪ 構造物の成分で
  3 セル以下のもののうち、成分の全セルが「5. の前の帯 ∪ 手塗り」に入り、しかも残す障害物を太らせた所に
  掛からないものだけを帯へ戻す（太らせない）。帯の縁にまたがる点・大きい障害物のすぐ脇の点は、
  縁そのものか障害物の一部とみなして残す（安全側）。戻した点は roadway にだけ足し、free の塗りには足さない。
  こうすると合成の gen_auto_free_mask.py が手塗りの内側と同じ規則で free_mask_auto に入れて消し、
  erased_by_free.png では自動（水色）の消去として残る（free にも塗ると人の塗りとの「両方」になり、どの規則で
  消えたか追えない）。消えたことは、合成した free_mask_auto の差分 == 戻した点、で確かめる。
  成分の取り方は --band-salt-gap-cells で選ぶ。既定の 1 は、1 セル以下の隙間で並ぶ点を 1 つの成分とみなす
  （2026-11-19 のレビュー MF-1 で既定にした）。生の 8 連結（0、gen_auto_free_mask と同じ）では、2 セルごと
  （間に 1 セル）の点の格子（10 cm ボクセルの生垣・柵）が 1 点ずつ 3 セル以下になって消え、残す障害物から
  外れるので太らせた壁も無くなる。すると格子の奥の候補のセルが帯につながり、補正後の縁まで FREE になる
  （原寸の切り出しで、帯の外の縁で消えた点は生垣の前の列だった）。R6 の回答は「大きい塊は残るので本物の
  障害物を消すおそれは小さい」という説明に基づいていたが、格子ではそれが成り立たない。ユーザーが
  決め直すまで、格子を残す側を既定にする。選ばれた点は生の成分も 3 セル以下なので、合成の
  gen_auto_free_mask はそのまま消す（規則は同じで、選ぶ点が少ないだけ）。生の 8 連結なら消えていた格子の
  点の数は summary の band_salt.cells_in_dot_lattice に残す。

  R3 は 2 回に分けて作る。1 回目の帯を区間に割り当て（集計と同じ割り当て）、帯のセルが --min-seg-cells
  以上で最大の幅が上限 − 半セル以上の区間を「上限まで広がった区間」とする（前の summary の
  derived.capped_* と同じ定義）。その区間に割り当てたセルを候補から除いて 2 回目を作る。候補が減るだけ
  なので、2 回目の帯は 1 回目の部分集合で、上限まで広がる区間は新たにはできない（checks で確かめる）。

合成は README の 3 節と同じ順（gen_auto_free_mask.py → compose_occupancy.py）。帯は roadway と free の
両方に足す（--free-policy paint、既定）。旧地図の直接の拡幅と同じ扱いで、roadway だけだと free_evidence
（レイの通過）の無いセルは UNKNOWN のまま残り、帯がまだらになる。帯から段・構造物を先に除いてあるので、
free に足しても機械の障害物は消えない。--free-policy evidence にすると roadway だけに足す（比較用）。

二重に広げないための止め（レビュー MF-2）：入力の手塗りが、前回このスクリプトが書いた広げた版
（--work-dir の roadway_widened.png、または --out-dir の summary.json に記録した sha256）と同じなら止める。
採用した後の手塗りを入力にすると、広げた縁からさらに広げてしまうため。

確かめて記録すること（レビュー SF-7。外れたら summary.json を書いた後で終了コード 1）：
  手塗りのまま合成した地図 == 今の既定の地図（--baseline-map）、増えた FREE == 帯、変わったセル ⊆ 帯、
  帯が手塗り・keepout・残した障害物と重ならない、帯で消えた点 == 戻した小さい障害物（R6）、
  drop のときは上限まで広がった区間が残らない（R3）。

既定の地図（occupancy_candidate.yaml、v2/candidate_final.pgm、v2/candidates/）・static.pcd・bag は読むだけで変えない。
出力は --out-dir（小さい画像と数値、Git 管理）、--work-dir（地図本体・原寸のマスク、Git 外）、
--gimp-dir の 19_widened_candidate.png（Git 外）。
"""

import argparse
import csv
import hashlib
import json
import math
import pathlib
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.spatial import cKDTree

REPO = pathlib.Path(__file__).resolve().parent.parent
D = REPO / 'docs/maps/campus-m8'
HOME = pathlib.Path.home()

OCC, FREE, UNKNOWN = 0, 254, 205
TRUSTED_KINDS = ('step', 'structure')

# Q2 の 4 か所と R1・R5。x の範囲（と広場・入口の y）はユーザーの回答の座標のまま。回答に y の無い所は、
# 手塗りの map の −y 側の縁（画像の下。方位ではほぼ西）の外を覆い、道の反対側（+y 側）の帯に掛からない
# 値を原寸の切り出しで決めた（発進点の道の −y 側の縁は y ≈ −1.5〜−2、北の道の −y 側の縁は y ≈ 159.5〜160、
# +y 側の縁は y ≈ 163.5）。
# R1：Q2 の発進点西の箱（x −10〜0）を、ユーザーの回答の x −77〜+6 へ延ばした（同じ帯なので 1 つの箱で
# 置き換える）。帯の −y 側のセルはこの x で y −4.87〜−1.42、+y 側のセルは y 0.52 以上（2026-11-19 に前回の
# 帯で実測）なので、y −9〜−1 は −y 側だけを覆う。−9 は、手塗りの縁が最も −y へ出る x −77（y −4.2）から
# 上限 1.5 m を引いても余る値。x −80〜−77 の −y 側の帯（正門前の手前）は回答の範囲の外なので残す。
# R5：判断票の箱は x −96.5〜−93.5、y 76〜92 だったが、点の密な所（x −99〜−95.5 で段 ∪ 構造物のセルが
# 3% 以上の、y 0.5 m 刻みの帯）は y 73〜94.5 まで続き、前回の帯は箱の外の y 75〜76（0.49 m²）と
# y 92〜94.5（0.96 m²）にも同じ点の間の隙間として残っていた。また y 94 付近の帯は x −96.68 まで出る。そこで y を密な所の端へ、x を
# −97 へ広げた。x −93.5 は道の +x 側（x ≈ −91）の帯に届かず、y 94.5 より +y 側の帯（点が 1% 未満の
# 舗装の脇）は切らない。
DEFAULT_EXCLUDE = [
    ('plaza_nw', -10.0, -2.0, 3.0, 15.0),            # 左上の広場
    ('diagonal_entrance', 30.0, 45.0, -8.0, -2.0),   # 斜めの舗装路の入口
    ('west_parking_strip', -77.0, 6.0, -9.0, -1.0),  # 発進点〜正門前の道の西（−y 側）の駐輪・駐車の帯（R1）
    ('north_parking_entrance', 18.0, 44.0, 150.0, 160.5),  # 北の道の駐車場の入口（−y 側）
    ('dot_field', -97.0, -93.5, 73.0, 94.5),          # 点の密な帯（R5）
]
# 集計の範囲。start_bldg3 は 202d の BOXES と同じ。north_road は 202d の箱を、目視の切り出し（x 96 まで）に合わせて東へ延ばした
REGIONS = {'start_bldg3': (-10.0, 70.0, -15.0, 15.0), 'north_road': (-125.0, 96.0, 145.0, 175.0)}
# 前回の報告で「要判断」とした所（数字を毎回出し直す）
WATCH = {'south_parking_strip_x-75_-10': (-75.0, -10.0, -6.0, -1.0), 'south_parking_strip_x0_5': (0.0, 5.0, -4.0, -1.0),
         'north_road_south_x44_60': (44.0, 60.0, 150.0, 160.5), 'capped_x-96_-93_y12_48': (-96.0, -93.0, 12.0, 48.0),
         'north_side_road_x21_27_y5_16': (21.0, 27.0, 5.0, 16.0),
         'dot_field_x-96.5_-93.5_y76_92': (-96.5, -93.5, 76.0, 92.0)}
# west・dotfield は R1・R5 の箱の確認用（2026-11-19 に足した）
CROPS = [('start', -15.0, 45.0, -9.0, 4.0), ('north', -12.0, 96.0, 151.0, 168.0),
         ('west', -82.0, -12.0, -8.0, 4.0), ('dotfield', -101.0, -89.0, 70.0, 98.0)]
KIND_COLOR = {'step': (230, 0, 0), 'structure': (255, 140, 0), 'intensity': (0, 110, 255),
              'none': (255, 120, 200), 'far': (150, 150, 150)}
BAND_COLOR = (255, 0, 255)
# 帯で消した小さい障害物（R6）。compose_occupancy の erased_by_free.png の自動の消去と同じ水色
SALT_COLOR = (0, 220, 220)


def parse_args(argv=None):
    b = HOME / 'bags/m8-experiments'
    p = argparse.ArgumentParser(description=__doc__.split('\n\n', 1)[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--layers-yaml', type=pathlib.Path, default=D / 'v2/v2_layers.yaml')
    p.add_argument('--hand-roadway', type=pathlib.Path, default=D / 'v2/candidates/candidate_roadway_merged.png',
                   help='広げる前の手塗りの roadway。採用した後は、採用前の版（git show で取り出した物）を渡す。')
    p.add_argument('--hand-free', type=pathlib.Path, default=D / 'v2/candidates/candidate_free_merged.png')
    p.add_argument('--keepout', type=pathlib.Path, default=D / 'v2/candidates/candidate_keepout_mask.png')
    p.add_argument('--baseline-map', type=pathlib.Path, default=D / 'occupancy_candidate.yaml',
                   help='比べる相手（今の既定の地図）。読むだけ。手塗りのまま合成した地図と一致することを確かめる。')
    p.add_argument('--waypoints', type=pathlib.Path, default=D / 'waypoints.yaml')
    p.add_argument('--gsi-road', type=pathlib.Path, default=b / '202d/gsi_snapped_road.png',
                   help='202d の補正後の地理院の道路の面（原寸、白・不透明 = 道）。')
    p.add_argument('--segments', type=pathlib.Path, default=b / '202d/edge_segments.csv',
                   help='202d の全区間の表（map_gsi_edge_snap.py の出力）。')
    p.add_argument('--nav2-params', type=pathlib.Path, default=REPO / 'src/whill_navigation/config/nav2_params.yaml')
    p.add_argument('--max-widen-m', type=float, default=1.5, help='手塗りから外へ広げる距離の上限 [m]。')
    p.add_argument('--tie-m', type=float, default=0.05,
                   help='赤・橙の区間が、それ以外の区間よりこれ以上近いセルだけを広げる [m]。')
    p.add_argument('--barrier-dilate-m', type=float, default=0.05,
                   help='段・構造物を太らせる半径 [m]（正方形）。1 セル（0.05）で 10 cm ボクセルの点の間を閉じる。')
    p.add_argument('--sample-m', type=float, default=0.025, help='区間を点に分ける間隔 [m]（最近傍の判定用）。')
    p.add_argument('--exclude-box', nargs=5, action='append', default=None,
                   metavar=('NAME', 'X0', 'X1', 'Y0', 'Y1'),
                   help='広げない箱（map 座標）。1 つでも渡すと既定の 5 か所（Q2・R1・R5）を置き換える。')
    p.add_argument('--capped-policy', choices=('drop', 'clip'), default='drop',
                   help='drop：上限まで広がった区間は、その区間の帯を全部広げない（R3、既定）。clip：上限で切って'
                        '広げる（前の振る舞い）。')
    # 上限 3 は gen_auto_free_mask.py の判定（per_cell_area <= 3）から来る。帯へ戻した点は合成でその判定に
    # 掛けて消すので、4 以上にすると戻したのに消えない点ができ、check が外れる。外れるのは地図や画像を
    # 書いた後なので、引数の段階で止める
    p.add_argument('--band-salt-max-cells', type=int, default=3, choices=range(0, 4), metavar='{0..3}',
                   help='帯の中の段 ∪ 構造物のうち、このセル数以下の成分を手塗りの内側と同じ規則で消す'
                        '（R6。gen_auto_free_mask.py の 3 が上限）。0 で消さない（前の振る舞い）。')
    p.add_argument('--band-salt-gap-cells', type=int, default=1, choices=range(0, 4), metavar='{0..3}',
                   help='R6 の成分を取るとき、このセル数以下の隙間で並ぶ点を 1 つの成分とみなす。1（既定）は'
                        '2 セルごと（間に 1 セル）の点の格子（10 cm ボクセルの生垣・柵）を大きい障害物として残す。'
                        '0 は gen_auto_free_mask.py と同じ生の 8 連結で、格子の点も 1 点ずつ消える。')
    p.add_argument('--compare-summary', type=pathlib.Path, default=None,
                   help='別の設定で作った summary.json（例：--band-salt-gap-cells 0 の版）。主な数字を'
                        'この summary の compare に写す（README の比較表の出所を 1 つにする）。')
    p.add_argument('--free-policy', choices=('paint', 'evidence'), default='paint',
                   help='paint：帯を roadway と free の両方に足す（既定）。evidence：roadway だけ（free_evidence の'
                        '無いセルは UNKNOWN のまま）。')
    p.add_argument('--hole-max-m2', type=float, default=1.0,
                   help='FREE の穴（帯の中に残る段・構造物の点とその周り）とみなす大きさの上限 [m²]。外の縁を取るとき'
                        'に埋める。これより大きい UNKNOWN（構内の島など）は埋めない。')
    p.add_argument('--out-dir', type=pathlib.Path, default=D / 'v2/experiments/154w')
    p.add_argument('--work-dir', type=pathlib.Path, default=b / '154w')
    p.add_argument('--gimp-dir', type=pathlib.Path, default=b / 'gimp-layers')
    p.add_argument('--no-gimp', action='store_true', help='GIMP 用の 19 番を書かない。')
    p.add_argument('--underlay-intensity', type=pathlib.Path, default=b / 'gimp-layers/04_intensity.png')
    p.add_argument('--underlay-photo', type=pathlib.Path, default=b / 'gimp-layers/01_gsi_photo.png')
    p.add_argument('--intensity-npz', type=pathlib.Path, default=b / '202c/ground_raster.npz',
                   help='確認だけに使う地面の反射強度（202c）。帯の中の明るいセルを数える。根拠には使わない。')
    p.add_argument('--grass-intensity', type=float, default=20.0,
                   help='これ以上を「明るい」とする（202c の壁と同じ）。縁石・側溝のコンクリートも明るいので参考の数字。')
    p.add_argument('--min-seg-cells', type=int, default=4,
                   help='区間を「広げた」と数える、その区間に割り当てた帯のセル数の下限（0.01 m²）。')
    return p.parse_args(argv)


# ---------------------------------------------------------------- 入出力

def load_grid(layers_yaml):
    doc = yaml.safe_load(layers_yaml.read_text())
    g = doc['grid']
    return doc, int(g['width']), int(g['height']), float(g['resolution']), \
        float(g['origin'][0]), float(g['origin'][1])


def load_mask(path, hw):
    """compose_occupancy.load_sidecar_mask と同じ解釈（白・不透明が塗り）。"""
    im = Image.open(path)
    m = np.array(im.convert('L')) >= 128
    if im.mode == 'RGBA':
        m &= np.array(im.getchannel('A')) > 0
    if m.shape != hw:
        raise SystemExit(f'{path}: 形 {m.shape} が格子 {hw} と違う')
    return m


def load_opaque(path, hw):
    """compose_occupancy.load_rgba_opaque と同じ（v2 の機械レイヤは α=255 が証拠）。"""
    a = np.array(Image.open(path).convert('RGBA'))[..., 3] == 255
    if a.shape != hw:
        raise SystemExit(f'{path}: 形 {a.shape} が格子 {hw} と違う')
    return a


def save_mask_rgba(path, mask):
    """手塗りの候補と同じ形式（白・不透明 / 透明）。compose_occupancy にそのまま渡せる。"""
    rgba = np.zeros(mask.shape + (4,), np.uint8)
    rgba[mask] = 255
    Image.fromarray(rgba, 'RGBA').save(path, optimize=True)


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def short_path(p):
    """summary.json に書くパス。リポジトリの中は相対、ホームの下は ~/ にする（作業場所の絶対パスを残さない）。"""
    p = pathlib.Path(p)
    try:
        return str(p.resolve().relative_to(REPO))
    except ValueError:
        pass
    try:
        return '~/' + str(p.resolve().relative_to(HOME.resolve()))
    except ValueError:
        return str(p)


def read_segments(path):
    """202d の区間の表を読み、補正後の位置の区間（中点 + offset × 外向きの法線、長さ len_m）にする。
    頂点の実際の位置は両隣の区間の移動の平均なので、区間の端で数 cm ずれうる（最近傍の判定には十分）。"""
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            rows.append(r)
    n = len(rows)
    mid = np.array([[float(r['x']), float(r['y'])] for r in rows]).reshape(n, 2)
    nv = np.array([[float(r['out_nx']), float(r['out_ny'])] for r in rows]).reshape(n, 2)
    off = np.array([float(r['offset_m']) for r in rows])
    L = np.array([float(r['len_m']) for r in rows])
    status = np.array([r['status'] for r in rows])
    kind = np.array([r['kind'] for r in rows])
    dup = np.array([r['dup'] == 'True' for r in rows])
    trusted = (status == 'snapped') & np.isin(kind, TRUSTED_KINDS)
    return {'mid': mid + off[:, None] * nv, 'nv': nv, 'len': L, 'off': off, 'status': status,
            'kind': kind, 'dup': dup, 'trusted': trusted, 'line': np.array([int(r['line']) for r in rows]),
            'seg': np.array([int(r['seg']) for r in rows])}


def sample_segments(seg, step):
    """区間を step 間隔の点に分ける。返り値は (点 [N,2]、点の元の区間の番号 [N])。"""
    pts, idx = [], []
    for i in range(len(seg['len'])):
        t = np.array([-seg['nv'][i, 1], seg['nv'][i, 0]])
        k = max(2, int(math.ceil(seg['len'][i] / step)) + 1)
        s = np.linspace(-seg['len'][i] / 2, seg['len'][i] / 2, k)
        pts.append(seg['mid'][i] + s[:, None] * t)
        idx.append(np.full(k, i))
    if not pts:
        return np.zeros((0, 2)), np.zeros(0, int)
    return np.vstack(pts), np.concatenate(idx)


def cell_xy(rows, cols, H, res, ox, oy):
    return ox + (cols + 0.5) * res, oy + (H - rows - 0.5) * res


def box_mask(hw, res, ox, oy, box):
    """map 座標の箱 (x0, x1, y0, y1) に中心が入るセル。画像の 0 行が map の y 最大。"""
    H, W = hw
    x0, x1, y0, y1 = box
    c0, c1 = max(0, int(math.ceil((x0 - ox) / res - 0.5))), min(W - 1, int(math.floor((x1 - ox) / res - 0.5)))
    r0 = max(0, int(math.ceil(H - 0.5 - (y1 - oy) / res)))
    r1 = min(H - 1, int(math.floor(H - 0.5 - (y0 - oy) / res)))
    m = np.zeros(hw, bool)
    if c0 <= c1 and r0 <= r1:
        m[r0:r1 + 1, c0:c1 + 1] = True
    return m


def square_dilate(m, r):
    """正方形（(2r+1)×(2r+1)）で太らせる。楕円の 3×3 は十字で、斜めの点の並びの間を閉じない（MF-1）。"""
    if r <= 0:
        return m.copy()
    return cv2.dilate(m.astype(np.uint8), np.ones((2 * r + 1, 2 * r + 1), np.uint8)).astype(bool)


def touching(b, hand):
    """b の 8 連結の成分のうち、手塗りに 8 近傍で接するものだけ。"""
    n, lab = cv2.connectedComponents(b.astype(np.uint8), connectivity=8)
    touch = square_dilate(hand, 1)
    ok = np.zeros(n, bool)
    ok[np.unique(lab[b & touch])] = True
    ok[0] = False
    return ok[lab]


def gap_dilate(m, gap):
    """各セルを、自分を角とする (gap+1)×(gap+1) の正方形に広げる（片側だけ）。2 点の正方形が 8 近傍で
    接するのは、各軸の差が gap + 1 以下、つまり間の隙間が gap セル以下のときだけ。中心に置いた半径 gap の
    正方形だと、2 つの正方形が接するだけで連結になり、隙間 2×gap まで繋がってしまう（レビュー SF-1）。"""
    if gap <= 0:
        return m.copy()
    k = np.ones((gap + 1, gap + 1), np.uint8)
    return cv2.dilate(m.astype(np.uint8), k, anchor=(0, 0)).astype(bool)


def small_components(m, max_cells, gap=0):
    """m の 8 連結の成分のうち max_cells 以下のもの。gap = 0 は gen_auto_free_mask.py の判定（生の格子、
    太らせない、8 連結、面積 ≤ 3）と同じ。gap > 0 は、gap セル以下の隙間で並ぶ点を 1 つの成分とみなし
    （gap_dilate で広げて連結を取る）、その成分の生のセル数で判定する。10 cm ボクセルの生垣・柵は 2 セル
    ごと（間に 1 セル）の点の格子になり、生の 8 連結では 1 点ずつ ≤ 3 になるため（MF-1 と同じ事情）。
    gap > 0 で選んだ点も生の成分は ≤ max_cells なので、合成の gen_auto_free_mask はそのまま消す。
    返り値は (そのセル、ラベル画像。gap > 0 では広げた格子のラベルで、m のセルは必ずラベルを持つ)。"""
    g = gap_dilate(m, gap)
    n, lab = cv2.connectedComponents(g.astype(np.uint8), connectivity=8)
    size = np.bincount(lab[m], minlength=n)
    small = size <= max_cells
    small[0] = False   # ラベル 0 は背景
    return m & small[lab], lab


def small_holes(mask, max_cells):
    """mask の中の、max_cells 以下の穴（4 連結の ~mask の成分で、格子の端に接しないもの）。"""
    n, lab, stats, _ = cv2.connectedComponentsWithStats((~mask).astype(np.uint8), connectivity=4)
    H, W = mask.shape
    x, y, w, h, area = (stats[:, i] for i in range(5))
    ok = (area <= max_cells) & (x > 0) & (y > 0) & (x + w < W) & (y + h < H)
    ok[0] = False   # ラベル 0 は mask 自身
    return ok[lab]


# ---------------------------------------------------------------- 広げる帯

def widen_band(hand, gsi, barrier, exclude, pts, pts_trusted, res, ox, oy, max_widen_m, tie_m,
               barrier_dilate_m=0.0, pts_nv=None, keepout=None, salt_max_cells=0, drop=None, salt_gap=0):
    """広げてよい帯を返す（モジュールの説明の 1〜8 と R6、R3 の 2 回目）。

    hand, gsi, barrier, exclude, keepout：同じ形の bool の格子。pts：補正後の縁の点（map 座標 [N,2]）、
    pts_trusted：その点が赤・橙の区間か、pts_nv：その点の区間の外向きの法線（[N,2]。None なら 4. を省く）。
    salt_max_cells：帯へ戻す小さい障害物の大きさの上限（R6。0 なら戻さない）。salt_gap：その成分を
    取るときに、この数のセルの隙間までを同じ成分とみなす（small_components）。drop：候補から除くセル
    （R3 の上限まで広がった区間のセル。None なら除かない）。
    返り値は dict：band（戻した小さい障害物を含む）、band_nobox（除外の箱が無いときの帯。戻す判定は箱の
    後の候補で行うので、箱の中の点は戻さない）、salt（band のうち戻した小さい障害物のセル）、
    barrier_kept（残した障害物）、stages（段階ごとのセル数）、near_idx（候補のセルごとの最も近い赤・橙の
    点の番号、-1 は対象外）、d_hand、outside_edge（4. で落としたセル）。"""
    H = hand.shape[0]
    stages = {}
    # 手塗りからの距離。cv2 の距離変換は 0 のセルまでの距離を測るので、手塗りを 0 にする
    d_hand = cv2.distanceTransform((~hand).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
    outside = gsi & ~hand
    stages['gsi_outside_hand'] = int(outside.sum())
    cand = outside & (d_hand <= max_widen_m + 1e-9)
    stages['within_cap'] = int(cand.sum())
    rr, cc = np.nonzero(cand)
    keep = np.zeros(rr.size, bool)
    beyond = np.zeros(rr.size, bool)
    near_idx = np.full(rr.size, -1)
    if rr.size and pts_trusted.any():
        x, y = cell_xy(rr, cc, H, res, ox, oy)
        q = np.column_stack([x, y])
        dT, iT = cKDTree(pts[pts_trusted]).query(q)
        if (~pts_trusted).any():
            dU, _ = cKDTree(pts[~pts_trusted]).query(q)
        else:
            dU = np.full(rr.size, np.inf)
        keep = dT + tie_m < dU
        near_idx = np.flatnonzero(pts_trusted)[iT]
        if pts_nv is not None:
            # 最も近い縁の点から見て、外向きの法線の側（縁の外）にあるセル
            beyond = keep & (np.einsum('ij,ij->i', q - pts[near_idx], pts_nv[near_idx]) > 0)
    stages['nearest_trusted'] = int(keep.sum())
    stages['outside_snapped_edge'] = int(beyond.sum())
    keep &= ~beyond
    b = np.zeros_like(hand)
    b[rr[keep], cc[keep]] = True
    outside_edge = np.zeros_like(hand)
    outside_edge[rr[beyond], cc[beyond]] = True
    # keepout と R3 の除外を障害物より先に引く。R6 で点を戻すかは「残った候補 ∪ 手塗り」に収まるかで
    # 決めるので、候補を先に確定させる（障害物を引いた後の段階のセル数の意味は前と同じ）
    if keepout is not None:
        b &= ~keepout
    stages['minus_keepout'] = int(b.sum())
    if drop is not None:
        b &= ~drop
    stages['minus_capped'] = int(b.sum())
    r = int(round(barrier_dilate_m / res))
    absorbed = np.zeros_like(hand)
    if salt_max_cells > 0:
        small, lab = small_components(barrier, salt_max_cells, salt_gap)
        room = (b & ~exclude) | hand
        ok = small
        # 戻せない点（候補の外にはみ出す・残す障害物の太らせた所に掛かる）を落とすと、その点は残す側に
        # 回り、その周りがまた太らせた所になる。r = 1 なら 8 近傍は同じ成分なので 1 回で止まるが、
        # r ≥ 2 でも正しいように、変わらなくなるまで回す
        while True:
            bad = ok & (~room | square_dilate(barrier & ~ok, r))
            if not bad.any():
                break
            lut = np.ones(int(lab.max()) + 1, bool)
            lut[np.unique(lab[bad])] = False
            ok = ok & lut[lab]
        absorbed = ok
    barrier_kept = barrier & ~absorbed
    b &= ~square_dilate(barrier_kept, r)
    stages['minus_barrier'] = int(b.sum())
    band_nobox = touching(b, hand)
    b &= ~exclude
    stages['minus_exclude'] = int(b.sum())
    band = touching(b, hand)
    stages['band'] = int(band.sum())
    stages['dropped_islands'] = stages['minus_exclude'] - stages['band']
    salt = band & absorbed
    stages['salt_cells'] = int(salt.sum())
    idx_img = np.full(hand.shape, -1, np.int64)
    idx_img[rr, cc] = near_idx
    return {'band': band, 'band_nobox': band_nobox, 'salt': salt, 'barrier_kept': barrier_kept,
            'stages': stages, 'near_idx': idx_img, 'd_hand': d_hand, 'outside_edge': outside_edge}


def assign_segments(band, pts, pts_trusted, pidx, dup_pts, H, res, ox, oy):
    """帯のセルを、最も近い赤・橙の区間に割り当てたラベル画像（区間の番号 + 1、0 は帯の外）を返す。
    タイルの重なり（dup）の区間は相方と同じ縁なので、重なりでない赤・橙の点の最近傍へ寄せる（SF-6）。
    R3 の判定と集計の両方でこれを使い、「上限まで広がった区間」の定義を 1 つにする。"""
    nd = pts_trusted & ~dup_pts
    lab_img = np.zeros(band.shape, np.int32)
    br, bc = np.nonzero(band)
    if br.size == 0 or not nd.any():
        return lab_img
    bx_, by_ = cell_xy(br, bc, H, res, ox, oy)
    _, j_nd = cKDTree(pts[nd]).query(np.column_stack([bx_, by_]))
    lab_img[br, bc] = pidx[np.flatnonzero(nd)[j_nd]] + 1
    return lab_img


def capped_segments(lab_img, d_hand, min_cells, thr):
    """帯のセルが min_cells 以上で、そのセルの手塗りからの距離の最大が thr 以上の区間の番号（R3）。
    集計の widened_segments.csv の width_max_m >= thr と同じ判定。"""
    n = int(lab_img.max())
    if n == 0:
        return np.zeros(0, int)
    cnt = np.bincount(lab_img.ravel(), minlength=n + 1)[1:]
    mx = np.asarray(ndimage.maximum(d_hand, lab_img, index=np.arange(1, n + 1)))
    return np.flatnonzero((cnt >= min_cells) & (mx >= thr))


# ---------------------------------------------------------------- 合成と確認

def run(cmd):
    print('  $', ' '.join(str(c) for c in cmd), flush=True)
    r = subprocess.run([str(c) for c in cmd], capture_output=True, text=True)
    if r.returncode not in (0, 1):
        raise SystemExit(f'失敗（{r.returncode}）: {cmd[1]}\n{r.stderr[-2000:]}')
    return r


def compose(layers_yaml, roadway, free, keepout, vd):
    """README 3 節と同じ順（gen_auto_free_mask → compose_occupancy）。"""
    s = REPO / 'scripts'
    vd.mkdir(parents=True, exist_ok=True)
    r = run([sys.executable, s / 'gen_auto_free_mask.py', '--layers-yaml', layers_yaml, '--roadway-mask', roadway,
             '--output-free-mask-auto', vd / 'free_mask_auto.png', '--output-salt-csv', vd / 'salt.csv'])
    if r.returncode:
        raise SystemExit(f'gen_auto_free_mask 失敗: {r.stderr[-2000:]}')
    r = run([sys.executable, s / 'compose_occupancy.py', '--layers-yaml', layers_yaml, '--roadway-mask', roadway,
             '--free-mask', free, '--free-mask-auto', vd / 'free_mask_auto.png', '--keepout-mask', keepout,
             '--output-pgm', vd / 'final.pgm'])
    if r.returncode:
        raise SystemExit(f'compose_occupancy 失敗: {r.stderr[-2000:]}')
    return vd / 'final.yaml'


def route_check(map_yaml, waypoints):
    r = run([sys.executable, REPO / 'scripts/map_route_check.py', map_yaml, waypoints])
    return json.loads(r.stdout)


def load_pgm(map_yaml):
    m = yaml.safe_load(pathlib.Path(map_yaml).read_text())
    return np.array(Image.open(pathlib.Path(map_yaml).parent / m['image']))


def nav2_footprint(path):
    """nav2_params.yaml の global_costmap の robot_radius と inflation_radius を読む（経路計画は global 側）。"""
    doc = yaml.safe_load(path.read_text())
    gc = doc['global_costmap']['global_costmap']['ros__parameters']
    lc = doc['local_costmap']['local_costmap']['ros__parameters']
    return {'global_robot_radius': float(gc['robot_radius']),
            'global_inflation_radius': float(gc['inflation_layer']['inflation_radius']),
            'global_cost_scaling_factor': float(gc['inflation_layer']['cost_scaling_factor']),
            'local_robot_radius': float(lc['robot_radius']),
            'local_inflation_radius': float(lc['inflation_layer']['inflation_radius'])}


def pct(a, q):
    return round(float(np.percentile(a, q)), 3) if len(a) else None


# ---------------------------------------------------------------- 画像

def crop_slice(H, res, ox, oy, box):
    x0, x1, y0, y1 = box
    c0, c1 = int((x0 - ox) / res), int(math.ceil((x1 - ox) / res))
    r0, r1 = int(H - (y1 - oy) / res), int(math.ceil(H - (y0 - oy) / res))
    return slice(max(0, r0), r1), slice(max(0, c0), c1)


def to_px(xy, H, res, ox, oy, r0, c0):
    return [((x - ox) / res - 0.5 - c0, (oy + H * res - y) / res - 0.5 - r0) for x, y in xy]


def draw_crop(under, sl, layers, seg, boxes, H, res, ox, oy, path):
    """原寸（5 cm/画素）の切り出し。layers：(マスク, 色, α) の順に重ねる。区間は補正後の位置を根拠の色で描く。"""
    rs, cs = sl
    base = np.array(Image.open(under).convert('RGB'))[rs, cs].astype(np.float32)
    for m, col, a in layers:
        mm = m[rs, cs]
        base[mm] = base[mm] * (1 - a) + np.array(col, np.float32) * a
    im = Image.fromarray(base.clip(0, 255).astype(np.uint8))
    d = ImageDraw.Draw(im)
    r0, c0 = rs.start, cs.start
    for i in range(len(seg['len'])):
        t = np.array([-seg['nv'][i, 1], seg['nv'][i, 0]])
        P = [seg['mid'][i] - t * seg['len'][i] / 2, seg['mid'][i] + t * seg['len'][i] / 2]
        px = to_px(P, H, res, ox, oy, r0, c0)
        if not any(-50 < u < im.size[0] + 50 and -50 < v < im.size[1] + 50 for u, v in px):
            continue
        st = seg['status'][i]
        k = seg['kind'][i] if st == 'snapped' else ('far' if st == 'far' else 'none')
        d.line(px, fill=KIND_COLOR.get(k, KIND_COLOR['none']), width=2)
    for name, x0, x1, y0, y1 in boxes:
        px = to_px([(x0, y1), (x1, y0)], H, res, ox, oy, r0, c0)
        d.rectangle([px[0], px[1]], outline=(0, 255, 255), width=2)
        d.text((px[0][0] + 4, px[0][1] + 4), name, fill=(0, 255, 255))
    im.save(path, optimize=True)
    return path.name


def edge_of(m):
    return m & ~cv2.erode(m.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)


# ---------------------------------------------------------------- 区間ごとの集計

def cluster_list(mask, H, res, ox, oy, top=15):
    """mask の 8 連結の成分を面積の大きい順に、重心（map 座標）と面積で返す（場所の列挙用）。"""
    n, lab, stats, cen = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    order = np.argsort(-stats[1:, cv2.CC_STAT_AREA])[:top] + 1
    out = []
    for k in order:
        x, y = cell_xy(cen[k, 1], cen[k, 0], H, res, ox, oy)
        out.append({'x': round(float(x), 1), 'y': round(float(y), 1),
                    'm2': round(float(stats[k, cv2.CC_STAT_AREA]) * res * res, 2)})
    return {'components': int(n - 1), 'cells': int(mask.sum()), 'largest': out}


def guard_double_widen(args, hand, hw):
    """入力の手塗りが、前回このスクリプトが書いた広げた版と同じなら止める（MF-2）。"""
    h_in = sha256(args.hand_roadway)
    prev_sum = args.out_dir / 'summary.json'
    if prev_sum.is_file():
        try:
            prev = json.loads(prev_sum.read_text()).get('outputs_sha256', {}).get('roadway_widened')
        except json.JSONDecodeError:
            prev = None
        if prev and prev == h_in:
            raise SystemExit(f'{args.hand_roadway} は前回の広げた版（{short_path(prev_sum)} の sha256）と同じ。'
                             '二重に広げないよう止める。採用前の版を --hand-roadway に渡す（README 154w 節）。')
    prev_png = args.work_dir / 'roadway_widened.png'
    if prev_png.is_file() and np.array_equal(load_mask(prev_png, hw), hand):
        raise SystemExit(f'{args.hand_roadway} の塗りは前回の広げた版（{short_path(prev_png)}）と同じ。'
                         '二重に広げないよう止める。採用前の版を --hand-roadway に渡す（README 154w 節）。')
    return h_in


def key_numbers(s):
    """比べるときに並べる数字（README の 2 つの版の表の出所）。summary.json の dict から取る。"""
    w = s['areas']['whole']
    occ = s['clearance']['occ']
    rw = s['routes']['widened']
    return {'band_m2': w['band_m2'], 'free_gained_m2': w['free_gained_m2'],
            'free_gained_pct': round(100 * w['free_gained_m2'] / w['free_baseline_m2'], 2),
            'salt_cells': s['band_salt']['cells'], 'salt_components': s['band_salt']['components'],
            'capped_segments': s['capped']['segments'], 'capped_band_m2': s['capped']['band_m2'],
            'gained_ge_robot_radius_m2': s['usable']['gained_ge_robot_radius_m2'],
            'gained_ge_inflation_m2': s['usable']['gained_ge_inflation_m2'],
            'newly_lt_inflation_segments': occ['newly_lt_inflation'], 'newly_lt_inflation_sites': len(occ['sites']),
            'all_connected': rw['all_connected'], 'eroded_free_components': rw.get('eroded_free_components'),
            'occ_cells': s['counts']['widened'][str(OCC)], 'checks_all_true': all(s['checks'].values()),
            'watch_band_m2': s['watch_band_m2']}


# ---------------------------------------------------------------- main

def main(argv=None):
    args = parse_args(argv)
    t0 = time.time()
    boxes = [(b[0], *map(float, b[1:])) for b in args.exclude_box] if args.exclude_box else DEFAULT_EXCLUDE
    doc, W, H, res, ox, oy = load_grid(args.layers_yaml)
    hw = (H, W)
    a = res * res
    ld = args.layers_yaml.parent
    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.work_dir.mkdir(parents=True, exist_ok=True)

    hand = load_mask(args.hand_roadway, hw)
    inputs_sha = {'hand_roadway': guard_double_widen(args, hand, hw)}
    for k_, p_ in (('hand_free', args.hand_free), ('keepout', args.keepout), ('gsi_road', args.gsi_road),
                   ('segments', args.segments)):
        inputs_sha[k_] = sha256(p_)
    hand_free = load_mask(args.hand_free, hw)
    keepout = load_mask(args.keepout, hw)
    gsi = load_mask(args.gsi_road, hw)
    step = load_opaque(ld / doc['layers']['occupied_step'], hw)
    struct = load_opaque(ld / doc['layers']['occupied_structure'], hw)
    barrier = step | struct
    # 塩の候補は段 ∪ 構造物と同じ集合のはず（確認用に読む）。違えば、帯の優先の前提が崩れるので止める
    salt = load_opaque(ld / doc['layers']['salt_candidates'], hw)
    salt_same = bool(np.array_equal(salt, barrier))
    if not salt_same:
        raise SystemExit('salt_candidates が occupied_step ∪ occupied_structure と一致しない。帯から除く集合を見直す。')
    del salt
    machine_free = np.array(Image.open(ld / doc['layers']['free_evidence']).convert('RGBA'))[..., 3] > 0
    exclude = np.zeros(hw, bool)
    box_m = {}
    for name, *bx in boxes:
        box_m[name] = box_mask(hw, res, ox, oy, bx)
        exclude |= box_m[name]

    seg = read_segments(args.segments)
    pts, pidx = sample_segments(seg, args.sample_m)
    pts_trusted = seg['trusted'][pidx]
    print(f'[区間] {len(seg["len"]):,}（赤・橙 {int(seg["trusted"].sum()):,}）、点 {len(pts):,}', flush=True)

    dup_pts = seg['dup'][pidx]
    cap_thr = args.max_widen_m - res / 2

    def band_pass(drop):
        return widen_band(hand, gsi, barrier, exclude, pts, pts_trusted, res, ox, oy, args.max_widen_m,
                          args.tie_m, args.barrier_dilate_m, seg['nv'][pidx], keepout,
                          args.band_salt_max_cells, drop, args.band_salt_gap_cells)

    wb = band_pass(None)
    # R3：1 回目の帯で上限まで広がった区間を求め、その区間のセルを除いて作り直す
    lab1 = assign_segments(wb['band'], pts, pts_trusted, pidx, dup_pts, H, res, ox, oy)
    cap1 = capped_segments(lab1, wb['d_hand'], args.min_seg_cells, cap_thr)
    cap_cells = np.isin(lab1, cap1 + 1)
    cap_rows = []
    for i in cap1:
        cap_rows.append({'line': int(seg['line'][i]), 'seg': int(seg['seg'][i]),
                         'x': round(float(seg['mid'][i, 0]), 2), 'y': round(float(seg['mid'][i, 1]), 2),
                         'kind': str(seg['kind'][i]), 'band_cells': int((lab1 == i + 1).sum())})
    capped = {'policy': args.capped_policy, 'segments': int(cap1.size),
              'band_cells': int(cap_cells.sum()), 'band_m2': round(float(cap_cells.sum()) * a, 2),
              'first_pass_band_m2': round(float(wb['band'].sum()) * a, 2),
              'first_pass_salt_cells': wb['stages']['salt_cells'], 'list': cap_rows}
    if args.capped_policy == 'drop' and cap1.size:
        wb = band_pass(cap_cells)
    del lab1
    if args.capped_policy != 'drop':
        cap_cells = np.zeros(hw, bool)
    band, stages, idx_img, d_hand = wb['band'], wb['stages'], wb['near_idx'], wb['d_hand']
    salt_band, barrier_kept = wb['salt'], wb['barrier_kept']
    print(f'[上限] {json.dumps({k_: v_ for k_, v_ in capped.items() if k_ != "list"}, ensure_ascii=False)}',
          flush=True)
    print(f'[帯] {stages}', flush=True)

    # 除外の箱（Q2・R1・R5）：補正後の面・箱が無いときの帯・箱の後の帯を実測で残す（箱が効いたかどうかの記録）
    q2 = {}
    for name, *bx in boxes:
        m = box_m[name]
        q2[name] = {'box': bx, 'gsi_outside_hand_m2': round(float((gsi & ~hand & m).sum()) * a, 2),
                    'gsi_outside_hand_within_cap_m2':
                        round(float((gsi & ~hand & m & (d_hand <= args.max_widen_m)).sum()) * a, 2),
                    'band_without_box_m2': round(float((wb['band_nobox'] & m).sum()) * a, 2),
                    'band_m2': round(float((band & m).sum()) * a, 2)}
    print(f'[Q2] {json.dumps(q2, ensure_ascii=False)}', flush=True)

    # ---- 合成（手塗りのまま・広げた版の 2 つ。手塗りのままは今の既定の地図と一致するはず）
    # R6 で戻した点は roadway にだけ足す。free にも塗ると、合成の gen_auto_free_mask の規則（≤3 セル）ではなく
    # 人の塗りとして消えたことになり、erased_by_free.png で区別できない（モジュールの説明の R6）
    roadway_w = hand | band
    free_w = hand_free | (band & ~salt_band) if args.free_policy == 'paint' else hand_free
    save_mask_rgba(args.work_dir / 'roadway_widened.png', roadway_w)
    save_mask_rgba(args.work_dir / 'free_widened.png', free_w)
    save_mask_rgba(args.work_dir / 'band.png', band)
    hand_yaml = compose(args.layers_yaml, args.hand_roadway, args.hand_free, args.keepout,
                        args.work_dir / 'compose_hand')
    new_yaml = compose(args.layers_yaml, args.work_dir / 'roadway_widened.png', args.work_dir / 'free_widened.png',
                       args.keepout, args.work_dir / 'compose')
    base_img = load_pgm(args.baseline_map)
    hand_img = load_pgm(hand_yaml)
    new_img = load_pgm(new_yaml)
    routes = {'baseline': route_check(args.baseline_map, args.waypoints),
              'widened': route_check(new_yaml, args.waypoints)}
    for k_, r_ in routes.items():
        wp = ' '.join(f'{w["name"]}:{w["free"]}/{w["component"]}/{w["snap_m"]}' for w in r_['waypoints'])
        print(f'[経路] {k_}: all_connected={r_["all_connected"]} {wp}', flush=True)

    # ---- 確かめること（SF-7）
    base_free, new_free = base_img == FREE, new_img == FREE
    gained = new_free & ~base_free
    lost = base_free & ~new_free
    changed = base_img != new_img
    # R6：帯で消えた点（合成の free_mask_auto の、手塗りのままとの差分）が、戻した点と 1 セルも違わない
    auto_new = load_mask(args.work_dir / 'compose/free_mask_auto.png', hw)
    auto_hand = load_mask(args.work_dir / 'compose_hand/free_mask_auto.png', hw)
    auto_gain = auto_new & ~auto_hand
    checks = {'hand_only_compose_equals_baseline': bool(np.array_equal(hand_img, base_img)),
              'changed_cells_within_band': int((changed & ~band).sum()) == 0,
              'band_disjoint_from_hand_keepout_barrier':
                  int((band & (hand | keepout | barrier_kept)).sum()) == 0,
              'band_salt_erased_by_auto_rule': bool(np.array_equal(auto_gain, salt_band))
              and int((auto_hand & ~auto_new).sum()) == 0,
              'salt_candidates_equal_step_or_structure': salt_same}
    if args.free_policy == 'paint':
        checks['gained_free_equals_band'] = bool(np.array_equal(gained, band))
    else:
        # evidence では帯を free に塗らないので、free_evidence の無いセルは UNKNOWN のまま。増えた FREE は帯の
        # 部分集合になるだけ（672bd81 までは paint と同じ一致を求めていて、evidence では必ず外れていた）
        checks['gained_free_within_band'] = int((gained & ~band).sum()) == 0
    if args.capped_policy == 'drop':
        # R3 で落とした区間のセルが 1 つも帯に残っていない（直接の確認。下の「上限まで届く区間が無い」とは別）
        checks['capped_segment_cells_absent_from_band'] = int((band & cap_cells).sum()) == 0
    del auto_new, auto_hand, auto_gain
    print(f'[確認] {checks}', flush=True)

    # ---- 面積
    area = {'whole': None}
    area.update({k_: box_mask(hw, res, ox, oy, v) for k_, v in REGIONS.items()})
    areas = {}
    for k_, m in area.items():
        mm = np.ones(hw, bool) if m is None else m
        areas[k_] = {'band_m2': round(float((band & mm).sum()) * a, 2),
                     'free_gained_m2': round(float((gained & mm).sum()) * a, 2),
                     'free_lost_m2': round(float((lost & mm).sum()) * a, 2),
                     'free_baseline_m2': round(float((base_free & mm).sum()) * a, 1),
                     'free_widened_m2': round(float((new_free & mm).sum()) * a, 1)}
    watch = {k_: round(float((band & box_mask(hw, res, ox, oy, v)).sum()) * a, 2) for k_, v in WATCH.items()}
    grass = np.zeros(hw, bool)
    if args.intensity_npz.is_file():
        with np.load(args.intensity_npz) as z:
            im_ = z['intensity_mean']
            grass = np.isfinite(im_) & (z['intensity_count'] > 0) & (im_ >= args.grass_intensity)
        del im_

    # ---- 区間ごとの集計
    # 帯のセルを、最も近い赤・橙の区間に割り当てる。タイルの重なり（dup）の区間は相方と同じ縁なので、
    # 集計では重なりでない赤・橙の点の最近傍へ寄せ直す（SF-6。寄せ直したセル数を記録する）
    br, bc = np.nonzero(band)
    reassigned = int(seg['dup'][pidx[idx_img[br, bc]]].sum())
    lab_img = assign_segments(band, pts, pts_trusted, pidx, dup_pts, H, res, ox, oy)

    occ_new = new_img == OCC
    occ_base = base_img == OCC

    def dist_to(m):
        return cv2.distanceTransform((~m).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res

    d_occ, d_step = dist_to(occ_new), dist_to(occ_new & step)
    d_occ_b, d_step_b = dist_to(occ_base), dist_to(occ_base & step)
    # 外の縁：小さい穴（帯の中に残る段・構造物の点と、太らせた分）を埋めた FREE の縁（SF-1）
    hmax = int(round(args.hole_max_m2 / a))
    new_outer = edge_of(new_free | small_holes(new_free, hmax))
    base_filled = base_free | small_holes(base_free, hmax)
    base_outer = edge_of(base_filled)
    fp = nav2_footprint(args.nav2_params)
    infl, rr_ = fp['global_inflation_radius'], fp['global_robot_radius']
    usable = {'gained_ge_robot_radius_m2': round(float((gained & (d_occ >= rr_)).sum()) * a, 2),
              'gained_ge_inflation_m2': round(float((gained & (d_occ >= infl)).sum()) * a, 2)}
    seg_rows = []
    objs = ndimage.find_objects(lab_img)
    for i0, sl in enumerate(objs):
        if sl is None:
            continue
        i = i0   # ラベルは区間の番号 + 1
        sl2 = tuple(slice(max(0, s.start - 2), s.stop + 2) for s in sl)
        m = lab_img[sl2] == i + 1
        n_ = int(m.sum())
        if n_ < args.min_seg_cells:
            continue
        widths = d_hand[sl2][m]
        e = m & new_outer[sl2]
        # 今の縁：この区間の帯のセルに 8 近傍で接する、今の FREE（穴を埋めた）の縁のセル（SF-2）
        eo = square_dilate(m, 1) & base_outer[sl2] & ~m
        x, y = seg['mid'][i]

        def mn(d, mm):
            return round(float(d[sl2][mm].min()), 3) if mm.any() else None
        seg_rows.append({'line': int(seg['line'][i]), 'seg': int(seg['seg'][i]), 'x': round(float(x), 2),
                         'y': round(float(y), 2), 'kind': str(seg['kind'][i]), 'offset_m': float(seg['off'][i]),
                         'band_cells': n_, 'width_max_m': round(float(widths.max()), 3),
                         'width_median_m': round(float(np.median(widths)), 3),
                         'bright_frac': round(float(grass[sl2][m].mean()), 3),
                         'new_edge_cells': int(e.sum()),
                         'new_edge_to_occ_min_m': mn(d_occ, e), 'new_edge_to_step_min_m': mn(d_step, e),
                         'old_edge_cells': int(eo.sum()),
                         'old_edge_to_occ_min_m': mn(d_occ_b, eo), 'old_edge_to_step_min_m': mn(d_step_b, eo)})
    wmax = np.array([r['width_max_m'] for r in seg_rows])
    by_kind = {k_: sum(1 for r in seg_rows if r['kind'] == k_) for k_ in TRUSTED_KINDS}

    def vals(key):
        return np.array([r[key] for r in seg_rows if r[key] is not None])

    def lt(key, thr):
        return sum(1 for r in seg_rows if r[key] is not None and r[key] < thr)
    clearance = {}
    for tgt in ('occ', 'step'):
        nk, ok_ = f'new_edge_to_{tgt}_min_m', f'old_edge_to_{tgt}_min_m'
        nv_, ov_ = vals(nk), vals(ok_)
        newly = [r for r in seg_rows if r[nk] is not None and r[nk] < infl
                 and r[ok_] is not None and r[ok_] >= infl]
        clearance[tgt] = {
            'segments_with_new_edge': int(nv_.size), 'new_lt_inflation': lt(nk, infl),
            'new_lt_robot_radius': lt(nk, rr_),
            'new_median_m': pct(nv_, 50), 'new_p90_m': pct(nv_, 90),
            'segments_with_old_edge': int(ov_.size), 'old_lt_inflation': lt(ok_, infl),
            'old_lt_inflation_share': round(lt(ok_, infl) / ov_.size, 3) if ov_.size else None,
            'old_median_m': pct(ov_, 50), 'newly_lt_inflation': len(newly)}
        if tgt == 'occ':
            newly_rows = newly
    # 今は 0.6 m 以上あり、広げて 0.6 m 未満になる区間（全 OCC まで）を、同じ線で続く区間ごとにまとめる
    sites = []
    for r in sorted(newly_rows, key=lambda r: (r['line'], r['seg'])):
        if sites and sites[-1]['line'] == r['line'] and r['seg'] - sites[-1]['seg_to'] <= 2:
            s_ = sites[-1]
            s_['seg_to'] = r['seg']
            s_['n_segments'] += 1
            s_['xs'].append(r['x'])
            s_['ys'].append(r['y'])
            s_['new_min'] = min(s_['new_min'], r['new_edge_to_occ_min_m'])
            s_['old_min'] = min(s_['old_min'], r['old_edge_to_occ_min_m'])
            s_['widen_max_m'] = max(s_['widen_max_m'], r['width_max_m'])
        else:
            sites.append({'line': r['line'], 'seg_from': r['seg'], 'seg_to': r['seg'], 'n_segments': 1,
                          'xs': [r['x']], 'ys': [r['y']], 'new_min': r['new_edge_to_occ_min_m'],
                          'old_min': r['old_edge_to_occ_min_m'], 'widen_max_m': r['width_max_m']})
    for s_ in sites:
        xs, ys = s_.pop('xs'), s_.pop('ys')
        s_['x_range'], s_['y_range'] = [min(xs), max(xs)], [min(ys), max(ys)]
    clearance['occ']['sites'] = sites
    with open(args.out_dir / 'widened_segments.csv', 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=list(seg_rows[0].keys()) if seg_rows else ['line'])
        wr.writeheader()
        wr.writerows(seg_rows)

    # ---- 数値（README に書く数字の出所。SF-4）
    capped_rows = [r for r in seg_rows if r['width_max_m'] >= cap_thr]
    if args.capped_policy == 'drop':
        # 2 回目は候補が減るだけなので、上限まで広がる区間は残らないはず（R3）
        checks['capped_policy_drop_leaves_no_capped_segment'] = len(capped_rows) == 0
    # 抜け穴の記録：帯のセルが --min-seg-cells 未満の区間は「上限まで広がった区間」に数えないので、上限まで
    # 届いていても R3 で落ちない。その区間の数とセル数を残す
    reach = capped_segments(lab_img, d_hand, 1, cap_thr)
    n_reach = np.bincount(lab_img.ravel(), minlength=int(lab_img.max()) + 1)[1:]
    small_reach = reach[n_reach[reach] < args.min_seg_cells] if reach.size else reach
    capped['small_segments_reaching_cap'] = {'segments': int(small_reach.size),
                                             'cells': int(n_reach[small_reach].sum()) if small_reach.size else 0}
    # R6 の点の数え方：戻した点の 8 連結の成分（手塗りにまたがる成分も 1 つと数える）と大きさ
    n_salt, salt_lab, salt_st, _ = cv2.connectedComponentsWithStats(salt_band.astype(np.uint8), connectivity=8)
    del salt_lab
    salt_sizes = salt_st[1:, cv2.CC_STAT_AREA]
    band_salt = {'max_cells': args.band_salt_max_cells, 'cells': int(salt_band.sum()),
                 'components': int(n_salt - 1),
                 'by_size': {str(k_): int((salt_sizes == k_).sum()) for k_ in range(1, args.band_salt_max_cells + 1)},
                 'm2': round(float(salt_band.sum()) * a, 4)}
    if args.band_salt_max_cells > 0:
        # 1 セルの隙間を閉じると 4 セル以上の塊になる点（2 セルごとの点の格子）。gap 0 のときに消える数。
        # gap ≥ 1 では、戻した点はすべて 1 セルの隙間でも孤立しているので 0 になる
        iso1, _ = small_components(barrier, args.band_salt_max_cells, 1)
        band_salt['cells_in_dot_lattice'] = int((salt_band & ~iso1).sum())
        del iso1
    # 帯から段・構造物を 1 セル太らせて除いたので、OCC は帯に 8 近傍では接しない。2 セル（0.1 m）以内で数える
    band_touch_occ = occ_new & square_dilate(band, 2) & ~hand
    new_holes = small_holes(roadway_w, hmax) & ~small_holes(hand, hmax)
    nh, _ = cv2.connectedComponents(new_holes.astype(np.uint8), connectivity=4)
    derived = {
        'capped_segments': len(capped_rows),
        'capped_band_m2': round(sum(r['band_cells'] for r in capped_rows) * a, 2),
        'capped_band_share': round(sum(r['band_cells'] for r in capped_rows) / max(1, int(band.sum())), 3),
        'bright_band_m2': round(float((band & grass).sum()) * a, 2),
        'bright_band_share': round(float((band & grass).sum()) / max(1, int(band.sum())), 3),
        'bright_hand_share': round(float((hand & grass).sum()) / max(1, int(hand.sum())), 3),
        'band_cells_without_free_evidence': int((band & ~machine_free).sum()),
        'occ_in_new_holes_cells': int((occ_new & new_holes).sum()),
        'occ_within_0_1m_of_band': {
            'cells': int(band_touch_occ.sum()),
            'components': int(cv2.connectedComponents(band_touch_occ.astype(np.uint8), connectivity=8)[0] - 1)},
        'new_holes_in_roadway_mask': {'components': int(nh - 1), 'cells': int(new_holes.sum())},
        'cells_reassigned_from_dup_segments': reassigned,
        'outside_snapped_edge': cluster_list(wb['outside_edge'], H, res, ox, oy),
    }

    # ---- 画像（原寸の切り出し。帯 = 桃、手塗りの roadway = 青の薄塗り、段 = 赤、構造物 = 橙）
    png = {}
    # 帯は薄く塗って縁を濃く描く（下絵の芝・舗装が見えるように）
    # 消した点（R6）は段・構造物の上に水色で重ねる（残した点と見分ける）
    lay = [(hand, (0, 90, 255), 0.25), (band, BAND_COLOR, 0.35), (edge_of(band), BAND_COLOR, 1.0),
           (step, (255, 0, 0), 0.9), (struct, (255, 150, 0), 0.9), (salt_band, SALT_COLOR, 1.0),
           (edge_of(hand), (0, 60, 255), 1.0)]
    for cname, *cb in CROPS:
        sl = crop_slice(H, res, ox, oy, cb)
        for uname, up in (('intensity', args.underlay_intensity), ('photo', args.underlay_photo)):
            if up.is_file():
                png[f'{cname}_{uname}'] = draw_crop(up, sl, lay, seg, boxes, H, res, ox, oy,
                                                    args.out_dir / f'{cname}_crop_{uname}_1to1.png')
    # 全体の 1/8（帯だけを太らせて見えるようにする）
    k = 8
    ov = np.full((H // k, W // k, 3), 255, np.uint8)
    ds = lambda m: m[:H // k * k, :W // k * k].reshape(H // k, k, W // k, k).any(axis=(1, 3))  # noqa: E731
    ov[ds(base_free)] = (190, 210, 255)
    ov[ds(occ_new)] = (60, 60, 60)
    ov[cv2.dilate(ds(band).astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)] = BAND_COLOR
    Image.fromarray(ov).save(args.out_dir / 'overview_band_8.png', optimize=True)
    png['overview'] = 'overview_band_8.png'

    if not args.no_gimp:
        args.gimp_dir.mkdir(parents=True, exist_ok=True)
        g = np.zeros((H, W, 4), np.uint8)
        g[band] = (*BAND_COLOR, 255)
        g[salt_band] = (*SALT_COLOR, 255)
        Image.fromarray(g, 'RGBA').save(args.gimp_dir / '19_widened_candidate.png', compress_level=3)
        print(f'[GIMP] {args.gimp_dir}/19_widened_candidate.png', flush=True)

    summary = {
        'params': {k_: (short_path(v) if isinstance(v, pathlib.Path) else v) for k_, v in vars(args).items()
                   if k_ != 'exclude_box'},
        'inputs_sha256': inputs_sha,
        'outputs_sha256': {'roadway_widened': sha256(args.work_dir / 'roadway_widened.png'),
                           'free_widened': sha256(args.work_dir / 'free_widened.png'),
                           'final_pgm': sha256(args.work_dir / 'compose/final.pgm')},
        'checks': checks,
        'exclude_boxes': [list(b) for b in boxes], 'regions': REGIONS, 'watch': WATCH,
        'segments': {'total': int(len(seg['len'])), 'trusted': int(seg['trusted'].sum()),
                     'trusted_step': int((seg['trusted'] & (seg['kind'] == 'step')).sum()),
                     'trusted_structure': int((seg['trusted'] & (seg['kind'] == 'structure')).sum()),
                     'trusted_non_dup': int((seg['trusted'] & ~seg['dup']).sum())},
        'stages_cells': stages, 'q2_boxes': q2, 'capped': capped, 'band_salt': band_salt,
        'areas': areas, 'watch_band_m2': watch,
        'band_cells': int(band.sum()), 'usable': usable, 'derived': derived,
        'widened_segments': {'count': len(seg_rows), 'by_kind': by_kind,
                             'width_max_median_m': pct(wmax, 50), 'width_max_p90_m': pct(wmax, 90),
                             'width_max_max_m': pct(wmax, 100)},
        'nav2': fp, 'clearance': clearance,
        'routes': routes,
        'counts': {'baseline': {str(v): int((base_img == v).sum()) for v in (OCC, FREE, UNKNOWN)},
                   'widened': {str(v): int((new_img == v).sum()) for v in (OCC, FREE, UNKNOWN)}},
        'png': png, 'elapsed_s': round(time.time() - t0, 1),
    }
    summary['key'] = key_numbers(summary)
    if args.compare_summary is not None:
        other = json.loads(args.compare_summary.read_text())
        summary['compare'] = {'summary': short_path(args.compare_summary),
                              'band_salt_gap_cells': other['params'].get('band_salt_gap_cells'),
                              'key': key_numbers(other)}
    for r_ in summary['routes'].values():
        r_['map'] = short_path(r_['map'])
    (args.out_dir / 'summary.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False) + '\n')
    print(json.dumps({k_: summary[k_] for k_ in ('areas', 'widened_segments', 'usable', 'derived', 'watch_band_m2',
                                                 'band_salt')}, ensure_ascii=False, indent=1))
    print(json.dumps({t: {k2: v2 for k2, v2 in c.items() if k2 != 'sites'} for t, c in clearance.items()},
                     ensure_ascii=False))
    print(f'[完了] {summary["elapsed_s"]} s')
    if not all(checks.values()):
        print(f'[失敗] 確認が外れた：{checks}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
