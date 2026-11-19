#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""campus-m8 の地図座標（map、GNSS なし）を国土地理院（GSI）の建物形状に合わせ、
外部の参照で走行可能域を点検する（#205）。

なぜ要るか：手塗りの roadway（`candidate_roadway_mask.png`）も、縁石からの自動抽出
（#202）も、どちらも自分たちの点群から作ったもので、互いに独立な検証にならない。
GSI の電子国土基本図（地図情報レベル 2500 の道路縁・建物）は第三者が航空写真から
作ったもので、点群とは独立。ただし map と GSI の対応は誰も測っていないので、
まず両者の建物の形を合わせて map → 平面直角座標系第 IX 系（EPSG:6677）を求める。

サブコマンド：
  fetch  地理院タイル（全国最新写真 z18、地理院ベクトル z16）を取得して raw に置く。
         既にあるタイルは取り直さない（再実行で通信しない）。
  align  構造物層と GSI の建物を合わせて剛体変換を推定し、georef.yaml を書く。
  report 変換を使って重ね絵と数値（道路域との IoU など）を作る。

外部ライブラリ（pyproj、shapely、mapbox_vector_tile）は本機に無く、入れると依存が
増えるので使わない。投影（ガウス・クリューゲル）と MVT の復号は数十行で書ける範囲で、
投影は下の単体テスト（scripts/tests/test_georef_gsi.py）で往復と既知値を確かめる。
"""

import argparse
import json
import math
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

REPO = Path(__file__).resolve().parent.parent

# --- 楕円体と投影 ---------------------------------------------------------------
# JGD2011 の楕円体は GRS80。第 IX 系の原点は 36°N、139°50'E、縮尺係数 0.9999
# （平成14年国土交通省告示第9号）。
GRS80_A = 6378137.0
GRS80_F = 1.0 / 298.257222101
ZONE9_LAT0 = 36.0
ZONE9_LON0 = 139.0 + 50.0 / 60.0
ZONE9_K0 = 0.9999

# 地理院タイルは球面メルカトル（EPSG:3857 と同じ式）。半径は WGS84 の長半径。
WEB_R = 6378137.0


def _tm_coeffs(f=GRS80_F):
    """Krüger 級数（n の 6 次まで）の係数。国土地理院「平面直角座標への換算」
    （河瀬 2011）と同じ展開で、日本の範囲では mm 未満の打ち切り誤差。"""
    n = f / (2.0 - f)
    A = (1 + n**2 / 4 + n**4 / 64 + n**6 / 256) / (1 + n)
    alpha = [
        n / 2 - 2 * n**2 / 3 + 5 * n**3 / 16 + 41 * n**4 / 180 - 127 * n**5 / 288 + 7891 * n**6 / 37800,
        13 * n**2 / 48 - 3 * n**3 / 5 + 557 * n**4 / 1440 + 281 * n**5 / 630 - 1983433 * n**6 / 1935360,
        61 * n**3 / 240 - 103 * n**4 / 140 + 15061 * n**5 / 26880 + 167603 * n**6 / 181440,
        49561 * n**4 / 161280 - 179 * n**5 / 168 + 6601661 * n**6 / 7257600,
        34729 * n**5 / 80640 - 3418889 * n**6 / 1995840,
        212378941 * n**6 / 319334400,
    ]
    beta = [
        n / 2 - 2 * n**2 / 3 + 37 * n**3 / 96 - n**4 / 360 - 81 * n**5 / 512 + 96199 * n**6 / 604800,
        n**2 / 48 + n**3 / 15 - 437 * n**4 / 1440 + 46 * n**5 / 105 - 1118711 * n**6 / 3870720,
        17 * n**3 / 480 - 37 * n**4 / 840 - 209 * n**5 / 4480 + 5569 * n**6 / 90720,
        4397 * n**4 / 161280 - 11 * n**5 / 504 - 830251 * n**6 / 7257600,
        4583 * n**5 / 161280 - 108847 * n**6 / 3991680,
        20648693 * n**6 / 638668800,
    ]
    delta = [
        2 * n - 2 * n**2 / 3 - 2 * n**3 + 116 * n**4 / 45 + 26 * n**5 / 45 - 2854 * n**6 / 675,
        7 * n**2 / 3 - 8 * n**3 / 5 - 227 * n**4 / 45 + 2704 * n**5 / 315 + 2323 * n**6 / 945,
        56 * n**3 / 15 - 136 * n**4 / 35 - 1262 * n**5 / 105 + 73814 * n**6 / 2835,
        4279 * n**4 / 630 - 332 * n**5 / 35 - 399572 * n**6 / 14175,
        4174 * n**5 / 315 - 144838 * n**6 / 6237,
        601676 * n**6 / 22275,
    ]
    return n, A, alpha, beta, delta


def _meridian_s(lat_rad, n, A, alpha):
    """赤道からの子午線弧長を a・k0 で割る前の量（等角緯度を経由する式の ξ に相当）。"""
    t = np.sinh(np.arctanh(np.sin(lat_rad)) - 2 * math.sqrt(n) / (1 + n)
                * np.arctanh(2 * math.sqrt(n) / (1 + n) * np.sin(lat_rad)))
    xi = np.arctan(t)
    s = xi.copy() if isinstance(xi, np.ndarray) else xi
    for j, a in enumerate(alpha, start=1):
        s = s + a * np.sin(2 * j * xi)
    return A * s


def tm_forward(lat, lon, lat0=ZONE9_LAT0, lon0=ZONE9_LON0, k0=ZONE9_K0, a=GRS80_A, f=GRS80_F):
    """緯度経度（度）→ 平面直角座標（easting, northing、m）。

    返り値は (E, N)。EPSG:6677 の軸順は (X=北, Y=東) なので、YAML には
    X/Y ではなく easting/northing と書いて取り違えを防ぐ。"""
    n, A, alpha, _, _ = _tm_coeffs(f)
    lat = np.radians(np.asarray(lat, float))
    lon = np.radians(np.asarray(lon, float))
    dl = lon - math.radians(lon0)
    c = 2 * math.sqrt(n) / (1 + n)
    t = np.sinh(np.arctanh(np.sin(lat)) - c * np.arctanh(c * np.sin(lat)))
    tb = np.sqrt(1 + t**2)
    xi_p = np.arctan2(t, np.cos(dl))
    eta_p = np.arctanh(np.sin(dl) / tb)
    xi = xi_p.copy()
    eta = eta_p.copy()
    for j, al in enumerate(alpha, start=1):
        xi = xi + al * np.sin(2 * j * xi_p) * np.cosh(2 * j * eta_p)
        eta = eta + al * np.cos(2 * j * xi_p) * np.sinh(2 * j * eta_p)
    s0 = _meridian_s(math.radians(lat0), n, A, alpha)
    N = k0 * a * (A * xi - s0)
    E = k0 * a * A * eta
    return E, N


def tm_inverse(E, N, lat0=ZONE9_LAT0, lon0=ZONE9_LON0, k0=ZONE9_K0, a=GRS80_A, f=GRS80_F):
    """平面直角座標（easting, northing、m）→ 緯度経度（度）。"""
    n, A, alpha, beta, delta = _tm_coeffs(f)
    E = np.asarray(E, float)
    N = np.asarray(N, float)
    s0 = _meridian_s(math.radians(lat0), n, A, alpha)
    xi = (N + k0 * a * s0) / (k0 * a * A)
    eta = E / (k0 * a * A)
    xi_p = xi.copy()
    eta_p = eta.copy()
    for j, be in enumerate(beta, start=1):
        xi_p = xi_p - be * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
        eta_p = eta_p - be * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
    chi = np.arcsin(np.sin(xi_p) / np.cosh(eta_p))
    lat = chi.copy()
    for j, de in enumerate(delta, start=1):
        lat = lat + de * np.sin(2 * j * chi)
    lon = math.radians(lon0) + np.arctan2(np.sinh(eta_p), np.cos(xi_p))
    return np.degrees(lat), np.degrees(lon)


def meridian_convergence_deg(lat, lon, lon0=ZONE9_LON0):
    """子午線収差（真北に対する座標北の角度の近似、度）。真北の向きを報告するために使う。
    1 次の近似 γ ≈ Δλ sinφ で、この範囲（Δλ 0.1°）では 1e-5° 未満の誤差。"""
    return (lon - lon0) * math.sin(math.radians(lat))


# --- 地理院タイル（球面メルカトル） -------------------------------------------------
def lonlat_to_global_px(lat, lon, z):
    """緯度経度 → ズーム z の全体画素座標（タイル番号 × 256 + タイル内画素）。"""
    lat = np.asarray(lat, float)
    lon = np.asarray(lon, float)
    n = 256.0 * 2**z
    x = (lon + 180.0) / 360.0 * n
    lr = np.radians(lat)
    y = (1 - np.log(np.tan(lr) + 1 / np.cos(lr)) / math.pi) / 2 * n
    return x, y


def global_px_to_lonlat(x, y, z):
    n = 256.0 * 2**z
    lon = np.asarray(x, float) / n * 360.0 - 180.0
    lat = np.degrees(np.arctan(np.sinh(math.pi * (1 - 2 * np.asarray(y, float) / n))))
    return lat, lon


# --- map 座標 ↔ 平面直角座標 ---------------------------------------------------------
def make_transform(yaw_deg, tE, tN, scale=1.0):
    """map (x, y) → (E, N) の 3x3 同次行列。yaw は map の +x 軸が座標東から反時計回りに
    何度か（座標北・真北との関係は georef.yaml に別に書く）。"""
    c = math.cos(math.radians(yaw_deg)) * scale
    s = math.sin(math.radians(yaw_deg)) * scale
    return np.array([[c, -s, tE], [s, c, tN], [0, 0, 1.0]])


def apply(T, x, y):
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    return T[0, 0] * x + T[0, 1] * y + T[0, 2], T[1, 0] * x + T[1, 1] * y + T[1, 2]


def map_to_lonlat(T, x, y):
    E, N = apply(T, x, y)
    return tm_inverse(E, N)


# --- MVT（Mapbox Vector Tile）の最小限の復号 --------------------------------------------
# 地理院ベクトルは標準の MVT（protobuf）。使うのはレイヤ名・属性・線と面の座標だけ。
def _varint(b, i):
    r = 0
    s = 0
    while True:
        c = b[i]
        i += 1
        r |= (c & 0x7F) << s
        s += 7
        if c < 0x80:
            return r, i


def _fields(b):
    i = 0
    while i < len(b):
        k, i = _varint(b, i)
        f, w = k >> 3, k & 7
        if w == 0:
            v, i = _varint(b, i)
        elif w == 1:
            v = b[i:i + 8]
            i += 8
        elif w == 2:
            ln, i = _varint(b, i)
            v = b[i:i + ln]
            i += ln
        elif w == 5:
            v = b[i:i + 4]
            i += 4
        else:
            raise ValueError(f'未対応の wire type {w}')
        yield f, w, v


def _packed(b):
    out = []
    i = 0
    while i < len(b):
        v, i = _varint(b, i)
        out.append(v)
    return out


def _zigzag(v):
    return (v >> 1) ^ -(v & 1)


def _mvt_value(b):
    for f, _, v in _fields(b):
        if f == 1:
            return v.decode('utf-8')
        if f == 2:
            return struct.unpack('<f', v)[0]
        if f == 3:
            return struct.unpack('<d', v)[0]
        if f in (4, 5):
            return v
        if f == 6:
            return _zigzag(v)
        if f == 7:
            return bool(v)
    return None


def _mvt_geometry(cmds):
    """コマンド列 → 部分（点列）のリスト。ClosePath は始点を足して閉じる。"""
    parts = []
    cur = None
    x = y = 0
    i = 0
    while i < len(cmds):
        c = cmds[i]
        i += 1
        cid, cnt = c & 7, c >> 3
        if cid == 7:
            if cur:
                cur.append(cur[0])
            continue
        for _ in range(cnt):
            x += _zigzag(cmds[i])
            y += _zigzag(cmds[i + 1])
            i += 2
            if cid == 1:
                cur = [(x, y)]
                parts.append(cur)
            else:
                cur.append((x, y))
    return parts


def mvt_decode(data):
    """{layer: {'extent': int, 'features': [{'type', 'props', 'parts'}]}}。
    type は 1=点、2=線、3=面（MVT の GeomType）。"""
    layers = {}
    for f, _, v in _fields(data):
        if f != 3:
            continue
        name, feats, keys, vals, ext = None, [], [], [], 4096
        for f2, _, v2 in _fields(v):
            if f2 == 1:
                name = v2.decode('utf-8')
            elif f2 == 2:
                feats.append(v2)
            elif f2 == 3:
                keys.append(v2.decode('utf-8'))
            elif f2 == 4:
                vals.append(_mvt_value(v2))
            elif f2 == 5:
                ext = v2
        out = []
        for fb in feats:
            tags, typ, geom = [], 0, []
            for f3, _, v3 in _fields(fb):
                if f3 == 2:
                    tags = _packed(v3)
                elif f3 == 3:
                    typ = v3
                elif f3 == 4:
                    geom = _packed(v3)
            props = {keys[tags[j]]: vals[tags[j + 1]] for j in range(0, len(tags), 2)}
            out.append({'type': typ, 'props': props, 'parts': _mvt_geometry(geom)})
        layers[name] = {'extent': ext, 'features': out}
    return layers


# --- 取得 ------------------------------------------------------------------------
UA = 'map-research-client/1.0'
ORTHO_URL = 'https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg'
BVMAP_URL = 'https://cyberjapandata.gsi.go.jp/xyz/experimental_bvmap/{z}/{x}/{y}.pbf'


def tile_range(lat, lon, half_m, z):
    """中心から ±half_m の正方形を覆うタイル番号の範囲（両端を含む）。"""
    dlat = half_m / 111_000.0
    dlon = half_m / (111_000.0 * math.cos(math.radians(lat)))
    x0, y0 = lonlat_to_global_px(lat + dlat, lon - dlon, z)
    x1, y1 = lonlat_to_global_px(lat - dlat, lon + dlon, z)
    return int(x0 // 256), int(y0 // 256), int(x1 // 256), int(y1 // 256)


def _curl(url, out, sleep):
    """curl で 1 枚取る。既にあれば取らない（再実行で負荷をかけない）。
    404 は「その場所・ズームのタイルが無い」なので失敗にせず記録する。"""
    if out.exists() and out.stat().st_size > 0:
        return 'cached'
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(out.suffix + '.part')
    r = subprocess.run(['curl', '-s', '-A', UA, '-o', str(tmp), '-w', '%{http_code}', url],
                       capture_output=True, text=True)
    time.sleep(sleep)  # 地理院タイルの利用規約に回数の上限は無いが、連続取得で負荷をかけない
    code = r.stdout.strip()
    if code == '200':
        tmp.rename(out)
        return 'ok'
    tmp.unlink(missing_ok=True)
    return f'http {code}'


def cmd_fetch(a):
    raw = Path(a.raw_dir).expanduser()
    log = {'center': [a.lat, a.lon], 'half_m': a.half_m, 'tiles': {}}
    for kind, url, z, ext in (('ortho', ORTHO_URL, a.ortho_zoom, 'jpg'),
                              ('bvmap', BVMAP_URL, a.vector_zoom, 'pbf')):
        x0, y0, x1, y1 = tile_range(a.lat, a.lon, a.half_m, z)
        stat = {}
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                s = _curl(url.format(z=z, x=x, y=y), raw / kind / str(z) / f'{x}_{y}.{ext}', a.sleep)
                stat[s] = stat.get(s, 0) + 1
        log['tiles'][kind] = {'zoom': z, 'x': [x0, x1], 'y': [y0, y1], 'status': stat}
        print(kind, z, (x0, y0, x1, y1), stat)
    # 取得日時は「最後に実際に通信した日時」。全部が手元にあった再実行では前回の値を残す
    # （タイルの版を後で辿れるようにするため）
    now = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    prev = raw / 'fetch_log.json'
    downloaded = any(v.get('ok', 0) for v in (t['status'] for t in log['tiles'].values()))
    if not downloaded and prev.exists():
        log['fetched_at'] = json.loads(prev.read_text()).get('fetched_at', now)
    else:
        log['fetched_at'] = now
    log['checked_at'] = now
    prev.write_text(json.dumps(log, ensure_ascii=False, indent=1))
    return 0


# --- 読み込み ------------------------------------------------------------------------
def load_grid(layers_yaml):
    doc = yaml.safe_load(Path(layers_yaml).read_text())
    g = doc['grid']
    return doc, int(g['width']), int(g['height']), float(g['resolution']), \
        float(g['origin'][0]), float(g['origin'][1])


def load_alpha(path):
    return np.array(Image.open(path).getchannel('A')) > 0


def load_white_mask(path):
    """compose_occupancy.load_sidecar_mask と同じ解釈（白・不透明が塗り）。"""
    im = Image.open(path)
    m = np.array(im.convert('L')) >= 128
    if im.mode == 'RGBA':
        m &= np.array(im.getchannel('A')) > 0
    return m


def load_ortho_mosaic(raw, z):
    """raw/ortho/z/*.jpg を 1 枚につなぐ。返り値は (RGB 配列, 左上の全体画素 x, y)。"""
    files = sorted((raw / 'ortho' / str(z)).glob('*.jpg'))
    if not files:
        raise SystemExit(f'{raw}/ortho/{z} にタイルが無い。先に fetch を実行する')
    xy = [tuple(int(v) for v in f.stem.split('_')) for f in files]
    xs = [p[0] for p in xy]
    ys = [p[1] for p in xy]
    x0, y0 = min(xs), min(ys)
    img = np.zeros(((max(ys) - y0 + 1) * 256, (max(xs) - x0 + 1) * 256, 3), np.uint8)
    for f, (x, y) in zip(files, xy):
        img[(y - y0) * 256:(y - y0 + 1) * 256, (x - x0) * 256:(x - x0 + 1) * 256] = \
            np.array(Image.open(f).convert('RGB'))
    return img, x0 * 256, y0 * 256


def load_vector(raw, z):
    """raw/bvmap/z/*.pbf の road・building を平面直角座標（E, N）の点列にして返す。
    タイル境界で切れた線・面はつながないまま返す（ラスタ化するので困らない）。"""
    files = sorted((raw / 'bvmap' / str(z)).glob('*.pbf'))
    if not files:
        raise SystemExit(f'{raw}/bvmap/{z} にタイルが無い。先に fetch を実行する')
    out = {'road': [], 'building': []}
    for f in files:
        tx, ty = (int(v) for v in f.stem.split('_'))
        L = mvt_decode(f.read_bytes())
        for lname in out:
            if lname not in L:
                continue
            ext = L[lname]['extent']
            for feat in L[lname]['features']:
                for part in feat['parts']:
                    p = np.array(part, float)
                    gx = (tx + p[:, 0] / ext) * 256.0
                    gy = (ty + p[:, 1] / ext) * 256.0
                    lat, lon = global_px_to_lonlat(gx, gy, z)
                    E, N = tm_forward(lat, lon)
                    out[lname].append({'type': feat['type'], 'props': feat['props'],
                                       'EN': np.stack([E, N], 1)})
    return out


# GSI の道路の地物コード（地理院地図Vector データ仕様、ズームレベル 16・17 の表）。
# 22xx は道路縁（道路の両側の線）、27xx は道路中心線。z16 のタイルに両方入っている。
ROAD_EDGE_CODES = {2201: '道路縁 通常部', 2203: '道路縁 通常部 橋・高架', 2204: '道路縁 通常部',
                   2241: '道路縁 通常部', 2243: '道路縁 通常部 橋・高架', 2244: '道路縁 通常部',
                   2221: '道路縁 庭園路', 2251: '道路縁 庭園路', 2224: '道路縁 庭園路',
                   2223: '道路縁 庭園路 橋・高架'}
ROAD_CENTER_CODES = {2701: '道路中心線 通常部', 2711: '道路中心線 庭園路', 2714: '道路中心線 庭園路 トンネル',
                     2721: '道路中心線 徒歩道'}


# --- ラスタ化 ------------------------------------------------------------------------
def rasterize_EN(features, T_inv, W, H, res, ox, oy, width=1, fill=False, scale=1):
    """平面直角座標の線・面を map の格子（1/scale に縮めた格子）に描く。
    T_inv は (E, N) → map の 3x3。"""
    w, h = W // scale, H // scale
    im = Image.new('L', (w, h), 0)
    d = ImageDraw.Draw(im)
    r = res * scale
    top = oy + H * res  # downsample_any は下と右の端数を捨てるので、上端を基準にする
    for ft in features:
        x, y = apply(T_inv, ft['EN'][:, 0], ft['EN'][:, 1])
        col = (x - ox) / r - 0.5
        row = (top - y) / r - 0.5
        pts = list(zip(col.tolist(), row.tolist()))
        if len(pts) < 2:
            continue
        if fill and ft['type'] == 3 and len(pts) >= 3:
            d.polygon(pts, fill=255)
        else:
            d.line(pts, fill=255, width=width)
    return np.array(im) > 0


def map_cell_xy(W, H, res, ox, oy, scale=1):
    """縮めた格子の各セルの中心の map 座標（2 次元配列 x, y）。上端を基準にする
    （downsample_any と同じく、下と右の端数を捨てる）。"""
    w, h = W // scale, H // scale
    r = res * scale
    x = ox + (np.arange(w) + 0.5) * r
    y = oy + H * res - (np.arange(h) + 0.5) * r
    return np.meshgrid(x, y)


def map_to_px(x, y, H, res, ox, oy, scale=1):
    """map 座標 → 縮めた格子の (col, row)（実数）。"""
    r = res * scale
    return (np.asarray(x) - ox) / r - 0.5, (oy + H * res - np.asarray(y)) / r - 0.5


def downsample_any(m, k):
    h, w = m.shape[0] // k * k, m.shape[1] // k * k
    return m[:h, :w].reshape(h // k, k, w // k, k).any(axis=(1, 3))


def downsample_mean(m, k):
    h, w = m.shape[0] // k * k, m.shape[1] // k * k
    return m[:h, :w].reshape(h // k, k, w // k, k).mean(axis=(1, 3))


# --- 位置合わせ ------------------------------------------------------------------------
# 向きの決め方（#205 の試行で分かったこと）：
# 構造物層（地面から 0.1〜2.2 m の点）には壁のほかに木・生垣・車・柵が多い。
# 「構造物の点 → GSI の輪郭」の距離で評価すると、GSI の輪郭が密な住宅地に地図を
# 置いたほうが点数が上がり、正しい置き方（キャンパス）より住宅地の誤った置き方が勝った
# （yaw 177° の誤答。内側の点の割合 0.32 対 正答 0.23）。
# 逆向き「GSI の輪郭 → 構造物」を、LiDAR が見た範囲（軌跡から cover_m 以内）に限って
# 測ると、正しい置き方では輪郭の 84% が 0.75 m 以内に構造物を持ち、誤答では 57〜61%。
# そこで粗い探索も精密化も、この逆向きの量で行う。分母（見た範囲にある GSI の輪郭の量）
# が置き方で変わるので、粗い探索は「近い輪郭の量 / 見た範囲の輪郭の量」の比で比べる。


def _sample_rings(features, step):
    """GSI の面の輪郭を step [m] おきの点にする。返り値は (点 (N,2), 建物番号 (N,))。"""
    pts, ids = [], []
    for i, ft in enumerate(features):
        P = ft['EN']
        for a, b in zip(P[:-1], P[1:]):
            L = float(np.hypot(*(b - a)))
            m = max(1, int(math.ceil(L / step)))
            t = (np.arange(m) / m)[:, None]
            pts.append(a + t * (b - a))
            ids.append(np.full(m, i))
    return np.concatenate(pts), np.concatenate(ids)


class MapField:
    """map の格子上の量（構造物までの距離、軌跡までの距離など）を map 座標で引く。"""

    def __init__(self, arr, H, res, ox, oy, scale, outside):
        self.a = arr
        self.H, self.res, self.ox, self.oy, self.k = H, res, ox, oy, scale
        self.outside = outside

    def __call__(self, x, y, order=1):
        from scipy import ndimage
        c, r = map_to_px(x, y, self.H, self.res, self.ox, self.oy, self.k)
        return ndimage.map_coordinates(self.a, [r, c], order=order, mode='constant', cval=self.outside)


def coarse_search(gsi_edge, S_fn, C_fn, grid0, gres, yaws, t_range, min_cells):
    """yaw ごとに平行移動を FFT の相関で探す。

    gsi_edge：平面直角座標の格子（左下 grid0、間隔 gres、行 = 北向き）の GSI の輪郭。
    S_fn(x, y)：map 座標での「構造物に近い」スコア（0〜1）。C_fn(x, y)：見た範囲なら 1。
    格子の添字 q（循環。fftfreq の順）を、平行移動 0 のときの平面直角座標の差 q*gres と
    みなし、map 座標 R(-yaw) q*gres でスコアを引いた画像を作る。
    num(t) = Σ_q edge(q + t) S(q)、den(t) = Σ_q edge(q + t) C(q) は相関なので FFT で一度に出る。
    t は格子の中心（初期値）から ±t_range に限る（循環の折り返しを拾わないため）。
    返り値は yaw ごとの (比, 近い輪郭のセル数, tE, tN)。"""
    n = gsi_edge.shape[0]
    F = np.fft.rfft2(gsi_edge.astype(float))
    q = np.fft.fftfreq(n, 1.0 / n) * gres
    QE, QN = np.meshgrid(q, q)  # 列 = 東、行 = 北
    c0 = n // 2
    m = int(t_range / gres)
    out = []
    for yaw in yaws:
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        x = c * QE + s * QN  # R(-yaw)
        y = -s * QE + c * QN
        S = S_fn(x, y)
        C = C_fn(x, y)
        num = np.fft.irfft2(F * np.conj(np.fft.rfft2(S)), s=(n, n))[c0 - m:c0 + m + 1, c0 - m:c0 + m + 1]
        den = np.fft.irfft2(F * np.conj(np.fft.rfft2(C)), s=(n, n))[c0 - m:c0 + m + 1, c0 - m:c0 + m + 1]
        ratio = np.where(den > min_cells, num / np.maximum(den, 1e-9), 0.0)
        ky, kx = np.unravel_index(np.argmax(ratio), ratio.shape)
        out.append((float(ratio[ky, kx]), float(num[ky, kx]),
                    grid0[0] + (kx + c0 - m) * gres, grid0[1] + (ky + c0 - m) * gres))
    return out


def refine(gsi_pts, dist_fn, cover_fn, T0, trunc, cover_m, float_scale=False):
    """GSI の輪郭の点（見た範囲のものだけ）から構造物までの距離の打ち切り誤差を最小にする。
    見た範囲は T0 で一度だけ決める（最適化の途中で点の集合が変わると目的関数が不連続になる）。
    打ち切りは、GSI にあって LiDAR の死角になった壁・その後に壊された建物などを外すため。"""
    from scipy.optimize import least_squares
    Tinv = np.linalg.inv(T0)
    x, y = apply(Tinv, gsi_pts[:, 0], gsi_pts[:, 1])
    sel = cover_fn(x, y) < cover_m
    P = gsi_pts[sel]

    def resid(p):
        T = make_transform(p[0], p[1], p[2], p[3] if float_scale else 1.0)
        xx, yy = apply(np.linalg.inv(T), P[:, 0], P[:, 1])
        return np.minimum(dist_fn(xx, yy), trunc)

    yaw0 = math.degrees(math.atan2(T0[1, 0], T0[0, 0]))
    p0 = [yaw0, T0[0, 2], T0[1, 2]] + ([1.0] if float_scale else [])
    r = least_squares(resid, p0, x_scale=[0.05, 0.2, 0.2] + ([1e-3] if float_scale else []),
                      diff_step=[1e-5, 1e-6, 1e-6] + ([1e-6] if float_scale else []),
                      loss='soft_l1', f_scale=trunc / 2)
    p = r.x
    T = make_transform(p[0], p[1], p[2], p[3] if float_scale else 1.0)
    return T, p, int(sel.sum())


def edge_residuals(gsi_pts, dist_fn, cover_fn, T, cover_m):
    """見た範囲の GSI の輪郭の点から構造物までの距離の統計。"""
    x, y = apply(np.linalg.inv(T), gsi_pts[:, 0], gsi_pts[:, 1])
    sel = cover_fn(x, y) < cover_m
    d = dist_fn(x[sel], y[sel])
    inl = d < 1.5
    return {
        'n_points': int(sel.sum()),
        'median_m': float(np.median(d)),
        'frac_within_0p75m': float((d < 0.75).mean()),
        'frac_within_1p5m': float(inl.mean()),
        'inlier_median_m': float(np.median(d[inl])),
        'inlier_p90_m': float(np.percentile(d[inl], 90)),
    }


def traj_mask(traj_path, H, W, res, ox, oy, scale):
    """軌跡（TUM 形式：t x y z qx qy qz qw）が通ったセル。"""
    tr = np.loadtxt(traj_path, usecols=(1, 2))
    h, w = H // scale, W // scale
    c, r = map_to_px(tr[:, 0], tr[:, 1], H, res, ox, oy, scale)
    c = np.round(c).astype(int)
    r = np.round(r).astype(int)
    ok = (c >= 0) & (r >= 0) & (c < w) & (r < h)
    m = np.zeros((h, w), bool)
    m[r[ok], c[ok]] = True
    return m


def cmd_align(a):
    from scipy import ndimage
    t_start = time.time()
    raw = Path(a.raw_dir).expanduser()
    out_dir = Path(a.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    _, W, H, res, ox, oy = load_grid(a.layers_yaml)
    struct = load_alpha(Path(a.layers_yaml).parent / 'occupied_structure.png')

    vec = load_vector(raw, a.vector_zoom)
    blds = [f for f in vec['building'] if f['type'] == 3]
    if not blds:
        raise SystemExit('GSI の建物（面）が無い')
    E0, N0 = (float(v) for v in tm_forward(a.init_lat, a.init_lon))

    # map 側の場：粗い探索用（0.5 m）と精密化用（原寸 0.05 m）
    kc = max(1, int(round(a.coarse_res / res)))
    sq = downsample_any(struct, kc)
    dq = ndimage.distance_transform_edt(~sq) * res * kc
    tm_q = traj_mask(a.traj, H, W, res, ox, oy, kc)
    dtraj_q = ndimage.distance_transform_edt(~tm_q) * res * kc
    cov = (dtraj_q < a.cover_m).astype(float)
    # 分子も見た範囲に限る（比が 0〜1 の「見た範囲の輪郭のうち構造物に近い割合」になる）
    S_q = MapField(np.exp(-0.5 * (dq / a.sigma) ** 2) * cov, H, res, ox, oy, kc, 0.0)
    C_q = MapField(cov, H, res, ox, oy, kc, 0.0)
    cover_fn = MapField(dtraj_q.astype(np.float32), H, res, ox, oy, kc, 1e3)
    dist_full = (ndimage.distance_transform_edt(~struct) * res).astype(np.float32)
    dist_fn = MapField(dist_full, H, res, ox, oy, 1, 1e3)

    # GSI の輪郭を、初期値を中心とする平面直角座標の格子に描く
    half = math.hypot(W * res, H * res) / 2 + a.search_m + 20
    gres = a.coarse_res
    n = 2 * int(half / gres)
    grid0 = (E0 - (n // 2) * gres, N0 - (n // 2) * gres)
    im = Image.new('L', (n, n), 0)
    d = ImageDraw.Draw(im)
    for ft in blds:
        d.line(list(zip(((ft['EN'][:, 0] - grid0[0]) / gres).tolist(),
                        ((ft['EN'][:, 1] - grid0[1]) / gres).tolist())), fill=255, width=1)
    gsi_edge = np.array(im) > 0

    yaws = np.arange(-180.0, 180.0, a.yaw_step)
    min_cells = a.min_edge_m / gres
    cs = coarse_search(gsi_edge, S_q, C_q, grid0, gres, yaws, a.search_m, min_cells)
    ratio = np.array([c[0] for c in cs])
    order = np.argsort(-ratio)
    cands = []
    for i in order:
        if all(abs(((yaws[i] - yaws[j]) + 180) % 360 - 180) > 10 for j in cands):
            cands.append(i)
        if len(cands) >= a.n_candidates:
            break
    print('粗い探索', f'{time.time() - t_start:.0f} s', [(float(yaws[i]), round(ratio[i], 3)) for i in cands])

    gsi_pts, gsi_ids = _sample_rings(blds, a.edge_step)
    results = []
    for rank, i in enumerate(cands):
        _, _, tE, tN = cs[i]
        T = make_transform(yaws[i], tE, tN)
        for tr in (3.0, 1.5, 0.75):
            T, p, nsel = refine(gsi_pts, dist_fn, cover_fn, T, tr, a.cover_m)
        st = edge_residuals(gsi_pts, dist_fn, cover_fn, T, a.cover_m)
        results.append({'rank': rank, 'coarse_yaw_deg': float(yaws[i]), 'coarse_ratio': float(ratio[i]),
                        'yaw_deg': float(p[0]), 'tE': float(p[1]), 'tN': float(p[2]), **st})
        print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in results[-1].items()})

    best = max(results, key=lambda r: r['frac_within_0p75m'])
    second = sorted(results, key=lambda r: -r['frac_within_0p75m'])[1] if len(results) > 1 else None
    T = make_transform(best['yaw_deg'], best['tE'], best['tN'])
    _, ps, _ = refine(gsi_pts, dist_fn, cover_fn, T, 0.75, a.cover_m, float_scale=True)

    # 感度：最良からずらしたときの残差。構造物層には木などが多く、でたらめに置いても輪郭の
    # 近くに何かがある（偶然の水準）。山の鋭さで、残差の数字がどれだけ置き方を区別するかを見る
    sens = {}
    for name, dp in (('dE', (0, 1, 0)), ('dN', (0, 0, 1)), ('dyaw_deg', (1, 0, 0))):
        prof = []
        for v in ((-2, -1, -0.5, 0, 0.5, 1, 2) if name != 'dyaw_deg' else (-1, -0.5, -0.2, 0, 0.2, 0.5, 1)):
            Tp = make_transform(best['yaw_deg'] + dp[0] * v, best['tE'] + dp[1] * v, best['tN'] + dp[2] * v)
            st = edge_residuals(gsi_pts, dist_fn, cover_fn, Tp, a.cover_m)
            prof.append([v, round(st['median_m'], 3), round(st['frac_within_0p75m'], 3)])
        sens[name] = prof

    # 不確かさ：建物を単位に復元抽出して推定し直した散らばり。点を単位にすると、同じ壁の
    # 点どうしの相関を無視して小さく出るので、建物を単位にする
    rng = np.random.default_rng(0)
    boot = []
    nb = len(blds)
    for _ in range(a.bootstrap):
        pick = rng.integers(0, nb, nb)
        cnt = np.bincount(pick, minlength=nb)
        w = cnt[gsi_ids]
        P = np.repeat(gsi_pts, w, axis=0)
        _, pb, _ = refine(P, dist_fn, cover_fn, T, 0.75, a.cover_m)
        boot.append(pb[:3])
    boot = np.array(boot)
    xy0 = np.array([0.0, 0.0])
    boot_origin = np.array([apply(make_transform(*b), xy0[0], xy0[1]) for b in boot]) if len(boot) else None

    E00, N00 = (float(v) for v in apply(T, 0.0, 0.0))
    lat00, lon00 = (float(v) for v in tm_inverse(E00, N00))
    gamma = meridian_convergence_deg(lat00, lon00)
    summary = {
        'method': '粗い探索（yaw 全周 × FFT 相関、GSI の輪郭→構造物の近さの比）→ 精密化（打ち切り 3/1.5/0.75 m の'
                  ' 最小二乗、soft_l1）。評価は軌跡から cover_m 以内の GSI の輪郭だけ',
        'candidates': results,
        'best': best,
        'second': second,
        'scale_if_free': float(ps[3]),
        'sensitivity_[offset,median_m,frac_within_0p75m]': sens,
        'origin_EN': [E00, N00],
        'origin_wgs84': [lat00, lon00],
        'init_wgs84': [a.init_lat, a.init_lon],
        'origin_minus_init_m': {'dE': E00 - E0, 'dN': N00 - N0, 'dist': math.hypot(E00 - E0, N00 - N0)},
        'meridian_convergence_deg': gamma,
        'bootstrap': {
            'n': int(len(boot)),
            'yaw_deg_std': float(boot[:, 0].std()) if len(boot) else None,
            'origin_E_std_m': float(boot_origin[:, 0].std()) if len(boot) else None,
            'origin_N_std_m': float(boot_origin[:, 1].std()) if len(boot) else None,
        },
        'gsi_buildings': nb,
        'elapsed_s': round(time.time() - t_start, 1),
        'params': {k: v for k, v in vars(a).items() if k != 'func'},
    }
    Path(a.summary_json).write_text(json.dumps(summary, ensure_ascii=False, indent=1))
    write_georef(a.georef_yaml, summary, a)
    keys = ('best', 'scale_if_free', 'origin_wgs84', 'origin_minus_init_m', 'bootstrap')
    print(json.dumps({k: summary[k] for k in keys}, ensure_ascii=False, indent=1))

    # 候補ごとの重ね絵（1/8、灰 = 構造物、赤 = GSI の建物）。曖昧なら目で比べるため全部を横に並べる
    tiles = []
    s8 = downsample_any(struct, 8)
    for r in results:
        Tc = make_transform(r['yaw_deg'], r['tE'], r['tN'])
        g = rasterize_EN(blds, np.linalg.inv(Tc), W, H, res, ox, oy, width=1, scale=8)
        rgb = np.full(g.shape + (3,), 255, np.uint8)
        rgb[s8[:g.shape[0], :g.shape[1]]] = (150, 150, 150)
        rgb[g] = (220, 0, 0)
        tl = Image.fromarray(rgb)
        # PIL の既定のフォントは日本語を描けないので、図の中の文字は英数字だけにする
        ImageDraw.Draw(tl).text((6, 6), f"#{r['rank']} yaw {r['yaw_deg']:.1f}  <0.75m {r['frac_within_0p75m']:.2f}",
                                fill=(0, 0, 255))
        tiles.append(tl)
    sheet = Image.new('RGB', (sum(t.width for t in tiles), tiles[0].height), 'white')
    xo = 0
    for t in tiles:
        sheet.paste(t, (xo, 0))
        xo += t.width
    sheet.save(out_dir / 'align_candidates_e.png', optimize=True)
    return 0


def write_georef(path, s, a):
    """georef.yaml（map → EPSG:6677 / WGS84）。"""
    b = s['best']
    lat, lon = s['origin_wgs84']
    gamma = s['meridian_convergence_deg']
    T = make_transform(b['yaw_deg'], b['tE'], b['tN'])
    # map の +x 軸の向き：座標北から時計回りの角度 = 90 - yaw。真北からは子午線収差 γ を足す
    # （第 IX 系の原点より東では座標北が真北より東に γ 傾くので、真北基準の方位は γ だけ大きい）
    hdg_grid = (90.0 - b['yaw_deg']) % 360
    doc = {
        'description': 'campus-m8 の map 座標を国土地理院の建物（電子国土基本図、地理院ベクトル z16）に'
                       '合わせた剛体変換（#205）。GNSS による測量ではない。docs/maps/campus-m8/reference-gsi.md',
        'map_grid': str(a.layers_yaml),
        'target_crs': 'EPSG:6677（JGD2011 平面直角座標系 第IX系）。EPSG の軸順は X=北, Y=東 なので、'
                      'ここでは easting / northing と書く（easting = EPSG の Y、northing = X）',
        'transform_map_to_epsg6677': {
            'model': 'easting = cos(yaw)*x - sin(yaw)*y + tE ; northing = sin(yaw)*x + cos(yaw)*y + tN（縮尺 1 に固定）',
            'yaw_deg': round(b['yaw_deg'], 4),
            'tE_m': round(b['tE'], 3),
            'tN_m': round(b['tN'], 3),
            'matrix_3x3': [[round(v, 9) if abs(v) < 10 else round(v, 4) for v in row] for row in T.tolist()],
            'scale_if_free': round(s['scale_if_free'], 6),
        },
        'map_origin': {
            'easting_m': round(s['origin_EN'][0], 3), 'northing_m': round(s['origin_EN'][1], 3),
            'lat_deg': round(lat, 8), 'lon_deg': round(lon, 8),
            'note': '緯度経度は JGD2011。WGS84 との差は数 cm〜数十 cm 程度で、この変換の不確かさより小さい',
        },
        'map_x_axis_heading_deg': {
            'from_grid_north_cw': round(hdg_grid, 3),
            'from_true_north_cw': round((hdg_grid + gamma) % 360, 3),
            'meridian_convergence_deg': round(gamma, 4),
        },
        'residual': {
            'what': '軌跡から ' + str(a.cover_m) + ' m 以内にある GSI の建物の輪郭（0.1 m おきの点）から、'
                    '構造物層（occupied_structure.png）の最も近いセルまでの距離',
            'median_m': round(b['median_m'], 3),
            'frac_within_0p75m': round(b['frac_within_0p75m'], 3),
            'frac_within_1p5m': round(b['frac_within_1p5m'], 3),
            'inlier_median_m': round(b['inlier_median_m'], 3),
            'inlier_p90_m': round(b['inlier_p90_m'], 3),
            'n_points': b['n_points'],
        },
        'uncertainty_1sigma_bootstrap_by_building': {
            'n': s['bootstrap']['n'],
            'yaw_deg': None if s['bootstrap']['yaw_deg_std'] is None else round(s['bootstrap']['yaw_deg_std'], 4),
            'origin_easting_m': None if s['bootstrap']['origin_E_std_m'] is None else round(s['bootstrap']['origin_E_std_m'], 3),
            'origin_northing_m': None if s['bootstrap']['origin_N_std_m'] is None else round(s['bootstrap']['origin_N_std_m'], 3),
            'note': 'GSI の建物の位置そのものの誤差（地図情報レベル 2500：水平位置の標準偏差 1.75 m 以内）は'
                    'ブートストラップに入らない。建物ごとに独立なら平均で小さくなるが、図化の系統誤差は残りうる',
        },
        'ambiguity': None if s['second'] is None else {
            'second_yaw_deg': round(s['second']['yaw_deg'], 2),
            'second_frac_within_0p75m': round(s['second']['frac_within_0p75m'], 3),
            'best_frac_within_0p75m': round(b['frac_within_0p75m'], 3),
        },
        'sanity_check': {
            'init_wgs84_user_google_maps': s['init_wgs84'],
            'origin_minus_init_m': round(s['origin_minus_init_m']['dist'], 2),
        },
        'method': s['method'],
        'source': '国土地理院 地理院ベクトル（提供実験）experimental_bvmap z16 の building。'
                  '出典：国土地理院ウェブサイト（https://maps.gsi.go.jp/development/vt_expt.html）を加工して作成',
    }
    Path(path).write_text('# 自動生成：scripts/map_georef_gsi.py align。手で直さない\n'
                          + yaml.safe_dump(doc, allow_unicode=True, sort_keys=False))


def load_georef(path):
    g = yaml.safe_load(Path(path).read_text())['transform_map_to_epsg6677']
    return make_transform(g['yaw_deg'], g['tE_m'], g['tN_m'])


# --- 外部の参照との比較 ---------------------------------------------------------------
def warp_ortho(T, X, Y, mosaic, gx0, gy0, z):
    """map 座標の格子 (X, Y) に、地理院の写真を双線形で写す。範囲外は灰。"""
    from scipy import ndimage
    lat, lon = map_to_lonlat(T, X, Y)
    px, py = lonlat_to_global_px(lat, lon, z)
    c = px - gx0 - 0.5
    r = py - gy0 - 0.5
    out = np.empty(X.shape + (3,), np.uint8)
    for ch in range(3):
        out[..., ch] = np.clip(ndimage.map_coordinates(mosaic[..., ch].astype(np.float32), [r, c], order=1,
                                                       mode='constant', cval=128), 0, 255)
    return out


def gsi_road_area(edges, centers, res_m, gap_m, max_half_width_m):
    """道路縁（線）と道路中心線から道路の面を作る。

    電子国土基本図の道路は、地図情報レベル 2500 では両側の道路縁（線）で描かれ、面は無い。
    中心線から、道路縁を越えずに広がる領域を道路とする（種 = GSI の中心線、壁 = GSI の道路縁）。
    道路縁の小さな切れ目は gap_m の膨張で閉じ、それでも切れ目から街区・構内の区画へ漏れる
    のを、中心線から max_half_width_m までに限って止める。最後に gap_m の帯を道路側に戻す。
    軌跡・点群など自分たちのデータは一切使わない（独立な参照にするため）。

    最初の試み（道路縁の補集合の連結成分を、最大内接半径で道路と街区に分ける）は、道路網が
    地図の端まで一つにつながり、端の広い所で半径の条件を外れて道路網ごと落ちたので止めた。"""
    from scipy import ndimage
    gp = max(1, int(round(gap_m / res_m)))
    ed = ndimage.binary_dilation(edges, iterations=gp)
    dc, (ir, ic) = ndimage.distance_transform_edt(~centers, return_indices=True)
    dc = dc * res_m
    cand = ~ed & (dc <= max_half_width_m)
    # 領域成長（中心線から道路縁を越えずに広がる）にすると、構内の区画の輪郭の小さな切れ目
    # から区画の中へ漏れた（#205 の 1 回目の試行）。そこで「最も近い中心線の点までの線分が
    # 道路縁を横切らない」セルだけを道路にする。漏れは切れ目を通る直線の上だけに限られる。
    rr, cc = np.nonzero(cand)
    tr, tc = ir[rr, cc], ic[rr, cc]
    ok = np.ones(rr.size, bool)
    nstep = int(math.ceil(max_half_width_m / res_m)) + 1
    for s in range(1, nstep):
        f = s / nstep
        pr = np.round(rr + (tr - rr) * f).astype(int)
        pc = np.round(cc + (tc - cc) * f).astype(int)
        ok &= ~ed[pr, pc]
    road = np.zeros_like(cand)
    road[rr[ok], cc[ok]] = True
    road |= centers & ~ed
    road = road | (ndimage.binary_dilation(road, iterations=gp + 1) & ed)
    return road, {'center_cells': int(centers.sum()), 'rejected_behind_edge_cells': int((~ok).sum())}


def overlap(a, b, region=None):
    """a（評価するマスク）と b（参照）の IoU・precision・recall。"""
    if region is not None:
        a = a & region
        b = b & region
    inter = int((a & b).sum())
    ua = int(a.sum())
    ub = int(b.sum())
    union = int((a | b).sum())
    return {'mask_cells': ua, 'ref_cells': ub, 'both_cells': inter,
            'iou': round(inter / union, 4) if union else None,
            'precision': round(inter / ua, 4) if ua else None,
            'recall': round(inter / ub, 4) if ub else None}


def box_mask(shape, H, res, ox, oy, k, box):
    X, Y = map_cell_xy(shape[1] * k, H, res, ox, oy, k)
    X, Y = X[:shape[0], :shape[1]], Y[:shape[0], :shape[1]]
    x0, x1, y0, y1 = box
    return (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)


def draw_waypoints(img, wps, H, res, ox, oy, k, r=6, color=(255, 0, 255), label=True):
    d = ImageDraw.Draw(img)
    for w in wps:
        c, rr = map_to_px(w['x'], w['y'], H, res, ox, oy, k)
        c, rr = float(c), float(rr)
        d.ellipse([c - r, rr - r, c + r, rr + r], outline=color, width=max(2, r // 3))
        if label:
            d.text((c + r + 2, rr - r - 2), w['name'], fill=color)


def save_png(img, path, limit_bytes):
    """PNG を書く。limit_bytes を超えたら 256 色に減色して書き直す（git に置く図の上限のため）。
    減色したかどうかは返り値で記録し、文書に書く。"""
    img.save(path, optimize=True)
    if Path(path).stat().st_size <= limit_bytes:
        return 'rgb'
    img.convert('RGB').quantize(256, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE).save(path, optimize=True)
    return 'quantized_256'


def cmd_report(a):
    from scipy import ndimage
    t_start = time.time()
    raw = Path(a.raw_dir).expanduser()
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    _, W, H, res, ox, oy = load_grid(a.layers_yaml)
    L = Path(a.layers_yaml).parent
    struct = load_alpha(L / 'occupied_structure.png')
    T = load_georef(a.georef_yaml)
    Tinv = np.linalg.inv(T)
    wps = yaml.safe_load(Path(a.waypoints).read_text())['waypoints']
    mosaic, gx0, gy0 = load_ortho_mosaic(raw, a.ortho_zoom)
    vec = load_vector(raw, a.vector_zoom)
    roads = vec['road']
    edges_f = [f for f in roads if int(f['props'].get('ftCode', 0)) // 100 == 22]
    cent_f = [f for f in roads if int(f['props'].get('ftCode', 0)) // 100 == 27]
    blds = [f for f in vec['building'] if f['type'] == 3]
    codes = {}
    for f in roads:
        c = int(f['props'].get('ftCode', 0))
        codes[c] = codes.get(c, 0) + 1

    masks = {
        'auto_202': load_white_mask(a.auto_mask),
        'hand_candidate': load_white_mask(a.hand_mask),
        'hand_merged': load_white_mask(a.hand_merged_mask),
    }
    log = {'png_mode': {}}

    # 評価の格子は 0.1 m（k=2）。GSI の線の位置の誤差（m 単位）に比べて十分細かい
    k = a.metric_scale
    rm = res * k
    e_m = rasterize_EN(edges_f, Tinv, W, H, res, ox, oy, width=1, scale=k)
    c_m = rasterize_EN(cent_f, Tinv, W, H, res, ox, oy, width=1, scale=k)
    road, rinfo = gsi_road_area(e_m, c_m, rm, a.gap_m, a.max_half_width_m)
    mk = {n: downsample_mean(m, k) >= 0.5 for n, m in masks.items()}
    tm = traj_mask(a.traj, H, W, res, ox, oy, k)
    dtraj = ndimage.distance_transform_edt(~tm) * rm
    near = dtraj <= a.near_m
    boxes = {'start_road': (-10, 70, -15, 15), 'seimon': (-100, -75, -20, 5), 'north_road': (-125, 75, 145, 175)}
    metrics = {'gsi_road_area': {'cells': int(road.sum()), 'area_m2': round(float(road.sum()) * rm * rm, 1),
                                 'cells_near_traj': int((road & near).sum()), **rinfo},
               'ftCode_counts': {str(k_): v for k_, v in sorted(codes.items())}}
    for name, m in mk.items():
        e = {'whole': overlap(m, road), f'near_traj_{a.near_m:g}m': overlap(m, road, near)}
        for bn, bx in boxes.items():
            e[bn] = overlap(m, road, box_mask(m.shape, H, res, ox, oy, k, bx))
        # GSI の中心線のうち、マスクの中にある割合（長さはセル数で近似）
        cl = c_m & near
        e['centerline_share_near_traj'] = round(float((m & cl).sum()) / max(1, int(cl.sum())), 4)
        e['centerline_cells_near_traj'] = int(cl.sum())
        # マスクの縁から最も近い GSI の道路縁までの距離（5 m 以内の縁だけ。縁の位置の合い方）
        b = m & ~ndimage.binary_erosion(m)
        de = ndimage.distance_transform_edt(~e_m) * rm
        dd = de[b & near]
        dd = dd[dd <= 5.0]
        e['edge_to_gsi_edge_m'] = {'n': int(dd.size), 'median': round(float(np.median(dd)), 3) if dd.size else None,
                                   'p90': round(float(np.percentile(dd, 90)), 3) if dd.size else None}
        metrics[name] = e
        print(name, json.dumps(e, ensure_ascii=False))

    # (a) 写真 + 構造物層 + 4 地点（1/4）
    q = 4
    Xq, Yq = map_cell_xy(W, H, res, ox, oy, q)
    ortho_q = warp_ortho(T, Xq, Yq, mosaic, gx0, gy0, a.ortho_zoom)
    s_q = downsample_any(struct, q)[:ortho_q.shape[0], :ortho_q.shape[1]]
    img = ortho_q.copy()
    img[s_q] = (255, 140, 0)
    pil = Image.fromarray(img)
    draw_waypoints(pil, wps, H, res, ox, oy, q, r=8)
    log['png_mode']['ortho_struct_q.png'] = save_png(pil, out / 'ortho_struct_q.png', a.png_limit)

    # (b) GSI の道路（縁 = 水色、中心線 = 黄、道路の面 = 薄い緑）とマスクの比較（1/4）
    eq = rasterize_EN(edges_f, Tinv, W, H, res, ox, oy, width=2, scale=q)
    cq = rasterize_EN(cent_f, Tinv, W, H, res, ox, oy, width=2, scale=q)
    road_q = downsample_mean(road, q // k) >= 0.5
    hq, wq = eq.shape
    road_q = road_q[:hq, :wq]
    for name, label in (('auto_202', 'auto'), ('hand_candidate', 'hand')):
        m = downsample_mean(masks[name], q) >= 0.5
        m = m[:hq, :wq]
        rgb = np.full((hq, wq, 3), 255, np.uint8)
        rgb[road_q] = (205, 235, 205)          # GSI の道路の面だけ
        rgb[m & ~road_q] = (240, 120, 120)     # マスクだけ（GSI では道路でない）
        rgb[m & road_q] = (120, 120, 120)      # 両方
        rgb[s_q[:hq, :wq]] = (0, 0, 0)
        rgb[eq] = (0, 160, 220)
        rgb[cq] = (230, 190, 0)
        pil = Image.fromarray(rgb)
        draw_waypoints(pil, wps, H, res, ox, oy, q, r=8)
        fn = f'gsi_roads_vs_{label}_q.png'
        log['png_mode'][fn] = save_png(pil, out / fn, a.png_limit)

    # (c) 発進点〜bldg3 の原寸（x -10〜70、y -15〜15）。写真の上に構造物・GSI・マスクの縁
    x0, x1, y0, y1 = a.crop
    c0, r0 = (int(round(v)) for v in map_to_px(x0, y1, H, res, ox, oy, 1))
    c1, r1 = (int(round(v)) for v in map_to_px(x1, y0, H, res, ox, oy, 1))
    Xf, Yf = map_cell_xy(W, H, res, ox, oy, 1)
    Xc, Yc = Xf[r0:r1 + 1, c0:c1 + 1], Yf[r0:r1 + 1, c0:c1 + 1]
    del Xf, Yf
    crop = warp_ortho(T, Xc, Yc, mosaic, gx0, gy0, a.ortho_zoom)
    sl = (slice(r0, r1 + 1), slice(c0, c1 + 1))
    st = struct[sl]
    crop[st] = (255, 140, 0)
    for name, col in (('hand_candidate', (0, 90, 255)), ('auto_202', (0, 220, 0))):
        m = masks[name][sl]
        crop[m & ~ndimage.binary_erosion(m, iterations=2)] = col
    pil = Image.fromarray(crop)
    d = ImageDraw.Draw(pil)
    for feats, col in ((edges_f, (0, 230, 255)), (cent_f, (255, 230, 0)), (blds, (255, 0, 0))):
        for ft in feats:
            x, y = apply(Tinv, ft['EN'][:, 0], ft['EN'][:, 1])
            cc, rr = map_to_px(x, y, H, res, ox, oy, 1)
            d.line(list(zip((cc - c0).tolist(), (rr - r0).tolist())), fill=col, width=3)
    # 切り出しの左上 (c0, r0) を原点にずらした格子として描く（map_to_px は上端基準なので oy を r0 行ぶん下げる）
    draw_waypoints(pil, wps, H, res, ox + c0 * res, oy - r0 * res, 1, r=12)
    log['png_mode']['start_crop_1to1.png'] = save_png(pil, out / 'start_crop_1to1.png', a.png_limit)

    metrics['elapsed_s'] = round(time.time() - t_start, 1)
    metrics['png'] = log['png_mode']
    metrics['params'] = {k_: v for k_, v in vars(a).items() if k_ != 'func'}
    Path(a.metrics_json).write_text(json.dumps(metrics, ensure_ascii=False, indent=1))
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    D = 'docs/maps/campus-m8'
    X = f'{D}/v2/experiments/205'
    RAW = '~/bags/m8-experiments/205/raw'

    f = sub.add_parser('fetch', help='地理院タイルを取得する（既にあるタイルは取らない）')
    # 中心の既定は、国土地理院の地名検索「宇都宮大学(工学部)」（36.55133, 139.92981）と
    # 利用者が Google マップで読んだ発進点（36.550694, 139.928528）のあいだ。
    f.add_argument('--lat', type=float, default=36.5505)
    f.add_argument('--lon', type=float, default=139.9295)
    f.add_argument('--half-m', type=float, default=350.0)
    f.add_argument('--ortho-zoom', type=int, default=18)  # 全国最新写真の上限は z18（z19 は 404）
    f.add_argument('--vector-zoom', type=int, default=16)  # 地理院ベクトルの上限は z16（z17 は 404）
    f.add_argument('--sleep', type=float, default=0.2)
    f.add_argument('--raw-dir', default=RAW)
    f.set_defaults(func=cmd_fetch)

    al = sub.add_parser('align', help='構造物層を GSI の建物に合わせ、georef.yaml を書く')
    al.add_argument('--layers-yaml', default=f'{D}/v2/v2_layers.yaml')
    al.add_argument('--traj', default=f'{D}/traj_lidar.txt')
    al.add_argument('--raw-dir', default=RAW)
    al.add_argument('--vector-zoom', type=int, default=16)
    # 初期値：利用者が Google マップで読んだ発進点（±10〜20 m の見込み）。平行移動の探索の中心と、
    # 結果の確かめにだけ使う。向き（yaw）は全周を探すので初期値を持たない
    al.add_argument('--init-lat', type=float, default=36.550694)
    al.add_argument('--init-lon', type=float, default=139.928528)
    al.add_argument('--search-m', type=float, default=60.0, help='平行移動の探索幅（初期値から ±）')
    al.add_argument('--coarse-res', type=float, default=0.5)
    al.add_argument('--sigma', type=float, default=1.0, help='粗い探索の「近さ」のガウスの幅 [m]')
    al.add_argument('--yaw-step', type=float, default=1.0)
    al.add_argument('--cover-m', type=float, default=25.0,
                    help='LiDAR が見たとみなす軌跡からの距離。これより遠い GSI の輪郭は評価に入れない')
    al.add_argument('--min-edge-m', type=float, default=500.0,
                    help='粗い探索で、見た範囲にある GSI の輪郭がこの長さ未満の置き方は採らない')
    al.add_argument('--edge-step', type=float, default=0.1)
    al.add_argument('--n-candidates', type=int, default=4)
    al.add_argument('--bootstrap', type=int, default=20)
    al.add_argument('--out-dir', default=X)
    al.add_argument('--summary-json', default=f'{X}/align_summary.json')
    al.add_argument('--georef-yaml', default=f'{D}/georef.yaml')
    al.set_defaults(func=cmd_align)

    rp = sub.add_parser('report', help='重ね絵と、GSI の道路の面に対する数値を作る')
    rp.add_argument('--layers-yaml', default=f'{D}/v2/v2_layers.yaml')
    rp.add_argument('--traj', default=f'{D}/traj_lidar.txt')
    rp.add_argument('--waypoints', default=f'{D}/waypoints.yaml')
    rp.add_argument('--georef-yaml', default=f'{D}/georef.yaml')
    rp.add_argument('--raw-dir', default=RAW)
    rp.add_argument('--ortho-zoom', type=int, default=18)
    rp.add_argument('--vector-zoom', type=int, default=16)
    rp.add_argument('--auto-mask', default=f'{D}/v2/experiments/202/roadway_auto.png')
    rp.add_argument('--hand-mask', default=f'{D}/v2/candidates/candidate_roadway_mask.png')
    # 2026-11-19 に candidate_roadway_merged.png は 154w で広げた版になったので、手塗りのままを残した
    # *_hand を既定にする（#205 の数字を出した時の入力と同じ物）。
    rp.add_argument('--hand-merged-mask', default=f'{D}/v2/candidates/candidate_roadway_merged_hand.png')
    rp.add_argument('--metric-scale', type=int, default=2, help='評価の格子（原寸の何倍か）。2 で 0.1 m')
    rp.add_argument('--gap-m', type=float, default=0.3, help='道路縁の切れ目を閉じる膨張 [m]')
    rp.add_argument('--max-half-width-m', type=float, default=10.0,
                    help='中心線から道路の面を広げる上限 [m]。GSI の道路縁は歩道を含む幅なので、'
                         '幅 20 m（片側 2 車線＋両側の歩道）の道路まで入る値')
    rp.add_argument('--near-m', type=float, default=8.0, help='#202 と同じ「軌跡の近く」の幅')
    rp.add_argument('--crop', type=float, nargs=4, default=[-10.0, 70.0, -15.0, 15.0], metavar=('X0', 'X1', 'Y0', 'Y1'))
    rp.add_argument('--png-limit', type=int, default=3_000_000)
    rp.add_argument('--out-dir', default=X)
    rp.add_argument('--metrics-json', default=f'{X}/metrics.json')
    rp.set_defaults(func=cmd_report)

    a = p.parse_args(argv)
    return a.func(a)


if __name__ == '__main__':
    sys.exit(main())
