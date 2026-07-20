#!/usr/bin/env bash
# m7_make_tls_cert.sh — 母艦 AP 用の自己署名 TLS 証明書を生成する。
#
# Why: iOS Safari の HTTPS-First で iPad が http://<母艦>:8000 を https に
# 格上げしてしまい、平文 http.server が TLS ハンドシェイクを 400 で弾く
# (2026-07-20 field で確定)。web UI (8000) と rosbridge (9090) を HTTPS/WSS で
# 出せば iPad の HTTPS-First と折り合う。ローカル IP には正規 CA 証明書が
# 取れない (Let's Encrypt はローカル IP 不可) ため自己署名にする。iPad には
# この証明書を 1 回だけ手動で信頼させる (README の手順)。
#
# SAN に母艦 AP の IP を入れるのが要点。CN だけの証明書は iOS が拒否する。
# AP の IP は既定 10.42.0.1 (nmcli hotspot の既定サブネット)。別 IP なら
# 第 1 引数で渡す: m7_make_tls_cert.sh 10.42.0.5
#
# 冪等: 既存の証明書があっても上書き再生成する (有効期限切れ対策)。

set -euo pipefail

IP="${1:-10.42.0.1}"
OUT_DIR="${2:-$HOME/.whill_dispatch_tls}"
DAYS=825   # iOS は 825 日超の証明書を拒否する

mkdir -p "$OUT_DIR"
CERT="$OUT_DIR/dispatch.crt"
KEY="$OUT_DIR/dispatch.key"

# SAN に IP と localhost を入れる。iOS は SAN 必須 (CN だけでは拒否)。
openssl req -x509 -newkey rsa:2048 -sha256 -days "$DAYS" -nodes \
  -keyout "$KEY" -out "$CERT" \
  -subj "/CN=whill-dispatch" \
  -addext "subjectAltName=IP:${IP},IP:127.0.0.1,DNS:localhost"

chmod 600 "$KEY"
echo "wrote $CERT"
echo "wrote $KEY"
echo "SAN: IP:${IP} (これを iPad の Safari で開く宛先にすること)"
echo ""
echo "次: この crt を iPad に信頼させる (src/whill_dispatch/README.md の HTTPS 節)"
echo "  1) iPad を whill-demo に接続後、Safari で https://${IP}:8000/dispatch.crt を開く"
echo "  2) iPad: 設定 → プロファイルがダウンロードされました → インストール"
echo "  3) iPad: 設定 → 一般 → 情報 → 証明書信頼設定 → whill-dispatch を全面的に信頼"
