#!/usr/bin/env python3
"""TLS 対応の静的ファイルサーバ — dispatch web UI を HTTPS で配信する。

stdlib の `python3 -m http.server` は TLS を話せず、iOS Safari の
HTTPS-First が投げてくる TLS ClientHello を 400 で弾いてしまう
(2026-07-20 field 実測)。このサーバは同じ静的配信を https で行うだけの
薄いラッパ。証明書は scripts/m7_make_tls_cert.sh が生成したものを使う。

証明書配布のため、cert ファイル (dispatch.crt) を配信ディレクトリに
--cert-serve-dir で見せると、iPad が https://<host>:<port>/dispatch.crt で
取得して信頼インストールできる (README の手順)。

使い方:
    m7_https_server.py --port 8000 --dir <web_dir> \
        --cert ~/.whill_dispatch_tls/dispatch.crt \
        --key  ~/.whill_dispatch_tls/dispatch.key
"""

import argparse
import functools
import http.server
import os
import shutil
import ssl
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--dir', required=True, help='static web dir to serve')
    ap.add_argument('--cert', required=True)
    ap.add_argument('--key', required=True)
    ap.add_argument('--bind', default='0.0.0.0')
    args = ap.parse_args()

    for p in (args.cert, args.key):
        if not os.path.isfile(p):
            sys.exit(f'm7_https_server: missing {p}. '
                     f'Run scripts/m7_make_tls_cert.sh first.')

    # cert を配信ディレクトリからも取れるようにコピー (iPad の信頼インストール
    # 用)。install の web/ 共有ディレクトリに置く。鍵 (.key) は絶対に置かない。
    try:
        dst = os.path.join(args.dir, 'dispatch.crt')
        if os.path.abspath(args.cert) != os.path.abspath(dst):
            shutil.copyfile(args.cert, dst)
    except OSError as e:
        print(f'warn: could not stage cert into web dir: {e}', flush=True)

    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler, directory=args.dir)
    httpd = http.server.ThreadingHTTPServer((args.bind, args.port), handler)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=args.cert, keyfile=args.key)
    httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)

    print(f'HTTPS static server on https://{args.bind}:{args.port} '
          f'serving {args.dir} (cert at /dispatch.crt)', flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
