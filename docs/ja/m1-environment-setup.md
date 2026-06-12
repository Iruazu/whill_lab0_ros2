# M1 — ROS 2 humble 環境構築

Language: [日本語](m1-environment-setup.md) | [English](../en/m1-environment-setup.md)

## ゴール

ラボ PC に動作する ROS 2 humble インストールを用意し、`~/ros2_ws` に空の colcon ワークスペースを切る。以降のマイルストーンがパッケージをビルドできる状態にする。

## ホスト

| | |
|--|--|
| OS | Ubuntu 22.04.5 LTS (jammy) |
| カーネル | 6.8.0-111-generic |
| ネットワーク | `proxy.cc.utsunomiya-u.ac.jp:8080` (宇都宮大学) 配下 |
| GitHub アクセス | HTTP CONNECT 経由で `ssh.github.com:443` にトンネル |

## 手順

本リポの `scripts/` ディレクトリで自動化済み。クリーンな PC で再現するには:

```bash
# 1. (任意) キャンパスプロキシ向けに apt + SSH を設定
./scripts/configure_proxy.sh

# 2. ROS 2 humble Desktop + dev tools をインストール
./scripts/install_ros2_humble.sh

# 3. ~/ros2_ws を作成し ~/.bashrc に source を追加
./scripts/setup_workspace.sh
```

各スクリプトは冪等で、認証情報を埋め込まない。

### インストールスクリプトの中身

1. ホストが jammy (Ubuntu 22.04) であることを確認する。
2. `locales` パッケージを入れて `en_US.UTF-8` を生成する。
3. apt の `universe` コンポーネントを有効化する。
4. `ros-infrastructure/ros-apt-source` から最新の `ros2-apt-source` deb (現行 1.2.0) をダウンロードしてインストールする。これは 2024 年以降の公式手順で、旧来の `ros-archive-keyring.gpg` 方式は非推奨になった。
5. `apt-get install ros-humble-desktop ros-dev-tools`。

### プロキシに関する注意

- このネットワークから GitHub への直接 SSH (port 22 や 443) はブロックされる。
- `~/.ssh/config` で `Host github.com` を `ProxyCommand nc -X connect -x proxy.cc.utsunomiya-u.ac.jp:8080 %h %p` 経由・`ssh.github.com:443` 宛に設定している。`git clone git@github.com:…` は最終的にこの経路を通る。
- `/etc/apt/apt.conf.d/95proxies` で apt 自身も同じプロキシを使うようにしてある。
- シェルレベルの `HTTP_PROXY` / `HTTPS_PROXY` 環境変数は本 PC では全ユーザー共通で設定済み。`sudo` 内でも保ちたい場合は `sudo -E` を使う。

## 検証

ワークスペースを切ったあと:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
# 別のシェルで:
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

talker が送る "I heard: …" メッセージが listener に表示されればよい。

## 成果物

- `/opt/ros/humble/` — ROS 2 humble インストール
- `~/ros2_ws/src/` — 空の colcon ソース空間。M2 以降の受け皿
- `~/.bashrc` — `humble` とワークスペースオーバーレイを自動 source

## ステータス

| ステップ | ステータス |
|---------|-----------|
| Apt プロキシ設定済 | 完了 |
| GitHub への SSH-via-proxy が通る | 完了 |
| `ros-humble-desktop` インストール済 | 完了 (`/opt/ros/humble/setup.bash` 確認) |
| `ros2 talker/listener` 往復 | 完了 (動作確認済) |
| `~/ros2_ws` 初期化済 | 完了 |
| `rosdep` 初期化・キャッシュ生成済 | 完了 |
| `.bashrc` への source 追加 | 完了 |

`ros2 pkg list` は検証済インストールで 273 パッケージを報告。
