# スクリプト一覧

`scripts/` の索引。起動順序は [runbook](runbook-live-demo.md)、ビルドは [開発環境](development.md)。

引数・依存・出力先は各スクリプトの冒頭を読む。引数なしで実行して使い方を調べない（セットアップや実機動作が始まるものがある）。

## セットアップ

| ファイル | 用途 |
| --- | --- |
| `install_ros2_humble.sh` | ROS 2環境の導入 |
| `setup_workspace.sh` | workspace準備・rosdep・bashrc追記。既定workspaceは `~/ros2_ws` |
| `import_upstream.sh` | `whill_lab.repos` の取得、Patchwork++除外設定、rosdep |
| `install_udev_rules.sh` | `/dev/whill` / `/dev/imu` のudev規則 |
| `grant_serial_access.sh` | serialアクセス権の設定 |
| `install_velodyne_network.sh` | 有線NICのnetplan設定。引数はNIC名 |
| `configure_proxy.sh` | proxy関連の設定 |
| `install_cuda.sh` | CUDA環境の導入 |
| `install_glim.sh` | GLIMの導入 |
| `install_gril_calib.sh` | GRIL-Calibの導入 |
| `install_nvidia_suspend_fix.sh` | NVIDIA suspend/resume対策 |

OS・権限・ネットワークを変えるものがある。中身を確認してから実行する。構築済みの PC では不要。

## 実機・配車・検証

| ファイル | 用途・注意 |
| --- | --- |
| `start_dispatch_demo.sh` | 実機bringup・Nav2・TLS配車Web・RVizの統合起動。--dry-run / --preflight対応 |
| `m6r_preflight.sh` | check 0〜5の走行前検査。check 4のrosout判定に制約あり |
| `m6r_smoke_test.sh` | bag再生でlocalizer等を検証する入口 |
| `m6r_record_calib_bag.sh` | 校正用bagの記録 |
| `m7_make_tls_cert.sh` | 配車Webの証明書作成。接続先IPを指定 |
| `m7_https_server.py` | dispatch launchが使うHTTPS配信 |
| `m7_make_web_map.py` | YAMLの画像からWeb背景とmetaを生成 |
| `mock_navigate_to_pose.py` | 模擬Nav2 action。実機bringupを再現しない |
| `m7_ws_smoke.py` | rosbridge経由の投入・取消を含むE2E検証。実機での受動診断には使わない |
| `m4r3_ekf_bench.py` | bagによるEKFの解析 |
| `check_static.py` | IMU bagの静止区間の解析 |

## SLAM・動的除去・校正

| ファイル | 用途 |
| --- | --- |
| `m5r3_fix_imu_bag.py` | オフライン用IMU bag補正 |
| `m5r3_run_glim.sh` | GLIMのオフライン実行 |
| `m5r3_export_merged_ply.py` | 結合点群の出力 |
| `m5r_glim_to_pcd.py` | GLIM結果のPCD化 |
| `m5r_run_dufomap.sh`, `m5r_run_dufomap_core.py` | DUFOMap処理 |
| `m5r_dufomap_diff.py` | 動的除去の差分解析 |
| `m5r3_loop_error.py`, `m5r3_b1_numeric.py` | 地図のループ・高さずれ解析 |
| `m5r4_run_gril_calib.sh` | LiDAR / IMU校正 |

## 占有格子・v2マスク

| ファイル | 用途 |
| --- | --- |
| `pcd_to_occupancy_grid.py`, `m5r_pcd_to_occupancy.py` | PCDから占有格子を生成 |
| `pcd_to_occupancy_v2.py` | v2用レイヤ・占有格子生成 |
| `clean_isolated_occupancy.py` | 孤立セルの除去 |
| `init_v2_sidecars.py` | v2マスクの初期化 |
| `gen_auto_free_mask.py` | 自動free mask生成 |
| `compose_occupancy.py` | レイヤとマスクの合成 |
| `pipeline_v2_verify.py` | v2成果物の検証 |
| `audit_free_leaks.py` | free領域の漏れ解析 |
| `pcd_to_traversability.py` | traversability生成 |
| `tests/test_step_accessible.py` | 段差処理のテスト |
| `tests/test_dispatch_demo_launcher.py` | `start_dispatch_demo.sh` の引数と終了処理のテスト（ROS・車体は起動しない） |

地図生成は成果物を書き換える。現用地図を再生成する前に入力・出力先・mask を確認する。
