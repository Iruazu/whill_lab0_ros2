# このPCにある bag の棚卸し（2026-11-27）

- 目的: VLA / VA 系研究の実験基盤として、手元の bag で「カメラ画像・自己位置・速度指令・TF が揃っているか」「手動と自律を分けられるか」を確かめ、次の記録（`scripts/record_avoidance_bag.sh`）がこの環境でそのまま動くかを確かめる。
- 日付: 2026-11-27（調査はすべて読み取りのみ。bag・設定・コードは変更していない。記録・ノード起動・トピックの publish はしていない）
- ホスト: WHILL CR2 実機スタックの PC。GPU は NVIDIA GeForce RTX 3080 Ti Laptop GPU（VRAM 16384 MiB）。ネットワークの情報は載せない。
- 画像について: 画像フレームは一切保存・抽出していない。解像度は 1 通目の `CompressedImage.data` の先頭バイト（JPEG の SOF マーカ）をメモリ上で読んだだけ。

## 要点

1. **bag は 88 本、合計 274G（`du -shc`）**。rosbag2 が 88 本（sqlite3 67 本・mcap 21 本）、すべて `metadata.yaml` あり。`ros2 bag info` は 88 本すべてで成功した。ほかに ROS 1 の `*.bag` が 3 本あるが、apt の `robot_localization` のテスト用データで、この研究の記録ではない。
   - 重複: `~/whill_platform/bags/2026-07-31-campus`（mcap）は `~/bags/m7-bench-data/2026-07-31-live-campus/...15_14_51`（sqlite3）の変換で、トピックと件数が一致する（同 README にも変換手順がある）。ほかに開始時刻がナノ秒まで一致する組が 5 組ある（`-hires`・`-imu-fixed`・`seg-A`・`first10` などの派生）。
   - リポジトリ内の `docs/m6r-bench-data/2026-07-12-acceptance-campus/bag` は git 管理の 24.9 KiB（`/tf_static` 1 通だけ）。main と 13 個の worktree に同じ中身（md5 一致）が 14 部あるが、1 本と数えた。
   - `~/bags/2026-11-14-static/` は空のディレクトリ。
2. **`/camera/camera/color/image_raw/compressed` があるのは 9 本だけ**（2026-11-16 の static 8 本と smoke 1 本）。どれも JPEG 640×480、約 30 Hz（平均 29.72〜30.05 Hz）、長さ 4.8〜11.9 s の静止確認。走行中の bag に圧縮カラーは 1 本も無い。生の `/camera/camera/color/image_raw`（非圧縮）は 2026-10-28 の sensor_check 11 本と m3-bench-data（2026-05）8 本の計 19 本にある（いずれも自己位置・cmd_vel は無い）。
3. **自己位置（`/pcl_pose`）・cmd_vel 系・`/tf` が同じ bag に揃うのは 15 本**（2026-07-22〜07-31 の m7 実走 14 本＋その mcap 変換 1 本。派生を除くと 13 記録）。**その 15 本にカメラ画像は無い**。カメラのある 9 本には自己位置も cmd_vel も無い。つまり「画像＋自己位置＋指令＋TF」が揃った bag は現時点で 0 本。
4. **手動と自律の区別**: twist_mux の入力（`/cmd_vel_nav`＝Nav2、`/cmd_vel_teleop`＝iPad の救出操縦、`/cmd_vel_safety`＝failsafe）が別に記録された 2026-07-22 以降の 14 本は、どの入力が出ていたかを時刻で分けられる。2026-07-20〜21 の 12 本は mux 出力の `/cmd_vel` だけだが、`/dispatch/state` の `phase`（ACTIVE＝Nav2 のゴール実行中）と、07-21 18:29 の 1 本だけ `teleop_active` で区間を分けられる。twist_mux の選択結果そのもの、ジョイスティック、Nav2 の action status はどの bag にも無い。車いす本体のジョイスティックでの手動走行は ROS のトピックに出ないため、cmd_vel 系の無い 61 本は手動か自律かを bag から判別できない。
5. **`record_avoidance_bag.sh` は今の環境ではそのままでは通らない**。RMW・DDS XML の指定・governor・プラグイン・保存先は合格。止まるのは (a) `check_dds_nic.py`（XML の NIC が今のPCに無く、LiDAR のサブネットを持つ NIC も無い。LiDAR の USB-LAN が未接続のため）と、(b) 入力トピックの確認（ロボットのノードが動いていない）。どちらも現地で m6r_bringup を起動し LiDAR をつなげば解消する見込みの前提条件で、スクリプトの不具合ではない。注意点として、`/velodyne_packets` の型 `velodyne_msgs` はワークスペースの `install/setup.bash` を source しないと見つからない。空き容量の確認はスクリプトに無い（今の空きは 561G）。
6. 空き容量: `/`（home と bag 置き場を含め全部同じパーティション）は 993G 中 561G 空き（使用 41%）。GPU は RTX 3080 Ti Laptop、16384 MiB、調査時の使用 375 MiB。

## 判別できなかったこと

- cmd_vel 系の無い 61 本（m3/m5r/m6r/m8 static/sensor_check など）が、手動で動かしたものか静止なのか自律なのか。bag に指令系のトピックが無く、`/whill/odom` があるのも 1 本（m6r 2026-07-14）だけ。
- twist_mux が各時刻にどの入力を選んだか（選択のトピックは記録されていない）。優先度（safety 100 > teleop 50 > navigation 10、timeout 0.5 s）と各入力の時刻から推定はできるが、本調査では推定値を出していない。
- 派生 bag（`-hires` など）の作り方の詳細。開始時刻と件数の一致だけを根拠に「同じ記録から派生」とした。`-hires` については `docs/lidar-yaw-measurement.md` に「#152 の方法で点群を Hi-Res に補正したもの」とある。
- `record_avoidance_bag.sh` の入力確認（`ros2 topic echo --once`）と記録そのもの。ノード起動と記録を禁じられているため実行していない。

## 1. このPCにある bag の一覧

### 実行したコマンドと出力

```bash
find / -xdev \( -path /proc -o -path /sys \) -prune -o \
  \( -name metadata.yaml -o -name "*.db3" -o -name "*.mcap" -o -name "*.bag" \) -print 2>/dev/null \
  | grep -v -E '/\.git/|/build/|/install/' | sort > found.txt
findmnt -rn -o TARGET,FSTYPE | grep -v -E 'proc|sys|cgroup|tmpfs|devpts|mqueue|squashfs|fuse|bpf|debugfs|tracefs|hugetlb|pstore|securityfs|configfs|autofs|efivarfs|binfmt'
ls -la /media/systemlab; ls /mnt; ls -la ~/bags/2026-11-14-static
```

マウント（`findmnt` の出力、ext4 のルート以外に bag を置けるマウントは無い）:

```
/ ext4
/boot/efi vfat
/run/snapd/ns/snapd-desktop-integration.mnt nsfs
```

```
/media/systemlab: 空（drwxr-x---+ root, 2026-07-22）
/mnt: 空
~/bags/2026-11-14-static: 空のディレクトリ（. と .. のみ）
```

`find` の生の出力（299 行、worktree 内の同一ファイルを含む）は付録 C。

```bash
# found.txt をディレクトリ単位にまとめ、.claude/worktrees/ と /opt を除いて
# metadata.yaml の先頭に rosbag2_bagfile_information があるもの、または *.mcap/*.db3 を含むものを bag とした
# （docs/maps/*/metadata.yaml は地図のメタデータなので除外される）→ bags.txt（88 行、NOMETA 0 件）
source /opt/ros/humble/setup.bash; source ~/whill_lab0_ros2/install/setup.bash
while read d; do echo "### $d"; echo "$ ros2 bag info $d"; ros2 bag info "$d"; echo "rc=$?"; done < bags.txt > info_raw.txt 2>&1
while read d; do du -sh "$d"; done < bags.txt > du_raw.txt
du -shc $(cat bags.txt) | tail -1
grep -c 'rc=0' info_raw.txt
# 重複の確認: データファイルのハードリンク、リポジトリ内 bag の worktree 間の一致
find $(cat bags.txt) -type f \( -name '*.db3' -o -name '*.mcap' \) -printf '%i %n %s %p\n' | awk '$2>1'
md5sum ~/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3 \
       ~/whill_lab0_ros2/.claude/worktrees/*/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3 | awk '{print $1}' | sort | uniq -c
du -sh /opt/ros/humble/share/robot_localization/test/*.bag
```

```
274G	合計
88
（ハードリンクされたデータファイル: 出力なし）
     14 1eb3c72dba48540078227d5b8fc713c4
1.5M	/opt/ros/humble/share/robot_localization/test/test1.bag
524K	/opt/ros/humble/share/robot_localization/test/test2.bag
25M	/opt/ros/humble/share/robot_localization/test/test3.bag
```

各 bag の `du -sh` の生の出力は付録 B、`ros2 bag info` の生の出力（88 本分、1764 行）は付録 A。

### 一覧（`ros2 bag info` の Start と Duration、`du -sh` から作成）

| # | bag（パス） | 形式 | 開始（JST） | 長さ [s] | du -sh | メッセージ数 |
|---|---|---|---|---:|---:|---:|
| 1 | `~/bags/2026-11-16-static/cone_1p5_141326` | mcap | 2026-11-16 14:13:26 | 11.8 | 386M | 4374 |
| 2 | `~/bags/2026-11-16-static/cone_2p5_141549` | mcap | 2026-11-16 14:15:49 | 11.8 | 390M | 4394 |
| 3 | `~/bags/2026-11-16-static/cone_2p_141506` | mcap | 2026-11-16 14:15:06 | 11.9 | 387M | 4365 |
| 4 | `~/bags/2026-11-16-static/cone_3p5_141752` | mcap | 2026-11-16 14:17:53 | 11.8 | 390M | 4397 |
| 5 | `~/bags/2026-11-16-static/cone_3p_141709` | mcap | 2026-11-16 14:17:09 | 11.8 | 386M | 4359 |
| 6 | `~/bags/2026-11-16-static/cone_4p_141838` | mcap | 2026-11-16 14:18:39 | 11.8 | 390M | 4387 |
| 7 | `~/bags/2026-11-16-static/cone_5p_141921` | mcap | 2026-11-16 14:19:21 | 11.8 | 390M | 4392 |
| 8 | `~/bags/2026-11-16-static/empty_start_140915` | mcap | 2026-11-16 14:09:16 | 11.8 | 390M | 4395 |
| 9 | `~/bags/2026-11-16-test/smoke_140741` | mcap | 2026-11-16 14:07:41 | 4.8 | 161M | 1812 |
| 10 | `~/bags/imu_investigation_20261030/seg_0_300_hires` | sqlite3 | 2026-07-10 15:07:43 | 300.0 | 1.8G | 32961 |
| 11 | `~/bags/imu_investigation_20261030/seg_0_300_orig` | sqlite3 | 2026-07-10 15:07:43 | 300.0 | 1.8G | 32961 |
| 12 | `~/bags/m3-bench-data/m3_chair_motion_2026-05-07` | sqlite3 | 2026-05-07 22:16:53 | 96.9 | 5.3G | 26137 |
| 13 | `~/bags/m3-bench-data/m3_chair_static_2026-05-07` | sqlite3 | 2026-05-07 22:07:41 | 19.9 | 1.1G | 5362 |
| 14 | `~/bags/m3-bench-data/m3_smoke_2026-05-07` | sqlite3 | 2026-05-07 21:38:57 | 7.8 | 437M | 2115 |
| 15 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08` | sqlite3 | 2026-05-08 10:53:52 | 59.8 | 3.3G | 16246 |
| 16 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run1` | sqlite3 | 2026-05-08 11:10:14 | 59.5 | 3.3G | 16240 |
| 17 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run2` | sqlite3 | 2026-05-08 11:12:15 | 59.5 | 3.3G | 16212 |
| 18 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run3` | sqlite3 | 2026-05-08 11:14:52 | 59.8 | 3.3G | 16172 |
| 19 | `~/bags/m3-bench-data/m5_chair_mapping_2026-05-08` | sqlite3 | 2026-05-08 12:11:45 | 119.8 | 6.6G | 32724 |
| 20 | `~/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag` | sqlite3 | 2026-06-21 13:38:02 | 199.3 | 1.2G | 27879 |
| 21 | `~/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag-imu-fixed` | sqlite3 | 2026-06-21 13:38:02 | 199.3 | 1.2G | 27879 |
| 22 | `~/bags/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag` | sqlite3 | 2026-06-24 13:49:42 | 239.4 | 1.5G | 26310 |
| 23 | `~/bags/m5r-bench-data/2026-07-07-campus-half-v3/bag` | sqlite3 | 2026-07-07 19:30:40 | 466.3 | 2.8G | 51220 |
| 24 | `~/bags/m5r-bench-data/2026-07-07-campus-loop/bag` | sqlite3 | 2026-07-07 19:08:32 | 821.5 | 4.8G | 88669 |
| 25 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-postgap/bag` | sqlite3 | 2026-07-08 16:33:02 | 1817.7 | 11G | 187770 |
| 26 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-pregap/bag` | sqlite3 | 2026-07-08 16:00:22 | 1864.5 | 12G | 204829 |
| 27 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2-first10/bag` | sqlite3 | 2026-07-08 18:52:24 | 599.6 | 3.6G | 65883 |
| 28 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2-pregap/bag` | sqlite3 | 2026-07-08 18:52:24 | 2602.1 | 16G | 284313 |
| 29 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2/bag` | sqlite3 | 2026-07-08 18:52:24 | 3482.5 | 20G | 366318 |
| 30 | `~/bags/m5r-bench-data/2026-07-08-campus-demo/bag` | sqlite3 | 2026-07-08 16:00:21 | 3778.4 | 22G | 393165 |
| 31 | `~/bags/m5r-bench-data/2026-07-08-campus-outer-debiased/bag` | sqlite3 | 2026-07-08 21:05:22 | 2813.3 | 17G | 308557 |
| 32 | `~/bags/m5r-bench-data/2026-07-08-campus-outer-seg-A/bag` | sqlite3 | 2026-07-08 21:05:22 | 1325.1 | 7.9G | 145573 |
| 33 | `~/bags/m5r-bench-data/2026-07-08-campus-outer-seg-B/bag` | sqlite3 | 2026-07-08 21:27:32 | 1483.5 | 8.9G | 162987 |
| 34 | `~/bags/m5r-bench-data/2026-07-08-campus-outer/bag` | sqlite3 | 2026-07-08 21:05:22 | 2813.3 | 17G | 308557 |
| 35 | `~/bags/m5r-bench-data/2026-07-08-indoor-calib/bag` | sqlite3 | 2026-07-08 10:32:15 | 365.8 | 2.2G | 40196 |
| 36 | `~/bags/m5r-bench-data/2026-07-08-outdoor-corrected/bag` | sqlite3 | 2026-07-08 12:22:55 | 745.4 | 4.5G | 81892 |
| 37 | `~/bags/m5r-bench-data/2026-07-09-indoor-calib/bag` | sqlite3 | 2026-07-09 16:26:54 | 209.1 | 1.3G | 22980 |
| 38 | `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag` | sqlite3 | 2026-07-10 15:07:43 | 2162.0 | 13G | 237506 |
| 39 | `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires` | sqlite3 | 2026-07-10 15:07:43 | 2162.0 | 13G | 237506 |
| 40 | `~/bags/m5r-bench-data/2026-07-10-dds-verify/bag` | sqlite3 | 2026-07-10 14:47:26 | 303.7 | 1.9G | 33366 |
| 41 | `~/bags/m6r-bench-data/2026-07-14-verify-campus/bag` | sqlite3 | 2026-07-14 13:55:36 | 627.7 | 3.8G | 95179 |
| 42 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41` | sqlite3 | 2026-07-20 15:13:41 | 783.8 | 4.7G | 127062 |
| 43 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43` | sqlite3 | 2026-07-20 15:29:44 | 257.6 | 1.6G | 40555 |
| 44 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08` | sqlite3 | 2026-07-20 15:44:08 | 270.9 | 1.7G | 39877 |
| 45 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27` | sqlite3 | 2026-07-20 15:59:27 | 632.6 | 3.8G | 107887 |
| 46 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07` | sqlite3 | 2026-07-20 17:03:07 | 462.9 | 2.8G | 79613 |
| 47 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28` | sqlite3 | 2026-07-20 18:09:28 | 543.0 | 3.3G | 93025 |
| 48 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48` | sqlite3 | 2026-07-20 19:58:48 | 275.5 | 1.7G | 47662 |
| 49 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01` | sqlite3 | 2026-07-21 16:34:01 | 255.8 | 1.6G | 42856 |
| 50 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48` | sqlite3 | 2026-07-21 16:50:48 | 119.2 | 735M | 20920 |
| 51 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31` | sqlite3 | 2026-07-21 16:53:32 | 506.1 | 3.1G | 85029 |
| 52 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34` | sqlite3 | 2026-07-21 17:20:34 | 595.1 | 3.7G | 99958 |
| 53 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14` | sqlite3 | 2026-07-21 18:29:14 | 337.4 | 2.1G | 56566 |
| 54 | `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15` | sqlite3 | 2026-07-22 18:01:15 | 132.5 | 876M | 31035 |
| 55 | `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13` | sqlite3 | 2026-07-22 18:23:13 | 174.4 | 1.1G | 40309 |
| 56 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43` | sqlite3 | 2026-07-24 15:21:43 | 121.0 | 44M | 10982 |
| 57 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46` | sqlite3 | 2026-07-24 15:23:46 | 63.1 | 44M | 9140 |
| 58 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10` | sqlite3 | 2026-07-24 15:25:10 | 62.0 | 188M | 12460 |
| 59 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44` | sqlite3 | 2026-07-24 15:27:45 | 36.4 | 102M | 7528 |
| 60 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02` | sqlite3 | 2026-07-24 15:32:02 | 121.6 | 546M | 27275 |
| 61 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36` | sqlite3 | 2026-07-24 16:13:36 | 277.2 | 1.5G | 64131 |
| 62 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38` | sqlite3 | 2026-07-24 16:19:38 | 38.5 | 93M | 8069 |
| 63 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23` | sqlite3 | 2026-07-24 17:42:23 | 37.5 | 120M | 6616 |
| 64 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27` | sqlite3 | 2026-07-24 18:11:27 | 497.7 | 2.7G | 103253 |
| 65 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26` | sqlite3 | 2026-07-24 19:05:26 | 842.9 | 5.3G | 199422 |
| 66 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires` | sqlite3 | 2026-07-24 19:05:26 | 842.9 | 5.3G | 199422 |
| 67 | `~/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51` | sqlite3 | 2026-07-31 15:14:51 | 235.1 | 1.5G | 54669 |
| 68 | `~/bags/m8-static-check-20261113/fixed_1` | sqlite3 | 2026-11-13 15:31:06 | 11.7 | 73M | 2467 |
| 69 | `~/bags/m8-static-check-20261113/fixed_2_place2` | sqlite3 | 2026-11-13 15:34:23 | 11.7 | 72M | 2468 |
| 70 | `~/bags/m8-static-check-20261113/fixed_3_place2_rot180` | sqlite3 | 2026-11-13 15:36:08 | 11.8 | 73M | 2467 |
| 71 | `~/bags/m8-static-check-20261113/fixed_4_place2_rot180_after_drive` | sqlite3 | 2026-11-13 15:40:58 | 11.7 | 73M | 2467 |
| 72 | `~/bags/m8-static-check-20261113/fixed_5_place3_rot180` | sqlite3 | 2026-11-13 15:43:44 | 11.8 | 73M | 2471 |
| 73 | `~/bags/m8-static-check-20261113/flat_1` | sqlite3 | 2026-11-13 11:42:16 | 11.8 | 73M | 2490 |
| 74 | `~/bags/m8-static-check-20261113/flat_2_rot180` | sqlite3 | 2026-11-13 11:44:49 | 11.8 | 73M | 2471 |
| 75 | `~/bags/m8-static-check-20261113/flat_3_seated` | sqlite3 | 2026-11-13 11:46:58 | 11.8 | 73M | 2471 |
| 76 | `~/bags/sensor_check_20261028/chk_bottle_2m_160742` | mcap | 2026-10-28 16:08:17 | 12.8 | 725M | 2186 |
| 77 | `~/bags/sensor_check_20261028/chk_bottle_3m_160831` | mcap | 2026-10-28 16:08:50 | 11.7 | 663M | 2004 |
| 78 | `~/bags/sensor_check_20261028/chk_bottle_4m_160903` | mcap | 2026-10-28 16:09:19 | 9.9 | 559M | 1689 |
| 79 | `~/bags/sensor_check_20261028/chk_bottle_5m_160929` | mcap | 2026-10-28 16:10:17 | 10.2 | 577M | 1744 |
| 80 | `~/bags/sensor_check_20261028/chk_chair_1.5m_160152` | mcap | 2026-10-28 16:02:05 | 21.9 | 1.3G | 3731 |
| 81 | `~/bags/sensor_check_20261028/chk_chair_1.5m_160421` | mcap | 2026-10-28 16:04:46 | 11.6 | 651M | 1971 |
| 82 | `~/bags/sensor_check_20261028/chk_chair_2m_160227` | mcap | 2026-10-28 16:03:20 | 11.0 | 624M | 1884 |
| 83 | `~/bags/sensor_check_20261028/chk_chair_2m_160458` | mcap | 2026-10-28 16:05:22 | 10.7 | 606M | 1832 |
| 84 | `~/bags/sensor_check_20261028/chk_chair_3m_160331` | mcap | 2026-10-28 16:03:58 | 12.4 | 702M | 2122 |
| 85 | `~/bags/sensor_check_20261028/chk_chair_3m_160533` | mcap | 2026-10-28 16:06:08 | 10.9 | 618M | 1868 |
| 86 | `~/bags/sensor_check_20261028/chk_empty_0m_160726` | mcap | 2026-10-28 16:07:30 | 11.4 | 646M | 1954 |
| 87 | `~/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag` | sqlite3 | 2026-07-12 19:08:40 | 0.0 | 32K | 1 |
| 88 | `~/whill_platform/bags/2026-07-31-campus` | mcap | 2026-07-31 15:14:51 | 235.1 | 1.5G | 54669 |

開始時刻（ナノ秒まで）が一致する bag の組（同じ記録から派生したもの）: #10/#11/#38/#39、#20/#21、#27/#28/#29、#31/#32/#34、#65/#66、#67/#88

- `metadata.yaml` が無い bag: 0 本。`ros2 bag reindex` は実行していない（bag を変更しないため）。
- ROS 1 の `*.bag`: `/opt/ros/humble/share/robot_localization/test/test{1,2,3}.bag`（apt パッケージのテスト用。この研究の記録ではないので表から除外）。
- 派生・重複の内訳（開始時刻の一致から）: #10/#11 は #38 の先頭 300 s の切り出し（IMU 調査用、`_hires` と `_orig`）、#39 は #38 の Hi-Res 版。#21 は #20 の IMU 修正版。#27/#28 は #29 の先頭部分。#32 と #34 は #31 と同じ開始（#31 は debiased 版、#32 は seg-A）。#66 は #65 の Hi-Res 版（`docs/lidar-yaw-measurement.md`）。#88 は #67 の mcap 変換（`~/whill_platform/bags/README.md`）。
- worktree: `~/whill_lab0_ros2/.claude/worktrees/*/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag` が 13 部あるが、git 管理のファイルで #87 と md5 が一致するため数えていない。
- シンボリックリンク: `~/bags/imu_investigation_20261030/glim_cfg/cpu_base/*.json` などの設定ファイルだけで、bag のリンクは無い。

## 2. bag ごとのトピック・型・件数・平均レート

```bash
# 付録 D の analyze.py を各 bag に実行（rosbag2_py で metadata を読み、件数 / Duration を平均レートとする）
bash run.sh   # → analysis.jsonl（生の出力は付録 E）
python3 gen.py
```

平均レートは「件数 ÷ bag の Duration」。トピックが途中から出た場合や、`/tf_static` のような latched なトピックでは実際の発行周期と一致しない。生の出力は付録 A（`ros2 bag info`）と付録 E（JSON）。

#### #1 `~/bags/2026-11-16-static/cone_1p5_141326`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 352 | 29.72 |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 352 | 29.72 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 352 | 29.72 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 352 | 29.72 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1185 | 100.05 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1174 | 99.12 |
| `/rosout` | `rcl_interfaces/msg/Log` | 16 | 1.35 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 356 | 30.06 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.34 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 115 | 9.71 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 116 | 9.79 |

#### #2 `~/bags/2026-11-16-static/cone_2p5_141549`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 355 | 29.99 |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 355 | 29.99 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 355 | 29.99 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 355 | 29.99 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1184 | 100.01 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1184 | 100.01 |
| `/rosout` | `rcl_interfaces/msg/Log` | 16 | 1.35 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 352 | 29.73 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.34 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 117 | 9.88 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 117 | 9.88 |

#### #3 `~/bags/2026-11-16-static/cone_2p_141506`（11.9 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 353 | 29.79 |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 353 | 29.79 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 352 | 29.70 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 352 | 29.70 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1175 | 99.15 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1175 | 99.15 |
| `/rosout` | `rcl_interfaces/msg/Log` | 16 | 1.35 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 352 | 29.70 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.34 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 116 | 9.79 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 117 | 9.87 |

#### #4 `~/bags/2026-11-16-static/cone_3p5_141752`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 355 | 29.99 |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 355 | 29.99 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 355 | 29.99 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 355 | 29.99 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1184 | 100.02 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1184 | 100.02 |
| `/rosout` | `rcl_interfaces/msg/Log` | 16 | 1.35 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 355 | 29.99 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.34 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 117 | 9.88 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 117 | 9.88 |

#### #5 `~/bags/2026-11-16-static/cone_3p_141709`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 351 | 29.74 |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 351 | 29.74 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 351 | 29.74 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 351 | 29.74 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1180 | 99.96 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1170 | 99.12 |
| `/rosout` | `rcl_interfaces/msg/Log` | 16 | 1.36 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 354 | 29.99 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.34 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 115 | 9.74 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 116 | 9.83 |

#### #6 `~/bags/2026-11-16-static/cone_4p_141838`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 355 | 29.97 |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 355 | 29.97 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 355 | 29.97 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 355 | 29.97 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1185 | 100.04 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1174 | 99.11 |
| `/rosout` | `rcl_interfaces/msg/Log` | 16 | 1.35 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 355 | 29.97 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.34 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 116 | 9.79 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 117 | 9.88 |

#### #7 `~/bags/2026-11-16-static/cone_5p_141921`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 356 | 30.05 |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 356 | 30.05 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 355 | 29.96 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 355 | 29.96 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1175 | 99.17 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1185 | 100.01 |
| `/rosout` | `rcl_interfaces/msg/Log` | 16 | 1.35 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 356 | 30.05 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.34 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 117 | 9.87 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 117 | 9.87 |

#### #8 `~/bags/2026-11-16-static/empty_start_140915`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 355 | 29.98 |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 355 | 29.98 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 355 | 29.98 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 355 | 29.98 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1184 | 99.99 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1184 | 99.99 |
| `/rosout` | `rcl_interfaces/msg/Log` | 14 | 1.18 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 355 | 29.98 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.34 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 117 | 9.88 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 117 | 9.88 |

#### #9 `~/bags/2026-11-16-test/smoke_140741`（4.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 145 | 29.94 |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 145 | 29.94 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 146 | 30.15 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 146 | 30.15 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 484 | 99.94 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 484 | 99.94 |
| `/rosout` | `rcl_interfaces/msg/Log` | 16 | 3.30 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 146 | 30.15 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.83 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 48 | 9.91 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 48 | 9.91 |

#### #10 `~/bags/imu_investigation_20261030/seg_0_300_hires`（300.0 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 30000 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2957 | 9.86 |

#### #11 `~/bags/imu_investigation_20261030/seg_0_300_orig`（300.0 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 30000 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2957 | 9.86 |

#### #12 `~/bags/m3-bench-data/m3_chair_motion_2026-05-07`（96.9 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 2904 | 29.98 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 2904 | 29.98 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 9687 | 100.01 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 9687 | 100.01 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.04 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 951 | 9.82 |

#### #13 `~/bags/m3-bench-data/m3_chair_static_2026-05-07`（19.9 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 595 | 29.97 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 595 | 29.97 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1986 | 100.05 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 1986 | 100.05 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.20 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 196 | 9.87 |

#### #14 `~/bags/m3-bench-data/m3_smoke_2026-05-07`（7.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 235 | 30.00 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 234 | 29.87 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 784 | 100.07 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 784 | 100.07 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 1 | 0.13 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 77 | 9.83 |

#### #15 `~/bags/m3-bench-data/m4_chair_live_2026-05-08`（59.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/Odometry` | `nav_msgs/msg/Odometry` | 48 | 0.80 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 1794 | 29.98 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 1794 | 29.98 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 5984 | 100.00 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 5984 | 100.00 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 48 | 0.80 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.07 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 590 | 9.86 |

#### #16 `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run1`（59.5 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/Odometry` | `nav_msgs/msg/Odometry` | 88 | 1.48 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 1784 | 29.98 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 1785 | 30.00 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 5952 | 100.03 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 5952 | 100.03 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 88 | 1.48 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.07 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 587 | 9.87 |

#### #17 `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run2`（59.5 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/Odometry` | `nav_msgs/msg/Odometry` | 81 | 1.36 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 1784 | 29.98 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 1781 | 29.93 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 5952 | 100.03 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 5952 | 100.03 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 81 | 1.36 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.07 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 577 | 9.70 |

#### #18 `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run3`（59.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/Odometry` | `nav_msgs/msg/Odometry` | 108 | 1.80 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 1794 | 29.98 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 1794 | 29.98 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 5887 | 98.38 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 5887 | 98.38 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 108 | 1.80 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.07 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 590 | 9.86 |

#### #19 `~/bags/m3-bench-data/m5_chair_mapping_2026-05-08`（119.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/Odometry` | `nav_msgs/msg/Odometry` | 290 | 2.42 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 3592 | 29.98 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 3592 | 29.98 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 11886 | 99.19 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 11886 | 99.19 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 290 | 2.42 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 6 | 0.05 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 1182 | 9.86 |

#### #20 `~/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag`（199.3 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 19931 | 100.01 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 5979 | 30.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.02 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 1965 | 9.86 |

#### #21 `~/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag-imu-fixed`（199.3 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 19931 | 100.01 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 5979 | 30.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.02 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 1965 | 9.86 |

#### #22 `~/bags/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag`（239.4 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 23945 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.02 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2361 | 9.86 |

#### #23 `~/bags/m5r-bench-data/2026-07-07-campus-half-v3/bag`（466.3 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 46626 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 4590 | 9.84 |

#### #24 `~/bags/m5r-bench-data/2026-07-07-campus-loop/bag`（821.5 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 80821 | 98.39 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 7844 | 9.55 |

#### #25 `~/bags/m5r-bench-data/2026-07-08-campus-demo-postgap/bag`（1817.7 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 170909 | 94.03 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 12 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 16849 | 9.27 |

#### #26 `~/bags/m5r-bench-data/2026-07-08-campus-demo-pregap/bag`（1864.5 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 186449 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 0 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 18380 | 9.86 |

#### #27 `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2-first10/bag`（599.6 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 59968 | 100.02 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 5911 | 9.86 |

#### #28 `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2-pregap/bag`（2602.1 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 258728 | 99.43 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 8 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 25577 | 9.83 |

#### #29 `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2/bag`（3482.5 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 333360 | 95.72 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 20 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 32938 | 9.46 |

#### #30 `~/bags/m5r-bench-data/2026-07-08-campus-demo/bag`（3778.4 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 357866 | 94.71 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 20 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 35279 | 9.34 |

#### #31 `~/bags/m5r-bench-data/2026-07-08-campus-outer-debiased/bag`（2813.3 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 280858 | 99.83 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 27695 | 9.84 |

#### #32 `~/bags/m5r-bench-data/2026-07-08-campus-outer-seg-A/bag`（1325.1 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 132507 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 13062 | 9.86 |

#### #33 `~/bags/m5r-bench-data/2026-07-08-campus-outer-seg-B/bag`（1483.5 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 148350 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 14633 | 9.86 |

#### #34 `~/bags/m5r-bench-data/2026-07-08-campus-outer/bag`（2813.3 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 280858 | 99.83 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 27695 | 9.84 |

#### #35 `~/bags/m5r-bench-data/2026-07-08-indoor-calib/bag`（365.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 36585 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 3607 | 9.86 |

#### #36 `~/bags/m5r-bench-data/2026-07-08-outdoor-corrected/bag`（745.4 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 74540 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 7348 | 9.86 |

#### #37 `~/bags/m5r-bench-data/2026-07-09-indoor-calib/bag`（209.1 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 20914 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.02 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2062 | 9.86 |

#### #38 `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag`（2162.0 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 216191 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 21311 | 9.86 |

#### #39 `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires`（2162.0 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 216191 | 100.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 21311 | 9.86 |

#### #40 `~/bags/m5r-bench-data/2026-07-10-dds-verify/bag`（303.7 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 30368 | 99.99 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2994 | 9.86 |

#### #41 `~/bags/m6r-bench-data/2026-07-14-verify-campus/bag`（627.7 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 62760 | 99.98 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 24663 | 39.29 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 6187 | 9.86 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 1569 | 2.50 |

#### #42 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41`（783.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 6699 | 8.55 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 826 | 1.05 |
| `/dispatch/state` | `std_msgs/msg/String` | 3919 | 5.00 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 77876 | 99.36 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 23363 | 29.81 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 6701 | 8.55 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 7678 | 9.80 |

#### #43 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43`（257.6 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 637 | 2.47 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 2591 | 10.06 |
| `/dispatch/state` | `std_msgs/msg/String` | 1288 | 5.00 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 25752 | 99.98 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 7727 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 21 | 0.08 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2539 | 9.86 |

#### #44 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08`（270.9 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 0 | 0.00 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 637 | 2.35 |
| `/dispatch/state` | `std_msgs/msg/String` | 1355 | 5.00 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 27088 | 100.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 8126 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 0 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2671 | 9.86 |

#### #45 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27`（632.6 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 6065 | 9.59 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 4531 | 7.16 |
| `/dispatch/state` | `std_msgs/msg/String` | 3171 | 5.01 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 62962 | 99.53 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 18889 | 29.86 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 6061 | 9.58 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 6208 | 9.81 |

#### #46 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07`（462.9 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 4564 | 9.86 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 3422 | 7.39 |
| `/dispatch/state` | `std_msgs/msg/String` | 2319 | 5.01 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 46293 | 100.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 13888 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 4564 | 9.86 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 4563 | 9.86 |

#### #47 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28`（543.0 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 5354 | 9.86 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 3653 | 6.73 |
| `/dispatch/state` | `std_msgs/msg/String` | 2719 | 5.01 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 54301 | 100.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 16291 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 5354 | 9.86 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 5353 | 9.86 |

#### #48 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48`（275.5 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 2717 | 9.86 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 2310 | 8.38 |
| `/dispatch/state` | `std_msgs/msg/String` | 1381 | 5.01 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 27554 | 100.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 8267 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 2717 | 9.86 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2716 | 9.86 |

#### #49 `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01`（255.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 2245 | 8.78 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 1225 | 4.79 |
| `/dispatch/state` | `std_msgs/msg/String` | 1283 | 5.02 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 25580 | 100.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 7674 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 2240 | 8.76 |
| `/plan` | `nav_msgs/msg/Path` | 88 | 0.34 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2521 | 9.86 |

#### #50 `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48`（119.2 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 1018 | 8.54 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 2030 | 17.03 |
| `/dispatch/state` | `std_msgs/msg/String` | 571 | 4.79 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 11917 | 100.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 3576 | 30.01 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 633 | 5.31 |
| `/plan` | `nav_msgs/msg/Path` | 0 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 1175 | 9.86 |

#### #51 `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31`（506.1 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 4677 | 9.24 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 2206 | 4.36 |
| `/dispatch/state` | `std_msgs/msg/String` | 2547 | 5.03 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 50613 | 100.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 15184 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 4647 | 9.18 |
| `/plan` | `nav_msgs/msg/Path` | 166 | 0.33 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 4989 | 9.86 |

#### #52 `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34`（595.1 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 5293 | 8.89 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 2955 | 4.97 |
| `/dispatch/state` | `std_msgs/msg/String` | 2988 | 5.02 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 59491 | 99.96 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 17853 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 5294 | 8.90 |
| `/plan` | `nav_msgs/msg/Path` | 218 | 0.37 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 5866 | 9.86 |

#### #53 `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14`（337.4 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 2958 | 8.77 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 1675 | 4.96 |
| `/dispatch/state` | `std_msgs/msg/String` | 1702 | 5.04 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 33738 | 100.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 10121 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 2950 | 8.74 |
| `/plan` | `nav_msgs/msg/Path` | 96 | 0.28 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 3326 | 9.86 |

#### #54 `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15`（132.5 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 1306 | 9.86 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 1019 | 7.69 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 1041 | 7.86 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 0 | 0.00 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 15 | 0.11 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.01 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 13247 | 100.01 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 156 | 1.18 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 662 | 5.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 3974 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 1306 | 9.86 |
| `/plan` | `nav_msgs/msg/Path` | 83 | 0.63 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 1306 | 9.86 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 5279 | 39.85 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.02 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 1306 | 9.86 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 331 | 2.50 |

#### #55 `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13`（174.4 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 1574 | 9.03 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 1577 | 9.04 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 758 | 4.35 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 819 | 4.70 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 0 | 0.00 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.01 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 17440 | 100.00 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 186 | 1.07 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 847 | 4.86 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 5232 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 1574 | 9.03 |
| `/plan` | `nav_msgs/msg/Path` | 58 | 0.33 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 1573 | 9.02 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 6805 | 39.02 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.02 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 1573 | 9.02 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 289 | 1.66 |

#### #56 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43`（121.0 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 0 | 0.00 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 843 | 6.97 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 1 | 0.01 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 843 | 6.97 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 0 | 0.00 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.01 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 5709 | 47.20 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.01 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 0 | 0.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 1716 | 14.19 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 1 | 0.01 |
| `/plan` | `nav_msgs/msg/Path` | 0 | 0.00 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 0 | 0.00 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 1716 | 14.19 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 6 | 0.05 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 0 | 0.00 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 145 | 1.20 |

#### #57 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46`（63.1 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 0 | 0.00 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 948 | 15.02 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 0 | 0.00 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 948 | 15.02 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 0 | 0.00 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.02 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 4384 | 69.48 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.02 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 41 | 0.65 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 1352 | 21.43 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 1 | 0.02 |
| `/plan` | `nav_msgs/msg/Path` | 0 | 0.00 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 0 | 0.00 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 1354 | 21.46 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.05 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 0 | 0.00 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 107 | 1.70 |

#### #58 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10`（62.0 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 72 | 1.16 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 761 | 12.26 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 82 | 1.32 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 761 | 12.26 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 0 | 0.00 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.02 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 6205 | 100.00 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 2 | 0.03 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 87 | 1.40 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 1862 | 30.01 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 73 | 1.18 |
| `/plan` | `nav_msgs/msg/Path` | 6 | 0.10 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 227 | 3.66 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 1935 | 31.19 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.05 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 228 | 3.67 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 155 | 2.50 |

#### #59 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44`（36.4 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 95 | 2.61 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 507 | 13.93 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 24 | 0.66 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 507 | 13.93 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 0 | 0.00 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.03 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 3640 | 100.02 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.03 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 98 | 2.69 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 1091 | 29.98 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 95 | 2.61 |
| `/plan` | `nav_msgs/msg/Path` | 2 | 0.05 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 94 | 2.58 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 1185 | 32.56 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.08 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 94 | 2.58 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 91 | 2.50 |

#### #60 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02`（121.6 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 621 | 5.11 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 1734 | 14.26 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 152 | 1.25 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 1729 | 14.22 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 39 | 0.32 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.01 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 12158 | 100.00 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 68 | 0.56 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 365 | 3.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 3648 | 30.01 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 606 | 4.98 |
| `/plan` | `nav_msgs/msg/Path` | 15 | 0.12 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 789 | 6.49 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 4254 | 34.99 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.02 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 789 | 6.49 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 304 | 2.50 |

#### #61 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36`（277.2 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 2353 | 8.49 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 2877 | 10.38 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 1153 | 4.16 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 1494 | 5.39 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 374 | 1.35 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.00 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 27714 | 99.98 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 229 | 0.83 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 1197 | 4.32 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 8316 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 2353 | 8.49 |
| `/plan` | `nav_msgs/msg/Path` | 93 | 0.34 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2260 | 8.15 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 10669 | 38.49 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2352 | 8.48 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 693 | 2.50 |

#### #62 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38`（38.5 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 79 | 2.05 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 612 | 15.89 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 112 | 2.91 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 588 | 15.27 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 0 | 0.00 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.03 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 3853 | 100.04 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 3 | 0.08 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 89 | 2.31 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 1156 | 30.01 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 79 | 2.05 |
| `/plan` | `nav_msgs/msg/Path` | 8 | 0.21 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 78 | 2.03 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 1234 | 32.04 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.08 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 78 | 2.03 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 96 | 2.49 |

#### #63 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23`（37.5 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 124 | 3.31 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 373 | 9.95 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 77 | 2.05 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 356 | 9.50 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 0 | 0.00 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.03 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 3112 | 83.00 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 3 | 0.08 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 94 | 2.51 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 943 | 25.15 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 124 | 3.31 |
| `/plan` | `nav_msgs/msg/Path` | 6 | 0.16 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 123 | 3.28 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 1066 | 28.43 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.08 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 123 | 3.28 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 88 | 2.35 |

#### #64 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27`（497.7 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 4131 | 8.30 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 5365 | 10.78 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 3905 | 7.85 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 1557 | 3.13 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 9 | 0.02 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.00 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 41897 | 84.18 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 635 | 1.28 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 2148 | 4.32 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 12767 | 25.65 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 4131 | 8.30 |
| `/plan` | `nav_msgs/msg/Path` | 303 | 0.61 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 4130 | 8.30 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 16897 | 33.95 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 4130 | 8.30 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 1244 | 2.50 |

#### #65 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26`（842.9 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 8310 | 9.86 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 7458 | 8.85 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 6598 | 7.83 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 587 | 0.70 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 381 | 0.45 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.00 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 84290 | 100.00 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1150 | 1.36 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 4215 | 5.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 25287 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 8303 | 9.85 |
| `/plan` | `nav_msgs/msg/Path` | 528 | 0.63 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 8309 | 9.86 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 33589 | 39.85 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 8309 | 9.86 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 2104 | 2.50 |

#### #66 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires`（842.9 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 8310 | 9.86 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 7458 | 8.85 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 6598 | 7.83 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 587 | 0.70 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 381 | 0.45 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.00 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 84290 | 100.00 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1150 | 1.36 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 4215 | 5.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 25287 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 8303 | 9.85 |
| `/plan` | `nav_msgs/msg/Path` | 528 | 0.63 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 8309 | 9.86 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 33589 | 39.85 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.00 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 8309 | 9.86 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 2104 | 2.50 |

#### #67 `~/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51`（235.1 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 2318 | 9.86 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 1621 | 6.90 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 1218 | 5.18 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 576 | 2.45 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 1 | 0.00 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.00 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 23509 | 100.00 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 194 | 0.83 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 1175 | 5.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 7053 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 2317 | 9.86 |
| `/plan` | `nav_msgs/msg/Path` | 93 | 0.40 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2317 | 9.86 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 9369 | 39.85 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2317 | 9.86 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 587 | 2.50 |

#### #68 `~/bags/m8-static-check-20261113/fixed_1`（11.7 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1174 | 100.07 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1174 | 100.07 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.26 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 116 | 9.89 |

#### #69 `~/bags/m8-static-check-20261113/fixed_2_place2`（11.7 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1175 | 100.08 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1175 | 100.08 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.26 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 115 | 9.79 |

#### #70 `~/bags/m8-static-check-20261113/fixed_3_place2_rot180`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1174 | 99.15 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1174 | 99.15 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.25 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 116 | 9.80 |

#### #71 `~/bags/m8-static-check-20261113/fixed_4_place2_rot180_after_drive`（11.7 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1174 | 100.08 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1174 | 100.08 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.26 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 116 | 9.89 |

#### #72 `~/bags/m8-static-check-20261113/fixed_5_place3_rot180`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1176 | 100.08 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1176 | 100.08 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.26 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 116 | 9.87 |

#### #73 `~/bags/m8-static-check-20261113/flat_1`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1185 | 100.05 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1185 | 100.05 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.25 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 117 | 9.88 |

#### #74 `~/bags/m8-static-check-20261113/flat_2_rot180`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1176 | 100.08 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1176 | 100.08 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.26 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 116 | 9.87 |

#### #75 `~/bags/m8-static-check-20261113/flat_3_seated`（11.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1176 | 100.07 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 1176 | 100.07 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.26 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 116 | 9.87 |

#### #76 `~/bags/sensor_check_20261028/chk_bottle_2m_160742`（12.8 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 384 | 29.96 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 384 | 29.96 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 384 | 29.96 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 384 | 29.96 |
| `/rosout` | `rcl_interfaces/msg/Log` | 13 | 1.01 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 382 | 29.80 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.31 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 125 | 9.75 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 126 | 9.83 |

#### #77 `~/bags/sensor_check_20261028/chk_bottle_3m_160831`（11.7 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 351 | 29.99 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 351 | 29.99 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 351 | 29.99 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 351 | 29.99 |
| `/rosout` | `rcl_interfaces/msg/Log` | 13 | 1.11 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 351 | 29.99 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.34 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 116 | 9.91 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 116 | 9.91 |

#### #78 `~/bags/sensor_check_20261028/chk_bottle_4m_160903`（9.9 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 296 | 29.98 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 296 | 29.98 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 296 | 29.98 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 296 | 29.98 |
| `/rosout` | `rcl_interfaces/msg/Log` | 13 | 1.32 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 293 | 29.67 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.41 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 98 | 9.93 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 97 | 9.82 |

#### #79 `~/bags/sensor_check_20261028/chk_bottle_5m_160929`（10.2 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 306 | 30.03 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 306 | 30.03 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 305 | 29.94 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 305 | 29.94 |
| `/rosout` | `rcl_interfaces/msg/Log` | 12 | 1.18 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 306 | 30.03 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.39 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 100 | 9.82 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 100 | 9.82 |

#### #80 `~/bags/sensor_check_20261028/chk_chair_1.5m_160152`（21.9 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 657 | 30.00 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 657 | 30.00 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 657 | 30.00 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 657 | 30.00 |
| `/rosout` | `rcl_interfaces/msg/Log` | 11 | 0.50 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 657 | 30.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.18 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 215 | 9.82 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 216 | 9.86 |

#### #81 `~/bags/sensor_check_20261028/chk_chair_1.5m_160421`（11.6 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 344 | 29.69 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 344 | 29.69 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 345 | 29.78 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 345 | 29.78 |
| `/rosout` | `rcl_interfaces/msg/Log` | 13 | 1.12 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 348 | 30.03 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.35 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 114 | 9.84 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 114 | 9.84 |

#### #82 `~/bags/sensor_check_20261028/chk_chair_2m_160227`（11.0 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 330 | 30.04 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 330 | 30.04 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 330 | 30.04 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 330 | 30.04 |
| `/rosout` | `rcl_interfaces/msg/Log` | 13 | 1.18 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 330 | 30.04 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.36 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 108 | 9.83 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 109 | 9.92 |

#### #83 `~/bags/sensor_check_20261028/chk_chair_2m_160458`（10.7 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 321 | 30.02 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 321 | 30.02 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 321 | 30.02 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 321 | 30.02 |
| `/rosout` | `rcl_interfaces/msg/Log` | 13 | 1.22 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 321 | 30.02 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.37 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 105 | 9.82 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 105 | 9.82 |

#### #84 `~/bags/sensor_check_20261028/chk_chair_3m_160331`（12.4 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 372 | 30.00 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 371 | 29.92 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 372 | 30.00 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 372 | 30.00 |
| `/rosout` | `rcl_interfaces/msg/Log` | 13 | 1.05 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 372 | 30.00 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.32 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 123 | 9.92 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 123 | 9.92 |

#### #85 `~/bags/sensor_check_20261028/chk_chair_3m_160533`（10.9 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 327 | 29.99 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 327 | 29.99 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 327 | 29.99 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 327 | 29.99 |
| `/rosout` | `rcl_interfaces/msg/Log` | 13 | 1.19 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 327 | 29.99 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.37 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 108 | 9.90 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 108 | 9.90 |

#### #86 `~/bags/sensor_check_20261028/chk_empty_0m_160726`（11.4 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 342 | 30.00 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 342 | 30.00 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 342 | 30.00 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 342 | 30.00 |
| `/rosout` | `rcl_interfaces/msg/Log` | 13 | 1.14 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 343 | 30.08 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 4 | 0.35 |
| `/velodyne_packets` | `velodyne_msgs/msg/VelodyneScan` | 113 | 9.91 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 113 | 9.91 |

#### #87 `~/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag`（0.0 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 1 | —（長さ 0） |

#### #88 `~/whill_platform/bags/2026-07-31-campus`（235.1 s）

| トピック | 型 | 件数 | 平均レート [Hz] |
|---|---|---:|---:|
| `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 2318 | 9.86 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 1621 | 6.90 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 1218 | 5.18 |
| `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 576 | 2.45 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | 1 | 0.00 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1 | 0.00 |
| `/imu/data_rep145` | `sensor_msgs/msg/Imu` | 23509 | 100.00 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 194 | 0.83 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 1175 | 5.00 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 7053 | 30.00 |
| `/pcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 2317 | 9.86 |
| `/plan` | `nav_msgs/msg/Path` | 93 | 0.40 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2317 | 9.86 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 9369 | 39.85 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 3 | 0.01 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 2317 | 9.86 |
| `/whill/odom` | `nav_msgs/msg/Odometry` | 587 | 2.50 |

## 3. `/camera/camera/color/image_raw/compressed` の有無・解像度・レート

```bash
# analyze.py の該当部分（付録 D）:
#  - /camera/camera/color/image_raw/compressed だけを StorageFilter で読み、1 通目を deserialize
#  - msg.data の先頭 64 KiB をメモリ上で走査し、JPEG の SOFn マーカ（または PNG の IHDR）から幅・高さを読む
#  - 画像データはファイルに書かない。比較のため /camera/camera/color/camera_info の 1 通目の width/height も読む
```

出力（付録 E の `camera` 欄から作成）:

| # | bag | compressed の有無 | 件数 | 平均レート [Hz] | 1 通目の format 欄 | 1 通目のヘッダから読んだ解像度 | color/camera_info の width×height | 生の /camera/camera/color/image_raw |
|---|---|---|---:|---:|---|---|---|---|
| 1 | `~/bags/2026-11-16-static/cone_1p5_141326` | **あり** | 352 | 29.72 | `rgb8; jpeg compressed bgr8` | JPEG 640×480 | 640×480 | なし |
| 2 | `~/bags/2026-11-16-static/cone_2p5_141549` | **あり** | 355 | 29.99 | `rgb8; jpeg compressed bgr8` | JPEG 640×480 | 640×480 | なし |
| 3 | `~/bags/2026-11-16-static/cone_2p_141506` | **あり** | 353 | 29.79 | `rgb8; jpeg compressed bgr8` | JPEG 640×480 | 640×480 | なし |
| 4 | `~/bags/2026-11-16-static/cone_3p5_141752` | **あり** | 355 | 29.99 | `rgb8; jpeg compressed bgr8` | JPEG 640×480 | 640×480 | なし |
| 5 | `~/bags/2026-11-16-static/cone_3p_141709` | **あり** | 351 | 29.74 | `rgb8; jpeg compressed bgr8` | JPEG 640×480 | 640×480 | なし |
| 6 | `~/bags/2026-11-16-static/cone_4p_141838` | **あり** | 355 | 29.97 | `rgb8; jpeg compressed bgr8` | JPEG 640×480 | 640×480 | なし |
| 7 | `~/bags/2026-11-16-static/cone_5p_141921` | **あり** | 356 | 30.05 | `rgb8; jpeg compressed bgr8` | JPEG 640×480 | 640×480 | なし |
| 8 | `~/bags/2026-11-16-static/empty_start_140915` | **あり** | 355 | 29.98 | `rgb8; jpeg compressed bgr8` | JPEG 640×480 | 640×480 | なし |
| 9 | `~/bags/2026-11-16-test/smoke_140741` | **あり** | 145 | 29.94 | `rgb8; jpeg compressed bgr8` | JPEG 640×480 | 640×480 | なし |
| 10 | `~/bags/imu_investigation_20261030/seg_0_300_hires` | なし | — | — | — | — | — | なし |
| 11 | `~/bags/imu_investigation_20261030/seg_0_300_orig` | なし | — | — | — | — | — | なし |
| 12 | `~/bags/m3-bench-data/m3_chair_motion_2026-05-07` | なし | — | — | — | — | — | あり（2904 件, 29.98 Hz） |
| 13 | `~/bags/m3-bench-data/m3_chair_static_2026-05-07` | なし | — | — | — | — | — | あり（595 件, 29.97 Hz） |
| 14 | `~/bags/m3-bench-data/m3_smoke_2026-05-07` | なし | — | — | — | — | — | あり（235 件, 30.00 Hz） |
| 15 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08` | なし | — | — | — | — | — | あり（1794 件, 29.98 Hz） |
| 16 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run1` | なし | — | — | — | — | — | あり（1784 件, 29.98 Hz） |
| 17 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run2` | なし | — | — | — | — | — | あり（1784 件, 29.98 Hz） |
| 18 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run3` | なし | — | — | — | — | — | あり（1794 件, 29.98 Hz） |
| 19 | `~/bags/m3-bench-data/m5_chair_mapping_2026-05-08` | なし | — | — | — | — | — | あり（3592 件, 29.98 Hz） |
| 20 | `~/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag` | なし | — | — | — | — | — | なし |
| 21 | `~/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag-imu-fixed` | なし | — | — | — | — | — | なし |
| 22 | `~/bags/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag` | なし | — | — | — | — | — | なし |
| 23 | `~/bags/m5r-bench-data/2026-07-07-campus-half-v3/bag` | なし | — | — | — | — | — | なし |
| 24 | `~/bags/m5r-bench-data/2026-07-07-campus-loop/bag` | なし | — | — | — | — | — | なし |
| 25 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-postgap/bag` | なし | — | — | — | — | — | なし |
| 26 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-pregap/bag` | なし | — | — | — | — | — | なし |
| 27 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2-first10/bag` | なし | — | — | — | — | — | なし |
| 28 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2-pregap/bag` | なし | — | — | — | — | — | なし |
| 29 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2/bag` | なし | — | — | — | — | — | なし |
| 30 | `~/bags/m5r-bench-data/2026-07-08-campus-demo/bag` | なし | — | — | — | — | — | なし |
| 31 | `~/bags/m5r-bench-data/2026-07-08-campus-outer-debiased/bag` | なし | — | — | — | — | — | なし |
| 32 | `~/bags/m5r-bench-data/2026-07-08-campus-outer-seg-A/bag` | なし | — | — | — | — | — | なし |
| 33 | `~/bags/m5r-bench-data/2026-07-08-campus-outer-seg-B/bag` | なし | — | — | — | — | — | なし |
| 34 | `~/bags/m5r-bench-data/2026-07-08-campus-outer/bag` | なし | — | — | — | — | — | なし |
| 35 | `~/bags/m5r-bench-data/2026-07-08-indoor-calib/bag` | なし | — | — | — | — | — | なし |
| 36 | `~/bags/m5r-bench-data/2026-07-08-outdoor-corrected/bag` | なし | — | — | — | — | — | なし |
| 37 | `~/bags/m5r-bench-data/2026-07-09-indoor-calib/bag` | なし | — | — | — | — | — | なし |
| 38 | `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag` | なし | — | — | — | — | — | なし |
| 39 | `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires` | なし | — | — | — | — | — | なし |
| 40 | `~/bags/m5r-bench-data/2026-07-10-dds-verify/bag` | なし | — | — | — | — | — | なし |
| 41 | `~/bags/m6r-bench-data/2026-07-14-verify-campus/bag` | なし | — | — | — | — | — | なし |
| 42 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41` | なし | — | — | — | — | — | なし |
| 43 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43` | なし | — | — | — | — | — | なし |
| 44 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08` | なし | — | — | — | — | — | なし |
| 45 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27` | なし | — | — | — | — | — | なし |
| 46 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07` | なし | — | — | — | — | — | なし |
| 47 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28` | なし | — | — | — | — | — | なし |
| 48 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48` | なし | — | — | — | — | — | なし |
| 49 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01` | なし | — | — | — | — | — | なし |
| 50 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48` | なし | — | — | — | — | — | なし |
| 51 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31` | なし | — | — | — | — | — | なし |
| 52 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34` | なし | — | — | — | — | — | なし |
| 53 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14` | なし | — | — | — | — | — | なし |
| 54 | `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15` | なし | — | — | — | — | — | なし |
| 55 | `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13` | なし | — | — | — | — | — | なし |
| 56 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43` | なし | — | — | — | — | — | なし |
| 57 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46` | なし | — | — | — | — | — | なし |
| 58 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10` | なし | — | — | — | — | — | なし |
| 59 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44` | なし | — | — | — | — | — | なし |
| 60 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02` | なし | — | — | — | — | — | なし |
| 61 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36` | なし | — | — | — | — | — | なし |
| 62 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38` | なし | — | — | — | — | — | なし |
| 63 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23` | なし | — | — | — | — | — | なし |
| 64 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27` | なし | — | — | — | — | — | なし |
| 65 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26` | なし | — | — | — | — | — | なし |
| 66 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires` | なし | — | — | — | — | — | なし |
| 67 | `~/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51` | なし | — | — | — | — | — | なし |
| 68 | `~/bags/m8-static-check-20261113/fixed_1` | なし | — | — | — | — | — | なし |
| 69 | `~/bags/m8-static-check-20261113/fixed_2_place2` | なし | — | — | — | — | — | なし |
| 70 | `~/bags/m8-static-check-20261113/fixed_3_place2_rot180` | なし | — | — | — | — | — | なし |
| 71 | `~/bags/m8-static-check-20261113/fixed_4_place2_rot180_after_drive` | なし | — | — | — | — | — | なし |
| 72 | `~/bags/m8-static-check-20261113/fixed_5_place3_rot180` | なし | — | — | — | — | — | なし |
| 73 | `~/bags/m8-static-check-20261113/flat_1` | なし | — | — | — | — | — | なし |
| 74 | `~/bags/m8-static-check-20261113/flat_2_rot180` | なし | — | — | — | — | — | なし |
| 75 | `~/bags/m8-static-check-20261113/flat_3_seated` | なし | — | — | — | — | — | なし |
| 76 | `~/bags/sensor_check_20261028/chk_bottle_2m_160742` | なし | — | — | — | — | 640×480 | あり（384 件, 29.96 Hz） |
| 77 | `~/bags/sensor_check_20261028/chk_bottle_3m_160831` | なし | — | — | — | — | 640×480 | あり（351 件, 29.99 Hz） |
| 78 | `~/bags/sensor_check_20261028/chk_bottle_4m_160903` | なし | — | — | — | — | 640×480 | あり（296 件, 29.98 Hz） |
| 79 | `~/bags/sensor_check_20261028/chk_bottle_5m_160929` | なし | — | — | — | — | 640×480 | あり（306 件, 30.03 Hz） |
| 80 | `~/bags/sensor_check_20261028/chk_chair_1.5m_160152` | なし | — | — | — | — | 640×480 | あり（657 件, 30.00 Hz） |
| 81 | `~/bags/sensor_check_20261028/chk_chair_1.5m_160421` | なし | — | — | — | — | 640×480 | あり（344 件, 29.69 Hz） |
| 82 | `~/bags/sensor_check_20261028/chk_chair_2m_160227` | なし | — | — | — | — | 640×480 | あり（330 件, 30.04 Hz） |
| 83 | `~/bags/sensor_check_20261028/chk_chair_2m_160458` | なし | — | — | — | — | 640×480 | あり（321 件, 30.02 Hz） |
| 84 | `~/bags/sensor_check_20261028/chk_chair_3m_160331` | なし | — | — | — | — | 640×480 | あり（371 件, 29.92 Hz） |
| 85 | `~/bags/sensor_check_20261028/chk_chair_3m_160533` | なし | — | — | — | — | 640×480 | あり（327 件, 29.99 Hz） |
| 86 | `~/bags/sensor_check_20261028/chk_empty_0m_160726` | なし | — | — | — | — | 640×480 | あり（342 件, 30.00 Hz） |
| 87 | `~/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag` | なし | — | — | — | — | — | なし |
| 88 | `~/whill_platform/bags/2026-07-31-campus` | なし | — | — | — | — | — | なし |
- compressed のある 9 本はすべて `format` が `rgb8; jpeg compressed bgr8`、JPEG ヘッダの解像度と camera_info がともに 640×480 で一致。
- 走行（cmd_vel 系がある）bag には、圧縮・非圧縮ともカラー画像が無い。

## 4. 自己位置・cmd_vel 系・TF の同居（bag ごとの表）

自己位置は NDT（`lidar_localization_ros2`）の出力 `/pcl_pose`（`geometry_msgs/msg/PoseWithCovarianceStamped`）で、整合の状態は `/alignment_status`。cmd_vel 系は twist_mux の入力 3 本（`/cmd_vel_safety`・`/cmd_vel_teleop`・`/cmd_vel_nav`）、出力 `/cmd_vel`、velocity_smoother の出力 `/whill/controller/cmd_vel`。

```bash
# 自己位置のトピック名の確認（リポジトリのコード）
grep -n '"pcl_pose"\|"alignment_status"' src/third_party/lidar_localization_ros2/src/lidar_localization_component.cpp
cat src/whill_safety/config/twist_mux.yaml
grep -n "remappings" src/whill_navigation/launch/nav_launch.py
# bag 内の有無と件数は付録 A / E から gen.py で表にした
```

```
962:  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
963:    "pcl_pose",
970:  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
971:    "alignment_status",
（twist_mux.yaml）safety: /cmd_vel_safety priority 100 / teleop: /cmd_vel_teleop priority 50 / navigation: /cmd_vel_nav priority 10、timeout 0.5 s。出力は safety_launch.py で /cmd_vel にリマップ
（nav_launch.py）controller_server ほか: ('/cmd_vel', '/cmd_vel_nav')、velocity_smoother: ('/cmd_vel_smoothed', '/whill/controller/cmd_vel')
```

| # | bag | `/pcl_pose` | `/alignment_status` | `/cmd_vel` | `/cmd_vel_nav` | `/cmd_vel_teleop` | `/cmd_vel_safety` | `/whill/controller/cmd_vel` | `/tf` | `/tf_static` | 3 点同居（`/pcl_pose`＋cmd_vel 系＋`/tf`） |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | `~/bags/2026-11-16-static/cone_1p5_141326` | — | — | — | — | — | — | — | 356 | 4 | × |
| 2 | `~/bags/2026-11-16-static/cone_2p5_141549` | — | — | — | — | — | — | — | 352 | 4 | × |
| 3 | `~/bags/2026-11-16-static/cone_2p_141506` | — | — | — | — | — | — | — | 352 | 4 | × |
| 4 | `~/bags/2026-11-16-static/cone_3p5_141752` | — | — | — | — | — | — | — | 355 | 4 | × |
| 5 | `~/bags/2026-11-16-static/cone_3p_141709` | — | — | — | — | — | — | — | 354 | 4 | × |
| 6 | `~/bags/2026-11-16-static/cone_4p_141838` | — | — | — | — | — | — | — | 355 | 4 | × |
| 7 | `~/bags/2026-11-16-static/cone_5p_141921` | — | — | — | — | — | — | — | 356 | 4 | × |
| 8 | `~/bags/2026-11-16-static/empty_start_140915` | — | — | — | — | — | — | — | 355 | 4 | × |
| 9 | `~/bags/2026-11-16-test/smoke_140741` | — | — | — | — | — | — | — | 146 | 4 | × |
| 10 | `~/bags/imu_investigation_20261030/seg_0_300_hires` | — | — | — | — | — | — | — | — | 4 | × |
| 11 | `~/bags/imu_investigation_20261030/seg_0_300_orig` | — | — | — | — | — | — | — | — | 4 | × |
| 12 | `~/bags/m3-bench-data/m3_chair_motion_2026-05-07` | — | — | — | — | — | — | — | — | 4 | × |
| 13 | `~/bags/m3-bench-data/m3_chair_static_2026-05-07` | — | — | — | — | — | — | — | — | 4 | × |
| 14 | `~/bags/m3-bench-data/m3_smoke_2026-05-07` | — | — | — | — | — | — | — | — | 1 | × |
| 15 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08` | — | — | — | — | — | — | — | 48 | 4 | × |
| 16 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run1` | — | — | — | — | — | — | — | 88 | 4 | × |
| 17 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run2` | — | — | — | — | — | — | — | 81 | 4 | × |
| 18 | `~/bags/m3-bench-data/m4_chair_live_2026-05-08_run3` | — | — | — | — | — | — | — | 108 | 4 | × |
| 19 | `~/bags/m3-bench-data/m5_chair_mapping_2026-05-08` | — | — | — | — | — | — | — | 290 | 6 | × |
| 20 | `~/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag` | — | — | — | — | — | — | — | 5979 | 4 | × |
| 21 | `~/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag-imu-fixed` | — | — | — | — | — | — | — | 5979 | 4 | × |
| 22 | `~/bags/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 23 | `~/bags/m5r-bench-data/2026-07-07-campus-half-v3/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 24 | `~/bags/m5r-bench-data/2026-07-07-campus-loop/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 25 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-postgap/bag` | — | — | — | — | — | — | — | — | 12 | × |
| 26 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-pregap/bag` | — | — | — | — | — | — | — | — | 0 | × |
| 27 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2-first10/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 28 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2-pregap/bag` | — | — | — | — | — | — | — | — | 8 | × |
| 29 | `~/bags/m5r-bench-data/2026-07-08-campus-demo-v2/bag` | — | — | — | — | — | — | — | — | 20 | × |
| 30 | `~/bags/m5r-bench-data/2026-07-08-campus-demo/bag` | — | — | — | — | — | — | — | — | 20 | × |
| 31 | `~/bags/m5r-bench-data/2026-07-08-campus-outer-debiased/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 32 | `~/bags/m5r-bench-data/2026-07-08-campus-outer-seg-A/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 33 | `~/bags/m5r-bench-data/2026-07-08-campus-outer-seg-B/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 34 | `~/bags/m5r-bench-data/2026-07-08-campus-outer/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 35 | `~/bags/m5r-bench-data/2026-07-08-indoor-calib/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 36 | `~/bags/m5r-bench-data/2026-07-08-outdoor-corrected/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 37 | `~/bags/m5r-bench-data/2026-07-09-indoor-calib/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 38 | `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 39 | `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires` | — | — | — | — | — | — | — | — | 4 | × |
| 40 | `~/bags/m5r-bench-data/2026-07-10-dds-verify/bag` | — | — | — | — | — | — | — | — | 4 | × |
| 41 | `~/bags/m6r-bench-data/2026-07-14-verify-campus/bag` | — | — | — | — | — | — | — | 24663 | — | × |
| 42 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41` | 6701 | 6699 | 826 | — | — | — | — | — | — | × |
| 43 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43` | 21 | 637 | 2591 | — | — | — | — | — | — | × |
| 44 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08` | 0 | 0 | 637 | — | — | — | — | — | — | × |
| 45 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27` | 6061 | 6065 | 4531 | — | — | — | — | — | — | × |
| 46 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07` | 4564 | 4564 | 3422 | — | — | — | — | — | — | × |
| 47 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28` | 5354 | 5354 | 3653 | — | — | — | — | — | — | × |
| 48 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48` | 2717 | 2717 | 2310 | — | — | — | — | — | — | × |
| 49 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01` | 2240 | 2245 | 1225 | — | — | — | — | — | — | × |
| 50 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48` | 633 | 1018 | 2030 | — | — | — | — | — | — | × |
| 51 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31` | 4647 | 4677 | 2206 | — | — | — | — | — | — | × |
| 52 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34` | 5294 | 5293 | 2955 | — | — | — | — | — | — | × |
| 53 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14` | 2950 | 2958 | 1675 | — | — | — | — | — | — | × |
| 54 | `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15` | 1306 | 1306 | 1019 | 1041 | 15 | 0 | — | 5279 | 3 | **○** |
| 55 | `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13` | 1574 | 1574 | 1577 | 758 | 0 | 819 | — | 6805 | 3 | **○** |
| 56 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43` | 1 | 0 | 843 | 1 | 0 | 843 | — | 1716 | 6 | **○** |
| 57 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46` | 1 | 0 | 948 | 0 | 0 | 948 | — | 1354 | 3 | **○** |
| 58 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10` | 73 | 72 | 761 | 82 | 0 | 761 | — | 1935 | 3 | **○** |
| 59 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44` | 95 | 95 | 507 | 24 | 0 | 507 | — | 1185 | 3 | **○** |
| 60 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02` | 606 | 621 | 1734 | 152 | 39 | 1729 | — | 4254 | 3 | **○** |
| 61 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36` | 2353 | 2353 | 2877 | 1153 | 374 | 1494 | — | 10669 | 3 | **○** |
| 62 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38` | 79 | 79 | 612 | 112 | 0 | 588 | — | 1234 | 3 | **○** |
| 63 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23` | 124 | 124 | 373 | 77 | 0 | 356 | — | 1066 | 3 | **○** |
| 64 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27` | 4131 | 4131 | 5365 | 3905 | 9 | 1557 | — | 16897 | 3 | **○** |
| 65 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26` | 8303 | 8310 | 7458 | 6598 | 381 | 587 | — | 33589 | 3 | **○** |
| 66 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires` | 8303 | 8310 | 7458 | 6598 | 381 | 587 | — | 33589 | 3 | **○** |
| 67 | `~/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51` | 2317 | 2318 | 1621 | 1218 | 1 | 576 | — | 9369 | 3 | **○** |
| 68 | `~/bags/m8-static-check-20261113/fixed_1` | — | — | — | — | — | — | — | — | 3 | × |
| 69 | `~/bags/m8-static-check-20261113/fixed_2_place2` | — | — | — | — | — | — | — | — | 3 | × |
| 70 | `~/bags/m8-static-check-20261113/fixed_3_place2_rot180` | — | — | — | — | — | — | — | — | 3 | × |
| 71 | `~/bags/m8-static-check-20261113/fixed_4_place2_rot180_after_drive` | — | — | — | — | — | — | — | — | 3 | × |
| 72 | `~/bags/m8-static-check-20261113/fixed_5_place3_rot180` | — | — | — | — | — | — | — | — | 3 | × |
| 73 | `~/bags/m8-static-check-20261113/flat_1` | — | — | — | — | — | — | — | — | 3 | × |
| 74 | `~/bags/m8-static-check-20261113/flat_2_rot180` | — | — | — | — | — | — | — | — | 3 | × |
| 75 | `~/bags/m8-static-check-20261113/flat_3_seated` | — | — | — | — | — | — | — | — | 3 | × |
| 76 | `~/bags/sensor_check_20261028/chk_bottle_2m_160742` | — | — | — | — | — | — | — | 382 | 4 | × |
| 77 | `~/bags/sensor_check_20261028/chk_bottle_3m_160831` | — | — | — | — | — | — | — | 351 | 4 | × |
| 78 | `~/bags/sensor_check_20261028/chk_bottle_4m_160903` | — | — | — | — | — | — | — | 293 | 4 | × |
| 79 | `~/bags/sensor_check_20261028/chk_bottle_5m_160929` | — | — | — | — | — | — | — | 306 | 4 | × |
| 80 | `~/bags/sensor_check_20261028/chk_chair_1.5m_160152` | — | — | — | — | — | — | — | 657 | 4 | × |
| 81 | `~/bags/sensor_check_20261028/chk_chair_1.5m_160421` | — | — | — | — | — | — | — | 348 | 4 | × |
| 82 | `~/bags/sensor_check_20261028/chk_chair_2m_160227` | — | — | — | — | — | — | — | 330 | 4 | × |
| 83 | `~/bags/sensor_check_20261028/chk_chair_2m_160458` | — | — | — | — | — | — | — | 321 | 4 | × |
| 84 | `~/bags/sensor_check_20261028/chk_chair_3m_160331` | — | — | — | — | — | — | — | 372 | 4 | × |
| 85 | `~/bags/sensor_check_20261028/chk_chair_3m_160533` | — | — | — | — | — | — | — | 327 | 4 | × |
| 86 | `~/bags/sensor_check_20261028/chk_empty_0m_160726` | — | — | — | — | — | — | — | 343 | 4 | × |
| 87 | `~/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag` | — | — | — | — | — | — | — | — | 1 | × |
| 88 | `~/whill_platform/bags/2026-07-31-campus` | 2317 | 2318 | 1621 | 1218 | 1 | 576 | — | 9369 | 3 | **○** |

数値は件数、— はトピック自体が bag に無いことを表す。

- 3 点が揃う（件数 > 0）のは #54〜#67 と #88 の 15 本。#56・#57 は `/pcl_pose` が 1 通だけで、自己位置としてはほぼ使えない。
- #42〜#53（2026-07-20〜21）は `/pcl_pose` と `/cmd_vel` はあるが `/tf`・`/tf_static` が無い。#44 は `/pcl_pose` が 0 件。
- `/whill/controller/cmd_vel`（車いすへの最終指令）はどの bag にも無い。

## 5. 手動走行と自律走行を区別できる情報

```bash
# 手動/自律の手がかりになるトピックの調査（コード側）
grep -n "create_publisher" src/third_party/ros2_whill/whill_driver/src/whill_node.cpp
cat src/third_party/ros2_whill_interfaces/whill_msgs/msg/ModelCr2State.msg
sed -n 680,705p src/whill_dispatch/whill_dispatch/dispatch_node.py   # /dispatch/state の JSON の中身
git log --format='%h %ad %s' --date=short -S"teleop_active" -- src/whill_dispatch/whill_dispatch/dispatch_node.py | tail -1
# bag 側: analyze.py が cmd_vel 系・/plan・/dispatch/state・/joy 系・/navigate_to_pose/_action/status を読み、
# 件数・非ゼロ件数（|linear.x| または |angular.z| > 1e-4）・最初と最後の時刻・/dispatch/state の phase と teleop_active の遷移を出す
grep -h "Topic:" info_raw.txt | sed -E 's/.*Topic: ([^ ]+) \| Type: ([^ ]+) .*/\1 \2/' | sort | uniq -c
# teleop_active の区間の /cmd_vel を数える（#53 のみ。win.py は付録 D）
bash run_win.sh ~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14
```

```
64:  states_model_cr2_pub_ = this->create_publisher<whill_msgs::msg::ModelCr2State>(
65:    "/whill/states/model_cr2", 10);
68:  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/whill/odom", 10);
（ModelCr2State.msg）battery_power, battery_current, right/left_motor_angle, right/left_motor_speed, power_on, speed_mode_indicator, error
  → 本体ジョイスティックの入力を表す欄は無い
（dispatch_node.py）snapshot = {job_id, phase, waypoint, progress, queue_len, pose, aligned, fitness, battery, teleop_active}
baab83f 2026-07-21 feat(m7): iPad 手動操縦による救出機能 + code-review 反映
（全 bag に出てくるトピックの種類と本数）
27 /alignment_status diagnostic_msgs/msg/DiagnosticArray
20 /camera/camera/color/camera_info sensor_msgs/msg/CameraInfo
9 /camera/camera/color/image_raw/compressed sensor_msgs/msg/CompressedImage
19 /camera/camera/color/image_raw sensor_msgs/msg/Image
20 /camera/camera/depth/camera_info sensor_msgs/msg/CameraInfo
28 /camera/camera/depth/image_rect_raw sensor_msgs/msg/Image
27 /cmd_vel geometry_msgs/msg/Twist
15 /cmd_vel_nav geometry_msgs/msg/Twist
15 /cmd_vel_safety geometry_msgs/msg/Twist
15 /cmd_vel_teleop geometry_msgs/msg/Twist
12 /dispatch/state std_msgs/msg/String
15 /global_costmap/costmap nav_msgs/msg/OccupancyGrid
27 /imu/data_raw sensor_msgs/msg/Imu
66 /imu/data_rep145 sensor_msgs/msg/Imu
8 /imu/mag sensor_msgs/msg/MagneticField
15 /local_costmap/costmap nav_msgs/msg/OccupancyGrid
15 /local_costmap/published_footprint geometry_msgs/msg/PolygonStamped
27 /odometry/filtered nav_msgs/msg/Odometry
5 /Odometry nav_msgs/msg/Odometry
27 /pcl_pose geometry_msgs/msg/PoseWithCovarianceStamped
20 /plan nav_msgs/msg/Path
20 /rosout rcl_interfaces/msg/Log
15 /scan sensor_msgs/msg/LaserScan
75 /tf_static tf2_msgs/msg/TFMessage
43 /tf tf2_msgs/msg/TFMessage
20 /velodyne_packets velodyne_msgs/msg/VelodyneScan
87 /velodyne_points sensor_msgs/msg/PointCloud2
16 /whill/odom nav_msgs/msg/Odometry
（win.py の出力、#53）
{"teleop_active=true: cmd_vel [n, nonzero]": [185, 151], "teleop_active=false: cmd_vel [n, nonzero]": [1490, 1110]}
```

### 区別に使える手がかり（どの bag にも無いものを含む）

| 手がかり | 意味（根拠） | bag 内の有無 |
|---|---|---|
| `/cmd_vel_nav` | Nav2 の controller_server / behavior_server の出力（`nav_launch.py` のリマップ）＝自律 | #54〜#67, #88 |
| `/cmd_vel_teleop` | iPad の救出用手動操縦（`dispatch_node.py` が `/dispatch/teleop` から変換）＝手動 | #54〜#67, #88（#55 を除く。件数が 0〜1 の bag もある） |
| `/cmd_vel_safety` | failsafe の停止指令（ゼロ速度） | #54〜#67, #88（#54 は 0 件）。非ゼロは 0 件 |
| `/cmd_vel` | twist_mux の出力。どの入力が選ばれたかは含まない | 27 本 |
| `/dispatch/state`（String の JSON） | `phase`＝配車ジョブの状態（ACTIVE はゴール実行中）、`teleop_active`＝手動操縦の ON/OFF（2026-07-21 の baab83f で追加） | #42〜#53。`teleop_active` 欄があるのは #53 だけ |
| `/plan` | Nav2 の経路が出ていた区間 | 20 本 |
| twist_mux の選択・lock のトピック | 設定に lock が無く、選択結果のトピックも記録されていない | なし |
| `/joy`・`/whill/controller/joy` | 記録されていない | なし |
| `/navigate_to_pose/_action/status` | 記録されていない（`record_avoidance_bag.sh` の drive プロファイルでは記録対象） | なし |
| 本体ジョイスティック | `ModelCr2State` に欄が無く、ROS に出ない | 判別不能 |
| bag の名前・記録プロファイル | `record_avoidance_bag.sh` の `<日付>-static` / `<日付>-drive`。static はセンサだけ。drive プロファイルの bag はまだ 1 本も無い | 2026-11-16-static（#1〜#8）、2026-11-16-test（#9） |

### cmd_vel 系がある bag の内訳

括弧内は非ゼロの件数。

| # | bag | `/cmd_vel_nav` 件数（非ゼロ） | `/cmd_vel_teleop` 件数（非ゼロ） | `/cmd_vel_safety` 件数（非ゼロ） | `/cmd_vel` 件数（非ゼロ） | `/plan` 件数 | `/dispatch/state` の phase / teleop_active の内訳 |
|---|---|---|---|---|---|---:|---|
| 42 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41` | — | — | — | 826（0） | — | {phase: IDLE}: 3919 |
| 43 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43` | — | — | — | 2591（0） | — | {phase: IDLE}: 1288 |
| 44 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08` | — | — | — | 637（0） | — | {phase: IDLE}: 1355 |
| 45 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27` | — | — | — | 4531（3923） | — | {phase: IDLE}: 720<br>{phase: QUEUED}: 4<br>{phase: ACTIVE}: 2063<br>{phase: ABORTED}: 384 |
| 46 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07` | — | — | — | 3422（2889） | — | {phase: IDLE}: 517<br>{phase: QUEUED}: 2<br>{phase: ACTIVE}: 1522<br>{phase: ABORTED}: 278 |
| 47 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28` | — | — | — | 3653（2571） | — | {phase: IDLE}: 948<br>{phase: QUEUED}: 2<br>{phase: ACTIVE}: 1315<br>{phase: SUCCEEDED}: 454 |
| 48 | `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48` | — | — | — | 2310（2309） | — | {phase: IDLE}: 97<br>{phase: QUEUED}: 2<br>{phase: ACTIVE}: 1157<br>{phase: SUCCEEDED}: 125 |
| 49 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01` | — | — | — | 1225（1134） | 88 | {phase: IDLE}: 616<br>{phase: QUEUED}: 2<br>{phase: ACTIVE}: 578<br>{phase: CANCELED}: 87 |
| 50 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48` | — | — | — | 2030（0） | — | {phase: IDLE}: 571 |
| 51 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31` | — | — | — | 2206（2107） | 166 | {phase: IDLE}: 579<br>{phase: QUEUED}: 8<br>{phase: ACTIVE}: 1247<br>{phase: SUCCEEDED}: 321<br>{phase: ABORTED}: 392 |
| 52 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34` | — | — | — | 2955（2825） | 218 | {phase: IDLE}: 1100<br>{phase: QUEUED}: 6<br>{phase: ACTIVE}: 1577<br>{phase: SUCCEEDED}: 169<br>{phase: ABORTED}: 136 |
| 53 | `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14` | — | — | — | 1675（1261） | 96 | {phase: IDLE, teleop_active: False}: 650<br>{phase: QUEUED, teleop_active: False}: 6<br>{phase: ACTIVE, teleop_active: False}: 654<br>{phase: SUCCEEDED, teleop_active: False}: 206<br>{phase: SUCCEEDED, teleop_active: True}: 186 |
| 54 | `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15` | 1041（1038） | 15（0） | — | 1019（1001） | 83 | — |
| 55 | `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13` | 758（755） | — | 819（0） | 1577（755） | 58 | — |
| 56 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43` | 1（0） | — | 843（0） | 843（0） | — | — |
| 57 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46` | — | — | 948（0） | 948（0） | — | — |
| 58 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10` | 82（81） | — | 761（0） | 761（0） | 6 | — |
| 59 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44` | 24（23） | — | 507（0） | 507（0） | 2 | — |
| 60 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02` | 152（142） | 39（24） | 1729（0） | 1734（5） | 15 | — |
| 61 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36` | 1153（1150） | 374（338） | 1494（0） | 2877（1377） | 93 | — |
| 62 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38` | 112（111） | — | 588（0） | 612（24） | 8 | — |
| 63 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23` | 77（75） | — | 356（0） | 373（26） | 6 | — |
| 64 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27` | 3905（3900） | 9（6） | 1557（0） | 5365（3804） | 303 | — |
| 65 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26` | 6598（6566） | 381（343） | 587（0） | 7458（6824） | 528 | — |
| 66 | `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires` | 6598（6566） | 381（343） | 587（0） | 7458（6824） | 528 | — |
| 67 | `~/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51` | 1218（1216） | 1（0） | 576（0） | 1621（1044） | 93 | — |
| 88 | `~/whill_platform/bags/2026-07-31-campus` | 1218（1216） | 1（0） | 576（0） | 1621（1044） | 93 | — |

時刻（bag 開始からの秒）:

- #42 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41`:  `/dispatch/state` の遷移: IDLE@0.2s
- #43 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43`:  `/dispatch/state` の遷移: IDLE@0.1s
- #44 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08`:  `/dispatch/state` の遷移: IDLE@0.0s
- #45 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27`:  `/dispatch/state` の遷移: IDLE@0.2s → QUEUED@144.1s → ACTIVE@144.1s → ABORTED@473.8s → QUEUED@544.8s → ACTIVE@544.8s → ABORTED@627.3s
- #46 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07`:  `/dispatch/state` の遷移: IDLE@0.1s → QUEUED@103.4s → ACTIVE@103.4s → ABORTED@407.6s
- #47 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28`:  `/dispatch/state` の遷移: IDLE@0.0s → QUEUED@189.6s → ACTIVE@189.6s → SUCCEEDED@452.4s
- #48 `~/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48`:  `/dispatch/state` の遷移: IDLE@0.2s → QUEUED@19.4s → ACTIVE@19.4s → SUCCEEDED@250.7s
- #49 `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01`:  `/dispatch/state` の遷移: IDLE@0.1s → QUEUED@123.1s → ACTIVE@123.1s → CANCELED@238.7s
- #50 `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48`:  `/dispatch/state` の遷移: IDLE@0.1s
- #51 `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31`:  `/dispatch/state` の遷移: IDLE@0.1s → QUEUED@115.8s → ACTIVE@115.8s → SUCCEEDED@171.1s → QUEUED@203.2s → ACTIVE@203.2s → SUCCEEDED@313.5s → QUEUED@345.4s → ACTIVE@345.4s → ABORTED@399.7s → QUEUED@464.4s → ACTIVE@464.4s → ABORTED@492.7s
- #52 `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34`:  `/dispatch/state` の遷移: IDLE@0.1s → QUEUED@219.9s → ACTIVE@219.9s → SUCCEEDED@350.6s → QUEUED@363.4s → ACTIVE@363.4s → SUCCEEDED@502.1s → QUEUED@522.8s → ACTIVE@522.8s → ABORTED@568.2s
- #53 `~/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14`:  `/dispatch/state` の遷移: IDLE@0.1s → QUEUED@130.0s → ACTIVE@130.0s → SUCCEEDED@169.3s → QUEUED@183.2s → ACTIVE@183.2s → SUCCEEDED@207.0s → SUCCEEDED(teleop)@216.9s → SUCCEEDED@230.6s → QUEUED@244.0s → ACTIVE@244.0s → SUCCEEDED@311.0s → SUCCEEDED(teleop)@314.4s
- #54 `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 27.0–132.4 s、`/cmd_vel_teleop` 125.0–130.2 s。
- #55 `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 37.3–112.8 s、`/cmd_vel_safety` 117.4–174.4 s。
- #56 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 41.1–41.1 s、`/cmd_vel_safety` 0.0–42.1 s。
- #57 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_safety` 15.8–63.1 s。
- #58 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 25.1–33.5 s、`/cmd_vel_safety` 15.7–62.0 s。
- #59 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 17.3–19.9 s、`/cmd_vel_safety` 11.1–36.4 s。
- #60 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 34.7–100.0 s、`/cmd_vel_teleop` 93.7–103.5 s、`/cmd_vel_safety` 16.9–121.5 s。
- #61 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 48.1–267.8 s、`/cmd_vel_teleop` 118.1–255.5 s、`/cmd_vel_safety` 71.1–277.2 s。
- #62 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 6.9–18.3 s、`/cmd_vel_safety` 9.2–38.5 s。
- #63 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 11.2–25.7 s、`/cmd_vel_safety` 13.8–37.5 s。
- #64 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 25.6–429.8 s、`/cmd_vel_teleop` 431.9–452.4 s、`/cmd_vel_safety` 419.9–497.7 s。
- #65 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 73.5–814.0 s、`/cmd_vel_teleop` 8.4–215.2 s、`/cmd_vel_safety` 0.0–835.3 s。
- #66 `~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 73.5–814.0 s、`/cmd_vel_teleop` 8.4–215.2 s、`/cmd_vel_safety` 0.0–835.3 s。
- #67 `~/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 101.9–223.5 s、`/cmd_vel_teleop` 156.4–156.4 s、`/cmd_vel_safety` 52.4–235.1 s。
- #88 `~/whill_platform/bags/2026-07-31-campus`: 最初と最後の受信（bag 開始からの秒）: `/cmd_vel_nav` 101.9–223.5 s、`/cmd_vel_teleop` 156.4–156.4 s、`/cmd_vel_safety` 52.4–235.1 s。

### bag ごとに言えること

- **#54〜#67、#88（2026-07-22〜07-31）**: twist_mux の入力が別々に記録されているので、`/cmd_vel_nav`（自律）と `/cmd_vel_teleop`（手動）が出ていた時刻を分けられる。teleop の非ゼロがあるのは #60（24 件）・#61（338 件）・#64（6 件）・#65/#66（343 件）。#54・#67/#88 の teleop は全部ゼロ（ボタンを離した状態など）、#55〜#59・#62・#63 には teleop が無いか 0 件。同じ時刻に両方が出ていたとき実際にどちらが通ったかは記録が無く、twist_mux の優先度（teleop 50 > navigation 10、timeout 0.5 s）から推定するしかない。#65/#66 は teleop が 8.4〜215.2 s、nav が 73.5〜814.0 s で重なる区間がある。
- **#45〜#49、#51〜#53（2026-07-20〜21）**: `/cmd_vel` は mux 出力だけ。`/dispatch/state` の `phase` が ACTIVE の区間は Nav2 のゴール実行中と言える。ただしこの時期に teleop 機能があったのは 07-21 以降で、`teleop_active` 欄があるのは #53 だけ。#53 では `teleop_active=true` の区間に `/cmd_vel` が 185 件（非ゼロ 151 件）あり、手動操縦の区間を切り出せる。#45〜#52 には teleop の欄が無いが、機能の追加前（07-20）または欄の追加前の bag なので、ACTIVE 以外で `/cmd_vel` が非ゼロの区間が何によるものかは bag からは決められない。
- **#42〜#44、#50**: `/cmd_vel` はすべてゼロ、`phase` は IDLE のまま。走行指令は出ていない。
- **#56・#57**: `/cmd_vel_safety` と `/cmd_vel` がすべてゼロ（failsafe の停止）で、`/cmd_vel_nav` は 0〜1 件。走行していない。
- **それ以外の 61 本（#1〜#41、#68〜#87）**: cmd_vel 系・`/dispatch/state`・`/plan` が無い。手動か自律かは bag からは判別できない。ディレクトリ名（`static`・`static-check`・`sensor_check`・`indoor-calib` など）は静止や校正を示唆するが、名前は根拠として弱く、動いたかどうかは点群やオドメトリの解析が要る（本調査ではしていない）。

## 6. GPU と空き容量

```bash
nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv
df -h / /home/systemlab /home/systemlab/bags /home/systemlab/whill_platform/bags /tmp
```

```
name, memory.total [MiB], memory.used [MiB]
NVIDIA GeForce RTX 3080 Ti Laptop GPU, 16384 MiB, 375 MiB

Filesystem      Size  Used Avail Use% Mounted on
/dev/nvme0n1p7  993G  382G  561G  41% /
/dev/nvme0n1p7  993G  382G  561G  41% /
/dev/nvme0n1p7  993G  382G  561G  41% /
/dev/nvme0n1p7  993G  382G  561G  41% /
/dev/nvme0n1p7  993G  382G  561G  41% /
```

`/`・home・`~/bags`・`~/whill_platform/bags`・`/tmp` はすべて同じパーティション。空きは 561G で、今ある bag の合計（274G）の約 2 倍。

## 7. `scripts/record_avoidance_bag.sh` がこの環境でそのまま動くか

スクリプトは `--help` だけが記録をせずに終わる（使い方を表示して exit 0）。`--dry-run` は無い。それ以外の起動は事前確認に通れば `ros2 bag record` まで進むので実行していない。事前確認の各項目を、スクリプトと同じ判定で個別に確かめた（`check7.sh`、付録 D）。入力トピックの確認（`ros2 topic echo --once`）はノードを作るため実行せず、代わりに `pgrep` でロボットのノードが動いていないことを確かめた。

```bash
bash check7.sh > check7.out 2>&1   # 出力のネットワーク関係の値は <redacted> にした
```

```
[0] bash scripts/record_avoidance_bag.sh --help
使用法: scripts/record_avoidance_bag.sh [オプション] <タグ>
  --profile static|drive  static: センサだけ（#149 静止確認） / drive: 走行一式（既定、#156）
  --run NAME              1 回の試験をまとめるディレクトリ名（既定: <YYYY-MM-DD>-<profile>）
  --duration SEC          指定秒数で自動停止（省略時は Ctrl-C で停止）
環境変数 RECORD_INPUT_TIMEOUT_S: 入力トピックの待ち時間（既定 8 秒）
  -h, --help              この説明を表示

保存先: ${WHILL_BAGS_ROOT:-~/bags}/<run>/<タグ>_<HHMMSS>/（mcap）。bag はリポジトリに置かない。
static: m6r_bringup_launch.py realsense:=true を起動しておく（D435 の深度・カラーが必須）。
drive : m6r_bringup + nav_launch を起動しておく。D435 は任意（無ければ警告して続ける）。
static の各 bag の後に、<run>/notes.csv に追記する行の雛形を表示する
（列は check_object_placement.py の bag,object,x_m,height_m,shape,expect,...。
何も置かない bag は object=empty で x_m・height_m を空にする）。
rc=0
[1a] RMW (before sourcing) = rmw_cyclonedds_cpp
[2a] CYCLONEDDS_URI set: yes; basename=cyclonedds-runtime-min.xml; readable=yes
[2b] repo configs/cyclonedds-runtime-min.xml exists: yes
[2c] CYCLONEDDS_URI points to same file as repo main checkout: yes
[2d] check_dds_nic.py:
XML の NIC が実機にありません: <redacted>（実機: <redacted>）
<redacted> を持つ NIC がありません。LiDAR の USB-LAN の接続を確認してください
rc=1
[3] governor: performance
     20 performance
[4a] ros2 before sourcing: /opt/ros/humble/bin/ros2
[4b] ros2 after sourcing /opt/ros/humble: /opt/ros/humble/bin/ros2
[1b] RMW after sourcing = rmw_cyclonedds_cpp
[4c] rosbag2_storage_mcap: /opt/ros/humble
[4d] compressed_image_transport: /opt/ros/humble
[4e] rmw_cyclonedds_cpp: /opt/ros/humble
[4f] velodyne_msgs (probe 型): Package not found
[4g] velodyne_msgs after workspace: /home/systemlab/whill_lab0_ros2/install/velodyne_msgs
[5] helpers:
  timeout  /usr/bin/timeout
  awk      /usr/bin/awk
  sed      /usr/bin/sed
  grep     /usr/bin/grep
  date     /usr/bin/date
  mkdir    /usr/bin/mkdir
  python3  /usr/bin/python3
  ip       /usr/sbin/ip
  timeout --foreground supported lines: 1
  -rwxrwxr-x /home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/scripts/check_dds_nic.py
  -rwxrwxr-x /home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/scripts/check_failsafe_inputs.sh
  -rwxrwxr-x /home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/scripts/record_avoidance_bag.sh
[6] output dir: WHILL_BAGS_ROOT=未設定 -> /home/systemlab/bags; writable=yes
Filesystem      Size  Used Avail Use% Mounted on
/dev/nvme0n1p7  993G  382G  561G  41% /
[7] running robot/ROS processes (pgrep, read-only):
  (none)
```

### 依存と前提条件ごとの判定

| # | スクリプトの確認（行の内容） | 調べた結果 | 判定 |
|---|---|---|---|
| 1 | `RMW_IMPLEMENTATION == rmw_cyclonedds_cpp` | この端末の環境で rmw_cyclonedds_cpp。`rmw_cyclonedds_cpp` パッケージもある | 合格 |
| 2 | `CYCLONEDDS_URI` が設定済みで読め、ファイル名が `cyclonedds-runtime-min.xml` | 設定済み・読める・ファイル名一致・リポジトリの `configs/cyclonedds-runtime-min.xml` と同じファイル | 合格 |
| 3 | `scripts/check_dds_nic.py "$XML"` が 0 で終わる（XML の NIC が実機にあり、LiDAR のサブネットの NIC が XML に入っている） | rc=1。XML の NIC がこのPCに無く、LiDAR のサブネットを持つ NIC も無い（LiDAR の USB-LAN が未接続） | **不合格（今は）** |
| 4 | CPU governor が performance | 全 20 CPU が performance | 合格 |
| 5 | `ros2` が PATH にある | `/opt/ros/humble/bin/ros2` | 合格 |
| 6 | `ros2 pkg prefix rosbag2_storage_mcap` | `/opt/ros/humble` | 合格 |
| 7 | `ros2 pkg prefix compressed_image_transport`（static では必須、drive では警告） | `/opt/ros/humble` | 合格 |
| 8 | 入力の型が解決できる（`ros2 topic echo` に型を明示する） | `velodyne_msgs` は `/opt/ros/humble` だけでは見つからず、`~/whill_lab0_ros2/install/setup.bash` を source すると見つかる | 条件付き合格（ワークスペースの source が要る） |
| 9 | 必須入力 `/velodyne_packets`・`/velodyne_points`・`/imu/data_rep145`・`/tf_static`（drive では `/odometry/filtered` も）を各 8 秒以内に 1 通受信 | ロボットのノードは動いていない（`pgrep` で該当なし）。今実行すれば最初の入力で止まる | **不合格（今は）。現地で bringup 後に確認** |
| 10 | カメラ入力（static では必須、drive では警告） | 9 と同じ理由で今は受信できない | 9 と同じ |
| 11 | drive: `scripts/check_failsafe_inputs.sh` が 0 で終わる（`/alignment_status`・`/pcl_pose`・`/velodyne_points_no_ground`・`/scan`） | スクリプトは存在し実行権あり。nav_launch が動いていないので今は通らない | **不合格（今は）** |
| 12 | 外部コマンド `timeout`（`--foreground`）・`awk`・`sed`・`grep`・`date`・`mkdir`・`python3`、`check_dds_nic.py` が使う `ip` | すべてある。`timeout --foreground` も使える | 合格 |
| 13 | 保存先 `${WHILL_BAGS_ROOT:-~/bags}` に `mkdir -p` できる | `WHILL_BAGS_ROOT` 未設定 → `~/bags`、書き込み可 | 合格 |
| 14 | 空き容量 | スクリプトは確認しない。今の空きは 561G（過去の実走 bag（m7）は 1 本あたり 44M〜5.3G、カメラ付きはこれより増える） | 確認なし（今の空きは十分） |

結論: 環境側（RMW・DDS XML の指定・governor・プラグイン・保存先・外部コマンド）はそろっている。今の状態で実行すると 3 の `check_dds_nic.py` で止まり、そこを越えても 9 の入力確認で止まる。どちらも LiDAR の USB-LAN をつなぎ、`m6r_bringup`（drive では `nav_launch` も）を起動した現地でのみ満たせる前提条件。実行する端末では `/opt/ros/humble/setup.bash` に加えて `~/whill_lab0_ros2/install/setup.bash` を source する必要がある（`velodyne_msgs` の型を解決するため）。

---

## 付録 A. `ros2 bag info` の生の出力（88 本、省略なし）

```
### /home/systemlab/bags/2026-11-16-static/cone_1p5_141326
$ ros2 bag info /home/systemlab/bags/2026-11-16-static/cone_1p5_141326

Files:             cone_1p5_141326_0.mcap
Bag size:          385.7 MiB
Storage id:        mcap
Duration:          11.844292065s
Start:             Nov 16 2026 14:13:26.912532752 (1794806006.912532752)
End:               Nov 16 2026 14:13:38.756824817 (1794806018.756824817)
Messages:          4374
Topic information: Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 115 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 352 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 352 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 352 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1174 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 356 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 352 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 116 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1185 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 16 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/2026-11-16-static/cone_2p5_141549
$ ros2 bag info /home/systemlab/bags/2026-11-16-static/cone_2p5_141549

Files:             cone_2p5_141549_0.mcap
Bag size:          389.6 MiB
Storage id:        mcap
Duration:          11.839043055s
Start:             Nov 16 2026 14:15:49.720038315 (1794806149.720038315)
End:               Nov 16 2026 14:16:01.559081370 (1794806161.559081370)
Messages:          4394
Topic information: Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 352 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1184 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 355 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 117 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1184 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 355 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 355 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 16 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 355 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 117 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/2026-11-16-static/cone_2p_141506
$ ros2 bag info /home/systemlab/bags/2026-11-16-static/cone_2p_141506

Files:             cone_2p_141506_0.mcap
Bag size:          386.1 MiB
Storage id:        mcap
Duration:          11.851269417s
Start:             Nov 16 2026 14:15:06.359508322 (1794806106.359508322)
End:               Nov 16 2026 14:15:18.210777739 (1794806118.210777739)
Messages:          4365
Topic information: Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 116 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 352 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1175 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 352 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 353 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1175 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 353 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 117 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 352 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 16 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/2026-11-16-static/cone_3p5_141752
$ ros2 bag info /home/systemlab/bags/2026-11-16-static/cone_3p5_141752

Files:             cone_3p5_141752_0.mcap
Bag size:          389.5 MiB
Storage id:        mcap
Duration:          11.837121879s
Start:             Nov 16 2026 14:17:53.073984570 (1794806273.073984570)
End:               Nov 16 2026 14:18:04.911106449 (1794806284.911106449)
Messages:          4397
Topic information: Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 117 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 117 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1184 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 355 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 355 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 355 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 355 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 16 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1184 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 355 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/2026-11-16-static/cone_3p_141709
$ ros2 bag info /home/systemlab/bags/2026-11-16-static/cone_3p_141709

Files:             cone_3p_141709_0.mcap
Bag size:          385.0 MiB
Storage id:        mcap
Duration:          11.804142540s
Start:             Nov 16 2026 14:17:09.872783339 (1794806229.872783339)
End:               Nov 16 2026 14:17:21.676925879 (1794806241.676925879)
Messages:          4359
Topic information: Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 115 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 351 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 351 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 351 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1170 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 354 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 116 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 351 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1180 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 16 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/2026-11-16-static/cone_4p_141838
$ ros2 bag info /home/systemlab/bags/2026-11-16-static/cone_4p_141838

Files:             cone_4p_141838_0.mcap
Bag size:          389.5 MiB
Storage id:        mcap
Duration:          11.845718341s
Start:             Nov 16 2026 14:18:39.085881287 (1794806319.085881287)
End:               Nov 16 2026 14:18:50.931599628 (1794806330.931599628)
Messages:          4387
Topic information: Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 116 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1174 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 355 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 355 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 355 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 355 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1185 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 117 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 16 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 355 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/2026-11-16-static/cone_5p_141921
$ ros2 bag info /home/systemlab/bags/2026-11-16-static/cone_5p_141921

Files:             cone_5p_141921_0.mcap
Bag size:          389.7 MiB
Storage id:        mcap
Duration:          11.848224143s
Start:             Nov 16 2026 14:19:21.785183118 (1794806361.785183118)
End:               Nov 16 2026 14:19:33.633407261 (1794806373.633407261)
Messages:          4392
Topic information: Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 117 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 355 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 355 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1175 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 117 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 356 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 356 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1185 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 356 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 16 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/2026-11-16-static/empty_start_140915
$ ros2 bag info /home/systemlab/bags/2026-11-16-static/empty_start_140915

Files:             empty_start_140915_0.mcap
Bag size:          389.7 MiB
Storage id:        mcap
Duration:          11.841227679s
Start:             Nov 16 2026 14:09:16.001339753 (1794805756.001339753)
End:               Nov 16 2026 14:09:27.842567432 (1794805767.842567432)
Messages:          4395
Topic information: Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 355 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1184 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 355 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1184 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 355 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 117 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 355 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 355 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 14 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 117 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/2026-11-16-test/smoke_140741
$ ros2 bag info /home/systemlab/bags/2026-11-16-test/smoke_140741

Files:             smoke_140741_0.mcap
Bag size:          160.0 MiB
Storage id:        mcap
Duration:          4.843063743s
Start:             Nov 16 2026 14:07:41.942980892 (1794805661.942980892)
End:               Nov 16 2026 14:07:46.786044635 (1794805666.786044635)
Messages:          1812
Topic information: Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 48 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 146 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 145 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 484 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 146 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 16 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 146 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 484 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 145 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 48 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/imu_investigation_20261030/seg_0_300_hires
$ ros2 bag info /home/systemlab/bags/imu_investigation_20261030/seg_0_300_hires

Files:             seg_0_300_hires_0.db3
Bag size:          1.8 GiB
Storage id:        sqlite3
Duration:          299.998928351s
Start:             Jul 10 2026 15:07:43.918712370 (1783663663.918712370)
End:               Jul 10 2026 15:12:43.917640721 (1783663963.917640721)
Messages:          32961
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2957 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 30000 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/imu_investigation_20261030/seg_0_300_orig
$ ros2 bag info /home/systemlab/bags/imu_investigation_20261030/seg_0_300_orig

Files:             seg_0_300_orig_0.db3
Bag size:          1.8 GiB
Storage id:        sqlite3
Duration:          299.998928351s
Start:             Jul 10 2026 15:07:43.918712370 (1783663663.918712370)
End:               Jul 10 2026 15:12:43.917640721 (1783663963.917640721)
Messages:          32961
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2957 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 30000 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m3-bench-data/m3_chair_motion_2026-05-07
$ ros2 bag info /home/systemlab/bags/m3-bench-data/m3_chair_motion_2026-05-07

Files:             m3_chair_motion_2026-05-07_0.db3
Bag size:          5.3 GiB
Storage id:        sqlite3
Duration:          96.857189845s
Start:             May  7 2026 22:16:53.562470446 (1778159813.562470446)
End:               May  7 2026 22:18:30.419660291 (1778159910.419660291)
Messages:          26137
Topic information: Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 2904 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 2904 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 9687 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 951 | Serialization Format: cdr
                   Topic: /imu/mag | Type: sensor_msgs/msg/MagneticField | Count: 9687 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m3-bench-data/m3_chair_static_2026-05-07
$ ros2 bag info /home/systemlab/bags/m3-bench-data/m3_chair_static_2026-05-07

Files:             m3_chair_static_2026-05-07_0.db3
Bag size:          1.1 GiB
Storage id:        sqlite3
Duration:          19.850282091s
Start:             May  7 2026 22:07:41.781267379 (1778159261.781267379)
End:               May  7 2026 22:08:01.631549470 (1778159281.631549470)
Messages:          5362
Topic information: Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 595 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 595 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1986 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 196 | Serialization Format: cdr
                   Topic: /imu/mag | Type: sensor_msgs/msg/MagneticField | Count: 1986 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m3-bench-data/m3_smoke_2026-05-07
$ ros2 bag info /home/systemlab/bags/m3-bench-data/m3_smoke_2026-05-07

Files:             m3_smoke_2026-05-07_0.db3
Bag size:          436.9 MiB
Storage id:        sqlite3
Duration:          7.834444034s
Start:             May  7 2026 21:38:57.511399105 (1778157537.511399105)
End:               May  7 2026 21:39:05.345843139 (1778157545.345843139)
Messages:          2115
Topic information: Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 235 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 234 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 784 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 77 | Serialization Format: cdr
                   Topic: /imu/mag | Type: sensor_msgs/msg/MagneticField | Count: 784 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08
$ ros2 bag info /home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08

Files:             m4_chair_live_2026-05-08_0.db3
Bag size:          3.3 GiB
Storage id:        sqlite3
Duration:          59.838429808s
Start:             May  8 2026 10:53:52.369641627 (1778205232.369641627)
End:               May  8 2026 10:54:52.208071435 (1778205292.208071435)
Messages:          16246
Topic information: Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 48 | Serialization Format: cdr
                   Topic: /imu/mag | Type: sensor_msgs/msg/MagneticField | Count: 5984 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 1794 | Serialization Format: cdr
                   Topic: /Odometry | Type: nav_msgs/msg/Odometry | Count: 48 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 1794 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 5984 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 590 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run1
$ ros2 bag info /home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run1

Files:             m4_chair_live_2026-05-08_run1_0.db3
Bag size:          3.2 GiB
Storage id:        sqlite3
Duration:          59.501778520s
Start:             May  8 2026 11:10:14.947801802 (1778206214.947801802)
End:               May  8 2026 11:11:14.449580322 (1778206274.449580322)
Messages:          16240
Topic information: Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 88 | Serialization Format: cdr
                   Topic: /Odometry | Type: nav_msgs/msg/Odometry | Count: 88 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 1784 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 1785 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 5952 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 587 | Serialization Format: cdr
                   Topic: /imu/mag | Type: sensor_msgs/msg/MagneticField | Count: 5952 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run2
$ ros2 bag info /home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run2

Files:             m4_chair_live_2026-05-08_run2_0.db3
Bag size:          3.2 GiB
Storage id:        sqlite3
Duration:          59.502864352s
Start:             May  8 2026 11:12:15.940332781 (1778206335.940332781)
End:               May  8 2026 11:13:15.443197133 (1778206395.443197133)
Messages:          16212
Topic information: Topic: /Odometry | Type: nav_msgs/msg/Odometry | Count: 81 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 1784 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 1781 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 577 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 5952 | Serialization Format: cdr
                   Topic: /imu/mag | Type: sensor_msgs/msg/MagneticField | Count: 5952 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 81 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run3
$ ros2 bag info /home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run3

Files:             m4_chair_live_2026-05-08_run3_0.db3
Bag size:          3.3 GiB
Storage id:        sqlite3
Duration:          59.839853600s
Start:             May  8 2026 11:14:52.993765624 (1778206492.993765624)
End:               May  8 2026 11:15:52.833619224 (1778206552.833619224)
Messages:          16172
Topic information: Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 108 | Serialization Format: cdr
                   Topic: /Odometry | Type: nav_msgs/msg/Odometry | Count: 108 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 1794 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 1794 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 5887 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 590 | Serialization Format: cdr
                   Topic: /imu/mag | Type: sensor_msgs/msg/MagneticField | Count: 5887 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m3-bench-data/m5_chair_mapping_2026-05-08
$ ros2 bag info /home/systemlab/bags/m3-bench-data/m5_chair_mapping_2026-05-08

Files:             m5_chair_mapping_2026-05-08_0.db3
Bag size:          6.5 GiB
Storage id:        sqlite3
Duration:          119.832125215s
Start:             May  8 2026 12:11:45.829222336 (1778209905.829222336)
End:               May  8 2026 12:13:45.661347551 (1778210025.661347551)
Messages:          32724
Topic information: Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 290 | Serialization Format: cdr
                   Topic: /imu/mag | Type: sensor_msgs/msg/MagneticField | Count: 11886 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 1182 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 11886 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 3592 | Serialization Format: cdr
                   Topic: /Odometry | Type: nav_msgs/msg/Odometry | Count: 290 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 3592 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 6 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag

Files:             bag_0.db3
Bag size:          1.2 GiB
Storage id:        sqlite3
Duration:          199.297559061s
Start:             Jun 21 2026 13:38:02.455624281 (1782016682.455624281)
End:               Jun 21 2026 13:41:21.753183342 (1782016881.753183342)
Messages:          27879
Topic information: Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 19931 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 1965 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 5979 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag-imu-fixed
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag-imu-fixed

Files:             bag-imu-fixed_0.db3
Bag size:          1.2 GiB
Storage id:        sqlite3
Duration:          199.297559061s
Start:             Jun 21 2026 13:38:02.455624281 (1782016682.455624281)
End:               Jun 21 2026 13:41:21.753183342 (1782016881.753183342)
Messages:          27879
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 5979 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 1965 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 19931 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag

Files:             bag_0.db3
Bag size:          1.4 GiB
Storage id:        sqlite3
Duration:          239.446921453s
Start:             Jun 24 2026 13:49:42.037883645 (1782276582.037883645)
End:               Jun 24 2026 13:53:41.484805098 (1782276821.484805098)
Messages:          26310
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 23945 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2361 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-07-campus-half-v3/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-07-campus-half-v3/bag

Files:             bag_0.db3
Bag size:          2.8 GiB
Storage id:        sqlite3
Duration:          466.260125959s
Start:             Jul  7 2026 19:30:40.129157468 (1783420240.129157468)
End:               Jul  7 2026 19:38:26.389283427 (1783420706.389283427)
Messages:          51220
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 4590 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 46626 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-07-campus-loop/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-07-campus-loop/bag

Files:             bag_0.db3
Bag size:          4.7 GiB
Storage id:        sqlite3
Duration:          821.453343469s
Start:             Jul  7 2026 19:08:32.923778807 (1783418912.923778807)
End:               Jul  7 2026 19:22:14.377122276 (1783419734.377122276)
Messages:          88669
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 7844 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 80821 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-postgap/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-postgap/bag

Files:             bag_0.db3
Bag size:          10.2 GiB
Storage id:        sqlite3
Duration:          1817.671593993s
Start:             Jul  8 2026 16:33:02.713583776 (1783495982.713583776)
End:               Jul  8 2026 17:03:20.385177769 (1783497800.385177769)
Messages:          187770
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 12 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 170909 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 16849 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-pregap/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-pregap/bag

Files:             bag_0.db3
Bag size:          11.1 GiB
Storage id:        sqlite3
Duration:          1864.494849877s
Start:             Jul  8 2026 16:00:22.000533605 (1783494022.000533605)
End:               Jul  8 2026 16:31:26.495383482 (1783495886.495383482)
Messages:          204829
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 0 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 186449 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 18380 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-first10/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-first10/bag

Files:             bag_0.db3
Bag size:          3.6 GiB
Storage id:        sqlite3
Duration:          599.579395776s
Start:             Jul  8 2026 18:52:24.414614520 (1783504344.414614520)
End:               Jul  8 2026 19:02:23.994010296 (1783504943.994010296)
Messages:          65883
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 59968 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 5911 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-pregap/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-pregap/bag

Files:             bag_0.db3
Bag size:          15.4 GiB
Storage id:        sqlite3
Duration:          2602.085145648s
Start:             Jul  8 2026 18:52:24.414614520 (1783504344.414614520)
End:               Jul  8 2026 19:35:46.499760168 (1783506946.499760168)
Messages:          284313
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 25577 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 8 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 258728 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2/bag

Files:             bag_0.db3
Bag size:          19.9 GiB
Storage id:        sqlite3
Duration:          3482.547239425s
Start:             Jul  8 2026 18:52:24.414614520 (1783504344.414614520)
End:               Jul  8 2026 19:50:26.961853945 (1783507826.961853945)
Messages:          366318
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 333360 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 20 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 32938 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo/bag

Files:             bag_0.db3
Bag size:          21.3 GiB
Storage id:        sqlite3
Duration:          3778.417638420s
Start:             Jul  8 2026 16:00:21.967539349 (1783494021.967539349)
End:               Jul  8 2026 17:03:20.385177769 (1783497800.385177769)
Messages:          393165
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 35279 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 357866 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 20 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-debiased/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-debiased/bag

Files:             bag_0.db3
Bag size:          16.7 GiB
Storage id:        sqlite3
Duration:          2813.311171606s
Start:             Jul  8 2026 21:05:22.532861237 (1783512322.532861237)
End:               Jul  8 2026 21:52:15.844032843 (1783515135.844032843)
Messages:          308557
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 280858 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 27695 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-A/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-A/bag

Files:             bag_0.db3
Bag size:          7.9 GiB
Storage id:        sqlite3
Duration:          1325.065490763s
Start:             Jul  8 2026 21:05:22.532861237 (1783512322.532861237)
End:               Jul  8 2026 21:27:27.598352000 (1783513647.598352000)
Messages:          145573
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 132507 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 13062 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-B/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-B/bag

Files:             bag_0.db3
Bag size:          8.8 GiB
Storage id:        sqlite3
Duration:          1483.475685843s
Start:             Jul  8 2026 21:27:32.368347000 (1783513652.368347000)
End:               Jul  8 2026 21:52:15.844032843 (1783515135.844032843)
Messages:          162987
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 148350 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 14633 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer/bag

Files:             bag_0.db3
Bag size:          16.7 GiB
Storage id:        sqlite3
Duration:          2813.311171606s
Start:             Jul  8 2026 21:05:22.532861237 (1783512322.532861237)
End:               Jul  8 2026 21:52:15.844032843 (1783515135.844032843)
Messages:          308557
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 280858 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 27695 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-indoor-calib/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-indoor-calib/bag

Files:             bag_0.db3
Bag size:          2.2 GiB
Storage id:        sqlite3
Duration:          365.849648203s
Start:             Jul  8 2026 10:32:15.828510530 (1783474335.828510530)
End:               Jul  8 2026 10:38:21.678158733 (1783474701.678158733)
Messages:          40196
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 3607 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 36585 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-08-outdoor-corrected/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-08-outdoor-corrected/bag

Files:             bag_0.db3
Bag size:          4.4 GiB
Storage id:        sqlite3
Duration:          745.400415540s
Start:             Jul  8 2026 12:22:55.634654316 (1783480975.634654316)
End:               Jul  8 2026 12:35:21.035069856 (1783481721.035069856)
Messages:          81892
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 7348 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 74540 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-09-indoor-calib/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-09-indoor-calib/bag

Files:             bag_0.db3
Bag size:          1.2 GiB
Storage id:        sqlite3
Duration:          209.149863567s
Start:             Jul  9 2026 16:26:54.866675279 (1783582014.866675279)
End:               Jul  9 2026 16:30:24.016538846 (1783582224.016538846)
Messages:          22980
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 20914 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2062 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag

Files:             bag_0.db3
Bag size:          12.8 GiB
Storage id:        sqlite3
Duration:          2161.984223891s
Start:             Jul 10 2026 15:07:43.918712370 (1783663663.918712370)
End:               Jul 10 2026 15:43:45.902936261 (1783665825.902936261)
Messages:          237506
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 216191 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 21311 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires

Files:             bag-hires_0.db3
Bag size:          12.8 GiB
Storage id:        sqlite3
Duration:          2161.984223891s
Start:             Jul 10 2026 15:07:43.918712370 (1783663663.918712370)
End:               Jul 10 2026 15:43:45.902936261 (1783665825.902936261)
Messages:          237506
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 21311 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 216191 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m5r-bench-data/2026-07-10-dds-verify/bag
$ ros2 bag info /home/systemlab/bags/m5r-bench-data/2026-07-10-dds-verify/bag

Files:             bag_0.db3
Bag size:          1.8 GiB
Storage id:        sqlite3
Duration:          303.712693566s
Start:             Jul 10 2026 14:47:26.382947360 (1783662446.382947360)
End:               Jul 10 2026 14:52:30.095640926 (1783662750.095640926)
Messages:          33366
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 30368 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2994 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m6r-bench-data/2026-07-14-verify-campus/bag
$ ros2 bag info /home/systemlab/bags/m6r-bench-data/2026-07-14-verify-campus/bag

Files:             bag_0.db3
Bag size:          3.7 GiB
Storage id:        sqlite3
Duration:          627.703662524s
Start:             Jul 14 2026 13:55:36.166902971 (1784004936.166902971)
End:               Jul 14 2026 14:06:03.870565495 (1784005563.870565495)
Messages:          95179
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 6187 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 62760 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 24663 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 1569 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41

Files:             rosbag2_2026_07_20-15_13_41_0.db3
Bag size:          4.7 GiB
Storage id:        sqlite3
Duration:          783.785092746s
Start:             Jul 20 2026 15:13:41.665185378 (1784528021.665185378)
End:               Jul 20 2026 15:26:45.450278124 (1784528805.450278124)
Messages:          127062
Topic information: Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 6701 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 23363 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 77876 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 3919 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 6699 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 826 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 7678 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43

Files:             rosbag2_2026_07_20-15_29_43_0.db3
Bag size:          1.5 GiB
Storage id:        sqlite3
Duration:          257.560123465s
Start:             Jul 20 2026 15:29:44.057756758 (1784528984.057756758)
End:               Jul 20 2026 15:34:01.617880223 (1784529241.617880223)
Messages:          40555
Topic information: Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 21 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 7727 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 25752 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 1288 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2539 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 2591 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 637 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08

Files:             rosbag2_2026_07_20-15_44_08_0.db3
Bag size:          1.6 GiB
Storage id:        sqlite3
Duration:          270.873453734s
Start:             Jul 20 2026 15:44:08.650242559 (1784529848.650242559)
End:               Jul 20 2026 15:48:39.523696293 (1784530119.523696293)
Messages:          39877
Topic information: Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 0 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 8126 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 27088 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 1355 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2671 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 637 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 0 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27

Files:             rosbag2_2026_07_20-15_59_27_0.db3
Bag size:          3.8 GiB
Storage id:        sqlite3
Duration:          632.601942113s
Start:             Jul 20 2026 15:59:27.624882293 (1784530767.624882293)
End:               Jul 20 2026 16:10:00.226824406 (1784531400.226824406)
Messages:          107887
Topic information: Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 6061 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 18889 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 62962 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 3171 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 6065 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 6208 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 4531 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07

Files:             rosbag2_2026_07_20-17_03_07_0.db3
Bag size:          2.8 GiB
Storage id:        sqlite3
Duration:          462.922195598s
Start:             Jul 20 2026 17:03:07.613544907 (1784534587.613544907)
End:               Jul 20 2026 17:10:50.535740505 (1784535050.535740505)
Messages:          79613
Topic information: Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 4564 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 13888 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 46293 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 2319 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 4563 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 3422 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 4564 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28

Files:             rosbag2_2026_07_20-18_09_28_0.db3
Bag size:          3.3 GiB
Storage id:        sqlite3
Duration:          543.006767426s
Start:             Jul 20 2026 18:09:28.982361202 (1784538568.982361202)
End:               Jul 20 2026 18:18:31.989128628 (1784539111.989128628)
Messages:          93025
Topic information: Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 5354 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 16291 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 54301 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 2719 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 5353 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 3653 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 5354 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48

Files:             rosbag2_2026_07_20-19_58_48_0.db3
Bag size:          1.7 GiB
Storage id:        sqlite3
Duration:          275.533821354s
Start:             Jul 20 2026 19:58:48.439445392 (1784545128.439445392)
End:               Jul 20 2026 20:03:23.973266746 (1784545403.973266746)
Messages:          47662
Topic information: Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 2717 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 8267 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 27554 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 1381 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2716 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 2310 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 2717 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01

Files:             rosbag2_2026_07_21-16_34_01_0.db3
Bag size:          1.5 GiB
Storage id:        sqlite3
Duration:          255.800057087s
Start:             Jul 21 2026 16:34:01.771628869 (1784619241.771628869)
End:               Jul 21 2026 16:38:17.571685956 (1784619497.571685956)
Messages:          42856
Topic information: Topic: /plan | Type: nav_msgs/msg/Path | Count: 88 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 2240 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 7674 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 25580 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 1283 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2521 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 1225 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 2245 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48

Files:             rosbag2_2026_07_21-16_50_48_0.db3
Bag size:          734.2 MiB
Storage id:        sqlite3
Duration:          119.166980930s
Start:             Jul 21 2026 16:50:48.724739797 (1784620248.724739797)
End:               Jul 21 2026 16:52:47.891720727 (1784620367.891720727)
Messages:          20920
Topic information: Topic: /plan | Type: nav_msgs/msg/Path | Count: 0 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 633 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 3576 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 11917 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 571 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 1175 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 2030 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 1018 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31

Files:             rosbag2_2026_07_21-16_53_31_0.db3
Bag size:          3.1 GiB
Storage id:        sqlite3
Duration:          506.126301567s
Start:             Jul 21 2026 16:53:32.053926478 (1784620412.053926478)
End:               Jul 21 2026 17:01:58.180228045 (1784620918.180228045)
Messages:          85029
Topic information: Topic: /plan | Type: nav_msgs/msg/Path | Count: 166 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 4647 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 15184 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 50613 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 2547 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 4989 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 2206 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 4677 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34

Files:             rosbag2_2026_07_21-17_20_34_0.db3
Bag size:          3.6 GiB
Storage id:        sqlite3
Duration:          595.139138775s
Start:             Jul 21 2026 17:20:34.997606606 (1784622034.997606606)
End:               Jul 21 2026 17:30:30.136745381 (1784622630.136745381)
Messages:          99958
Topic information: Topic: /plan | Type: nav_msgs/msg/Path | Count: 218 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 5294 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 17853 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 59491 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 2988 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 5866 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 2955 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 5293 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14

Files:             rosbag2_2026_07_21-18_29_14_0.db3
Bag size:          2.0 GiB
Storage id:        sqlite3
Duration:          337.380523968s
Start:             Jul 21 2026 18:29:14.230012670 (1784626154.230012670)
End:               Jul 21 2026 18:34:51.610536638 (1784626491.610536638)
Messages:          56566
Topic information: Topic: /plan | Type: nav_msgs/msg/Path | Count: 96 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 2950 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 10121 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 33738 | Serialization Format: cdr
                   Topic: /dispatch/state | Type: std_msgs/msg/String | Count: 1702 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 3326 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 1675 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 2958 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15

Files:             rosbag2_2026_07_22-18_01_15_0.db3
Bag size:          875.9 MiB
Storage id:        sqlite3
Duration:          132.461762187s
Start:             Jul 22 2026 18:01:15.535629800 (1784710875.535629800)
End:               Jul 22 2026 18:03:27.997391987 (1784711007.997391987)
Messages:          31035
Topic information: Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 662 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 156 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 1306 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 83 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 1306 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 13247 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 331 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 5279 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 1306 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 1019 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 3974 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 1306 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 1041 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 0 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 15 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13

Files:             rosbag2_2026_07_22-18_23_13_0.db3
Bag size:          1.0 GiB
Storage id:        sqlite3
Duration:          174.392238293s
Start:             Jul 22 2026 18:23:13.694766403 (1784712193.694766403)
End:               Jul 22 2026 18:26:08.087004696 (1784712368.087004696)
Messages:          40309
Topic information: Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 847 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 186 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 1574 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 58 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 1573 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 17440 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 289 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 6805 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 1573 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 1577 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 5232 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 1574 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 758 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 819 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 0 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43

Files:             rosbag2_2026_07_24-15_21_43_0.db3
Bag size:          43.9 MiB
Storage id:        sqlite3
Duration:          120.959567458s
Start:             Jul 24 2026 15:21:43.231724569 (1784874103.231724569)
End:               Jul 24 2026 15:23:44.191292027 (1784874224.191292027)
Messages:          10982
Topic information: Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 0 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 1 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 0 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 0 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 5709 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 145 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1716 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 0 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 843 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 1716 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 6 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 0 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 1 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 843 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 0 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46

Files:             rosbag2_2026_07_24-15_23_46_0.db3
Bag size:          43.1 MiB
Storage id:        sqlite3
Duration:          63.098820427s
Start:             Jul 24 2026 15:23:46.155963749 (1784874226.155963749)
End:               Jul 24 2026 15:24:49.254784176 (1784874289.254784176)
Messages:          9140
Topic information: Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 41 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 1 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 0 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 0 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 4384 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 107 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1354 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 0 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 948 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 1352 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 0 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 0 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 948 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 0 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10

Files:             rosbag2_2026_07_24-15_25_10_0.db3
Bag size:          187.1 MiB
Storage id:        sqlite3
Duration:          62.047053311s
Start:             Jul 24 2026 15:25:10.630161229 (1784874310.630161229)
End:               Jul 24 2026 15:26:12.677214540 (1784874372.677214540)
Messages:          12460
Topic information: Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 82 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 2 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 155 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1935 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 761 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 0 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 73 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 228 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 1862 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 761 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 72 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 6205 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 6 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 227 | Serialization Format: cdr
                   Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 87 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44

Files:             rosbag2_2026_07_24-15_27_44_0.db3
Bag size:          101.5 MiB
Storage id:        sqlite3
Duration:          36.391547008s
Start:             Jul 24 2026 15:27:45.118395811 (1784874465.118395811)
End:               Jul 24 2026 15:28:21.509942819 (1784874501.509942819)
Messages:          7528
Topic information: Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 98 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 95 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 2 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 94 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 3640 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 91 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1185 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 94 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 507 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 1091 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 95 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 24 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 507 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 0 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02

Files:             rosbag2_2026_07_24-15_32_02_0.db3
Bag size:          545.2 MiB
Storage id:        sqlite3
Duration:          121.575196100s
Start:             Jul 24 2026 15:32:02.148881047 (1784874722.148881047)
End:               Jul 24 2026 15:34:03.724077147 (1784874843.724077147)
Messages:          27275
Topic information: Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 365 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 68 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 606 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 15 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 789 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 12158 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 304 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 4254 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 789 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 1734 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 3648 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 621 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 152 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 1729 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 39 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36

Files:             rosbag2_2026_07_24-16_13_36_0.db3
Bag size:          1.5 GiB
Storage id:        sqlite3
Duration:          277.195982718s
Start:             Jul 24 2026 16:13:36.051196267 (1784877216.051196267)
End:               Jul 24 2026 16:18:13.247178985 (1784877493.247178985)
Messages:          64131
Topic information: Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 693 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2352 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 2877 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 8316 | Serialization Format: cdr
                   Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 1197 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 229 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 1153 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 2353 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 10669 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 27714 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 2260 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 93 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 2353 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 374 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 1494 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38

Files:             rosbag2_2026_07_24-16_19_38_0.db3
Bag size:          92.4 MiB
Storage id:        sqlite3
Duration:          38.515727792s
Start:             Jul 24 2026 16:19:38.535579582 (1784877578.535579582)
End:               Jul 24 2026 16:20:17.051307374 (1784877617.051307374)
Messages:          8069
Topic information: Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 89 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 3 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 79 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 8 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 78 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 3853 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 96 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1234 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 78 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 612 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 1156 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 79 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 112 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 588 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 0 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23

Files:             rosbag2_2026_07_24-17_42_23_0.db3
Bag size:          120.0 MiB
Storage id:        sqlite3
Duration:          37.492145517s
Start:             Jul 24 2026 17:42:23.464787320 (1784882543.464787320)
End:               Jul 24 2026 17:43:00.956932837 (1784882580.956932837)
Messages:          6616
Topic information: Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 94 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 3 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 124 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 6 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 123 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 3112 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 88 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1066 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 123 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 373 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 943 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 124 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 77 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 356 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 0 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27

Files:             rosbag2_2026_07_24-18_11_27_0.db3
Bag size:          2.6 GiB
Storage id:        sqlite3
Duration:          497.711519134s
Start:             Jul 24 2026 18:11:27.814708392 (1784884287.814708392)
End:               Jul 24 2026 18:19:45.526227526 (1784884785.526227526)
Messages:          103253
Topic information: Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 1244 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 16897 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 4130 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 3905 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 4131 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 4131 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 635 | Serialization Format: cdr
                   Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 2148 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 41897 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 4130 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 303 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 1557 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 12767 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 5365 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26

Files:             rosbag2_2026_07_24-19_05_26_0.db3
Bag size:          5.2 GiB
Storage id:        sqlite3
Duration:          842.885597147s
Start:             Jul 24 2026 19:05:26.516010853 (1784887526.516010853)
End:               Jul 24 2026 19:19:29.401608000 (1784888369.401608000)
Messages:          199422
Topic information: Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 4215 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1150 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 8303 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 528 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 8309 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 84290 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 2104 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 33589 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 8309 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 7458 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 25287 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 8310 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 6598 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 587 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 381 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires

Files:             rosbag2_2026_07_24-19_05_26-hires_0.db3
Bag size:          5.2 GiB
Storage id:        sqlite3
Duration:          842.885597147s
Start:             Jul 24 2026 19:05:26.516010853 (1784887526.516010853)
End:               Jul 24 2026 19:19:29.401608000 (1784888369.401608000)
Messages:          199422
Topic information: Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 381 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 587 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 6598 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 8310 | Serialization Format: cdr
                   Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 4215 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1150 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 8303 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 8309 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 528 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 84290 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 2104 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 33589 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 8309 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 7458 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 25287 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51
$ ros2 bag info /home/systemlab/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51

Files:             rosbag2_2026_07_31-15_14_51_0.db3
Bag size:          1.5 GiB
Storage id:        sqlite3
Duration:          235.081798969s
Start:             Jul 31 2026 15:14:51.709657463 (1785478491.709657463)
End:               Jul 31 2026 15:18:46.791456432 (1785478726.791456432)
Messages:          54669
Topic information: Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 1175 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 194 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 2317 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 93 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 2317 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 23509 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 587 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 9369 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2317 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 1621 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 7053 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 2318 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 1218 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 576 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 1 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m8-static-check-20261113/fixed_1
$ ros2 bag info /home/systemlab/bags/m8-static-check-20261113/fixed_1

Files:             fixed_1_0.db3
Bag size:          72.1 MiB
Storage id:        sqlite3
Duration:          11.731595423s
Start:             Nov 13 2026 15:31:06.889849929 (1794551466.889849929)
End:               Nov 13 2026 15:31:18.621445352 (1794551478.621445352)
Messages:          2467
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1174 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 116 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1174 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m8-static-check-20261113/fixed_2_place2
$ ros2 bag info /home/systemlab/bags/m8-static-check-20261113/fixed_2_place2

Files:             fixed_2_place2_0.db3
Bag size:          71.5 MiB
Storage id:        sqlite3
Duration:          11.740689389s
Start:             Nov 13 2026 15:34:23.825061440 (1794551663.825061440)
End:               Nov 13 2026 15:34:35.565750829 (1794551675.565750829)
Messages:          2468
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1175 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 115 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1175 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m8-static-check-20261113/fixed_3_place2_rot180
$ ros2 bag info /home/systemlab/bags/m8-static-check-20261113/fixed_3_place2_rot180

Files:             fixed_3_place2_rot180_0.db3
Bag size:          72.1 MiB
Storage id:        sqlite3
Duration:          11.840354241s
Start:             Nov 13 2026 15:36:08.577397991 (1794551768.577397991)
End:               Nov 13 2026 15:36:20.417752232 (1794551780.417752232)
Messages:          2467
Topic information: Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1174 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 116 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1174 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m8-static-check-20261113/fixed_4_place2_rot180_after_drive
$ ros2 bag info /home/systemlab/bags/m8-static-check-20261113/fixed_4_place2_rot180_after_drive

Files:             fixed_4_place2_rot180_after_drive_0.db3
Bag size:          72.1 MiB
Storage id:        sqlite3
Duration:          11.730850519s
Start:             Nov 13 2026 15:40:58.922158131 (1794552058.922158131)
End:               Nov 13 2026 15:41:10.653008650 (1794552070.653008650)
Messages:          2467
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1174 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 116 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1174 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m8-static-check-20261113/fixed_5_place3_rot180
$ ros2 bag info /home/systemlab/bags/m8-static-check-20261113/fixed_5_place3_rot180

Files:             fixed_5_place3_rot180_0.db3
Bag size:          72.1 MiB
Storage id:        sqlite3
Duration:          11.750680508s
Start:             Nov 13 2026 15:43:44.985212758 (1794552224.985212758)
End:               Nov 13 2026 15:43:56.735893266 (1794552236.735893266)
Messages:          2471
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1176 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 116 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1176 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m8-static-check-20261113/flat_1
$ ros2 bag info /home/systemlab/bags/m8-static-check-20261113/flat_1

Files:             flat_1_0.db3
Bag size:          72.7 MiB
Storage id:        sqlite3
Duration:          11.843723369s
Start:             Nov 13 2026 11:42:16.983260314 (1794537736.983260314)
End:               Nov 13 2026 11:42:28.826983683 (1794537748.826983683)
Messages:          2490
Topic information: Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1185 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 117 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1185 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m8-static-check-20261113/flat_2_rot180
$ ros2 bag info /home/systemlab/bags/m8-static-check-20261113/flat_2_rot180

Files:             flat_2_rot180_0.db3
Bag size:          72.1 MiB
Storage id:        sqlite3
Duration:          11.750682810s
Start:             Nov 13 2026 11:44:49.329044153 (1794537889.329044153)
End:               Nov 13 2026 11:45:01.079726963 (1794537901.079726963)
Messages:          2471
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1176 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 116 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1176 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/m8-static-check-20261113/flat_3_seated
$ ros2 bag info /home/systemlab/bags/m8-static-check-20261113/flat_3_seated

Files:             flat_3_seated_0.db3
Bag size:          72.1 MiB
Storage id:        sqlite3
Duration:          11.751782546s
Start:             Nov 13 2026 11:46:58.152320739 (1794538018.152320739)
End:               Nov 13 2026 11:47:09.904103285 (1794538029.904103285)
Messages:          2471
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 1176 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 116 | Serialization Format: cdr
                   Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 1176 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_bottle_2m_160742
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_bottle_2m_160742

Files:             chk_bottle_2m_160742_0.mcap
Bag size:          724.5 MiB
Storage id:        mcap
Duration:          12.817846100s
Start:             Oct 28 2026 16:08:17.879993533 (1793171297.879993533)
End:               Oct 28 2026 16:08:30.697839633 (1793171310.697839633)
Messages:          2186
Topic information: Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 382 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 126 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 384 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 384 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 125 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 384 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 384 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 13 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_bottle_3m_160831
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_bottle_3m_160831

Files:             chk_bottle_3m_160831_0.mcap
Bag size:          662.9 MiB
Storage id:        mcap
Duration:          11.702101807s
Start:             Oct 28 2026 16:08:50.989242120 (1793171330.989242120)
End:               Oct 28 2026 16:09:02.691343927 (1793171342.691343927)
Messages:          2004
Topic information: Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 351 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 351 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 13 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 351 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 351 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 351 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 116 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 116 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_bottle_4m_160903
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_bottle_4m_160903

Files:             chk_bottle_4m_160903_0.mcap
Bag size:          558.5 MiB
Storage id:        mcap
Duration:          9.873895187s
Start:             Oct 28 2026 16:09:19.403941217 (1793171359.403941217)
End:               Oct 28 2026 16:09:29.277836404 (1793171369.277836404)
Messages:          1689
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 97 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 293 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 296 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 98 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 296 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 296 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 13 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 296 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_bottle_5m_160929
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_bottle_5m_160929

Files:             chk_bottle_5m_160929_0.mcap
Bag size:          576.4 MiB
Storage id:        mcap
Duration:          10.188271765s
Start:             Oct 28 2026 16:10:17.857497157 (1793171417.857497157)
End:               Oct 28 2026 16:10:28.045768922 (1793171428.045768922)
Messages:          1744
Topic information: Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 306 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 306 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 306 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 100 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 305 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 305 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 12 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 100 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160152
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160152

Files:             chk_chair_1.5m_160152_0.mcap
Bag size:          1.2 GiB
Storage id:        mcap
Duration:          21.897645993s
Start:             Oct 28 2026 16:02:05.533583301 (1793170925.533583301)
End:               Oct 28 2026 16:02:27.431229294 (1793170947.431229294)
Messages:          3731
Topic information: Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 657 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 657 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 657 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 215 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 657 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 657 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 11 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 216 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160421
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160421

Files:             chk_chair_1.5m_160421_0.mcap
Bag size:          650.7 MiB
Storage id:        mcap
Duration:          11.586497367s
Start:             Oct 28 2026 16:04:46.155587523 (1793171086.155587523)
End:               Oct 28 2026 16:04:57.742084890 (1793171097.742084890)
Messages:          1971
Topic information: Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 345 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 344 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 114 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 344 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 114 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 345 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 348 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 13 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160227
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160227

Files:             chk_chair_2m_160227_0.mcap
Bag size:          623.1 MiB
Storage id:        mcap
Duration:          10.985278366s
Start:             Oct 28 2026 16:03:20.192446296 (1793171000.192446296)
End:               Oct 28 2026 16:03:31.177724662 (1793171011.177724662)
Messages:          1884
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 109 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 330 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 330 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 108 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 330 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 330 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 330 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 13 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160458
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160458

Files:             chk_chair_2m_160458_0.mcap
Bag size:          605.5 MiB
Storage id:        mcap
Duration:          10.694174822s
Start:             Oct 28 2026 16:05:22.073949913 (1793171122.073949913)
End:               Oct 28 2026 16:05:32.768124735 (1793171132.768124735)
Messages:          1832
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 105 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 321 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 321 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 105 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 321 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 321 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 13 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 321 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160331
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160331

Files:             chk_chair_3m_160331_0.mcap
Bag size:          701.7 MiB
Storage id:        mcap
Duration:          12.398923208s
Start:             Oct 28 2026 16:03:58.319133178 (1793171038.319133178)
End:               Oct 28 2026 16:04:10.718056386 (1793171050.718056386)
Messages:          2122
Topic information: Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 372 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 372 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 13 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 372 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 372 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 371 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 123 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 123 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160533
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160533

Files:             chk_chair_3m_160533_0.mcap
Bag size:          617.5 MiB
Storage id:        mcap
Duration:          10.903786310s
Start:             Oct 28 2026 16:06:08.243811718 (1793171168.243811718)
End:               Oct 28 2026 16:06:19.147598028 (1793171179.147598028)
Messages:          1868
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 108 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 327 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 327 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 327 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 108 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 327 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 327 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 13 | Serialization Format: cdr

rc=0
### /home/systemlab/bags/sensor_check_20261028/chk_empty_0m_160726
$ ros2 bag info /home/systemlab/bags/sensor_check_20261028/chk_empty_0m_160726

Files:             chk_empty_0m_160726_0.mcap
Bag size:          645.9 MiB
Storage id:        mcap
Duration:          11.401345908s
Start:             Oct 28 2026 16:07:30.129750534 (1793171250.129750534)
End:               Oct 28 2026 16:07:41.531096442 (1793171261.531096442)
Messages:          1954
Topic information: Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 113 | Serialization Format: cdr
                   Topic: /camera/camera/depth/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 342 | Serialization Format: cdr
                   Topic: /velodyne_packets | Type: velodyne_msgs/msg/VelodyneScan | Count: 113 | Serialization Format: cdr
                   Topic: /camera/camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 342 | Serialization Format: cdr
                   Topic: /camera/camera/color/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 342 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 13 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 343 | Serialization Format: cdr
                   Topic: /camera/camera/depth/image_rect_raw | Type: sensor_msgs/msg/Image | Count: 342 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 4 | Serialization Format: cdr

rc=0
### /home/systemlab/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag
$ ros2 bag info /home/systemlab/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag

Files:             bag_0.db3
Bag size:          24.9 KiB
Storage id:        sqlite3
Duration:          0.000000000s
Start:             Jul 12 2026 19:08:40.823039815 (1783850920.823039815)
End:               Jul 12 2026 19:08:40.823039815 (1783850920.823039815)
Messages:          1
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr

rc=0
### /home/systemlab/whill_platform/bags/2026-07-31-campus
$ ros2 bag info /home/systemlab/whill_platform/bags/2026-07-31-campus

Files:             2026-07-31-campus_0.mcap
Bag size:          1.5 GiB
Storage id:        mcap
Duration:          235.081798969s
Start:             Jul 31 2026 15:14:51.709657463 (1785478491.709657463)
End:               Jul 31 2026 15:18:46.791456432 (1785478726.791456432)
Messages:          54669
Topic information: Topic: /plan | Type: nav_msgs/msg/Path | Count: 93 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 2317 | Serialization Format: cdr
                   Topic: /imu/data_rep145 | Type: sensor_msgs/msg/Imu | Count: 23509 | Serialization Format: cdr
                   Topic: /whill/odom | Type: nav_msgs/msg/Odometry | Count: 587 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 9369 | Serialization Format: cdr
                   Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 2317 | Serialization Format: cdr
                   Topic: /odometry/filtered | Type: nav_msgs/msg/Odometry | Count: 7053 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 1621 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 3 | Serialization Format: cdr
                   Topic: /alignment_status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 2318 | Serialization Format: cdr
                   Topic: /cmd_vel_nav | Type: geometry_msgs/msg/Twist | Count: 1218 | Serialization Format: cdr
                   Topic: /cmd_vel_safety | Type: geometry_msgs/msg/Twist | Count: 576 | Serialization Format: cdr
                   Topic: /cmd_vel_teleop | Type: geometry_msgs/msg/Twist | Count: 1 | Serialization Format: cdr
                   Topic: /pcl_pose | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 2317 | Serialization Format: cdr
                   Topic: /global_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 1 | Serialization Format: cdr
                   Topic: /local_costmap/costmap | Type: nav_msgs/msg/OccupancyGrid | Count: 194 | Serialization Format: cdr
                   Topic: /local_costmap/published_footprint | Type: geometry_msgs/msg/PolygonStamped | Count: 1175 | Serialization Format: cdr

rc=0
```

## 付録 B. `du -sh` の生の出力（88 本、省略なし）

```
386M	/home/systemlab/bags/2026-11-16-static/cone_1p5_141326
390M	/home/systemlab/bags/2026-11-16-static/cone_2p5_141549
387M	/home/systemlab/bags/2026-11-16-static/cone_2p_141506
390M	/home/systemlab/bags/2026-11-16-static/cone_3p5_141752
386M	/home/systemlab/bags/2026-11-16-static/cone_3p_141709
390M	/home/systemlab/bags/2026-11-16-static/cone_4p_141838
390M	/home/systemlab/bags/2026-11-16-static/cone_5p_141921
390M	/home/systemlab/bags/2026-11-16-static/empty_start_140915
161M	/home/systemlab/bags/2026-11-16-test/smoke_140741
1.8G	/home/systemlab/bags/imu_investigation_20261030/seg_0_300_hires
1.8G	/home/systemlab/bags/imu_investigation_20261030/seg_0_300_orig
5.3G	/home/systemlab/bags/m3-bench-data/m3_chair_motion_2026-05-07
1.1G	/home/systemlab/bags/m3-bench-data/m3_chair_static_2026-05-07
437M	/home/systemlab/bags/m3-bench-data/m3_smoke_2026-05-07
3.3G	/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08
3.3G	/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run1
3.3G	/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run2
3.3G	/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run3
6.6G	/home/systemlab/bags/m3-bench-data/m5_chair_mapping_2026-05-08
1.2G	/home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag
1.2G	/home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag-imu-fixed
1.5G	/home/systemlab/bags/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag
2.8G	/home/systemlab/bags/m5r-bench-data/2026-07-07-campus-half-v3/bag
4.8G	/home/systemlab/bags/m5r-bench-data/2026-07-07-campus-loop/bag
11G	/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-postgap/bag
12G	/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-pregap/bag
3.6G	/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-first10/bag
16G	/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-pregap/bag
20G	/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2/bag
22G	/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo/bag
17G	/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-debiased/bag
7.9G	/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-A/bag
8.9G	/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-B/bag
17G	/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer/bag
2.2G	/home/systemlab/bags/m5r-bench-data/2026-07-08-indoor-calib/bag
4.5G	/home/systemlab/bags/m5r-bench-data/2026-07-08-outdoor-corrected/bag
1.3G	/home/systemlab/bags/m5r-bench-data/2026-07-09-indoor-calib/bag
13G	/home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag
13G	/home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires
1.9G	/home/systemlab/bags/m5r-bench-data/2026-07-10-dds-verify/bag
3.8G	/home/systemlab/bags/m6r-bench-data/2026-07-14-verify-campus/bag
4.7G	/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41
1.6G	/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43
1.7G	/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08
3.8G	/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27
2.8G	/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07
3.3G	/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28
1.7G	/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48
1.6G	/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01
735M	/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48
3.1G	/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31
3.7G	/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34
2.1G	/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14
876M	/home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15
1.1G	/home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13
44M	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43
44M	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46
188M	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10
102M	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44
546M	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02
1.5G	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36
93M	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38
120M	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23
2.7G	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27
5.3G	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26
5.3G	/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires
1.5G	/home/systemlab/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51
73M	/home/systemlab/bags/m8-static-check-20261113/fixed_1
72M	/home/systemlab/bags/m8-static-check-20261113/fixed_2_place2
73M	/home/systemlab/bags/m8-static-check-20261113/fixed_3_place2_rot180
73M	/home/systemlab/bags/m8-static-check-20261113/fixed_4_place2_rot180_after_drive
73M	/home/systemlab/bags/m8-static-check-20261113/fixed_5_place3_rot180
73M	/home/systemlab/bags/m8-static-check-20261113/flat_1
73M	/home/systemlab/bags/m8-static-check-20261113/flat_2_rot180
73M	/home/systemlab/bags/m8-static-check-20261113/flat_3_seated
725M	/home/systemlab/bags/sensor_check_20261028/chk_bottle_2m_160742
663M	/home/systemlab/bags/sensor_check_20261028/chk_bottle_3m_160831
559M	/home/systemlab/bags/sensor_check_20261028/chk_bottle_4m_160903
577M	/home/systemlab/bags/sensor_check_20261028/chk_bottle_5m_160929
1.3G	/home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160152
651M	/home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160421
624M	/home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160227
606M	/home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160458
702M	/home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160331
618M	/home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160533
646M	/home/systemlab/bags/sensor_check_20261028/chk_empty_0m_160726
32K	/home/systemlab/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag
1.5G	/home/systemlab/whill_platform/bags/2026-07-31-campus
```

## 付録 C. `find` の生の出力（found.txt、299 行、省略なし）

worktree 内の行は git 管理のファイル（地図の metadata.yaml とリポジトリ内の bag）の写し。

```
/home/systemlab/bags/2026-11-16-static/cone_1p5_141326/cone_1p5_141326_0.mcap
/home/systemlab/bags/2026-11-16-static/cone_1p5_141326/metadata.yaml
/home/systemlab/bags/2026-11-16-static/cone_2p_141506/cone_2p_141506_0.mcap
/home/systemlab/bags/2026-11-16-static/cone_2p_141506/metadata.yaml
/home/systemlab/bags/2026-11-16-static/cone_2p5_141549/cone_2p5_141549_0.mcap
/home/systemlab/bags/2026-11-16-static/cone_2p5_141549/metadata.yaml
/home/systemlab/bags/2026-11-16-static/cone_3p_141709/cone_3p_141709_0.mcap
/home/systemlab/bags/2026-11-16-static/cone_3p_141709/metadata.yaml
/home/systemlab/bags/2026-11-16-static/cone_3p5_141752/cone_3p5_141752_0.mcap
/home/systemlab/bags/2026-11-16-static/cone_3p5_141752/metadata.yaml
/home/systemlab/bags/2026-11-16-static/cone_4p_141838/cone_4p_141838_0.mcap
/home/systemlab/bags/2026-11-16-static/cone_4p_141838/metadata.yaml
/home/systemlab/bags/2026-11-16-static/cone_5p_141921/cone_5p_141921_0.mcap
/home/systemlab/bags/2026-11-16-static/cone_5p_141921/metadata.yaml
/home/systemlab/bags/2026-11-16-static/empty_start_140915/empty_start_140915_0.mcap
/home/systemlab/bags/2026-11-16-static/empty_start_140915/metadata.yaml
/home/systemlab/bags/2026-11-16-test/smoke_140741/metadata.yaml
/home/systemlab/bags/2026-11-16-test/smoke_140741/smoke_140741_0.mcap
/home/systemlab/bags/imu_investigation_20261030/seg_0_300_hires/metadata.yaml
/home/systemlab/bags/imu_investigation_20261030/seg_0_300_hires/seg_0_300_hires_0.db3
/home/systemlab/bags/imu_investigation_20261030/seg_0_300_orig/metadata.yaml
/home/systemlab/bags/imu_investigation_20261030/seg_0_300_orig/seg_0_300_orig_0.db3
/home/systemlab/bags/m3-bench-data/m3_chair_motion_2026-05-07/m3_chair_motion_2026-05-07_0.db3
/home/systemlab/bags/m3-bench-data/m3_chair_motion_2026-05-07/metadata.yaml
/home/systemlab/bags/m3-bench-data/m3_chair_static_2026-05-07/m3_chair_static_2026-05-07_0.db3
/home/systemlab/bags/m3-bench-data/m3_chair_static_2026-05-07/metadata.yaml
/home/systemlab/bags/m3-bench-data/m3_smoke_2026-05-07/m3_smoke_2026-05-07_0.db3
/home/systemlab/bags/m3-bench-data/m3_smoke_2026-05-07/metadata.yaml
/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08/m4_chair_live_2026-05-08_0.db3
/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08/metadata.yaml
/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run1/m4_chair_live_2026-05-08_run1_0.db3
/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run1/metadata.yaml
/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run2/m4_chair_live_2026-05-08_run2_0.db3
/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run2/metadata.yaml
/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run3/m4_chair_live_2026-05-08_run3_0.db3
/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run3/metadata.yaml
/home/systemlab/bags/m3-bench-data/m5_chair_mapping_2026-05-08/m5_chair_mapping_2026-05-08_0.db3
/home/systemlab/bags/m3-bench-data/m5_chair_mapping_2026-05-08/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag-imu-fixed/bag-imu-fixed_0.db3
/home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag-imu-fixed/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-07-campus-half-v3/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-07-campus-half-v3/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-07-campus-loop/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-07-campus-loop/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-postgap/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-postgap/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-pregap/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-pregap/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-first10/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-first10/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-pregap/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-pregap/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-debiased/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-debiased/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-A/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-A/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-B/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-B/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-indoor-calib/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-indoor-calib/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-08-outdoor-corrected/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-08-outdoor-corrected/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-09-indoor-calib/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-09-indoor-calib/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires/bag-hires_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag/metadata.yaml
/home/systemlab/bags/m5r-bench-data/2026-07-10-dds-verify/bag/bag_0.db3
/home/systemlab/bags/m5r-bench-data/2026-07-10-dds-verify/bag/metadata.yaml
/home/systemlab/bags/m6r-bench-data/2026-07-14-verify-campus/bag/bag_0.db3
/home/systemlab/bags/m6r-bench-data/2026-07-14-verify-campus/bag/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41/rosbag2_2026_07_20-15_13_41_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43/rosbag2_2026_07_20-15_29_43_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08/rosbag2_2026_07_20-15_44_08_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27/rosbag2_2026_07_20-15_59_27_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07/rosbag2_2026_07_20-17_03_07_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28/rosbag2_2026_07_20-18_09_28_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48/rosbag2_2026_07_20-19_58_48_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01/rosbag2_2026_07_21-16_34_01_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48/rosbag2_2026_07_21-16_50_48_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31/rosbag2_2026_07_21-16_53_31_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34/rosbag2_2026_07_21-17_20_34_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14/rosbag2_2026_07_21-18_29_14_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15/rosbag2_2026_07_22-18_01_15_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13/rosbag2_2026_07_22-18_23_13_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43/rosbag2_2026_07_24-15_21_43_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46/rosbag2_2026_07_24-15_23_46_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10/rosbag2_2026_07_24-15_25_10_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44/rosbag2_2026_07_24-15_27_44_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02/rosbag2_2026_07_24-15_32_02_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36/rosbag2_2026_07_24-16_13_36_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38/rosbag2_2026_07_24-16_19_38_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23/rosbag2_2026_07_24-17_42_23_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27/rosbag2_2026_07_24-18_11_27_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires/rosbag2_2026_07_24-19_05_26-hires_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26/rosbag2_2026_07_24-19_05_26_0.db3
/home/systemlab/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51/metadata.yaml
/home/systemlab/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51/rosbag2_2026_07_31-15_14_51_0.db3
/home/systemlab/bags/m8-static-check-20261113/fixed_1/fixed_1_0.db3
/home/systemlab/bags/m8-static-check-20261113/fixed_1/metadata.yaml
/home/systemlab/bags/m8-static-check-20261113/fixed_2_place2/fixed_2_place2_0.db3
/home/systemlab/bags/m8-static-check-20261113/fixed_2_place2/metadata.yaml
/home/systemlab/bags/m8-static-check-20261113/fixed_3_place2_rot180/fixed_3_place2_rot180_0.db3
/home/systemlab/bags/m8-static-check-20261113/fixed_3_place2_rot180/metadata.yaml
/home/systemlab/bags/m8-static-check-20261113/fixed_4_place2_rot180_after_drive/fixed_4_place2_rot180_after_drive_0.db3
/home/systemlab/bags/m8-static-check-20261113/fixed_4_place2_rot180_after_drive/metadata.yaml
/home/systemlab/bags/m8-static-check-20261113/fixed_5_place3_rot180/fixed_5_place3_rot180_0.db3
/home/systemlab/bags/m8-static-check-20261113/fixed_5_place3_rot180/metadata.yaml
/home/systemlab/bags/m8-static-check-20261113/flat_1/flat_1_0.db3
/home/systemlab/bags/m8-static-check-20261113/flat_1/metadata.yaml
/home/systemlab/bags/m8-static-check-20261113/flat_2_rot180/flat_2_rot180_0.db3
/home/systemlab/bags/m8-static-check-20261113/flat_2_rot180/metadata.yaml
/home/systemlab/bags/m8-static-check-20261113/flat_3_seated/flat_3_seated_0.db3
/home/systemlab/bags/m8-static-check-20261113/flat_3_seated/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_bottle_2m_160742/chk_bottle_2m_160742_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_bottle_2m_160742/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_bottle_3m_160831/chk_bottle_3m_160831_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_bottle_3m_160831/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_bottle_4m_160903/chk_bottle_4m_160903_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_bottle_4m_160903/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_bottle_5m_160929/chk_bottle_5m_160929_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_bottle_5m_160929/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160152/chk_chair_1.5m_160152_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160152/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160421/chk_chair_1.5m_160421_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160421/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160227/chk_chair_2m_160227_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160227/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160458/chk_chair_2m_160458_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160458/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160331/chk_chair_3m_160331_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160331/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160533/chk_chair_3m_160533_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160533/metadata.yaml
/home/systemlab/bags/sensor_check_20261028/chk_empty_0m_160726/chk_empty_0m_160726_0.mcap
/home/systemlab/bags/sensor_check_20261028/chk_empty_0m_160726/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/docs/maps/campus-m8/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a5ff978baf8d33a7e/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a5ff978baf8d33a7e/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a5ff978baf8d33a7e/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a5ff978baf8d33a7e/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a5ff978baf8d33a7e/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a5ff978baf8d33a7e/docs/maps/campus-m8/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a5ff978baf8d33a7e/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a5ff978baf8d33a7e/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a5ff978baf8d33a7e/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a6be4559bf67aff21/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a6be4559bf67aff21/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a6be4559bf67aff21/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a6be4559bf67aff21/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a6be4559bf67aff21/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a6be4559bf67aff21/docs/maps/campus-m8/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a6be4559bf67aff21/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a6be4559bf67aff21/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a6be4559bf67aff21/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aea1688e3f7101320/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aea1688e3f7101320/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aea1688e3f7101320/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aea1688e3f7101320/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aea1688e3f7101320/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aea1688e3f7101320/docs/maps/campus-m8/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aea1688e3f7101320/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aea1688e3f7101320/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aea1688e3f7101320/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aecfa03d91f4cdcc6/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aecfa03d91f4cdcc6/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aecfa03d91f4cdcc6/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aecfa03d91f4cdcc6/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aecfa03d91f4cdcc6/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aecfa03d91f4cdcc6/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aecfa03d91f4cdcc6/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aecfa03d91f4cdcc6/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aefeb9215fde1cac3/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aefeb9215fde1cac3/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aefeb9215fde1cac3/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aefeb9215fde1cac3/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aefeb9215fde1cac3/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aefeb9215fde1cac3/docs/maps/campus-m8/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aefeb9215fde1cac3/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aefeb9215fde1cac3/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-aefeb9215fde1cac3/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-baseline/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-baseline/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-baseline/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-baseline/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-baseline/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-baseline/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-baseline/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-baseline/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c2/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c2/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c2/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c2/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c2/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c2/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c3/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c3/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c3/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c3/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c3/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c3/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c3/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/avoidance-c3/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/footprint-143/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/footprint-143/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/footprint-143/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/footprint-143/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/footprint-143/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/footprint-143/docs/maps/campus-m8/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/footprint-143/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/footprint-143/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/footprint-143/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/m8-integration/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/m8-integration/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/m8-integration/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/m8-integration/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/m8-integration/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/m8-integration/docs/maps/campus-m8/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/m8-integration/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/m8-integration/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/m8-integration/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/outdoor-149/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/outdoor-149/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/outdoor-149/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/outdoor-149/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/outdoor-149/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/outdoor-149/docs/maps/campus-m8/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/outdoor-149/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/outdoor-149/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/outdoor-149/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/record-fg/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/record-fg/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/.claude/worktrees/record-fg/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/record-fg/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/record-fg/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/record-fg/docs/maps/campus-m8/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/record-fg/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/record-fg/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/.claude/worktrees/record-fg/docs/maps/_template/metadata.yaml
/home/systemlab/whill_lab0_ros2/docs/m5r-bench-data/2026-07-07-campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/bag_0.db3
/home/systemlab/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag/metadata.yaml
/home/systemlab/whill_lab0_ros2/docs/maps/campus-demo/metadata.yaml
/home/systemlab/whill_lab0_ros2/docs/maps/campus-half-v3/metadata.yaml
/home/systemlab/whill_lab0_ros2/docs/maps/campus-m8/metadata.yaml
/home/systemlab/whill_lab0_ros2/docs/maps/campus/metadata.yaml
/home/systemlab/whill_lab0_ros2/docs/maps/campus-outdoor-corrected/metadata.yaml
/home/systemlab/whill_lab0_ros2/docs/maps/_template/metadata.yaml
/home/systemlab/whill_platform/bags/2026-07-31-campus/2026-07-31-campus_0.mcap
/home/systemlab/whill_platform/bags/2026-07-31-campus/metadata.yaml
/opt/ros/humble/share/robot_localization/test/test1.bag
/opt/ros/humble/share/robot_localization/test/test2.bag
/opt/ros/humble/share/robot_localization/test/test3.bag
```

## 付録 D. 使ったスクリプト（すべて読み取りのみ）

### analyze.py

```python
#!/usr/bin/env python3
"""Read-only bag analysis. Never writes image data anywhere; decodes only header bytes in memory."""
import json, struct, sys, collections, datetime
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CAM = '/camera/camera/color/image_raw/compressed'
CAM_INFO = '/camera/camera/color/camera_info'
CMD = ['/cmd_vel', '/cmd_vel_nav', '/cmd_vel_teleop', '/cmd_vel_safety', '/whill/controller/cmd_vel']
MODE_TOPICS = CMD + ['/dispatch/state', '/plan', '/dispatch/teleop', '/whill/controller/joy', '/joy',
                     '/navigate_to_pose/_action/status']


def jpeg_size(b):
    if b[:2] != b'\xff\xd8':
        return None
    i = 2
    while i + 9 < len(b):
        if b[i] != 0xFF:
            i += 1
            continue
        m = b[i + 1]
        if m in (0xD8, 0x01) or 0xD0 <= m <= 0xD7:
            i += 2
            continue
        L = struct.unpack('>H', b[i + 2:i + 4])[0]
        if m in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            h, w = struct.unpack('>HH', b[i + 5:i + 9])
            return ('jpeg', w, h)
        i += 2 + L
    return None


def png_size(b):
    if b[:8] != b'\x89PNG\r\n\x1a\n':
        return None
    w, h = struct.unpack('>II', b[16:24])
    return ('png', w, h)


def reader_for(path, sid, topics=None):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=sid),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def main(path):
    info = rosbag2_py.Info().read_metadata(path, '')
    sid = info.storage_identifier
    dur = info.duration.nanoseconds / 1e9
    start = info.starting_time
    res = {'path': path, 'storage': sid, 'duration_s': dur,
           'start': str(start), 'size': info.bag_size, 'messages': info.message_count,
           'topics': []}
    types = {}
    for t in info.topics_with_message_count:
        m = t.topic_metadata
        types[m.name] = m.type
        res['topics'].append({'name': m.name, 'type': m.type, 'count': t.message_count,
                              'rate_hz': (t.message_count / dur) if dur > 0 else None})
    res['topics'].sort(key=lambda x: x['name'])
    # camera
    cam = {}
    if CAM in types:
        r = reader_for(path, sid, [CAM])
        if r.has_next():
            topic, data, ts = r.read_next()
            msg = deserialize_message(data, get_message(types[CAM]))
            b = bytes(msg.data[:65536])
            cam['format_field'] = msg.format
            cam['header_size'] = jpeg_size(b) or png_size(b)
            cam['first_stamp_ns'] = ts
        del r
    if CAM_INFO in types:
        r = reader_for(path, sid, [CAM_INFO])
        if r.has_next():
            topic, data, ts = r.read_next()
            msg = deserialize_message(data, get_message(types[CAM_INFO]))
            cam['camera_info_wh'] = (msg.width, msg.height)
        del r
    res['camera'] = cam
    # mode evidence
    present = [t for t in MODE_TOPICS if t in types]
    ev = {}
    if present:
        r = reader_for(path, sid, present)
        st = collections.defaultdict(lambda: {'n': 0, 'first': None, 'last': None, 'nonzero': 0})
        disp = collections.Counter()
        disp_seq = []
        while r.has_next():
            topic, data, ts = r.read_next()
            s = st[topic]
            s['n'] += 1
            s['first'] = s['first'] if s['first'] is not None else ts
            s['last'] = ts
            if topic in CMD:
                msg = deserialize_message(data, get_message(types[topic]))
                if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
                    s['nonzero'] += 1
            elif topic == '/dispatch/state':
                msg = deserialize_message(data, get_message(types[topic]))
                val = msg.data
                try:
                    j = json.loads(val)
                    key = j.get('state') if isinstance(j, dict) else None
                    mode_keys = {k: j.get(k) for k in ('phase', 'teleop_active') if isinstance(j, dict) and k in j}
                    val = json.dumps(mode_keys, ensure_ascii=False, sort_keys=True)
                except Exception:
                    val = val[:80]
                disp[val] += 1
                if not disp_seq or disp_seq[-1][0] != val:
                    disp_seq.append((val, ts))
        del r
        for k, v in st.items():
            ev[k] = {'n': v['n'], 'nonzero': v['nonzero'] if k in CMD else None,
                     'first_rel_s': (v['first'] - start.nanoseconds) / 1e9 if v['first'] else None,
                     'last_rel_s': (v['last'] - start.nanoseconds) / 1e9 if v['last'] else None}
        if disp:
            ev['/dispatch/state#values'] = dict(disp)
            ev['/dispatch/state#transitions'] = [(v, round((t - start.nanoseconds) / 1e9, 1)) for v, t in disp_seq][:40]
    res['mode'] = ev
    print(json.dumps(res, ensure_ascii=False, default=str))


if __name__ == '__main__':
    main(sys.argv[1])
```

### run.sh

```bash
#!/usr/bin/env bash
# Read-only: analyze each bag listed in bags.txt
# no set -u: ROS setup.bash references unset vars
source /opt/ros/humble/setup.bash
source /home/systemlab/whill_lab0_ros2/install/setup.bash
D=$(dirname "$0")
if [[ $# -gt 0 ]]; then python3 "$D/analyze.py" "$1"; exit; fi
: > "$D/analysis.jsonl"
while read -r b; do
  python3 "$D/analyze.py" "$b" >> "$D/analysis.jsonl" 2>> "$D/analysis.err" || echo "FAIL $b" >> "$D/analysis.err"
done < "$D/bags.txt"
```

### gen.py（表の作成）

```python
#!/usr/bin/env python3
"""Generate markdown tables from analysis.jsonl / du_raw.txt (read-only inputs)."""
import json, re, os, datetime, collections, sys
S = os.path.dirname(os.path.abspath(__file__))
H = '/home/systemlab/'
rows = [json.loads(l) for l in open(f'{S}/analysis.jsonl')]
du = {}
for l in open(f'{S}/du_raw.txt'):
    sz, p = l.rstrip('\n').split('\t')
    du[p] = sz
JST = datetime.timezone(datetime.timedelta(hours=9))


def short(p):
    return '~/' + p[len(H):] if p.startswith(H) else p


def start_ns(r):
    return int(re.search(r'nanoseconds=(\d+)', r['start']).group(1))


def fmt_start(r):
    return datetime.datetime.fromtimestamp(start_ns(r) / 1e9, JST).strftime('%Y-%m-%d %H:%M:%S')


def has(r, t):
    for x in r['topics']:
        if x['name'] == t:
            return x
    return None


out = []
w = out.append
# ---- section 1
w('| # | bag（パス） | 形式 | 開始（JST） | 長さ [s] | du -sh | メッセージ数 |')
w('|---|---|---|---|---:|---:|---:|')
for i, r in enumerate(rows, 1):
    r['idx'] = i
    w(f"| {i} | `{short(r['path'])}` | {r['storage']} | {fmt_start(r)} | {r['duration_s']:.1f} | {du[r['path']]} | {r['messages']} |")
w('')
groups = collections.defaultdict(list)
for r in rows:
    groups[start_ns(r)].append(r['idx'])
dups = [v for v in groups.values() if len(v) > 1]
w('開始時刻（ナノ秒まで）が一致する bag の組（同じ記録から派生したもの）: ' +
  ('、'.join('#' + '/#'.join(map(str, v)) for v in dups) if dups else 'なし'))
w('')
sec1 = '\n'.join(out)

# ---- section 2
out = []
w = out.append
for r in rows:
    d = r['duration_s']
    w(f"#### #{r['idx']} `{short(r['path'])}`（{d:.1f} s）")
    w('')
    w('| トピック | 型 | 件数 | 平均レート [Hz] |')
    w('|---|---|---:|---:|')
    for t in r['topics']:
        rate = f"{t['rate_hz']:.2f}" if t['rate_hz'] is not None else '—（長さ 0）'
        w(f"| `{t['name']}` | `{t['type']}` | {t['count']} | {rate} |")
    w('')
sec2 = '\n'.join(out)

# ---- section 3
out = []
w = out.append
CAM = '/camera/camera/color/image_raw/compressed'
w('| # | bag | compressed の有無 | 件数 | 平均レート [Hz] | 1 通目の format 欄 | 1 通目のヘッダから読んだ解像度 | color/camera_info の width×height | 生の /camera/camera/color/image_raw |')
w('|---|---|---|---:|---:|---|---|---|---|')
for r in rows:
    c = has(r, CAM)
    raw = has(r, '/camera/camera/color/image_raw')
    cam = r['camera']
    ci = cam.get('camera_info_wh')
    ci_s = f"{ci[0]}×{ci[1]}" if ci else '—'
    raw_s = f"あり（{raw['count']} 件, {raw['rate_hz']:.2f} Hz）" if raw else 'なし'
    if c:
        hs = cam.get('header_size')
        hs_s = f"{hs[0].upper()} {hs[1]}×{hs[2]}" if hs else '読めず'
        w(f"| {r['idx']} | `{short(r['path'])}` | **あり** | {c['count']} | {c['rate_hz']:.2f} | `{cam.get('format_field')}` | {hs_s} | {ci_s} | {raw_s} |")
    else:
        w(f"| {r['idx']} | `{short(r['path'])}` | なし | — | — | — | — | {ci_s} | {raw_s} |")
w('')
sec3 = '\n'.join(out)

# ---- section 4
out = []
w = out.append
cols = ['/pcl_pose', '/alignment_status', '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_teleop', '/cmd_vel_safety',
        '/whill/controller/cmd_vel', '/tf', '/tf_static']
w('| # | bag | ' + ' | '.join(f'`{c}`' for c in cols) + ' | 3 点同居（`/pcl_pose`＋cmd_vel 系＋`/tf`） |')
w('|---|---|' + '---:|' * len(cols) + '---|')
for r in rows:
    cells = []
    for c in cols:
        t = has(r, c)
        cells.append(str(t['count']) if t else '—')
    cmd_any = any(has(r, c) and has(r, c)['count'] > 0 for c in cols[2:7])
    pp = has(r, '/pcl_pose') and has(r, '/pcl_pose')['count'] > 0
    tf = has(r, '/tf') and has(r, '/tf')['count'] > 0
    w(f"| {r['idx']} | `{short(r['path'])}` | " + ' | '.join(cells) + f" | {'**○**' if (cmd_any and pp and tf) else '×'} |")
w('')
w('数値は件数、— はトピック自体が bag に無いことを表す。')
sec4 = '\n'.join(out)

# ---- section 5 table
out = []
w = out.append
w('| # | bag | `/cmd_vel_nav` 件数（非ゼロ） | `/cmd_vel_teleop` 件数（非ゼロ） | `/cmd_vel_safety` 件数（非ゼロ） | `/cmd_vel` 件数（非ゼロ） | `/plan` 件数 | `/dispatch/state` の phase / teleop_active の内訳 |')
w('|---|---|---|---|---|---|---:|---|')
for r in rows:
    m = r['mode']
    if not m:
        continue

    def cell(t):
        v = m.get(t)
        return f"{v['n']}（{v['nonzero']}）" if v else '—'
    dv = m.get('/dispatch/state#values')
    ds = '<br>'.join(f"{json.loads(k)}: {n}" for k, n in dv.items()).replace("'", '') if dv else '—'
    pl = m.get('/plan', {}).get('n', '—')
    w(f"| {r['idx']} | `{short(r['path'])}` | {cell('/cmd_vel_nav')} | {cell('/cmd_vel_teleop')} | {cell('/cmd_vel_safety')} | {cell('/cmd_vel')} | {pl} | {ds} |")
w('')
sec5 = '\n'.join(out)

# transitions detail
out = []
w = out.append
for r in rows:
    m = r['mode']
    tr = m.get('/dispatch/state#transitions')
    seg = []
    for t in ('/cmd_vel_nav', '/cmd_vel_teleop', '/cmd_vel_safety'):
        if t in m:
            seg.append(f"`{t}` {m[t]['first_rel_s']:.1f}–{m[t]['last_rel_s']:.1f} s")
    if tr or seg:
        line = f"- #{r['idx']} `{short(r['path'])}`: "
        if seg:
            line += '最初と最後の受信（bag 開始からの秒）: ' + '、'.join(seg) + '。'
        if tr:
            line += ' `/dispatch/state` の遷移: ' + ' → '.join(
                f"{json.loads(v).get('phase')}{'(teleop)' if json.loads(v).get('teleop_active') else ''}@{t}s" for v, t in tr)
        w(line)
sec5b = '\n'.join(out)

for name, body in [('sec1', sec1), ('sec2', sec2), ('sec3', sec3), ('sec4', sec4), ('sec5', sec5), ('sec5b', sec5b)]:
    open(f'{S}/{name}.md', 'w').write(body + '\n')
tot = sum(r['size'] for r in rows)
print('bags', len(rows), 'metadata bag_size total GiB', round(tot / 2**30, 1))
print('dups', dups)
```

### win.py / run_win.sh

```python
"""Count /cmd_vel messages (and non-zero ones) inside teleop_active windows of one bag (read-only)."""
import json, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
from std_msgs.msg import String
p = sys.argv[1]
info = rosbag2_py.Info().read_metadata(p, '')
t0 = info.starting_time.nanoseconds
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=p, storage_id=info.storage_identifier), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/cmd_vel', '/dispatch/state']))
teleop = False
cnt = {True: [0, 0], False: [0, 0]}
while r.has_next():
    t, d, ts = r.read_next()
    if t == '/dispatch/state':
        teleop = bool(json.loads(deserialize_message(d, String).data).get('teleop_active'))
    else:
        m = deserialize_message(d, Twist)
        cnt[teleop][0] += 1
        if abs(m.linear.x) > 1e-4 or abs(m.angular.z) > 1e-4:
            cnt[teleop][1] += 1
print(json.dumps({'teleop_active=true: cmd_vel [n, nonzero]': cnt[True], 'teleop_active=false: cmd_vel [n, nonzero]': cnt[False]}))
```

```bash
source /opt/ros/humble/setup.bash
python3 "$(dirname "$0")/win.py" "$1" 2>/dev/null
```

### check7.sh

```bash
#!/usr/bin/env bash
# Read-only dependency checks mirroring scripts/record_avoidance_bag.sh (no recording, no publishing).
R=/home/systemlab/whill_lab0_ros2/.claude/worktrees/agent-a2c7f7e00817bd323
echo "[0] bash scripts/record_avoidance_bag.sh --help"; bash "$R/scripts/record_avoidance_bag.sh" --help; echo "rc=$?"
echo "[1a] RMW (before sourcing) = ${RMW_IMPLEMENTATION:-未設定}"
X="${CYCLONEDDS_URI:-}"; X="${X#file://}"
echo "[2a] CYCLONEDDS_URI set: $([[ -n $X ]] && echo yes || echo no); basename=$(basename "${X:-none}"); readable=$([[ -r $X ]] && echo yes || echo no)"
echo "[2b] repo configs/cyclonedds-runtime-min.xml exists: $([[ -r $R/configs/cyclonedds-runtime-min.xml ]] && echo yes || echo no)"
echo "[2c] CYCLONEDDS_URI points to same file as repo main checkout: $([[ "$(readlink -f "$X")" == "$(readlink -f /home/systemlab/whill_lab0_ros2/configs/cyclonedds-runtime-min.xml)" ]] && echo yes || echo no)"
echo "[2d] check_dds_nic.py:"; python3 "$R/scripts/check_dds_nic.py" "$X"; echo "rc=$?"
echo "[3] governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"; cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c
echo "[4a] ros2 before sourcing: $(command -v ros2 || echo none)"
source /opt/ros/humble/setup.bash
echo "[4b] ros2 after sourcing /opt/ros/humble: $(command -v ros2 || echo none)"
echo "[1b] RMW after sourcing = ${RMW_IMPLEMENTATION:-未設定}"
echo "[4c] rosbag2_storage_mcap: $(ros2 pkg prefix rosbag2_storage_mcap 2>&1)"
echo "[4d] compressed_image_transport: $(ros2 pkg prefix compressed_image_transport 2>&1)"
echo "[4e] rmw_cyclonedds_cpp: $(ros2 pkg prefix rmw_cyclonedds_cpp 2>&1)"
echo "[4f] velodyne_msgs (probe 型): $(ros2 pkg prefix velodyne_msgs 2>&1)"
source /home/systemlab/whill_lab0_ros2/install/setup.bash
echo "[4g] velodyne_msgs after workspace: $(ros2 pkg prefix velodyne_msgs 2>&1)"
echo "[5] helpers:"; for c in timeout awk sed grep date mkdir python3 ip; do printf '  %-8s %s\n' $c "$(command -v $c || echo MISSING)"; done
timeout --help | grep -c -- '--foreground' | sed 's/^/  timeout --foreground supported lines: /'
ls -l "$R/scripts/check_failsafe_inputs.sh" "$R/scripts/check_dds_nic.py" "$R/scripts/record_avoidance_bag.sh" | awk '{print "  "$1, $NF}'
echo "[6] output dir: WHILL_BAGS_ROOT=${WHILL_BAGS_ROOT:-未設定} -> ${WHILL_BAGS_ROOT:-$HOME/bags}; writable=$([[ -w ${WHILL_BAGS_ROOT:-$HOME/bags} ]] && echo yes || echo no)"
df -h "${WHILL_BAGS_ROOT:-$HOME/bags}"
echo "[7] running robot/ROS processes (pgrep, read-only):"
pgrep -af 'ros2 launch|m6r_bringup|velodyne|whill_node|lidar_localization|realsense|twist_mux|failsafe|controller_server|ros2 bag record|_ros2_daemon' | grep -v pgrep || echo "  (none)"
```

## 付録 E. analyze.py の生の出力（analysis.jsonl、88 行、省略なし）

`/dispatch/state` の値は、判定に使う `phase` と `teleop_active` だけを残している（位置やバッテリーの欄は出力していない）。

```json
{"path": "/home/systemlab/bags/2026-11-16-static/cone_1p5_141326", "storage": "mcap", "duration_s": 11.844292065, "start": "Time(nanoseconds=1794806006912532752, clock_type=SYSTEM_TIME)", "size": 404455447, "messages": 4374, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 352, "rate_hz": 29.718956444865412}, {"name": "/camera/camera/color/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "count": 352, "rate_hz": 29.718956444865412}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 352, "rate_hz": 29.718956444865412}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 352, "rate_hz": 29.718956444865412}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1185, "rate_hz": 100.04819144081112}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1174, "rate_hz": 99.11947405190908}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 16, "rate_hz": 1.3508616565847915}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 356, "rate_hz": 30.05667185901161}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.3377154141461979}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 115, "rate_hz": 9.709318156703189}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 116, "rate_hz": 9.793747010239738}], "camera": {"format_field": "rgb8; jpeg compressed bgr8", "header_size": ["jpeg", 640, 480], "first_stamp_ns": 1794806007028613625, "camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/2026-11-16-static/cone_2p5_141549", "storage": "mcap", "duration_s": 11.839043055, "start": "Time(nanoseconds=1794806149720038315, clock_type=SYSTEM_TIME)", "size": 408512595, "messages": 4394, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 355, "rate_hz": 29.98553163045322}, {"name": "/camera/camera/color/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "count": 355, "rate_hz": 29.98553163045322}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 355, "rate_hz": 29.98553163045322}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 355, "rate_hz": 29.98553163045322}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1184, "rate_hz": 100.0080829590327}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1184, "rate_hz": 100.0080829590327}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 16, "rate_hz": 1.351460580527469}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 352, "rate_hz": 29.73213277160432}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.3378651451318673}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 117, "rate_hz": 9.882555495107118}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 117, "rate_hz": 9.882555495107118}], "camera": {"format_field": "rgb8; jpeg compressed bgr8", "header_size": ["jpeg", 640, 480], "first_stamp_ns": 1794806149734164168, "camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/2026-11-16-static/cone_2p_141506", "storage": "mcap", "duration_s": 11.851269417, "start": "Time(nanoseconds=1794806106359508322, clock_type=SYSTEM_TIME)", "size": 404815669, "messages": 4365, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 353, "rate_hz": 29.785838763705833}, {"name": "/camera/camera/color/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "count": 353, "rate_hz": 29.785838763705833}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 352, "rate_hz": 29.70145961706644}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 352, "rate_hz": 29.70145961706644}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1175, "rate_hz": 99.14549730128712}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1175, "rate_hz": 99.14549730128712}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 16, "rate_hz": 1.3500663462302926}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 352, "rate_hz": 29.70145961706644}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.33751658655757316}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 116, "rate_hz": 9.787981010169622}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 117, "rate_hz": 9.872360156809016}], "camera": {"format_field": "rgb8; jpeg compressed bgr8", "header_size": ["jpeg", 640, 480], "first_stamp_ns": 1794806106469167835, "camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/2026-11-16-static/cone_3p5_141752", "storage": "mcap", "duration_s": 11.837121879, "start": "Time(nanoseconds=1794806273073984570, clock_type=SYSTEM_TIME)", "size": 408400374, "messages": 4397, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 355, "rate_hz": 29.99039831040334}, {"name": "/camera/camera/color/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "count": 355, "rate_hz": 29.99039831040334}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 355, "rate_hz": 29.99039831040334}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 355, "rate_hz": 29.99039831040334}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1184, "rate_hz": 100.02431436483818}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1184, "rate_hz": 100.02431436483818}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 16, "rate_hz": 1.3516799238491646}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 355, "rate_hz": 29.99039831040334}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.33791998096229114}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 117, "rate_hz": 9.884159443147016}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 117, "rate_hz": 9.884159443147016}], "camera": {"format_field": "rgb8; jpeg compressed bgr8", "header_size": ["jpeg", 640, 480], "first_stamp_ns": 1794806273092731776, "camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/2026-11-16-static/cone_3p_141709", "storage": "mcap", "duration_s": 11.80414254, "start": "Time(nanoseconds=1794806229872783339, clock_type=SYSTEM_TIME)", "size": 403694716, "messages": 4359, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 351, "rate_hz": 29.7353237484711}, {"name": "/camera/camera/color/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "count": 351, "rate_hz": 29.7353237484711}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 351, "rate_hz": 29.7353237484711}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 351, "rate_hz": 29.7353237484711}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1180, "rate_hz": 99.96490604899117}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1170, "rate_hz": 99.117745828237}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 16, "rate_hz": 1.35545635320666}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 354, "rate_hz": 29.98947181469735}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.338864088301665}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 115, "rate_hz": 9.742342538672867}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 116, "rate_hz": 9.827058560748284}], "camera": {"format_field": "rgb8; jpeg compressed bgr8", "header_size": ["jpeg", 640, 480], "first_stamp_ns": 1794806229993791215, "camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/2026-11-16-static/cone_4p_141838", "storage": "mcap", "duration_s": 11.845718341, "start": "Time(nanoseconds=1794806319085881287, clock_type=SYSTEM_TIME)", "size": 408384467, "messages": 4387, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 355, "rate_hz": 29.96863421708129}, {"name": "/camera/camera/color/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "count": 355, "rate_hz": 29.96863421708129}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 355, "rate_hz": 29.96863421708129}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 355, "rate_hz": 29.96863421708129}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1185, "rate_hz": 100.0361452034967}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1174, "rate_hz": 99.10753963620685}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 16, "rate_hz": 1.3506990069670441}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 355, "rate_hz": 29.96863421708129}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.33767475174176104}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 116, "rate_hz": 9.792567800511069}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 117, "rate_hz": 9.87698648844651}], "camera": {"format_field": "rgb8; jpeg compressed bgr8", "header_size": ["jpeg", 640, 480], "first_stamp_ns": 1794806319092814741, "camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/2026-11-16-static/cone_5p_141921", "storage": "mcap", "duration_s": 11.848224143, "start": "Time(nanoseconds=1794806361785183118, clock_type=SYSTEM_TIME)", "size": 408677611, "messages": 4392, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 356, "rate_hz": 30.046696931398525}, {"name": "/camera/camera/color/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "count": 356, "rate_hz": 30.046696931398525}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 355, "rate_hz": 29.962296097321563}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 355, "rate_hz": 29.962296097321563}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1175, "rate_hz": 99.17098004043052}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1185, "rate_hz": 100.01498838120014}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 16, "rate_hz": 1.3504133452313944}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 356, "rate_hz": 30.046696931398525}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.3376033363078486}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 117, "rate_hz": 9.874897587004572}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 117, "rate_hz": 9.874897587004572}], "camera": {"format_field": "rgb8; jpeg compressed bgr8", "header_size": ["jpeg", 640, 480], "first_stamp_ns": 1794806361791399429, "camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/2026-11-16-static/empty_start_140915", "storage": "mcap", "duration_s": 11.841227679, "start": "Time(nanoseconds=1794805756001339753, clock_type=SYSTEM_TIME)", "size": 408618552, "messages": 4395, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 355, "rate_hz": 29.979999508799246}, {"name": "/camera/camera/color/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "count": 355, "rate_hz": 29.979999508799246}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 355, "rate_hz": 29.979999508799246}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 355, "rate_hz": 29.979999508799246}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1184, "rate_hz": 99.98963216455861}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1184, "rate_hz": 99.98963216455861}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 14, "rate_hz": 1.1823098397836322}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 355, "rate_hz": 29.979999508799246}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.33780281136675205}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 117, "rate_hz": 9.880732232477497}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 117, "rate_hz": 9.880732232477497}], "camera": {"format_field": "rgb8; jpeg compressed bgr8", "header_size": ["jpeg", 640, 480], "first_stamp_ns": 1794805756010217074, "camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/2026-11-16-test/smoke_140741", "storage": "mcap", "duration_s": 4.843063743, "start": "Time(nanoseconds=1794805661942980892, clock_type=SYSTEM_TIME)", "size": 167793431, "messages": 1812, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 145, "rate_hz": 29.93972569730846}, {"name": "/camera/camera/color/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "count": 145, "rate_hz": 29.93972569730846}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 146, "rate_hz": 30.14620656418645}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 146, "rate_hz": 30.14620656418645}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 484, "rate_hz": 99.93673956894686}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 484, "rate_hz": 99.93673956894686}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 16, "rate_hz": 3.30369387004783}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 146, "rate_hz": 30.14620656418645}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.8259234675119576}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 48, "rate_hz": 9.91108161014349}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 48, "rate_hz": 9.91108161014349}], "camera": {"format_field": "rgb8; jpeg compressed bgr8", "header_size": ["jpeg", 640, 480], "first_stamp_ns": 1794805661974757736, "camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/imu_investigation_20261030/seg_0_300_hires", "storage": "sqlite3", "duration_s": 299.998928351, "start": "Time(nanoseconds=1783663663918712370, clock_type=SYSTEM_TIME)", "size": 1914370874, "messages": 32961, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 30000, "rate_hz": 100.00035721760936}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.013333380962347915}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2957, "rate_hz": 9.856701876415697}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/imu_investigation_20261030/seg_0_300_orig", "storage": "sqlite3", "duration_s": 299.998928351, "start": "Time(nanoseconds=1783663663918712370, clock_type=SYSTEM_TIME)", "size": 1914370872, "messages": 32961, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 30000, "rate_hz": 100.00035721760936}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.013333380962347915}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2957, "rate_hz": 9.856701876415697}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m3-bench-data/m3_chair_motion_2026-05-07", "storage": "sqlite3", "duration_s": 96.857189845, "start": "Time(nanoseconds=1778159813562470446, clock_type=SYSTEM_TIME)", "size": 5670277391, "messages": 26137, "topics": [{"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 2904, "rate_hz": 29.98228633978804}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 2904, "rate_hz": 29.98228633978804}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 9687, "rate_hz": 100.01322581733014}, {"name": "/imu/mag", "type": "sensor_msgs/msg/MagneticField", "count": 9687, "rate_hz": 100.01322581733014}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.041297915068578564}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 951, "rate_hz": 9.818579307554554}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m3-bench-data/m3_chair_static_2026-05-07", "storage": "sqlite3", "duration_s": 19.850282091, "start": "Time(nanoseconds=1778159261781267379, clock_type=SYSTEM_TIME)", "size": 1162510603, "messages": 5362, "topics": [{"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 595, "rate_hz": 29.974385113134964}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 595, "rate_hz": 29.974385113134964}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1986, "rate_hz": 100.04895602468241}, {"name": "/imu/mag", "type": "sensor_msgs/msg/MagneticField", "count": 1986, "rate_hz": 100.04895602468241}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.20150847134880648}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 196, "rate_hz": 9.873915096091517}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m3-bench-data/m3_smoke_2026-05-07", "storage": "sqlite3", "duration_s": 7.834444034, "start": "Time(nanoseconds=1778157537511399105, clock_type=SYSTEM_TIME)", "size": 458108270, "messages": 2115, "topics": [{"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 235, "rate_hz": 29.995746855825967}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 234, "rate_hz": 29.86810537984373}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 784, "rate_hz": 100.07091717007472}, {"name": "/imu/mag", "type": "sensor_msgs/msg/MagneticField", "count": 784, "rate_hz": 100.07091717007472}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 1, "rate_hz": 0.12764147598223816}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 77, "rate_hz": 9.828393650632337}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08", "storage": "sqlite3", "duration_s": 59.838429808, "start": "Time(nanoseconds=1778205232369641627, clock_type=SYSTEM_TIME)", "size": 3504554890, "messages": 16246, "topics": [{"name": "/Odometry", "type": "nav_msgs/msg/Odometry", "count": 48, "rate_hz": 0.8021600859851226}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 1794, "rate_hz": 29.980733213693956}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 1794, "rate_hz": 29.980733213693956}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 5984, "rate_hz": 100.00262405281194}, {"name": "/imu/mag", "type": "sensor_msgs/msg/MagneticField", "count": 5984, "rate_hz": 100.00262405281194}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 48, "rate_hz": 0.8021600859851226}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.06684667383209354}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 590, "rate_hz": 9.859884390233798}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run1", "storage": "sqlite3", "duration_s": 59.50177852, "start": "Time(nanoseconds=1778206214947801802, clock_type=SYSTEM_TIME)", "size": 3486045076, "messages": 16240, "topics": [{"name": "/Odometry", "type": "nav_msgs/msg/Odometry", "count": 88, "rate_hz": 1.478947389285533}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 1784, "rate_hz": 29.98229707369762}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 1785, "rate_hz": 29.999103294030412}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 5952, "rate_hz": 100.03062342076694}, {"name": "/imu/mag", "type": "sensor_msgs/msg/MagneticField", "count": 5952, "rate_hz": 100.03062342076694}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 88, "rate_hz": 1.478947389285533}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.06722488133116059}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 587, "rate_hz": 9.865251335347816}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run2", "storage": "sqlite3", "duration_s": 59.502864352, "start": "Time(nanoseconds=1778206335940332781, clock_type=SYSTEM_TIME)", "size": 3476329364, "messages": 16212, "topics": [{"name": "/Odometry", "type": "nav_msgs/msg/Odometry", "count": 81, "rate_hz": 1.3612790053404789}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 1784, "rate_hz": 29.98174994478289}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 1781, "rate_hz": 29.931332203844356}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 5952, "rate_hz": 100.02879802205592}, {"name": "/imu/mag", "type": "sensor_msgs/msg/MagneticField", "count": 5952, "rate_hz": 100.02879802205592}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 81, "rate_hz": 1.3612790053404789}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.067223654584715}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 577, "rate_hz": 9.69701217384514}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m3-bench-data/m4_chair_live_2026-05-08_run3", "storage": "sqlite3", "duration_s": 59.8398536, "start": "Time(nanoseconds=1778206492993765624, clock_type=SYSTEM_TIME)", "size": 3504624836, "messages": 16172, "topics": [{"name": "/Odometry", "type": "nav_msgs/msg/Odometry", "count": 108, "rate_hz": 1.804817249753432}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 1794, "rate_hz": 29.98001987090423}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 1794, "rate_hz": 29.98001987090423}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 5887, "rate_hz": 98.3792513823931}, {"name": "/imu/mag", "type": "sensor_msgs/msg/MagneticField", "count": 5887, "rate_hz": 98.3792513823931}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 108, "rate_hz": 1.804817249753432}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.06684508332420118}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 590, "rate_hz": 9.859649790319674}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m3-bench-data/m5_chair_mapping_2026-05-08", "storage": "sqlite3", "duration_s": 119.832125215, "start": "Time(nanoseconds=1778209905829222336, clock_type=SYSTEM_TIME)", "size": 7017353833, "messages": 32724, "topics": [{"name": "/Odometry", "type": "nav_msgs/msg/Odometry", "count": 290, "rate_hz": 2.420052214543377}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 3592, "rate_hz": 29.97526742979245}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 3592, "rate_hz": 29.97526742979245}, {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 11886, "rate_hz": 99.18876076573302}, {"name": "/imu/mag", "type": "sensor_msgs/msg/MagneticField", "count": 11886, "rate_hz": 99.18876076573302}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 290, "rate_hz": 2.420052214543377}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 6, "rate_hz": 0.050070045818138835}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 1182, "rate_hz": 9.86379902617335}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag", "storage": "sqlite3", "duration_s": 199.297559061, "start": "Time(nanoseconds=1782016682455624281, clock_type=SYSTEM_TIME)", "size": 1273847028, "messages": 27879, "topics": [{"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 19931, "rate_hz": 100.00624239406575}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 5979, "rate_hz": 30.000367431344095}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.020070491675092218}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 1965, "rate_hz": 9.859629035389052}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-06-21-loop-outdoor/bag-imu-fixed", "storage": "sqlite3", "duration_s": 199.297559061, "start": "Time(nanoseconds=1782016682455624281, clock_type=SYSTEM_TIME)", "size": 1273847048, "messages": 27879, "topics": [{"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 19931, "rate_hz": 100.00624239406575}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 5979, "rate_hz": 30.000367431344095}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.020070491675092218}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 1965, "rate_hz": 9.859629035389052}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag", "storage": "sqlite3", "duration_s": 239.446921453, "start": "Time(nanoseconds=1782276582037883645, clock_type=SYSTEM_TIME)", "size": 1529207283, "messages": 26310, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 23945, "rate_hz": 100.00128569078329}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.01670516361508178}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2361, "rate_hz": 9.86022282380202}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-07-campus-half-v3/bag", "storage": "sqlite3", "duration_s": 466.260125959, "start": "Time(nanoseconds=1783420240129157468, clock_type=SYSTEM_TIME)", "size": 2973195041, "messages": 51220, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 46626, "rate_hz": 99.99997298525159}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.008578902156329223}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 4590, "rate_hz": 9.844290224387782}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-07-campus-loop/bag", "storage": "sqlite3", "duration_s": 821.453343469, "start": "Time(nanoseconds=1783418912923778807, clock_type=SYSTEM_TIME)", "size": 5078779799, "messages": 88669, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 80821, "rate_hz": 98.38781550111254}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.0048694183690433206}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 7844, "rate_hz": 9.54892942169395}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-postgap/bag", "storage": "sqlite3", "duration_s": 1817.671593993, "start": "Time(nanoseconds=1783495982713583776, clock_type=SYSTEM_TIME)", "size": 10908141471, "messages": 187770, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 170909, "rate_hz": 94.02633598105191}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 12, "rate_hz": 0.006601852633697599}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 16849, "rate_hz": 9.269551252097571}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-pregap/bag", "storage": "sqlite3", "duration_s": 1864.494849877, "start": "Time(nanoseconds=1783494022000533605, clock_type=SYSTEM_TIME)", "size": 11899332510, "messages": 204829, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 186449, "rate_hz": 99.99973988252098}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 0, "rate_hz": 0.0}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 18380, "rate_hz": 9.85789797231809}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-first10/bag", "storage": "sqlite3", "duration_s": 599.579395776, "start": "Time(nanoseconds=1783504344414614520, clock_type=SYSTEM_TIME)", "size": 3826817826, "messages": 65883, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 59968, "rate_hz": 100.016779132957}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.006671343325303962}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 5911, "rate_hz": 9.85857759896793}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2-pregap/bag", "storage": "sqlite3", "duration_s": 2602.085145648, "start": "Time(nanoseconds=1783504344414614520, clock_type=SYSTEM_TIME)", "size": 16558459688, "messages": 284313, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 258728, "rate_hz": 99.43102762518123}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 8, "rate_hz": 0.0030744574263375044}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 25577, "rate_hz": 9.829424699179294}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo-v2/bag", "storage": "sqlite3", "duration_s": 3482.547239425, "start": "Time(nanoseconds=1783504344414614520, clock_type=SYSTEM_TIME)", "size": 21324012329, "messages": 366318, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 333360, "rate_hz": 95.72303750143551}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 20, "rate_hz": 0.00574292281626083}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 32938, "rate_hz": 9.458019586099962}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-demo/bag", "storage": "sqlite3", "duration_s": 3778.41763842, "start": "Time(nanoseconds=1783494021967539349, clock_type=SYSTEM_TIME)", "size": 22839855007, "messages": 393165, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 357866, "rate_hz": 94.71319325876502}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 20, "rate_hz": 0.005293221108390572}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 35279, "rate_hz": 9.33697737414555}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-debiased/bag", "storage": "sqlite3", "duration_s": 2813.311171606, "start": "Time(nanoseconds=1783512322532861237, clock_type=SYSTEM_TIME)", "size": 17929881804, "messages": 308557, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 280858, "rate_hz": 99.83182906840345}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.0014218121480378476}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 27695, "rate_hz": 9.844271859977047}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-A/bag", "storage": "sqlite3", "duration_s": 1325.065490763, "start": "Time(nanoseconds=1783512322532861237, clock_type=SYSTEM_TIME)", "size": 8456411340, "messages": 145573, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 132507, "rate_hz": 100.00034030295343}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.0030187187183455494}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 13062, "rate_hz": 9.857625974757392}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer-seg-B/bag", "storage": "sqlite3", "duration_s": 1483.475685843, "start": "Time(nanoseconds=1783513652368347000, clock_type=SYSTEM_TIME)", "size": 9473509580, "messages": 162987, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 148350, "rate_hz": 100.00163899936021}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.0026963704482469895}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 14633, "rate_hz": 9.863997192299548}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-campus-outer/bag", "storage": "sqlite3", "duration_s": 2813.311171606, "start": "Time(nanoseconds=1783512322532861237, clock_type=SYSTEM_TIME)", "size": 17929881804, "messages": 308557, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 280858, "rate_hz": 99.83182906840345}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.0014218121480378476}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 27695, "rate_hz": 9.844271859977047}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-indoor-calib/bag", "storage": "sqlite3", "duration_s": 365.849648203, "start": "Time(nanoseconds=1783474335828510530, clock_type=SYSTEM_TIME)", "size": 2335181602, "messages": 40196, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 36585, "rate_hz": 100.00009615890072}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.010933453181238291}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 3607, "rate_hz": 9.859241406181628}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-08-outdoor-corrected/bag", "storage": "sqlite3", "duration_s": 745.40041554, "start": "Time(nanoseconds=1783480975634654316, clock_type=SYSTEM_TIME)", "size": 4757129414, "messages": 81892, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 74540, "rate_hz": 99.99994425278128}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.0053662433191725934}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 7348, "rate_hz": 9.857788977320054}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-09-indoor-calib/bag", "storage": "sqlite3", "duration_s": 209.149863567, "start": "Time(nanoseconds=1783582014866675279, clock_type=SYSTEM_TIME)", "size": 1334950690, "messages": 22980, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 20914, "rate_hz": 99.99528397158296}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.019125042358531696}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2062, "rate_hz": 9.858959335823089}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag", "storage": "sqlite3", "duration_s": 2161.984223891, "start": "Time(nanoseconds=1783663663918712370, clock_type=SYSTEM_TIME)", "size": 13796883240, "messages": 237506, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 216191, "rate_hz": 99.99656686250621}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.0018501522609638}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 21311, "rate_hz": 9.857148708349886}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires", "storage": "sqlite3", "duration_s": 2161.984223891, "start": "Time(nanoseconds=1783663663918712370, clock_type=SYSTEM_TIME)", "size": 13796883252, "messages": 237506, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 216191, "rate_hz": 99.99656686250621}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.0018501522609638}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 21311, "rate_hz": 9.857148708349886}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m5r-bench-data/2026-07-10-dds-verify/bag", "storage": "sqlite3", "duration_s": 303.712693566, "start": "Time(nanoseconds=1783662446382947360, clock_type=SYSTEM_TIME)", "size": 1938340642, "messages": 33366, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 30368, "rate_hz": 99.98923536398294}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.013170341855108396}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2994, "rate_hz": 9.858000878548633}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m6r-bench-data/2026-07-14-verify-campus/bag", "storage": "sqlite3", "duration_s": 627.703662524, "start": "Time(nanoseconds=1784004936166902971, clock_type=SYSTEM_TIME)", "size": 4014115189, "messages": 95179, "topics": [{"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 62760, "rate_hz": 99.98348543585308}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 24663, "rate_hz": 39.290833354117986}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 6187, "rate_hz": 9.856561892791955}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 1569, "rate_hz": 2.499587135896327}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_13_41", "storage": "sqlite3", "duration_s": 783.785092746, "start": "Time(nanoseconds=1784528021665185378, clock_type=SYSTEM_TIME)", "size": 5032136563, "messages": 127062, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 6699, "rate_hz": 8.54698572606169}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 826, "rate_hz": 1.0538603089605845}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 3919, "rate_hz": 5.000095097840837}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 77876, "rate_hz": 99.35886854795942}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 23363, "rate_hz": 29.807915736375467}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 6701, "rate_hz": 8.549537445938109}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 7678, "rate_hz": 9.796052605568242}], "camera": {}, "mode": {"/dispatch/state": {"n": 3919, "nonzero": null, "first_rel_s": 0.184847332, "last_rel_s": 783.785092746}, "/cmd_vel": {"n": 826, "nonzero": 0, "first_rel_s": 99.17346082, "last_rel_s": 769.823114204}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 3919}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.2]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_29_43", "storage": "sqlite3", "duration_s": 257.560123465, "start": "Time(nanoseconds=1784528984057756758, clock_type=SYSTEM_TIME)", "size": 1656975214, "messages": 40555, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 637, "rate_hz": 2.4732089402285222}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 2591, "rate_hz": 10.059787070851023}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 1288, "rate_hz": 5.000774120901627}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 25752, "rate_hz": 99.98442170920707}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 7727, "rate_hz": 30.00076213680658}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 21, "rate_hz": 0.08153436066687436}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2539, "rate_hz": 9.85789246348543}], "camera": {}, "mode": {"/dispatch/state": {"n": 1288, "nonzero": null, "first_rel_s": 0.107699165, "last_rel_s": 257.507494928}, "/cmd_vel": {"n": 2591, "nonzero": 0, "first_rel_s": 57.998615903, "last_rel_s": 257.54803891}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 1288}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.1]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_44_08", "storage": "sqlite3", "duration_s": 270.873453734, "start": "Time(nanoseconds=1784529848650242559, clock_type=SYSTEM_TIME)", "size": 1740369770, "messages": 39877, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 0, "rate_hz": 0.0}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 637, "rate_hz": 2.351651633701763}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 1355, "rate_hz": 5.0023358927251}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 27088, "rate_hz": 100.00241672482473}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 8126, "rate_hz": 29.999248313124845}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 0, "rate_hz": 0.0}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2671, "rate_hz": 9.860693113999071}], "camera": {}, "mode": {"/dispatch/state": {"n": 1355, "nonzero": null, "first_rel_s": 0.031533996, "last_rel_s": 270.831303841}, "/cmd_vel": {"n": 637, "nonzero": 0, "first_rel_s": 228.05442204, "last_rel_s": 270.854582218}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 1355}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.0]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-15_59_27", "storage": "sqlite3", "duration_s": 632.601942113, "start": "Time(nanoseconds=1784530767624882293, clock_type=SYSTEM_TIME)", "size": 4070809460, "messages": 107887, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 6065, "rate_hz": 9.587387575418834}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 4531, "rate_hz": 7.16248196277374}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 3171, "rate_hz": 5.012630832918899}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 62962, "rate_hz": 99.52862267494157}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 18889, "rate_hz": 29.85921911163831}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 6061, "rate_hz": 9.581064483860438}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 6208, "rate_hz": 9.813438098631512}], "camera": {}, "mode": {"/dispatch/state": {"n": 3171, "nonzero": null, "first_rel_s": 0.20185277, "last_rel_s": 632.601942113}, "/cmd_vel": {"n": 4531, "nonzero": 3923, "first_rel_s": 14.353830438, "last_rel_s": 629.653686907}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 720, "{\"phase\": \"QUEUED\"}": 4, "{\"phase\": \"ACTIVE\"}": 2063, "{\"phase\": \"ABORTED\"}": 384}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.2], ["{\"phase\": \"QUEUED\"}", 144.1], ["{\"phase\": \"ACTIVE\"}", 144.1], ["{\"phase\": \"ABORTED\"}", 473.8], ["{\"phase\": \"QUEUED\"}", 544.8], ["{\"phase\": \"ACTIVE\"}", 544.8], ["{\"phase\": \"ABORTED\"}", 627.3]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-17_03_07", "storage": "sqlite3", "duration_s": 462.922195598, "start": "Time(nanoseconds=1784534587613544907, clock_type=SYSTEM_TIME)", "size": 2992553842, "messages": 79613, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 4564, "rate_hz": 9.859108168499576}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 3422, "rate_hz": 7.392170936153714}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 2319, "rate_hz": 5.009481122425617}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 46293, "rate_hz": 100.00168589928809}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 13888, "rate_hz": 30.000721788808526}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 4564, "rate_hz": 9.859108168499576}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 4563, "rate_hz": 9.856947978278608}], "camera": {}, "mode": {"/dispatch/state": {"n": 2319, "nonzero": null, "first_rel_s": 0.099418742, "last_rel_s": 462.899471169}, "/cmd_vel": {"n": 3422, "nonzero": 2889, "first_rel_s": 45.119863745, "last_rel_s": 452.569449197}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 517, "{\"phase\": \"QUEUED\"}": 2, "{\"phase\": \"ACTIVE\"}": 1522, "{\"phase\": \"ABORTED\"}": 278}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.1], ["{\"phase\": \"QUEUED\"}", 103.4], ["{\"phase\": \"ACTIVE\"}", 103.4], ["{\"phase\": \"ABORTED\"}", 407.6]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-18_09_28", "storage": "sqlite3", "duration_s": 543.006767426, "start": "Time(nanoseconds=1784538568982361202, clock_type=SYSTEM_TIME)", "size": 3511025522, "messages": 93025, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 5354, "rate_hz": 9.859913947996299}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 3653, "rate_hz": 6.727356304077415}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 2719, "rate_hz": 5.007304076317134}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 54301, "rate_hz": 100.00059531007602}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 16291, "rate_hz": 30.001467711394792}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 5354, "rate_hz": 9.859913947996299}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 5353, "rate_hz": 9.858072350322038}], "camera": {}, "mode": {"/dispatch/state": {"n": 2719, "nonzero": null, "first_rel_s": 0.031346442, "last_rel_s": 542.831424692}, "/cmd_vel": {"n": 3653, "nonzero": 2571, "first_rel_s": 20.008509137, "last_rel_s": 452.38115772}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 948, "{\"phase\": \"QUEUED\"}": 2, "{\"phase\": \"ACTIVE\"}": 1315, "{\"phase\": \"SUCCEEDED\"}": 454}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.0], ["{\"phase\": \"QUEUED\"}", 189.6], ["{\"phase\": \"ACTIVE\"}", 189.6], ["{\"phase\": \"SUCCEEDED\"}", 452.4]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-20-live-campus/rosbag2_2026_07_20-19_58_48", "storage": "sqlite3", "duration_s": 275.533821354, "start": "Time(nanoseconds=1784545128439445392, clock_type=SYSTEM_TIME)", "size": 1781186417, "messages": 47662, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 2717, "rate_hz": 9.860858411676642}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 2310, "rate_hz": 8.383725775109696}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 1381, "rate_hz": 5.012088872479}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 27554, "rate_hz": 100.002242427434}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 8267, "rate_hz": 30.003576183044093}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 2717, "rate_hz": 9.860858411676642}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2716, "rate_hz": 9.857229093159278}], "camera": {}, "mode": {"/dispatch/state": {"n": 1381, "nonzero": null, "first_rel_s": 0.177124032, "last_rel_s": 275.377000857}, "/cmd_vel": {"n": 2310, "nonzero": 2309, "first_rel_s": 19.760570781, "last_rel_s": 250.659332342}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 97, "{\"phase\": \"QUEUED\"}": 2, "{\"phase\": \"ACTIVE\"}": 1157, "{\"phase\": \"SUCCEEDED\"}": 125}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.2], ["{\"phase\": \"QUEUED\"}", 19.4], ["{\"phase\": \"ACTIVE\"}", 19.4], ["{\"phase\": \"SUCCEEDED\"}", 250.7]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_34_01", "storage": "sqlite3", "duration_s": 255.800057087, "start": "Time(nanoseconds=1784619241771628869, clock_type=SYSTEM_TIME)", "size": 1655136578, "messages": 42856, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 2245, "rate_hz": 8.776385844341132}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 1225, "rate_hz": 4.788896507491263}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 1283, "rate_hz": 5.015636097233706}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 25580, "rate_hz": 99.99997768296042}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 7674, "rate_hz": 29.999993304888125}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 2240, "rate_hz": 8.756839327984023}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 88, "rate_hz": 0.34401868788508666}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2521, "rate_hz": 9.855353547253449}], "camera": {}, "mode": {"/dispatch/state": {"n": 1283, "nonzero": null, "first_rel_s": 0.125504454, "last_rel_s": 255.725470128}, "/cmd_vel": {"n": 1225, "nonzero": 1134, "first_rel_s": 27.872031555, "last_rel_s": 238.741035308}, "/plan": {"n": 88, "nonzero": null, "first_rel_s": 123.380699573, "last_rel_s": 238.801421246}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 616, "{\"phase\": \"QUEUED\"}": 2, "{\"phase\": \"ACTIVE\"}": 578, "{\"phase\": \"CANCELED\"}": 87}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.1], ["{\"phase\": \"QUEUED\"}", 123.1], ["{\"phase\": \"ACTIVE\"}", 123.1], ["{\"phase\": \"CANCELED\"}", 238.7]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_50_48", "storage": "sqlite3", "duration_s": 119.16698093, "start": "Time(nanoseconds=1784620248724739797, clock_type=SYSTEM_TIME)", "size": 769900863, "messages": 20920, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 1018, "rate_hz": 8.54263481423587}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 2030, "rate_hz": 17.034920110902572}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 571, "rate_hz": 4.791595755332693}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 11917, "rate_hz": 100.00253347863347}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 3576, "rate_hz": 30.008312471225416}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 633, "rate_hz": 5.311874103547452}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 0, "rate_hz": 0.0}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 1175, "rate_hz": 9.860113857295822}], "camera": {}, "mode": {"/dispatch/state": {"n": 571, "nonzero": null, "first_rel_s": 0.07204625, "last_rel_s": 114.071927169}, "/cmd_vel": {"n": 2030, "nonzero": 0, "first_rel_s": 15.704247168, "last_rel_s": 119.153784737}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 571}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.1]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-16_53_31", "storage": "sqlite3", "duration_s": 506.126301567, "start": "Time(nanoseconds=1784620412053926478, clock_type=SYSTEM_TIME)", "size": 3289731396, "messages": 85029, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 4677, "rate_hz": 9.24077643370776}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 2206, "rate_hz": 4.358595854770006}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 2547, "rate_hz": 5.03234072624624}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 50613, "rate_hz": 100.00073073321592}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 15184, "rate_hz": 30.00041679910597}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 4647, "rate_hz": 9.181502691349147}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 166, "rate_hz": 0.327981374384325}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 4989, "rate_hz": 9.857223354237334}], "camera": {}, "mode": {"/dispatch/state": {"n": 2547, "nonzero": null, "first_rel_s": 0.0905093, "last_rel_s": 506.090549164}, "/cmd_vel": {"n": 2206, "nonzero": 2107, "first_rel_s": 31.587400442, "last_rel_s": 492.667347714}, "/plan": {"n": 166, "nonzero": null, "first_rel_s": 116.08323897, "last_rel_s": 491.789470388}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 579, "{\"phase\": \"QUEUED\"}": 8, "{\"phase\": \"ACTIVE\"}": 1247, "{\"phase\": \"SUCCEEDED\"}": 321, "{\"phase\": \"ABORTED\"}": 392}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.1], ["{\"phase\": \"QUEUED\"}", 115.8], ["{\"phase\": \"ACTIVE\"}", 115.8], ["{\"phase\": \"SUCCEEDED\"}", 171.1], ["{\"phase\": \"QUEUED\"}", 203.2], ["{\"phase\": \"ACTIVE\"}", 203.2], ["{\"phase\": \"SUCCEEDED\"}", 313.5], ["{\"phase\": \"QUEUED\"}", 345.4], ["{\"phase\": \"ACTIVE\"}", 345.4], ["{\"phase\": \"ABORTED\"}", 399.7], ["{\"phase\": \"QUEUED\"}", 464.4], ["{\"phase\": \"ACTIVE\"}", 464.4], ["{\"phase\": \"ABORTED\"}", 492.7]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-17_20_34", "storage": "sqlite3", "duration_s": 595.139138775, "start": "Time(nanoseconds=1784622034997606606, clock_type=SYSTEM_TIME)", "size": 3874980164, "messages": 99958, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 5293, "rate_hz": 8.893718552765334}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 2955, "rate_hz": 4.965225453130845}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 2988, "rate_hz": 5.020674671389159}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 59491, "rate_hz": 99.96149828501086}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 17853, "rate_hz": 29.998027077747874}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 5294, "rate_hz": 8.895398832106494}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 218, "rate_hz": 0.3663008963731046}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 5866, "rate_hz": 9.856518615250604}], "camera": {}, "mode": {"/dispatch/state": {"n": 2988, "nonzero": null, "first_rel_s": 0.064657821, "last_rel_s": 595.066259541}, "/cmd_vel": {"n": 2955, "nonzero": 2825, "first_rel_s": 35.84643122, "last_rel_s": 568.134742655}, "/plan": {"n": 218, "nonzero": null, "first_rel_s": 220.160234684, "last_rel_s": 567.976687639}, "/dispatch/state#values": {"{\"phase\": \"IDLE\"}": 1100, "{\"phase\": \"QUEUED\"}": 6, "{\"phase\": \"ACTIVE\"}": 1577, "{\"phase\": \"SUCCEEDED\"}": 169, "{\"phase\": \"ABORTED\"}": 136}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\"}", 0.1], ["{\"phase\": \"QUEUED\"}", 219.9], ["{\"phase\": \"ACTIVE\"}", 219.9], ["{\"phase\": \"SUCCEEDED\"}", 350.6], ["{\"phase\": \"QUEUED\"}", 363.4], ["{\"phase\": \"ACTIVE\"}", 363.4], ["{\"phase\": \"SUCCEEDED\"}", 502.1], ["{\"phase\": \"QUEUED\"}", 522.8], ["{\"phase\": \"ACTIVE\"}", 522.8], ["{\"phase\": \"ABORTED\"}", 568.2]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-21-live-campus/rosbag2_2026_07_21-18_29_14", "storage": "sqlite3", "duration_s": 337.380523968, "start": "Time(nanoseconds=1784626154230012670, clock_type=SYSTEM_TIME)", "size": 2183209283, "messages": 56566, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 2958, "rate_hz": 8.76754818331055}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 1675, "rate_hz": 4.964720489197151}, {"name": "/dispatch/state", "type": "std_msgs/msg/String", "count": 1702, "rate_hz": 5.044748819470777}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 33738, "rate_hz": 99.99984469524387}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 10121, "rate_hz": 29.99876780368022}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 2950, "rate_hz": 8.743836085451699}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 96, "rate_hz": 0.2845451743062248}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 3326, "rate_hz": 9.858304684817746}], "camera": {}, "mode": {"/dispatch/state": {"n": 1702, "nonzero": null, "first_rel_s": 0.108108313, "last_rel_s": 337.308181656}, "/cmd_vel": {"n": 1675, "nonzero": 1261, "first_rel_s": 37.145501515, "last_rel_s": 328.70894935}, "/plan": {"n": 96, "nonzero": null, "first_rel_s": 130.2781576, "last_rel_s": 310.894520435}, "/dispatch/state#values": {"{\"phase\": \"IDLE\", \"teleop_active\": false}": 650, "{\"phase\": \"QUEUED\", \"teleop_active\": false}": 6, "{\"phase\": \"ACTIVE\", \"teleop_active\": false}": 654, "{\"phase\": \"SUCCEEDED\", \"teleop_active\": false}": 206, "{\"phase\": \"SUCCEEDED\", \"teleop_active\": true}": 186}, "/dispatch/state#transitions": [["{\"phase\": \"IDLE\", \"teleop_active\": false}", 0.1], ["{\"phase\": \"QUEUED\", \"teleop_active\": false}", 130.0], ["{\"phase\": \"ACTIVE\", \"teleop_active\": false}", 130.0], ["{\"phase\": \"SUCCEEDED\", \"teleop_active\": false}", 169.3], ["{\"phase\": \"QUEUED\", \"teleop_active\": false}", 183.2], ["{\"phase\": \"ACTIVE\", \"teleop_active\": false}", 183.2], ["{\"phase\": \"SUCCEEDED\", \"teleop_active\": false}", 207.0], ["{\"phase\": \"SUCCEEDED\", \"teleop_active\": true}", 216.9], ["{\"phase\": \"SUCCEEDED\", \"teleop_active\": false}", 230.6], ["{\"phase\": \"QUEUED\", \"teleop_active\": false}", 244.0], ["{\"phase\": \"ACTIVE\", \"teleop_active\": false}", 244.0], ["{\"phase\": \"SUCCEEDED\", \"teleop_active\": false}", 311.0], ["{\"phase\": \"SUCCEEDED\", \"teleop_active\": true}", 314.4]]}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15", "storage": "sqlite3", "duration_s": 132.461762187, "start": "Time(nanoseconds=1784710875535629800, clock_type=SYSTEM_TIME)", "size": 918456666, "messages": 31035, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 1306, "rate_hz": 9.859449085059603}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 1019, "rate_hz": 7.692786077852785}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 1041, "rate_hz": 7.858871743910449}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 0, "rate_hz": 0.0}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 15, "rate_hz": 0.11324022685749928}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.007549348457166619}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 13247, "rate_hz": 100.00621901208619}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 156, "rate_hz": 1.1776983593179924}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 662, "rate_hz": 4.997668678644302}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 3974, "rate_hz": 30.00111076878014}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 1306, "rate_hz": 9.859449085059603}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 83, "rate_hz": 0.6265959219448294}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 1306, "rate_hz": 9.859449085059603}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 5279, "rate_hz": 39.85301050538258}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.022648045371499854}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 1306, "rate_hz": 9.859449085059603}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 331, "rate_hz": 2.498834339322151}], "camera": {}, "mode": {"/plan": {"n": 83, "nonzero": null, "first_rel_s": 26.983625913, "last_rel_s": 132.075163116}, "/cmd_vel_nav": {"n": 1041, "nonzero": 1038, "first_rel_s": 27.002442053, "last_rel_s": 132.43204469}, "/cmd_vel": {"n": 1019, "nonzero": 1001, "first_rel_s": 27.002648928, "last_rel_s": 132.432038059}, "/cmd_vel_teleop": {"n": 15, "nonzero": 0, "first_rel_s": 124.969567478, "last_rel_s": 130.245990401}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_23_13", "storage": "sqlite3", "duration_s": 174.392238293, "start": "Time(nanoseconds=1784712193694766403, clock_type=SYSTEM_TIME)", "size": 1095051610, "messages": 40309, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 1574, "rate_hz": 9.025631045319175}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 1577, "rate_hz": 9.042833645786745}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 758, "rate_hz": 4.346523718139729}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 819, "rate_hz": 4.696309927647016}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 0, "rate_hz": 0.0}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.005734200155857162}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 17440, "rate_hz": 100.00445071814892}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 186, "rate_hz": 1.0665612289894322}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 847, "rate_hz": 4.856867532011017}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 5232, "rate_hz": 30.001335215444676}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 1574, "rate_hz": 9.025631045319175}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 58, "rate_hz": 0.33258360903971546}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 1573, "rate_hz": 9.019896845163316}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 6805, "rate_hz": 39.02123206060799}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.017202600467571487}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 1573, "rate_hz": 9.019896845163316}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 289, "rate_hz": 1.65718384504272}], "camera": {}, "mode": {"/plan": {"n": 58, "nonzero": null, "first_rel_s": 37.232367486, "last_rel_s": 109.716574247}, "/cmd_vel_nav": {"n": 758, "nonzero": 755, "first_rel_s": 37.251473739, "last_rel_s": 112.829951233}, "/cmd_vel": {"n": 1577, "nonzero": 755, "first_rel_s": 37.251628749, "last_rel_s": 174.376434628}, "/cmd_vel_safety": {"n": 819, "nonzero": 0, "first_rel_s": 117.426579609, "last_rel_s": 174.376244333}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_21_43", "storage": "sqlite3", "duration_s": 120.959567458, "start": "Time(nanoseconds=1784874103231724569, clock_type=SYSTEM_TIME)", "size": 46016837, "messages": 10982, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 0, "rate_hz": 0.0}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 843, "rate_hz": 6.969270953227486}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 1, "rate_hz": 0.008267225330044467}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 843, "rate_hz": 6.969270953227486}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 0, "rate_hz": 0.0}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.008267225330044467}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 5709, "rate_hz": 47.197589409223866}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.008267225330044467}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 0, "rate_hz": 0.0}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 1716, "rate_hz": 14.186558666356305}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 1, "rate_hz": 0.008267225330044467}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 0, "rate_hz": 0.0}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 0, "rate_hz": 0.0}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 1716, "rate_hz": 14.186558666356305}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 6, "rate_hz": 0.0496033519802668}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 0, "rate_hz": 0.0}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 145, "rate_hz": 1.1987476728564477}], "camera": {}, "mode": {"/cmd_vel_safety": {"n": 843, "nonzero": 0, "first_rel_s": 0.046260385, "last_rel_s": 42.146440909}, "/cmd_vel": {"n": 843, "nonzero": 0, "first_rel_s": 0.046317702, "last_rel_s": 42.146405953}, "/cmd_vel_nav": {"n": 1, "nonzero": 0, "first_rel_s": 41.084834078, "last_rel_s": 41.084834078}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_23_46", "storage": "sqlite3", "duration_s": 63.098820427, "start": "Time(nanoseconds=1784874226155963749, clock_type=SYSTEM_TIME)", "size": 45177154, "messages": 9140, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 0, "rate_hz": 0.0}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 948, "rate_hz": 15.024052646067384}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 0, "rate_hz": 0.0}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 948, "rate_hz": 15.024052646067384}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 0, "rate_hz": 0.0}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.015848156799649137}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 4384, "rate_hz": 69.47831940966182}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.015848156799649137}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 41, "rate_hz": 0.6497744287856146}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 1352, "rate_hz": 21.426707993125635}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 1, "rate_hz": 0.015848156799649137}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 0, "rate_hz": 0.0}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 0, "rate_hz": 0.0}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 1354, "rate_hz": 21.458404306724933}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.047544470398947415}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 0, "rate_hz": 0.0}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 107, "rate_hz": 1.695752777562458}], "camera": {}, "mode": {"/cmd_vel_safety": {"n": 948, "nonzero": 0, "first_rel_s": 15.750004349, "last_rel_s": 63.098533928}, "/cmd_vel": {"n": 948, "nonzero": 0, "first_rel_s": 15.750508588, "last_rel_s": 63.098820427}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_25_10", "storage": "sqlite3", "duration_s": 62.047053311, "start": "Time(nanoseconds=1784874310630161229, clock_type=SYSTEM_TIME)", "size": 196167709, "messages": 12460, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 72, "rate_hz": 1.1604096594098128}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 761, "rate_hz": 12.264885427928714}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 82, "rate_hz": 1.3215776676611755}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 761, "rate_hz": 12.264885427928714}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 0, "rate_hz": 0.0}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.016116800825136286}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 6205, "rate_hz": 100.00474911997067}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 2, "rate_hz": 0.03223360165027257}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 87, "rate_hz": 1.402161671786857}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 1862, "rate_hz": 30.009483136403766}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 73, "rate_hz": 1.176526460234949}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 6, "rate_hz": 0.09670080495081773}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 227, "rate_hz": 3.6585137873059375}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 1935, "rate_hz": 31.186009596638716}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.048350402475408866}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 228, "rate_hz": 3.6746305881310737}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 155, "rate_hz": 2.4981041278961245}], "camera": {}, "mode": {"/cmd_vel": {"n": 761, "nonzero": 0, "first_rel_s": 15.742556891, "last_rel_s": 62.042795006}, "/cmd_vel_safety": {"n": 761, "nonzero": 0, "first_rel_s": 15.742566728, "last_rel_s": 62.042656948}, "/plan": {"n": 6, "nonzero": null, "first_rel_s": 25.037374504, "last_rel_s": 33.640531465}, "/cmd_vel_nav": {"n": 82, "nonzero": 81, "first_rel_s": 25.05506548, "last_rel_s": 33.457837495}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_27_44", "storage": "sqlite3", "duration_s": 36.391547008, "start": "Time(nanoseconds=1784874465118395811, clock_type=SYSTEM_TIME)", "size": 106436934, "messages": 7528, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 95, "rate_hz": 2.6104963325443684}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 507, "rate_hz": 13.931806743157841}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 24, "rate_hz": 0.6594938103269984}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 507, "rate_hz": 13.931806743157841}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 0, "rate_hz": 0.0}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.027478908763624934}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 3640, "rate_hz": 100.02322789959476}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.027478908763624934}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 98, "rate_hz": 2.6929330588352434}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 1091, "rate_hz": 29.979489461114802}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 95, "rate_hz": 2.6104963325443684}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 2, "rate_hz": 0.05495781752724987}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 94, "rate_hz": 2.5830174237807437}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 1185, "rate_hz": 32.56250688489555}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.0824367262908748}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 94, "rate_hz": 2.5830174237807437}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 91, "rate_hz": 2.5005806974898688}], "camera": {}, "mode": {"/cmd_vel_safety": {"n": 507, "nonzero": 0, "first_rel_s": 11.074624535, "last_rel_s": 36.374543472}, "/cmd_vel": {"n": 507, "nonzero": 0, "first_rel_s": 11.07464108, "last_rel_s": 36.374805904}, "/plan": {"n": 2, "nonzero": null, "first_rel_s": 17.259566234, "last_rel_s": 18.883840336}, "/cmd_vel_nav": {"n": 24, "nonzero": 23, "first_rel_s": 17.294604265, "last_rel_s": 19.895345756}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-15_32_02", "storage": "sqlite3", "duration_s": 121.5751961, "start": "Time(nanoseconds=1784874722148881047, clock_type=SYSTEM_TIME)", "size": 571717975, "messages": 27275, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 621, "rate_hz": 5.107949811482969}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 1734, "rate_hz": 14.262777734479016}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 152, "rate_hz": 1.2502550263211132}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 1729, "rate_hz": 14.221650924402663}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 39, "rate_hz": 0.32078911859554876}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.008225362015270481}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 12158, "rate_hz": 100.00395138165851}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 68, "rate_hz": 0.5593246170383928}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 365, "rate_hz": 3.0022571355737258}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 3648, "rate_hz": 30.006120631706718}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 606, "rate_hz": 4.984569381253912}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 15, "rate_hz": 0.12338043022905723}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 789, "rate_hz": 6.48981063004841}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 4254, "rate_hz": 34.99069001296063}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.024676086045811445}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 789, "rate_hz": 6.48981063004841}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 304, "rate_hz": 2.5005100526422264}], "camera": {}, "mode": {"/cmd_vel_safety": {"n": 1729, "nonzero": 0, "first_rel_s": 16.889051978, "last_rel_s": 121.538682005}, "/cmd_vel": {"n": 1734, "nonzero": 5, "first_rel_s": 16.889182832, "last_rel_s": 121.538805734}, "/plan": {"n": 15, "nonzero": null, "first_rel_s": 34.70804448, "last_rel_s": 99.21554418}, "/cmd_vel_nav": {"n": 152, "nonzero": 142, "first_rel_s": 34.731483188, "last_rel_s": 100.049651506}, "/cmd_vel_teleop": {"n": 39, "nonzero": 24, "first_rel_s": 93.746830842, "last_rel_s": 103.464913356}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_13_36", "storage": "sqlite3", "duration_s": 277.195982718, "start": "Time(nanoseconds=1784877216051196267, clock_type=SYSTEM_TIME)", "size": 1608938058, "messages": 64131, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 2353, "rate_hz": 8.488579007992982}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 2877, "rate_hz": 10.378938294090867}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 1153, "rate_hz": 4.159511940593246}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 1494, "rate_hz": 5.389688498912671}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 374, "rate_hz": 1.3492259026729176}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.00360755588949978}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 27714, "rate_hz": 99.9798039215969}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 229, "rate_hz": 0.8261302986954495}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 1197, "rate_hz": 4.3182443997312365}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 8316, "rate_hz": 30.00043477708017}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 2353, "rate_hz": 8.488579007992982}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 93, "rate_hz": 0.3355026977234795}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 2260, "rate_hz": 8.153076310269503}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 10669, "rate_hz": 38.48901378507315}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.010822667668499339}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2352, "rate_hz": 8.484971452103482}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 693, "rate_hz": 2.5000362314233473}], "camera": {}, "mode": {"/plan": {"n": 93, "nonzero": null, "first_rel_s": 48.110463441, "last_rel_s": 195.447929503}, "/cmd_vel_nav": {"n": 1153, "nonzero": 1150, "first_rel_s": 48.130629762, "last_rel_s": 267.783604168}, "/cmd_vel": {"n": 2877, "nonzero": 1377, "first_rel_s": 48.130716599, "last_rel_s": 277.154460062}, "/cmd_vel_safety": {"n": 1494, "nonzero": 0, "first_rel_s": 71.054450619, "last_rel_s": 277.154154381}, "/cmd_vel_teleop": {"n": 374, "nonzero": 338, "first_rel_s": 118.141739558, "last_rel_s": 255.473240614}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-16_19_38", "storage": "sqlite3", "duration_s": 38.515727792, "start": "Time(nanoseconds=1784877578535579582, clock_type=SYSTEM_TIME)", "size": 96885063, "messages": 8069, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 79, "rate_hz": 2.051110144578623}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 612, "rate_hz": 15.88961276559642}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 112, "rate_hz": 2.907902989782351}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 588, "rate_hz": 15.266490696357344}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 0, "rate_hz": 0.0}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.025963419551628137}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 3853, "rate_hz": 100.03705553242321}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 3, "rate_hz": 0.07789025865488441}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 89, "rate_hz": 2.3107443400949044}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 1156, "rate_hz": 30.013713001682127}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 79, "rate_hz": 2.051110144578623}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 8, "rate_hz": 0.2077073564130251}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 78, "rate_hz": 2.0251467250269948}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 1234, "rate_hz": 32.03885972670912}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.07789025865488441}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 78, "rate_hz": 2.0251467250269948}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 96, "rate_hz": 2.4924882769563013}], "camera": {}, "mode": {"/plan": {"n": 8, "nonzero": null, "first_rel_s": 6.845481436, "last_rel_s": 17.024191158}, "/cmd_vel_nav": {"n": 112, "nonzero": 111, "first_rel_s": 6.850271329, "last_rel_s": 18.25352902}, "/cmd_vel": {"n": 612, "nonzero": 24, "first_rel_s": 6.850353061, "last_rel_s": 38.51382099}, "/cmd_vel_safety": {"n": 588, "nonzero": 0, "first_rel_s": 9.168164588, "last_rel_s": 38.513684669}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-17_42_23", "storage": "sqlite3", "duration_s": 37.492145517, "start": "Time(nanoseconds=1784882543464787320, clock_type=SYSTEM_TIME)", "size": 125782345, "messages": 6616, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 124, "rate_hz": 3.3073594026187405}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 373, "rate_hz": 9.948750461103147}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 77, "rate_hz": 2.05376350001325}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 356, "rate_hz": 9.495322155905416}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 0, "rate_hz": 0.0}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.026672253246925327}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 3112, "rate_hz": 83.00405210443162}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 3, "rate_hz": 0.08001675974077598}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 94, "rate_hz": 2.5071918052109807}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 943, "rate_hz": 25.151934811850584}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 124, "rate_hz": 3.3073594026187405}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 6, "rate_hz": 0.16003351948155195}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 123, "rate_hz": 3.2806871493718153}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 1066, "rate_hz": 28.4326219612224}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.08001675974077598}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 123, "rate_hz": 3.2806871493718153}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 88, "rate_hz": 2.347158285729429}], "camera": {}, "mode": {"/plan": {"n": 6, "nonzero": null, "first_rel_s": 11.193270068, "last_rel_s": 17.977069368}, "/cmd_vel_nav": {"n": 77, "nonzero": 75, "first_rel_s": 11.217581523, "last_rel_s": 25.70642265}, "/cmd_vel": {"n": 373, "nonzero": 26, "first_rel_s": 11.217771744, "last_rel_s": 37.482247096}, "/cmd_vel_safety": {"n": 356, "nonzero": 0, "first_rel_s": 13.782496989, "last_rel_s": 37.482200246}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-18_11_27", "storage": "sqlite3", "duration_s": 497.711519134, "start": "Time(nanoseconds=1784884287814708392, clock_type=SYSTEM_TIME)", "size": 2838514019, "messages": 103253, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 4131, "rate_hz": 8.299988730796889}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 5365, "rate_hz": 10.779336611165652}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 3905, "rate_hz": 7.845910431799044}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 1557, "rate_hz": 3.1283181926532935}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 9, "rate_hz": 0.0180827641193832}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.0020091960132648}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 41897, "rate_hz": 84.17928536775533}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 635, "rate_hz": 1.275839468423148}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 2148, "rate_hz": 4.31575303649279}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 12767, "rate_hz": 25.6514055013517}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 4131, "rate_hz": 8.299988730796889}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 303, "rate_hz": 0.6087863920192343}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 4130, "rate_hz": 8.297979534783623}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 16897, "rate_hz": 33.949385036135325}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.0060275880397944}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 4130, "rate_hz": 8.297979534783623}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 1244, "rate_hz": 2.499439840501411}], "camera": {}, "mode": {"/plan": {"n": 303, "nonzero": null, "first_rel_s": 25.554407242, "last_rel_s": 429.361230231}, "/cmd_vel_nav": {"n": 3905, "nonzero": 3900, "first_rel_s": 25.577596185, "last_rel_s": 429.83000067}, "/cmd_vel": {"n": 5365, "nonzero": 3804, "first_rel_s": 25.577684639, "last_rel_s": 497.711519134}, "/cmd_vel_safety": {"n": 1557, "nonzero": 0, "first_rel_s": 419.911386948, "last_rel_s": 497.711220173}, "/cmd_vel_teleop": {"n": 9, "nonzero": 6, "first_rel_s": 431.867736432, "last_rel_s": 452.447697796}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26", "storage": "sqlite3", "duration_s": 842.885597147, "start": "Time(nanoseconds=1784887526516010853, clock_type=SYSTEM_TIME)", "size": 5636172133, "messages": 199422, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 8310, "rate_hz": 9.858989201058478}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 7458, "rate_hz": 8.84817586780916}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 6598, "rate_hz": 7.827871329552809}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 587, "rate_hz": 0.6964171673912547}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 381, "rate_hz": 0.45201863846008183}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.00118640062587948}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 84290, "rate_hz": 100.00170875538136}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1150, "rate_hz": 1.364360719761402}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 4215, "rate_hz": 5.000678638082008}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 25287, "rate_hz": 30.00051262661441}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 8303, "rate_hz": 9.850684396677321}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 528, "rate_hz": 0.6264195304643654}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 8309, "rate_hz": 9.8578028004326}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 33589, "rate_hz": 39.85001062266585}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.00355920187763844}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 8309, "rate_hz": 9.8578028004326}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 2104, "rate_hz": 2.496186916850426}], "camera": {}, "mode": {"/cmd_vel_safety": {"n": 587, "nonzero": 0, "first_rel_s": 0.005986255, "last_rel_s": 835.305968621}, "/cmd_vel": {"n": 7458, "nonzero": 6824, "first_rel_s": 0.056105762, "last_rel_s": 835.306016833}, "/cmd_vel_teleop": {"n": 381, "nonzero": 343, "first_rel_s": 8.395868126, "last_rel_s": 215.172353073}, "/plan": {"n": 528, "nonzero": null, "first_rel_s": 73.503644902, "last_rel_s": 813.980985435}, "/cmd_vel_nav": {"n": 6598, "nonzero": 6566, "first_rel_s": 73.517170365, "last_rel_s": 814.043549676}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26-hires", "storage": "sqlite3", "duration_s": 842.885597147, "start": "Time(nanoseconds=1784887526516010853, clock_type=SYSTEM_TIME)", "size": 5636172145, "messages": 199422, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 8310, "rate_hz": 9.858989201058478}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 7458, "rate_hz": 8.84817586780916}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 6598, "rate_hz": 7.827871329552809}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 587, "rate_hz": 0.6964171673912547}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 381, "rate_hz": 0.45201863846008183}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.00118640062587948}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 84290, "rate_hz": 100.00170875538136}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1150, "rate_hz": 1.364360719761402}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 4215, "rate_hz": 5.000678638082008}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 25287, "rate_hz": 30.00051262661441}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 8303, "rate_hz": 9.850684396677321}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 528, "rate_hz": 0.6264195304643654}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 8309, "rate_hz": 9.8578028004326}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 33589, "rate_hz": 39.85001062266585}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.00355920187763844}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 8309, "rate_hz": 9.8578028004326}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 2104, "rate_hz": 2.496186916850426}], "camera": {}, "mode": {"/cmd_vel_safety": {"n": 587, "nonzero": 0, "first_rel_s": 0.005986255, "last_rel_s": 835.305968621}, "/cmd_vel": {"n": 7458, "nonzero": 6824, "first_rel_s": 0.056105762, "last_rel_s": 835.306016833}, "/cmd_vel_teleop": {"n": 381, "nonzero": 343, "first_rel_s": 8.395868126, "last_rel_s": 215.172353073}, "/plan": {"n": 528, "nonzero": null, "first_rel_s": 73.503644902, "last_rel_s": 813.980985435}, "/cmd_vel_nav": {"n": 6598, "nonzero": 6566, "first_rel_s": 73.517170365, "last_rel_s": 814.043549676}}}
{"path": "/home/systemlab/bags/m7-bench-data/2026-07-31-live-campus/rosbag2_2026_07_31-15_14_51", "storage": "sqlite3", "duration_s": 235.081798969, "start": "Time(nanoseconds=1785478491709657463, clock_type=SYSTEM_TIME)", "size": 1588660572, "messages": 54669, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 2318, "rate_hz": 9.860397572955753}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 1621, "rate_hz": 6.895472159517375}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 1218, "rate_hz": 5.181175256195043}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 576, "rate_hz": 2.4502109585947}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 1, "rate_hz": 0.004253838469782465}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.004253838469782465}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 23509, "rate_hz": 100.00348858611596}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 194, "rate_hz": 0.8252446631377982}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 1175, "rate_hz": 4.9982602019943965}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 7053, "rate_hz": 30.002322727375724}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 2317, "rate_hz": 9.85614373448597}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 93, "rate_hz": 0.3956069776897692}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 2317, "rate_hz": 9.85614373448597}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 9369, "rate_hz": 39.854212623391916}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.012761515409347394}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2317, "rate_hz": 9.85614373448597}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 587, "rate_hz": 2.497003181762307}], "camera": {}, "mode": {"/cmd_vel_safety": {"n": 576, "nonzero": 0, "first_rel_s": 52.416901278, "last_rel_s": 235.066542902}, "/cmd_vel": {"n": 1621, "nonzero": 1044, "first_rel_s": 52.417033865, "last_rel_s": 235.066627135}, "/plan": {"n": 93, "nonzero": null, "first_rel_s": 101.910999527, "last_rel_s": 222.678347539}, "/cmd_vel_nav": {"n": 1218, "nonzero": 1216, "first_rel_s": 101.9298168, "last_rel_s": 223.547404379}, "/cmd_vel_teleop": {"n": 1, "nonzero": 0, "first_rel_s": 156.385029978, "last_rel_s": 156.385029978}}}
{"path": "/home/systemlab/bags/m8-static-check-20261113/fixed_1", "storage": "sqlite3", "duration_s": 11.731595423, "start": "Time(nanoseconds=1794551466889849929, clock_type=SYSTEM_TIME)", "size": 75623379, "messages": 2467, "topics": [{"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1174, "rate_hz": 100.07164052881949}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1174, "rate_hz": 100.07164052881949}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.25571969470737516}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 116, "rate_hz": 9.88782819535184}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m8-static-check-20261113/fixed_2_place2", "storage": "sqlite3", "duration_s": 11.740689389, "start": "Time(nanoseconds=1794551663825061440, clock_type=SYSTEM_TIME)", "size": 74976225, "messages": 2468, "topics": [{"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1175, "rate_hz": 100.07930208092144}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1175, "rate_hz": 100.07930208092144}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.2555216223342675}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 115, "rate_hz": 9.794995522813588}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m8-static-check-20261113/fixed_3_place2_rot180", "storage": "sqlite3", "duration_s": 11.840354241, "start": "Time(nanoseconds=1794551768577397991, clock_type=SYSTEM_TIME)", "size": 75618707, "messages": 2467, "topics": [{"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1174, "rate_hz": 99.15243886325209}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1174, "rate_hz": 99.15243886325209}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.2533707977766237}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 116, "rate_hz": 9.797004180696117}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m8-static-check-20261113/fixed_4_place2_rot180_after_drive", "storage": "sqlite3", "duration_s": 11.730850519, "start": "Time(nanoseconds=1794552058922158131, clock_type=SYSTEM_TIME)", "size": 75619335, "messages": 2467, "topics": [{"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1174, "rate_hz": 100.077995035272}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1174, "rate_hz": 100.077995035272}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.2557359327988211}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 116, "rate_hz": 9.888456068221084}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m8-static-check-20261113/fixed_5_place3_rot180", "storage": "sqlite3", "duration_s": 11.750680508, "start": "Time(nanoseconds=1794552224985212758, clock_type=SYSTEM_TIME)", "size": 75623407, "messages": 2471, "topics": [{"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1176, "rate_hz": 100.07931023223425}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1176, "rate_hz": 100.07931023223425}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.25530436283733227}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 116, "rate_hz": 9.871768696376847}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m8-static-check-20261113/flat_1", "storage": "sqlite3", "duration_s": 11.843723369, "start": "Time(nanoseconds=1794537736983260314, clock_type=SYSTEM_TIME)", "size": 76274641, "messages": 2490, "topics": [{"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1185, "rate_hz": 100.05299542048094}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1185, "rate_hz": 100.05299542048094}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.25329872258349606}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 117, "rate_hz": 9.878650180756345}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m8-static-check-20261113/flat_2_rot180", "storage": "sqlite3", "duration_s": 11.75068281, "start": "Time(nanoseconds=1794537889329044153, clock_type=SYSTEM_TIME)", "size": 75623391, "messages": 2471, "topics": [{"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1176, "rate_hz": 100.07929062634616}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1176, "rate_hz": 100.07929062634616}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.25530431282231164}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 116, "rate_hz": 9.871766762462716}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/m8-static-check-20261113/flat_3_seated", "storage": "sqlite3", "duration_s": 11.751782546, "start": "Time(nanoseconds=1794538018152320739, clock_type=SYSTEM_TIME)", "size": 75623391, "messages": 2471, "topics": [{"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu", "count": 1176, "rate_hz": 100.0699251706525}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 1176, "rate_hz": 100.0699251706525}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.2552804213537053}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 116, "rate_hz": 9.87084295900994}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_bottle_2m_160742", "storage": "mcap", "duration_s": 12.8178461, "start": "Time(nanoseconds=1793171297879993533, clock_type=SYSTEM_TIME)", "size": 759669883, "messages": 2186, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 384, "rate_hz": 29.95823143796367}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 384, "rate_hz": 29.95823143796367}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 384, "rate_hz": 29.95823143796367}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 384, "rate_hz": 29.95823143796367}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 13, "rate_hz": 1.014210960139395}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 382, "rate_hz": 29.802198982557606}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.3120649108121215}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 125, "rate_hz": 9.752028462878798}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 126, "rate_hz": 9.830044690581829}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_bottle_3m_160831", "storage": "mcap", "duration_s": 11.702101807, "start": "Time(nanoseconds=1793171330989242120, clock_type=SYSTEM_TIME)", "size": 695097449, "messages": 2004, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 351, "rate_hz": 29.994611719241558}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 351, "rate_hz": 29.994611719241558}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 351, "rate_hz": 29.994611719241558}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 351, "rate_hz": 29.994611719241558}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 13, "rate_hz": 1.1109115451570948}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 351, "rate_hz": 29.994611719241558}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.34181893697141374}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 116, "rate_hz": 9.912749172170999}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 116, "rate_hz": 9.912749172170999}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_bottle_4m_160903", "storage": "mcap", "duration_s": 9.873895187, "start": "Time(nanoseconds=1793171359403941217, clock_type=SYSTEM_TIME)", "size": 585668433, "messages": 1689, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 296, "rate_hz": 29.978037481065677}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 296, "rate_hz": 29.978037481065677}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 296, "rate_hz": 29.978037481065677}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 296, "rate_hz": 29.978037481065677}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 13, "rate_hz": 1.3166029974792357}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 293, "rate_hz": 29.67420602010893}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.4051086146089956}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 98, "rate_hz": 9.925161057920393}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 97, "rate_hz": 9.823883904268143}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_bottle_5m_160929", "storage": "mcap", "duration_s": 10.188271765, "start": "Time(nanoseconds=1793171417857497157, clock_type=SYSTEM_TIME)", "size": 604348423, "messages": 1744, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 306, "rate_hz": 30.03453451754288}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 306, "rate_hz": 30.03453451754288}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 305, "rate_hz": 29.936382443956138}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 305, "rate_hz": 29.936382443956138}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 12, "rate_hz": 1.1778248830408973}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 306, "rate_hz": 30.03453451754288}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.39260829434696576}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 100, "rate_hz": 9.815207358674144}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 100, "rate_hz": 9.815207358674144}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160152", "storage": "mcap", "duration_s": 21.897645993, "start": "Time(nanoseconds=1793170925533583301, clock_type=SYSTEM_TIME)", "size": 1300095364, "messages": 3731, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 657, "rate_hz": 30.003225013776483}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 657, "rate_hz": 30.003225013776483}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 657, "rate_hz": 30.003225013776483}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 657, "rate_hz": 30.003225013776483}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 11, "rate_hz": 0.5023371006872774}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 657, "rate_hz": 30.003225013776483}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.18266803661355546}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 215, "rate_hz": 9.818406967978605}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 216, "rate_hz": 9.864073977131994}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_chair_1.5m_160421", "storage": "mcap", "duration_s": 11.586497367, "start": "Time(nanoseconds=1793171086155587523, clock_type=SYSTEM_TIME)", "size": 682270403, "messages": 1971, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 344, "rate_hz": 29.689731858029948}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 344, "rate_hz": 29.689731858029948}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 345, "rate_hz": 29.77603921808236}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 345, "rate_hz": 29.77603921808236}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 13, "rate_hz": 1.1219956806813642}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 348, "rate_hz": 30.034961298239597}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.3452294402096506}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 114, "rate_hz": 9.83903904597504}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 114, "rate_hz": 9.83903904597504}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160227", "storage": "mcap", "duration_s": 10.985278366, "start": "Time(nanoseconds=1793171000192446296, clock_type=SYSTEM_TIME)", "size": 653370340, "messages": 1884, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 330, "rate_hz": 30.04020371676397}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 330, "rate_hz": 30.04020371676397}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 330, "rate_hz": 30.04020371676397}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 330, "rate_hz": 30.04020371676397}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 13, "rate_hz": 1.1834019645997926}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 330, "rate_hz": 30.04020371676397}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.3641236814153208}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 108, "rate_hz": 9.831339398213663}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 109, "rate_hz": 9.922370318567493}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_chair_2m_160458", "storage": "mcap", "duration_s": 10.694174822, "start": "Time(nanoseconds=1793171122073949913, clock_type=SYSTEM_TIME)", "size": 634891967, "messages": 1832, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 321, "rate_hz": 30.01634117104954}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 321, "rate_hz": 30.01634117104954}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 321, "rate_hz": 30.01634117104954}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 321, "rate_hz": 30.01634117104954}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 13, "rate_hz": 1.2156150630020062}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 321, "rate_hz": 30.01634117104954}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.3740354040006173}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 105, "rate_hz": 9.818429355016205}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 105, "rate_hz": 9.818429355016205}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160331", "storage": "mcap", "duration_s": 12.398923208, "start": "Time(nanoseconds=1793171038319133178, clock_type=SYSTEM_TIME)", "size": 735801756, "messages": 2122, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 372, "rate_hz": 30.002605368180614}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 371, "rate_hz": 29.92195320321239}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 372, "rate_hz": 30.002605368180614}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 372, "rate_hz": 30.002605368180614}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 13, "rate_hz": 1.048478144586957}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 372, "rate_hz": 30.002605368180614}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.3226086598729098}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 123, "rate_hz": 9.920216291091977}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 123, "rate_hz": 9.920216291091977}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_chair_3m_160533", "storage": "mcap", "duration_s": 10.90378631, "start": "Time(nanoseconds=1793171168243811718, clock_type=SYSTEM_TIME)", "size": 647510340, "messages": 1868, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 327, "rate_hz": 29.98958258197927}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 327, "rate_hz": 29.98958258197927}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 327, "rate_hz": 29.98958258197927}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 327, "rate_hz": 29.98958258197927}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 13, "rate_hz": 1.1922464023416834}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 327, "rate_hz": 29.98958258197927}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.36684504687436414}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 108, "rate_hz": 9.904816265607833}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 108, "rate_hz": 9.904816265607833}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/bags/sensor_check_20261028/chk_empty_0m_160726", "storage": "mcap", "duration_s": 11.401345908, "start": "Time(nanoseconds=1793171250129750534, clock_type=SYSTEM_TIME)", "size": 677255989, "messages": 1954, "topics": [{"name": "/camera/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 342, "rate_hz": 29.996458554952564}, {"name": "/camera/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "count": 342, "rate_hz": 29.996458554952564}, {"name": "/camera/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "count": 342, "rate_hz": 29.996458554952564}, {"name": "/camera/camera/depth/image_rect_raw", "type": "sensor_msgs/msg/Image", "count": 342, "rate_hz": 29.996458554952564}, {"name": "/rosout", "type": "rcl_interfaces/msg/Log", "count": 13, "rate_hz": 1.14021626086077}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 343, "rate_hz": 30.0841674980957}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 4, "rate_hz": 0.3508357725725446}, {"name": "/velodyne_packets", "type": "velodyne_msgs/msg/VelodyneScan", "count": 113, "rate_hz": 9.911110575174385}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 113, "rate_hz": 9.911110575174385}], "camera": {"camera_info_wh": [640, 480]}, "mode": {}}
{"path": "/home/systemlab/whill_lab0_ros2/docs/m6r-bench-data/2026-07-12-acceptance-campus/bag", "storage": "sqlite3", "duration_s": 0.0, "start": "Time(nanoseconds=1783850920823039815, clock_type=SYSTEM_TIME)", "size": 25520, "messages": 1, "topics": [{"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 1, "rate_hz": null}], "camera": {}, "mode": {}}
{"path": "/home/systemlab/whill_platform/bags/2026-07-31-campus", "storage": "mcap", "duration_s": 235.081798969, "start": "Time(nanoseconds=1785478491709657463, clock_type=SYSTEM_TIME)", "size": 1578149311, "messages": 54669, "topics": [{"name": "/alignment_status", "type": "diagnostic_msgs/msg/DiagnosticArray", "count": 2318, "rate_hz": 9.860397572955753}, {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist", "count": 1621, "rate_hz": 6.895472159517375}, {"name": "/cmd_vel_nav", "type": "geometry_msgs/msg/Twist", "count": 1218, "rate_hz": 5.181175256195043}, {"name": "/cmd_vel_safety", "type": "geometry_msgs/msg/Twist", "count": 576, "rate_hz": 2.4502109585947}, {"name": "/cmd_vel_teleop", "type": "geometry_msgs/msg/Twist", "count": 1, "rate_hz": 0.004253838469782465}, {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 1, "rate_hz": 0.004253838469782465}, {"name": "/imu/data_rep145", "type": "sensor_msgs/msg/Imu", "count": 23509, "rate_hz": 100.00348858611596}, {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "count": 194, "rate_hz": 0.8252446631377982}, {"name": "/local_costmap/published_footprint", "type": "geometry_msgs/msg/PolygonStamped", "count": 1175, "rate_hz": 4.9982602019943965}, {"name": "/odometry/filtered", "type": "nav_msgs/msg/Odometry", "count": 7053, "rate_hz": 30.002322727375724}, {"name": "/pcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "count": 2317, "rate_hz": 9.85614373448597}, {"name": "/plan", "type": "nav_msgs/msg/Path", "count": 93, "rate_hz": 0.3956069776897692}, {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "count": 2317, "rate_hz": 9.85614373448597}, {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "count": 9369, "rate_hz": 39.854212623391916}, {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "count": 3, "rate_hz": 0.012761515409347394}, {"name": "/velodyne_points", "type": "sensor_msgs/msg/PointCloud2", "count": 2317, "rate_hz": 9.85614373448597}, {"name": "/whill/odom", "type": "nav_msgs/msg/Odometry", "count": 587, "rate_hz": 2.497003181762307}], "camera": {}, "mode": {"/cmd_vel_safety": {"n": 576, "nonzero": 0, "first_rel_s": 52.416901278, "last_rel_s": 235.066542902}, "/cmd_vel": {"n": 1621, "nonzero": 1044, "first_rel_s": 52.417033865, "last_rel_s": 235.066627135}, "/plan": {"n": 93, "nonzero": null, "first_rel_s": 101.910999527, "last_rel_s": 222.678347539}, "/cmd_vel_nav": {"n": 1218, "nonzero": 1216, "first_rel_s": 101.9298168, "last_rel_s": 223.547404379}, "/cmd_vel_teleop": {"n": 1, "nonzero": 0, "first_rel_s": 156.385029978, "last_rel_s": 156.385029978}}}
```

