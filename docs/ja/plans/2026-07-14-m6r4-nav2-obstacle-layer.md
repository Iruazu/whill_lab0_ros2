# M6R-4 実行計画: Nav2 obstacle layer 復活 + collision detection 復帰

Language: 日本語

- 日付: 2026-07-14 起草
- 状態: **accepted** (2026-07-14 ユーザー承認済。3 点確認完了 = デモ日未確定 / M6R4-4 は M6R4-3 完了時判断 / `allow_unknown: false` 反転)
- 親方針: [`docs/ja/plans/2026-06-11-platform-pivot.md`](2026-06-11-platform-pivot.md)
  §2 (P5), §3.3 (Nav2), §4 (M6-R), §6 M6-R
- 上位フェーズ計画: [`docs/ja/plans/2026-06-24-m6r-localization.md`](2026-06-24-m6r-localization.md)
  §6 M6R-4
- 直前依存: M6R-2 (`whill_safety/m6r_bringup_launch.py`) live PASS 済 (2026-07-14
  §10.4 verify走行、`docs/m6r-bench-data/2026-07-14-verify-campus/`)、
  M6R-3 **lite 版**が同日中に merge 予定
  (twist_mux + failsafe A/B のみ、jump 検知と SAFE_HOLD は demo 後の
  バックログへ降格。ADR-0007 §「M6R-3 lite への縮小」節に記録)

## 0. ユーザー要件の理解

M6R-4 のゴール:「Web/CLI から NavigateToPose ゴール送信 → キャンパス
マップ上で自律走行 → 前方の人 (obstacle layer) で停止 → 人が退くと再開」の
成立。オープンキャンパスデモ (2026-07 中旬〜下旬想定) の走行本体を担う。

親方針 §2 P5 (地図品質の問題が安全機能を連鎖停止) の残り = Nav2 統合側の
解消がこのフェーズの主目的。M5-R (地図生成) と M6R-2 (localizer) が
仕上がった今、`use_collision_detection: false` と obstacle layer 不在の
2 点を落とすと親方針 R5 (歩行者の動的回避、無人走行安全) の最低条件が
成立する。

## 1. 解決すべき問題

現状 `nav_launch.py` は M4-R 完了時点から意図的な半壊状態:

- default map yaml が `lab-legacy-m5b` (M5-b 期の旧屋内マップ) を指す
- localization include が抜けている (M6R-2 で `m6r_bringup_launch.py`
  内に実体化済のためこちらが正)
- `nav2_params.yaml` の costmap plugins は `[static_layer, inflation_layer]`
  のみで **obstacle layer 不在**
- `FollowPath.use_collision_detection: false` (M5-b のゴースト障害物を
  避けるための応急処置)
- `/cmd_vel` が failsafe を素通り (M6R-3 lite が landing 後に `/cmd_vel_nav`
  へ remap 必須)

これら 5 点を M6R-4 で一度に落とすには手戻りが多いので、後述の 4 phase に
分けて段階的にゲートを開ける。

## 2. スコープ

### 2.1 扱うもの (in-scope)

1. `src/whill_navigation/` の中身を M6R-4 用に書き直す:
   - `launch/nav_launch.py`: localizer は include しない (`m6r_bringup_launch.py`
     が上位で立てる前提)。default map を `docs/maps/campus/occupancy.yaml`
     へ切替、`site` launch argument で `docs/maps/<site>/occupancy.yaml`
     を解決 (M6R-2 と同じ規約)
   - `config/nav2_params.yaml`: obstacle layer 追加、collision detection ON、
     map_server の paramsは新 site 前提に更新
2. `pointcloud_to_laserscan_node` の追加:
   - `/velodyne_points` (best-effort sensor QoS) → `/scan` (reliable) の
     bridge。M5-c のコメントで既知の QoS 不整合を解消
   - パラメータは smoke test で調整、初期値は §3.M6R4-2 に列挙
3. Nav2 の cmd_vel を `/cmd_vel_nav` に remap:
   - controller_server, behavior_server 両方の `/cmd_vel` → `/cmd_vel_nav`
   - velocity_smoother は twist_mux 出力 (`/cmd_vel`) を subscribe する
     配置に変更 (§8 統合ポイント)
4. デモ用の最小 CLI ラッパ (M6R4-4、判断次第):
   - 名前付き waypoint yaml (`config/demo_waypoints.yaml`) → `NavigateToPose`
     action call の shell script or 小さい Python ノード
   - **これは M7 (`whill_dispatch`) の前哨。ROS 側のみで完結する必要
     最小限に留める** (Web / 認証 / ジョブキューは M7 で書き直す)

### 2.2 扱わないもの (out-of-scope)

- **`whill_dispatch` パッケージ本体**: M7 担当
- **Web / rosbridge / タブレット UI**: M8 担当
- **twist_mux 本体 / failsafe_node 本体の実装**: M6R-3 lite で完了する前提
- **jump 検知 / SAFE_HOLD / G4 実機 3 試験**: ADR-0007 lite 縮小分に従い
  demo 後のバックログ
- **DWB / MPPI コントローラ選定**: RegulatedPurePursuit を継続。屋外
  歩行者環境で振る舞いが破綻したときに再検討し、ADR 起票
- **local costmap を `odom` frame に切り替える**: 現状 `map` frame。
  M4-R EKF は odom → base_link を供給しているので技術的には可能だが、
  変更すると M5-c コメントとの整合を全部書き直すことになる。本フェーズは
  `map` frame 維持
- **carrot / recovery behavior のチューニング**: `spin/backup/wait` の
  既存構成で通す。デモで挙動不良が出たら 7/下旬 予備日にチューニング
- **v2 マップ (キャンパス内側補完)**: `docs/maps/campus/README.md` §2 に
  ある通り、外周のみで走る運用。内側 unknown を経路計画に使わない
  (`allow_unknown: false` に落とす。§3.M6R4-1 参照)
- **camera_link 再校正**: 旧 M6R-6、demo 後
- **`use_collision_detection` の DWB 版**: RPP のみ有効化

## 3. Phase 分解

### M6R4-1: Nav2 bringup 骨格 (localizer 込みで NavigateToPose が経路を吐くまで)

- **目的**: 現状の半壊 `nav_launch.py` を M6R-2 と組み合わせて lifecycle
  activate まで持っていき、`ros2 action send_goal /navigate_to_pose ...` で
  planner_server が経路を計算・publish するところまで戻す。動的障害物は
  この phase では扱わない (obstacle layer 不在のまま、static のみ)
- **担当 agent**: `ros2-implementer` → `debugger` (lifecycle failure 時)
  → `code-reviewer`
- **入力**:
  - `docs/maps/campus/occupancy.yaml` (M5-R 本番マップ)
  - M6R-2 の `m6r_bringup_launch.py` (別ターミナルで立てる)
  - M6R-3 lite の twist_mux 出力 topic 名 (`/cmd_vel` 想定、§8 で確定)
- **成果物**:
  - `src/whill_navigation/launch/nav_launch.py` の refactor:
    - `site` launch argument (default `campus`)、`docs/maps/<site>/occupancy.yaml`
      を map_server に注入
    - `map_server` は既に居るので `yaml_filename` を新 site に向け直すのみ
    - default map yaml のハードコード (M5-b) を撤去、Docstring を M6R-4
      現状に更新
    - controller_server / behavior_server の `/cmd_vel` → `/cmd_vel_nav`
      remap を追加 (M6R-3 lite の twist_mux 入力に合わせる)
    - velocity_smoother の subscribe topic を twist_mux 出力 (`/cmd_vel`)
      に合わせて配置 (現状既に `/cmd_vel` を subscribe しているので実質
      変更なし。Docstring の cmd_vel routing 図を更新)
  - `src/whill_navigation/config/nav2_params.yaml` の更新:
    - `bt_navigator.transform_tolerance`: 0.5 → 0.3 に締める (M4-R EKF が
      30 Hz、M6R-2 localizer が 10 Hz、旧 0.5 は FAST-LIO 7 Hz 時の緩め設定)
    - `bt_navigator.odom_topic`: `/Odometry` → `/odometry/filtered`
      (M4-R EKF 出力に合わせる。旧 `/Odometry` は FAST-LIO)
    - `controller_server.odom_topic`: 同上
    - `velocity_smoother.odom_topic`: 同上
    - `planner_server.allow_unknown`: true → **false** に反転
      (`campus` マップは外周のみ走行のため、内側 unknown 領域を通したく
      ない。M6R4-3 走行で問題が出たら再検討)
    - static / inflation layer は既存を継続 (obstacle は M6R4-2 で追加)
    - map_server の `yaml_filename` は launch 側で上書きするため空継続
  - `src/whill_navigation/README.md` を M6R-4 現状に書き直し (半壊状態の
    説明を削除、`m6r_bringup_launch.py` 前提の運用手順を追加)
- **観測可能な受け入れ基準**:
  - [ ] **T1**: `ros2 launch whill_safety m6r_bringup_launch.py site:=campus`
    に続けて別ターミナルで `ros2 launch whill_navigation nav_launch.py site:=campus`
    を実行後、90 秒以内に:
    ```bash
    ros2 lifecycle get /map_server           # -> active [3]
    ros2 lifecycle get /planner_server       # -> active [3]
    ros2 lifecycle get /controller_server    # -> active [3]
    ros2 lifecycle get /behavior_server      # -> active [3]
    ros2 lifecycle get /bt_navigator         # -> active [3]
    ros2 lifecycle get /velocity_smoother    # -> active [3]
    ```
    すべて `active [3]`。
  - [ ] **T2**: `ros2 topic list | grep -E '(map|cmd_vel_nav|pcl_pose|odometry/filtered)$'`
    で 4 topic 全て見える (map: static map、cmd_vel_nav: Nav2 出力、
    pcl_pose: localizer、odometry/filtered: EKF)
  - [ ] **T3**: RViz で `/initialpose` publish 後、`ros2 topic echo /alignment_status --once`
    が `has_converged: true` かつ `message: ok` を返し、`ros2 topic hz /pcl_pose`
    が 8-12 Hz を 30 秒安定
  - [ ] **T4**: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0},
    orientation: {w: 1.0}}}}"` (initial pose 近傍の 5 m 先) で 5 秒以内に
    `ros2 topic echo /plan --once` が 1 本以上の PoseStamped 列を含む Path
    を返す。**走行はまだしない** (実機 cmd_vel は twist_mux 経由で止めておく)
  - [ ] **T5**: `ros2 run tf2_tools view_frames` の `frames.pdf` が
    `map -> odom -> base_link -> {imu_link, velodyne, camera_link}` の
    一本鎖のみ (publisher 二重なし)
- **検証方法**:
  - T1〜T5 を静止状態 (WHILL 電源 OFF or ジョイスティック手動制御下) で
    ユーザー実施。bag record は不要 (次 phase で採取)
- **依存**: M6R-2 merged (済) + M6R-3 lite merged (今日中)

### M6R4-2: pointcloud_to_laserscan 橋 + obstacle layer 有効化 (静止環境)

- **目的**: `/velodyne_points` を 2D 化して obstacle layer 入力に接続、
  local costmap に静的障害物 (壁・柱) が乗り、Nav2 が既に地図に載っている
  静的障害物と一致することを確認。歩行者テストはまだしない。
- **担当 agent**: `ros2-implementer` → `debugger` (QoS 不一致 debug 用)
  → `code-reviewer`
- **入力**: M6R4-1 完了状態、`ros-humble-pointcloud-to-laserscan` (apt 提供、
  BSD-3-Clause)
- **成果物**:
  - `src/whill_navigation/package.xml` に
    `<exec_depend>pointcloud_to_laserscan</exec_depend>` 追加
  - `src/whill_navigation/launch/nav_launch.py` に `pointcloud_to_laserscan_node`
    を追加。remap: `cloud_in` → `/velodyne_points`、`scan` → `/scan`。
    QoS 上書きは launch 側で対応 (次項参照)
  - `src/whill_navigation/config/pointcloud_to_laserscan.yaml` (新規、初期値):
    ```yaml
    pointcloud_to_laserscan:
      ros__parameters:
        target_frame: base_link         # scan は base_link で表現
        transform_tolerance: 0.1
        min_height: -0.2                # base_link 面-20 cm (路面ノイズ除去)
        max_height:  1.6                # WHILL 高さ (搭乗者含む) を超える範囲は不要
        angle_min: -3.141592
        angle_max:  3.141592
        angle_increment: 0.00873        # 0.5 deg (VLP-16 の水平分解能に合わせる)
        scan_time: 0.1
        range_min: 0.5                  # 車体反射の除去
        range_max: 25.0                 # obstacle layer の raytrace 範囲と一致
        use_inf: true
        inf_epsilon: 1.0
    ```
    **これらの数値は smoke test で検証必須** (§4 リスク R2)。特に
    `min_height` / `max_height` は WHILL の base_link 高さと LiDAR 実測
    高さ (`static_tf_launch.py`) から逆算する必要がある。M6R4-2 debug の
    最初のタスクとして「WHILL の 1 m 先に人を立たせて `/scan` の
    `ranges[]` に 1.0 前後が出るか」を目視確認する
  - `src/whill_navigation/config/nav2_params.yaml` の costmap plugins:
    ```yaml
    local_costmap:
      local_costmap:
        ros__parameters:
          plugins: [static_layer, obstacle_layer, inflation_layer]  # obstacle 追加
          obstacle_layer:
            plugin: nav2_costmap_2d::ObstacleLayer
            enabled: true
            observation_sources: scan
            scan:
              topic: /scan
              sensor_frame: base_link
              max_obstacle_height: 2.0
              clearing: true
              marking: true
              data_type: LaserScan
              raytrace_max_range: 25.0
              raytrace_min_range: 0.0
              obstacle_max_range: 20.0
              obstacle_min_range: 0.0
    global_costmap:
      global_costmap:
        ros__parameters:
          plugins: [static_layer, obstacle_layer, inflation_layer]
          # obstacle_layer は local と同構成、topic /scan 共有
    ```
    **QoS 対応**: Nav2 humble の `ObstacleLayer` は subscription QoS 上書きが
    未対応なので、`pointcloud_to_laserscan` 側の出力 QoS を `reliable` に
    合わせる (default reliable なので上書き不要)。`/velodyne_points` 側が
    `best-effort sensor QoS` なのを `pointcloud_to_laserscan_node` が
    subscribe できるかは v0.4 系で override 可能。実機で dropped msg が
    出る場合は `qos_overrides` で `best_effort` 側に落として `/scan` 側は
    `reliable` に上げる (bridge の本来の役目)
  - `docs/m6r-bench-data/2026-07-XX-m6r4-costmap-static/` を作成、bag と
    RViz スクショを保存
- **観測可能な受け入れ基準**:
  - [ ] **U1**: `ros2 topic hz /scan` が 9-11 Hz で 60 秒安定 (VLP-16 の
    10 Hz 出力を落とさない)
  - [ ] **U2**: `ros2 topic hz /local_costmap/costmap` が 4-6 Hz で 30 秒安定
    (現行 `update_frequency: 5.0` に一致)
  - [ ] **U3**: 静止状態で `ros2 topic echo /scan --once` の
    `ranges[]` に有限値が最低 200 個以上入り (360 度分解能 720 の 28% 以上)、
    かつ全 nan / 全 inf でない
  - [ ] **U4**: WHILL の前方 2 m 位置に立つ人を含む RViz スクショで、
    `/local_costmap/costmap_updates` に人相当の障害物が **描画される**
    (M5-c コメントの「obstacle layer なし」が完全解消)
  - [ ] **U5**: 静止状態 60 秒間、`nav2_msgs/action/NavigateToPose` を発火
    しない状態で `/local_costmap/costmap` が「動的に湧く false positive」を
    含まない (走行沿いの壁・柱以外の空間に inflation が伸びていない。
    Costmap の Occupied cell 数の時系列を bag record して手集計で確認)
  - [ ] **U6**: `use_collision_detection` はまだ **false のまま**
    (この phase では触らない。次 phase の M6R4-3 で反転)
- **検証方法**:
  - ユーザー実機で WHILL 電源 ON 静止、`m6r_bringup + nav_launch` 起動、
    RViz で local_costmap を可視化。前方 2 m に人を立たせて U4 を確認。
  - bag record: `/scan`, `/local_costmap/costmap`, `/pcl_pose`, `/tf` を
    5 分間。派生: costmap から Occupied cell 数の時系列 CSV を出す
    scripts/m6r4_costmap_stats.py を M6R4-2 内で新設。
- **依存**: M6R4-1 完了

### M6R4-3: collision_detection 復帰 + E2E 走行 (歩行者テスト)

- **目的**: `use_collision_detection: true` に反転。実機で NavigateToPose
  発火 → 走行 → 前方に人 → 停止 → 人が退く → 再開の一連が成立することを
  観測可能な形で確認。デモ本番と同じ条件。
- **担当 agent**: `ros2-implementer` → `debugger` (発散・振動時)
  → `code-reviewer`
- **入力**: M6R4-2 完了状態、実機 WHILL + 屋外キャンパス走行環境
- **成果物**:
  - `src/whill_navigation/config/nav2_params.yaml` の変更:
    - `FollowPath.use_collision_detection: false` → **true**
    - `FollowPath.max_allowed_time_to_collision_up_to_carrot: 1.0` は継続
    - `FollowPath.desired_linear_vel: 0.3` は継続 (実機で振動する場合のみ
      予備日に 0.2 へ落とす)
    - `local_costmap.inflation_layer.inflation_radius: 0.5` は継続。ゴースト
      障害物が消えた本番マップでは締めても走れる想定。実走行で「経路が
      狭すぎる」なら 0.4 に落とす
    - コメントを M5-b/M5-e の応急処置説明から「M6R-4 で復帰、根拠:
      DUFOMap 動的除去済 static map + obstacle layer」に書き直す
  - `docs/m6r-bench-data/2026-07-XX-m6r4-e2e/` に E2E bag + 動画 (可能なら)
- **観測可能な受け入れ基準**:
  - [ ] **V1** (走行成立): 発進点 → 10 m 先 goal を NavigateToPose で
    送信、`ros2 topic echo /navigate_to_pose/_action/status --once` で
    `STATUS_SUCCEEDED (4)` が返る。走行時間は 40 秒以内 (0.3 m/s × 10 m
    = 33 秒 + 減速余裕)
  - [ ] **V2** (歩行者停止): 走行中に前方 1.5 m に人が横切る → **1 秒以内**に
    `/cmd_vel_nav` の `linear.x` が 0.05 m/s 未満に落ちる。bag 解析で確認:
    ```bash
    scripts/m6r4_cmd_vel_stop_latency.py <bag> --obstacle-time <t>
    # 期待: latency < 1.0 s, min_v < 0.05
    ```
    (script は M6R4-3 内で新設。 obstacle-time は RViz スクショの人の初出
    タイムスタンプ)
  - [ ] **V3** (歩行者退去後再開): 人が退いてから **5 秒以内**に
    `/cmd_vel_nav.linear.x` が 0.1 m/s を回復し、goal に向かって走行再開
    (recovery behavior の spin/backup が起動しないこと。起動した場合は
    条件見直し)
  - [ ] **V4** (30 分連続 TF 破綻ゼロ): 30 分間、経路上を巡回走行して:
    ```bash
    scripts/m6r_tf_jump_check.py <30min-bag> --parent map --child odom \
      --threshold 0.5
    # 期待: violation count = 0
    ```
    かつ M6R-3 lite failsafe が **false positive で発火しない** (bag 中の
    `/cmd_vel_safety` publish 回数 = 0)
  - [ ] **V5** (安全性): `use_collision_detection: true` 状態で
    `/local_costmap/costmap` に lethal (254) が乗ったセルが RPP の
    lookahead 範囲に来た瞬間、`/cmd_vel_nav.linear.x` が減速する
    (V2 と重複だが collision_detection 機能単独の確認)
  - [ ] **V6** (M6R-3 lite との協調): V2 で停止した後、失探ではないので
    failsafe A/B 層は発火せず、Nav2 の cmd_vel_nav = 0 で twist_mux は
    nav 側を通し続ける (safety 側優先発火が false positive でない)
- **検証方法**:
  - 実機屋外走行、ユーザー実施 + 安全操作者随伴。デモ本番と同じ経路で
    30 分連続 (V4)。人の横断は 3 回 × 別ロケーションで再現性確認 (V2/V3)。
  - 天候: 晴れ or 曇り。雨天は M6R-4 スコープ外 (glass reflection false
    positive のリスク別 phase)
- **依存**: M6R4-2 完了 + M6R-3 lite merged 済 (twist_mux が /cmd_vel_nav
  を通す設定)

### M6R4-4: デモ用 CLI ラッパ (M7 前哨、判断で入れる)

- **目的**: 当日デモの操作者が「発進点 → A地点 → 発進点」の巡回を
  ROS 側から 1 コマンドで叩けるようにする。**Web / rosbridge は M7 で
  やる**。ここは python action client の薄い wrapper に留める
- **担当 agent**: `ros2-implementer` → `code-reviewer`
- **入力**: M6R4-3 完了、デモ経路の 3-5 waypoint 座標 (ユーザーが実測)
- **成果物**:
  - `src/whill_navigation/config/demo_waypoints.yaml` (新規):
    ```yaml
    waypoints:
      - name: "start"
        x: 0.0
        y: 0.0
        yaw: 0.0
      - name: "landmark_a"
        x: 20.0
        y: 5.0
        yaw: 1.57
      # ... デモ経路
    ```
  - `src/whill_navigation/scripts/nav_to_waypoint.py` (新規、実行可能):
    - CLI: `ros2 run whill_navigation nav_to_waypoint <name>` or
      `<name1> <name2> ... <nameN>` で順次実行 (シーケンス発火)
    - 内部で `NavigateToPose` action client を launch、成功したら次の
      waypoint へ、失敗 (aborted/canceled) したら停止 & log
  - `README.md` にデモ用 quick reference (「操作者は Terminal D で
    `ros2 run whill_navigation nav_to_waypoint start landmark_a start` を
    叩く」)
- **観測可能な受け入れ基準**:
  - [ ] **W1**: `ros2 run whill_navigation nav_to_waypoint landmark_a` を
    実行 → 5 秒以内に `/navigate_to_pose/_action/status` が `STATUS_EXECUTING`
    (2) → 完了時 `STATUS_SUCCEEDED` (4)。手戻り走行なし
  - [ ] **W2**: 3 waypoint 順次発火 (start → landmark_a → start) で全
    waypoint に到着 (`xy_goal_tolerance: 0.25` 内)
  - [ ] **W3**: 途中で人が横切って停止 → 再開 → 到着の V2/V3 動作が
    ラッパ経由でも同様に成立
- **検証方法**: 実機屋外走行、デモリハーサル兼用
- **依存**: M6R4-3 完了
- **位置付け**: **optional。M6R4-3 の V1-V6 全 pass 後に着手要否を判断** (ユーザー確定 2026-07-14)
- **判断条件**: 以下 2 つが両方成立するとき着手する。
  - (a) M7 (`whill_dispatch`) の着手が遠く、デモ当日までに Web/rosbridge
    経由の goal 送信が用意できない
  - (b) 当日 operator が `ros2 action send_goal /navigate_to_pose ...` を
    素で叩くのを避けたい (waypoint 名で呼びたい)
  - どちらか一方でも成立しなければ、Terminal での `ros2 action send_goal`
    or RViz Nav2 Goal ボタンで本番運用し、M6R4-4 は skip

## 4. 依存関係

```
M6R-2 (merged 2026-07-14)
   │
   ├──▶ M6R-3 lite (今日 merge)
   │       │
   │       └── twist_mux /cmd_vel_nav / /cmd_vel_safety / /cmd_vel の 3 topic
   │           確定 + failsafe A/B のみ
   │
   ▼
M6R4-1 (Nav2 骨格)  ← twist_mux 出力 topic 名の確定に依存
   │
   ▼
M6R4-2 (obstacle layer)  ← 静止環境で確認、実機必須
   │
   ▼
M6R4-3 (collision detection + E2E)  ← V4 の 30 分走行が最重量、天候依存
   │
   ▼
M6R4-4 (CLI wrapper、判断)  ← 残時間次第で skip 可
```

**並列可能な作業**:
- M6R4-1 の launch refactor と M6R4-2 の pointcloud_to_laserscan yaml 起草
  は別ファイルなので **同一 PR / 同一日に着手可**。ただし観測可能な受入は
  順序があるので merge は M6R4-1 → M6R4-2 の順
- M6R4-3 の走行と `scripts/m6r4_cmd_vel_stop_latency.py` の実装は並列可
  (script は bag 到着次第書ける)

**単一開発者 + 単一実機ボトルネック**: M6R4-2 と M6R4-3 の実機走行は連続で
1 日に押し込むのが効率的 (bringup 起動 30 分、実走行 1-2 時間、bag 解析
30 分 = 半日で 1 phase 分)。天候リスクで別日に分散させる場合はデモ本番前
に 2 日分の走行日程を確保する。

## 5. リスクと緩和策

- **R1 (QoS 不整合による /scan 未接続)**:
  内容: `/velodyne_points` は best-effort sensor QoS、`nav2_costmap_2d` の
  `ObstacleLayer` は reliable 期待。M5-c コメントの「obstacle layer なし」
  の直接原因
  緩和: `pointcloud_to_laserscan_node` を **明示的な QoS bridge として** 挟む
  (sub: best_effort、pub: reliable)。M6R4-2 U1 で `/scan` が 10 Hz 出続けることを
  必ず観測。出ない場合は `qos_overrides` を launch 側で明示指定
- **R2 (min_height/max_height の初期値ズレで人を見逃す or 地面を拾う)**:
  内容: WHILL の base_link 高さ = 地上何 cm か、Velodyne mount 高さ = 何 cm か
  を初期値のまま実測せずデモ日を迎えると誤検知。static_tf_launch.py の
  実測値 (M4-R PR #61) を参照して逆算
  緩和: M6R4-2 debug 開始時に「WHILL の 1 m 先に人を立たせて `/scan` の
  該当方向の range が 1.0 ± 0.1 m」を **最初のチェック** として実施。
  外れたら height 範囲を実測し ADR-0008 (§6 候補) に記載
- **R3 (localizer 発散との相互作用)**:
  内容: M6R-2 verify走行で reject 0 だったが、Nav2 走行時 (旋回・
  加減速多め) に fitness が悪化して失探する可能性
  緩和 A: M6R-3 lite の B 層 (fitness > 1.0 継続 or /pcl_pose 途絶) が発火
  したら twist_mux が safety 側に切替、Nav2 は継続だが cmd_vel は 0。
  Nav2 の transform_tolerance (0.3 s) 超えで controller が abort し、
  bt_navigator は re-plan を試みる (ここは Nav2 標準挙動)
  緩和 B: 失探した状態で 30 秒経過なら操作者が手動で
  `/reinitialization_requested` を publish (A 層発火 → cmd_vel zero →
  RViz で /initialpose 再設定)
  緩和 C: デモ経路は M6R-2 verify で走った同じ経路を優先。initial pose も
  同じ (origin identity)。
- **R4 (costmap 更新周期不足で人を捉えない)**:
  内容: 現行 `update_frequency: 5.0` = 0.2 s 周期。WHILL 0.3 m/s と歩行者
  1.4 m/s の相対速度 ~1.7 m/s では 0.34 m/更新。lookahead 0.6 m で 1-2 更新
  以内に検知必須
  緩和: M6R4-3 V2 の合格条件 = 「1.0 秒以内に減速開始」= 5 更新分の余裕。
  V2 で miss したら update_frequency を 10 Hz に上げる (CPU 余力あり、
  M6R-2 verify で bringup 11 ノード時のロード余裕を確認済)
- **R5 (屋外での false positive)**:
  内容: 太陽光反射 (ガラス面、水たまり)、揺れる植生、風で舞う紙屑等が
  obstacle layer に一過性障害物を焼き付けると走行不能
  緩和: `obstacle_layer` の default (marking + clearing 有効) に加えて、
  `raytrace_max_range: 25.0` で通過後の cell を確実に clearing。V4 の
  30 分走行で「stuck (recovery 3 回連続)」が発生したら raytrace 距離
  短縮 or `voxel_layer` への切替 (obstacle_layer 2D は Z 情報を捨てるので
  高い障害物が下から見えて誤検知するケースがある。voxel_layer は 3D 保持)
  → ADR-0008 候補
- **R6 (デモ前日のパラメータチューニング不足)**:
  内容: 屋外実機は再現性が低く、V4 (30 分走行) の合格判定が遅れると
  デモ前日に発覚
  緩和: **デモ日程逆算 (§6) で M6R4-3 完了目標を「デモ 5 日前」に置く**。
  以降は tuning 予備日 & リハーサル。M6R4-4 は判断で skip 可
- **R7 (M6R-3 lite の failsafe が Nav2 走行中に頻発)**:
  内容: B 層閾値 fitness > 1.0 継続は M6R-2 verify での 0.02-0.3 レンジに
  余裕 3x で設定したが、Nav2 走行時は旋回・加減速で fitness 上振れの
  可能性。1 秒 (仮) 継続で発火だと歩行中に false positive
  緩和: M6R4-2 の 60 秒静止で fitness の分布 (mean / 99 percentile) を
  ヒストグラム保存、M6R-3 lite の 閾値ドキュメント (ADR-0007) と突き合わせ。
  超える場合は M6R-3 lite の閾値見直し PR を M6R4-3 前に入れる
- **R8 (map -> odom TF 遅延)**:
  内容: M6R-2 localizer は 10 Hz publish、TF は `enable_map_odom_tf: true`
  で毎 scan publish。Nav2 controller は 10 Hz、transform_tolerance 0.3 s
  = 3 frame 分。TF 遅延が 200 ms を超えると controller が abort
  緩和: M6R4-1 T2 で `ros2 topic hz /tf` 確認、`ros2 topic delay
  /pcl_pose` (もしあれば) or bag record で TF stamp と現在時刻の差を計測。
  200 ms 超えなら transform_tolerance を 0.5 s に戻す

## 6. スケジュール (phase 順序と依存)

**デモ本番日は未確定** (2026-07-14 時点)。日付ベースの逆算は本節では
行わず、phase 順序と依存関係のみ守る。デモ日が確定次第、本節に
「クリティカルパス上の phase 完了目標日」を追記する。

**phase 順序 (直列必須)**:

```
M6R-3 lite merged (2026-07-14 中に完了予定)
   │
   ▼
M6R4-1 (Nav2 骨格)         ─ 実機不要 (bringup 静止確認のみ)、半日〜1 日
   │
   ▼
M6R4-2 (obstacle layer)    ─ 実機静止テスト必須 (屋外、~1 時間)
   │
   ▼
M6R4-3 (collision + E2E)   ─ 実機屋外走行必須。V4 (30 分連続) がクリティカルパス
   │                        天候依存。1 回で pass しない場合は再走日を確保
   │
   ▼
M6R4-4 (CLI wrapper)       ─ optional。着手判定は M6R4-3 完了時、§3 M6R4-4 の
                             判断条件参照
```

**クリティカルパス**: M6R4-3 V4 (30 分連続走行、TF 破綻ゼロ)。ここが
最大リスクなので、V4 を通過してからデモ日をコミットするのが安全。

**バッファ確保**: M6R4-4 を skip 判断できる余地があるため、M6R4-3 完了
から実質 2 日はリハ・予備チューニングに使える。Web/rosbridge 経由の
配車 UI は M7/M8 スコープであり、デモ当日は「操作者が Terminal or
RViz から goal を送信する」で成立する。

**デモ日確定時の TODO**: (a) M6R4-3 完了目標日を「デモ本番 - 5 日」以上
に置く、(b) M6R4-4 着手要否をこの逆算から判定、(c) リハーサル日程を
2 回分確保。

## 7. ADR 候補

以下が本フェーズで発生する重要判断。M6R4-3 完了時までに proposed 起案、
デモ後 accepted 化予定:

- **ADR-0008 (候補): Nav2 costmap 構成の選定**
  - static + obstacle (2D LaserScan) + inflation の 3 層。
    voxel_layer への昇格条件は R5 の false positive 発生時
  - 動的除去済み static map + 実時間 obstacle layer の役割分担を明記
- **ADR-0009 (候補): pointcloud_to_laserscan パラメータの選定**
  - min_height/max_height の実測根拠、angle_increment の分解能選定、
    range_min の車体除去、range_max の raytrace 一致
  - VLP-16 の 16 ring から `.range_min/range_max` を落とす際の
    Z 情報損失の是非
- **ADR-0010 (候補): Nav2 planner + controller の選定 + allow_unknown**
  - NavfnPlanner (grid Dijkstra) 継続、RegulatedPurePursuit 継続の根拠
  - 屋外広域走行での DWB / MPPI 比較は demo 後の課題として明記
  - `allow_unknown: false` (M6R4-1 変更) の根拠 = 外周走行に限定
    (2026-07-14 ユーザー承認済)

**起票タイミング**:
- **ADR-0010** proposed = M6R4-1 完了時 (`allow_unknown: false` 反転が
  ユーザー承認済で M6R4-1 の params 変更と同期する必要があるため、
  M6R4-3 完了待ちから前倒し)
- **ADR-0008 / 0009** proposed = M6R4-2 完了時
- 全 ADR accepted 化 = デモ後 (実走行データが揃った時点)

## 8. M6R-3 lite との統合ポイント

M6R-3 lite が今日中に merge される想定で、以下の topic 契約を M6R4-1 側で
遵守する:

| topic | publisher | subscriber | 内容 |
|-------|-----------|------------|------|
| `/cmd_vel_nav` | controller_server, behavior_server (remap) | twist_mux | Nav2 出力 (Twist) |
| `/cmd_vel_safety` | failsafe_node | twist_mux | 停止 twist (zero) |
| `/cmd_vel` | twist_mux | velocity_smoother | 選択後 twist (優先度: safety=100 > nav=10) |
| `/whill/controller/cmd_vel` | velocity_smoother | whill_driver | rate-limited 最終値 |
| `/reinitialization_requested` | operator (RViz plugin or CLI) | failsafe_node | A 層発火トリガ |
| `/alignment_status` | lidar_localization | failsafe_node | B 層観測 |
| `/pcl_pose` | lidar_localization | failsafe_node | B 層観測 (途絶検知) |

**failsafe 発火時の Nav2 挙動**:
- twist_mux が safety 側を通す → Nav2 の cmd_vel_nav は publish 継続だが
  車体は動かない
- Nav2 側は controller_server が `progress_checker` (`movement_time_allowance: 10.0`)
  で 10 秒進捗なしを検出 → controller が失敗、bt_navigator が recovery
  (spin/backup/wait) を試みる
- **重要**: recovery の spin/backup は failsafe 継続中も cmd_vel_nav に
  出力される。twist_mux が safety 側を通している限り実効は zero
- 復帰: failsafe A 層 (reinit) は operator が initial pose を再入力するまで
  発火継続。B 層は fitness が閾値以下 & pcl_pose 復活で自動復帰
  (lite 版に SAFE_HOLD なし = 復帰は即時)。Nav2 側は progress が回復
  次第 controller_server が正常戻り

**M6R4 の設計判断**:
- Nav2 側では **failsafe 発火を検知しない** (疎結合維持)。Nav2 は
  cmd_vel_nav を出し続け、実効停止は twist_mux が担う
- **cancel goal の判定は operator 責任**: failsafe 発火が長引いた場合、
  operator が `ros2 action send_goal --cancel-all /navigate_to_pose` で
  goal を明示的に取り消す (M6R4-4 CLI ラッパでもこのコマンドを提供)

**推測が入っている項目 (M6R-3 lite の実装確認要)**:
- twist_mux の topic 名 (`/cmd_vel_nav`, `/cmd_vel_safety`, `/cmd_vel`) が
  ADR-0007 §4 と一致するか (これは M6R-3 lite が merge されたら
  `config/twist_mux.yaml` を読んで確定)
- velocity_smoother が twist_mux の下流に置かれるか、それとも twist_mux が
  velocity_smoother 出力を差し替える構成か。ADR-0007 §4 の記述からは
  「twist_mux 出力 = `/cmd_vel` → velocity_smoother がそのまま subscribe」
  と読める。この解釈で M6R4-1 を進める

---

## 承認前チェック

### 前提のうち "推測で置いた" 項目 (実 params が読めなかった箇所)

1. **M6R-3 lite の twist_mux topic 契約** — ADR-0007 §4 記載の
   `/cmd_vel_nav` / `/cmd_vel_safety` / `/cmd_vel` の 3 topic を採用と
   仮定した。M6R-3 lite 実装で異なる名前が採用される場合、M6R4-1 の
   remap を合わせて修正
2. **M6R-3 lite の B 層閾値** — ADR-0007 §3 の `FITNESS_MAX=1.0` /
   `WINDOW_S=2.0` を採用と仮定。lite 縮小で異なる場合、R7 のヒスト
   グラム観測結果と突き合わせて再調整
3. **WHILL CR2 のフットプリント** — `robot_radius: 0.45` は既存 params
   (1.0 m × 0.6 m の外接円 0.45) を継続。実測 URDF が landing すれば
   polygon footprint に切替
4. **pointcloud_to_laserscan の height 範囲** — `min_height: -0.2` /
   `max_height: 1.6` は WHILL base_link + 搭乗者高さの仮の値。R2 に
   従い M6R4-2 debug 冒頭で実測補正
5. **NavfnPlanner + RegulatedPurePursuit の継続採用** — 既存 params の
   継続。屋外広域で不足なら demo 後に ADR-0010 で SmacPlanner2D / MPPI
   比較
6. **local costmap の `map` frame 継続** — M4-R EKF が `odom` frame を
   供給しているので技術的には odom frame に切り替え可能だが、M5-c の
   コメント整合を全部書き直す作業を回避
7. **update_frequency: 5.0 Hz の継続** — R4 で不足時に 10 Hz へ引き
   上げると記載したが、初期値は既存継続

**2026-07-14 ユーザー承認で確定した項目 (推測ではなくなった)**:

- **`allow_unknown: false` への反転** — `campus` マップ中央 unknown を
  通行禁止扱い。外周のみ走行のデモ経路に合致 (V4 走行で経路が引けなく
  なる懸念があれば ADR-0010 で真偽検証)
- **デモ日程** — 未確定。§6 の日付ベース逆算は保留、phase 順序のみ守る
- **M6R4-4 の着手判断** — M6R4-3 完了時に §3 M6R4-4 の判断条件で決定

### 実機検証が必要でユーザーに手渡す作業

- **M6R4-1**: T1-T5 の bringup 静止確認 (実機電源 ON、走行しない)
- **M6R4-2**:
  - U1-U5 の静止 obstacle layer 確認 (屋外、~1 時間)
  - U4 の人立たせテスト (人手 1 名)
- **M6R4-3**:
  - V1-V3 の人横断テスト (人手 2 名 = 走行監視 + 横断)、~2 時間
  - V4 の 30 分連続走行 (安全操作者 1 名随伴)、天候安定日
  - V5/V6 の safety 相互作用確認
- **M6R4-4** (着手判断次第):
  - W1-W3 の CLI シーケンス確認、リハーサル兼用
- **デモ waypoint 座標の実測** (M6R4-4 の入力): メジャー実測 or
  RViz で発進点から走行して `/pcl_pose` を採取

### ADR 起票が必要な項目

- **ADR-0008**: Nav2 costmap 構成 (static + obstacle + inflation 3 層)
  — proposed 起票: M6R4-2 完了時
- **ADR-0009**: pointcloud_to_laserscan パラメータ選定
  — proposed 起票: M6R4-2 完了時
- **ADR-0010**: Nav2 planner + controller の継続採用 (NavfnPlanner +
  RegulatedPurePursuit) と `allow_unknown: false` の根拠
  — proposed 起票: M6R4-3 完了時
- 全 ADR の accepted 化: デモ後 (実走行データ揃い次第)

### 追加質問

2026-07-14 に 3 点全て回答済 (§ 冒頭 accepted 記録参照)。追加の未解決
質問なし。M6R4-1 着手可。
