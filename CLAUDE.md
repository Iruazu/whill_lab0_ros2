# whill_lab0_ros2 — 作業指針

このPCのWHILL Model CR2実機スタック。ROS 2 Humbleを使用する。現用コードを根拠に更新し、実装済みと実機受入済みを区別する。

## 読む文書

- 概要: `README.md`
- 運用: `docs/runbook-live-demo.md`
- 構成: `docs/architecture-overview.md`
- スクリプト: `docs/scripts.md`
- 開発: `docs/development.md`
- 未解決事項: `docs/repository-maintenance.md`
- パッケージ変更前に該当 `src/whill_*/README.md` を読む。

## 現行方針

文書は日本語のみを正本とする。日英同期は不要。過去の計画や実測記録を現在の手順として使わず、コード・設定と照合する。8月デモの受入結果をコードの存在だけで完了扱いしない。

地図作成はGLIM / DUFOMapのオフライン処理。runtime localizerはNDT scan-to-map。FAST-LIO runtime launchとidentity TFは削除済みで、復活させない。

実機起動はm6r_bringup + nav_launch + dispatch_launch。センサ / odom / safetyの重複起動を避ける。自律走行の検証はfailsafe / twist_mux有効の構成で行う。

全ターミナルのRMW・domain・DDS XMLを合わせる。現用runbookは `configs/cyclonedds-runtime-min.xml` を使用。旧runtime / bag-record XMLとの混在を避ける。NIC名は現物確認し、旧名に上書きしない。bashrcは自動編集しない。

## 作業規約

- コメントには判断理由を書く。スクリプトは再実行可能にする。
- 自作コードは既存のBSD-3-Clause方針に合わせ、上流ライセンスは個別に尊重する。
- package.xmlの依存を実装と揃える。
- launch変更時は `ros2 launch <pkg> <launch>.py --show-args` で評価する。
- mainへ直接commit / pushしない。ブランチとPRで提示し、マージはユーザーが行う。
- 通常のADR採択は人が行う。ユーザーの明示指示による文書言語変更はADR-0001の追記に記録済み。
- 既存の未コミット変更を破棄しない。bag・地図PCD・校正値を整理の名目で削除しない。
- 旧noetic調査は `docs/legacy-index.md` を起点とする。
