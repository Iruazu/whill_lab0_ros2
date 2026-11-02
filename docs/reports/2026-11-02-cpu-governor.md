# #136 CPU governorの永続化

2026-11-02。導入前はintel_pstate active、20 policyすべてpowersave。
cpupowerは導入済み。cpufrequtilsとTLPは未導入。

`scripts/cpu_governor.py` は `whill-cpu-performance.service` を追加する。
起動時に既存power-profiles-daemonの後で全CPUへperformanceを設定し、
全policyを読み返して検査する。既存サービス・GRUB・デモスクリプトは変更しない。
performance/powersaveはintel_pstateの制御方式であり、powersaveでも動的に周波数は上がる。
本対応は録画スクリプトのperformance要件を再起動後も満たすためのもの。
仕様は [Linux kernel文書](https://docs.kernel.org/6.1/admin-guide/pm/intel_pstate.html) を参照。

## ホストで導入

```bash
# 本スクリプトを含むリポジトリのルートで実行する。
sudo python3 scripts/cpu_governor.py install
```

導入前のpolicy値とboot IDは `/var/lib/whill-cpu-performance/before.json` に保持する。
インストール先は `/usr/local/lib/whill-cpu-performance.py` と
`/etc/systemd/system/whill-cpu-performance.service`。
異なる既存ファイルは上書きしない。同じ内容での再実行は可能。
導入は即時にgovernorを変更するので、並行測定の途中では実行しない。
自動再起動はしない。

## 確認

```bash
python3 /usr/local/lib/whill-cpu-performance.py check
# 並行作業を終えて通常の再起動を行った後:
python3 /usr/local/lib/whill-cpu-performance.py verify-reboot
```

後者はboot ID変化、サービスenabled/active、全policyのperformanceを検査する。
導入直後の成功だけでは#136を完了扱いしない。
GUIの電源プロファイル変更・サスペンド復帰・CPU hotplugでの再適用は対象外。
値が変わった場合は `sudo systemctl restart whill-cpu-performance` で再適用し確認する。
再起動後に他の電源管理が上書きする場合は、その競合を調べる。定期的な上書きはしない。

## 取り消し

```bash
sudo python3 /usr/local/lib/whill-cpu-performance.py remove
```

サービスを停止・無効化して削除し、導入前のpolicyごとのgovernorに戻す。
バックアップは保持する。導入ファイルが第三者に変更されていれば削除を拒否する。

## 現時点の検証

Python構文・差分の空白検査に成功。実機の読み取り検査で20 policyすべての
powersaveを検出した。隔離した仮のsysfsとsystemctlで、導入・同内容の再導入・
同一boot IDの拒否・再起動後の検査・外部編集の削除拒否・policyごとの復元を確認した。
これらは実際のsystemdサービス起動試験ではない。サンドボックスはsystemdと/etc変更に
アクセスできないため、ホスト導入と再起動後の検証はユーザーが実施した（下記）。
Claudeの並行作業用ファイルとGPU実験は変更しない。

## ホスト受入結果（2026-11-02）

ユーザーがinstall、再起動、verify-rebootを実行し、全20 policyがperformance、
サービスがenabledかつactive、再起動後検査PASSである出力を共有した。
こちらでも読み取りで全20 policyのperformanceとインストール済みunitを確認した。

- 導入前boot ID: `12b20993-1146-4db1-85bc-55316099a1af`
- 再起動後boot ID: `d16e8176-1257-4a4d-a7b9-d24f402d23d8`
- `/var/lib/whill-cpu-performance/before.json` は導入前の全policyのpowersaveを保持。

#136の完了条件「再起動後にgovernorがperformance」は満たした。
ホストへの導入・受入は完了。スクリプトと本報告のmainへの反映はPRのマージで行う。
