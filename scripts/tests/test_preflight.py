"""m6r_preflight.sh、check_dds_nic.py、check_device_aliases.sh を、偽の ros2・ip・udevadm と
偽の /dev で検証する。実機は使わない。"""
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
from check_dds_nic import find_problems  # noqa: E402

SUBNET = '192.168.1.0/24'
NIC = 'enxcurrent'

# SILENT に挙げたトピックは受信待ちのまま止まる。実物と同じく、QoS が合わないと届かない。
FAKE_ROS2 = r"""#!/usr/bin/env bash
args="$*"
case "$args" in
  'topic list') exit 0 ;;
  'param get'*) echo 'Boolean value is: True'; exit 0 ;;
  'node list') echo /failsafe_node; exit 0 ;;
esac
for t in $SILENT; do [[ "$args" == *" $t"* ]] && exec sleep 100; done
case "$args" in
  *'/scan'*|*'/velodyne_points_no_ground'*)
    [[ "$args" == *'--qos-profile sensor_data'* ]] || exec sleep 100 ;;
  *'/pcl_pose'*|*'/alignment_status'*)
    [[ "$args" == *'--qos-reliability reliable'* ]] || exec sleep 100 ;;
  *'/rosout'*) exec sleep 100 ;;
esac
# 型を渡さないと、実物は発見の前に即座に失敗する。
case "$args" in
  *'/alignment_status diagnostic_msgs/msg/DiagnosticArray'*) ;;
  *'/pcl_pose geometry_msgs/msg/PoseWithCovarianceStamped'*) ;;
  *'/velodyne_points_no_ground sensor_msgs/msg/PointCloud2'*) ;;
  *'/scan sensor_msgs/msg/LaserScan'*) ;;
  *'/alignment_status'*|*'/pcl_pose'*|*'/velodyne_points_no_ground'*|*'/scan'*) exit 1 ;;
esac
case "$args" in
  *'/cmd_vel_safety --field'*) for _ in $(seq 60); do echo ---; done; exit 0 ;;
esac
exit 0
"""


# 偽の udevadm。FAKE_UDEV_IDS の「名前 VID:PID」から attribute-walk を作る。実物と同じく
# tty の上に interface、usb_device、hub の順に並べ、hub の値を拾う誤りも検出できるようにする。
FAKE_UDEVADM = r"""#!/usr/bin/env bash
for a in "$@"; do case "$a" in --name=*) name=${a#--name=} ;; esac; done
base=$(basename "$name")
ids=$(awk -v b="$base" '$1 == b {print $2}' "$FAKE_UDEV_IDS")
[[ -n "$ids" ]] || exit 1
cat <<EOT
  looking at device '/devices/pci0000:00/usb3/3-2/3-2.4/3-2.4:1.0/tty/$base':
    KERNEL=="$base"
    SUBSYSTEM=="tty"
  looking at parent device '/devices/pci0000:00/usb3/3-2/3-2.4/3-2.4:1.0':
    ATTRS{bInterfaceNumber}=="00"
  looking at parent device '/devices/pci0000:00/usb3/3-2/3-2.4':
    ATTRS{idProduct}=="${ids#*:}"
    ATTRS{idVendor}=="${ids%:*}"
  looking at parent device '/devices/pci0000:00/usb3/3-2':
    ATTRS{idProduct}=="0610"
    ATTRS{idVendor}=="05e3"
EOT
"""

RULES = SCRIPTS.parent / 'udev' / '99-whill-stack.rules'
WHILL_AND_IMU = {'ttyUSB0': '067b:2303', 'ttyACM0': '2b72:0003'}


def executable(path, text):
    path.write_text(text)
    path.chmod(path.stat().st_mode | stat.S_IEXEC)


class FakeHost:
    """偽の ros2・ip・udevadm と /dev を用意する。既定は WHILL と IMU が正しく繋がった状態。"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        executable(root / 'ros2', FAKE_ROS2)
        links = [{'ifname': 'lo', 'addr_info': [{'local': '127.0.0.1'}]},
                 {'ifname': NIC, 'addr_info': [{'local': '192.168.1.100'}]}]
        executable(root / 'ip', "#!/bin/sh\necho '%s'\n" % json.dumps(links))
        executable(root / 'udevadm', FAKE_UDEVADM)
        self.root = root
        self.xml = self.write_xml(NIC)
        self.dev = root / 'dev'
        self.dev.mkdir()
        self.plug(WHILL_AND_IMU, {'whill': 'ttyUSB0', 'imu': 'ttyACM0'})

    def plug(self, ttys, aliases):
        """偽の /dev に tty と別名を作る。udev の代わりに、別名を作るかを試験が決める。"""
        for entry in self.dev.iterdir():
            entry.unlink()
        for name in ttys:
            (self.dev / name).write_text('')
        for alias, target in aliases.items():
            (self.dev / alias).symlink_to(target)
        (self.root / 'ids').write_text(''.join(f'{n} {i}\n' for n, i in ttys.items()))

    def env(self, **extra):
        env = dict(os.environ, PATH=f'{self.root}:{os.environ["PATH"]}',
                   FAKE_UDEV_IDS=str(self.root / 'ids'), WHILL_DEV_DIR=str(self.dev),
                   WHILL_UDEV_INSTALLED=str(RULES))
        env.update(extra)
        return env

    def write_xml(self, nic):
        path = self.root / 'dds.xml'
        path.write_text('<CycloneDDS xmlns="https://cdds.io/config"><Domain><General><Interfaces>'
                        '<NetworkInterface name="lo"/><NetworkInterface name="%s"/>'
                        '</Interfaces></General></Domain></CycloneDDS>' % nic)
        return path


class PreflightTests(FakeHost, unittest.TestCase):
    def run_preflight(self, silent='', uri=None):
        env = self.env(SILENT=silent, PREFLIGHT_INPUT_TIMEOUT_S='1')
        env['CYCLONEDDS_URI'] = f'file://{self.xml}' if uri is None else uri
        if uri == '':
            del env['CYCLONEDDS_URI']
        return subprocess.run(['bash', str(SCRIPTS / 'm6r_preflight.sh')], env=env, text=True,
                              capture_output=True, timeout=60)

    def test_all_good_passes_without_touching_rosout(self):
        result = self.run_preflight()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('preflight PASS', result.stdout)

    def test_silent_input_fails_in_bounded_time(self):
        start = time.monotonic()
        result = self.run_preflight(silent='/scan')
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertLess(time.monotonic() - start, 15)
        self.assertRegex(result.stdout, r'/scan +Layer D +1 秒以内に届かない')
        self.assertNotIn('Layer D live fire', result.stdout)

    def test_all_silent_inputs_are_named(self):
        result = self.run_preflight(silent='/pcl_pose /velodyne_points_no_ground')
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertRegex(result.stdout, r'/pcl_pose +Layer B +1 秒以内')
        self.assertRegex(result.stdout, r'/velodyne_points_no_ground +Layer C +1 秒以内')

    def test_old_nic_name_fails_before_any_ros_check(self):
        self.write_xml('enxold')
        result = self.run_preflight()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('enxold', result.stdout)
        self.assertNotIn('1. use_collision_detection', result.stdout)

    def test_unset_uri_fails(self):
        result = self.run_preflight(uri='')
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('CYCLONEDDS_URI is not set', result.stdout)

    def test_missing_xml_fails(self):
        result = self.run_preflight(uri='file:///nonexistent.xml')
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('missing file', result.stdout)

    def test_missing_imu_alias_fails_before_param_checks(self):
        self.plug(WHILL_AND_IMU, {'whill': 'ttyUSB0'})
        result = self.run_preflight()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('sudo udevadm trigger --action=change --name-match=/dev/ttyACM0', result.stdout)
        self.assertNotIn('1. use_collision_detection', result.stdout)


class DeviceAliasTests(FakeHost, unittest.TestCase):
    def run_check(self, **extra):
        return subprocess.run(['bash', str(SCRIPTS / 'check_device_aliases.sh')],
                              env=self.env(**extra), text=True, capture_output=True, timeout=30)

    def test_both_aliases_ok(self):
        result = self.run_check()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertRegex(result.stdout, r'/dev/whill .*ttyUSB0 +ok \(067b:2303\)')
        self.assertRegex(result.stdout, r'/dev/imu .*ttyACM0 +ok \(2b72:0003\)')

    def test_missing_alias_with_matching_tty_prints_recovery(self):
        # 2026-10-30 の状態。ttyACM0 は規則に合うが /dev/imu が無い。
        self.plug(WHILL_AND_IMU, {'whill': 'ttyUSB0'})
        result = self.run_check()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('/dev/ttyACM0 が規則（2b72:0003）に合っている', result.stdout)
        self.assertIn('udevadm info --query=all --name=/dev/ttyACM0', result.stdout)
        self.assertIn('sudo udevadm trigger --action=change --name-match=/dev/ttyACM0', result.stdout)
        self.assertRegex(result.stdout, r'/dev/whill .*ok')

    def test_unplugged_devices_are_all_named(self):
        self.plug({}, {})
        result = self.run_check()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertRegex(result.stdout, r'/dev/whill +無し。規則（067b:2303）に合う ttyACM / ttyUSB も無い')
        self.assertRegex(result.stdout, r'/dev/imu +無し。規則（2b72:0003）に合う ttyACM / ttyUSB も無い')
        self.assertNotIn('udevadm trigger', result.stdout)

    def test_alias_to_wrong_device_fails(self):
        self.plug(WHILL_AND_IMU, {'whill': 'ttyUSB0', 'imu': 'ttyUSB0'})
        result = self.run_check()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('規則は 2b72:0003、実物は 067b:2303', result.stdout)

    def test_ids_come_from_rules_file(self):
        rules = self.root / 'rules'
        rules.write_text(RULES.read_text().replace('2b72', 'abcd'))
        result = self.run_check(WHILL_UDEV_RULES=str(rules))
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('規則は abcd:0003、実物は 2b72:0003', result.stdout)
        # 入っている規則（ここでは RULES）とも違うので、入れ直しを促す。
        self.assertIn('リポジトリと違います', result.stdout)

    def test_comment_only_difference_is_not_reported(self):
        # 実機の /etc の規則は、コメントだけがリポジトリと違う（2026-11-12 に確認）。
        installed = self.root / 'installed'
        installed.write_text('# 古いコメント\n' + RULES.read_text())
        self.plug({'ttyUSB0': '067b:2303'}, {'whill': 'ttyUSB0'})
        result = self.run_check(WHILL_UDEV_INSTALLED=str(installed))
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertNotIn('リポジトリと違います', result.stdout)
        self.assertNotIn('規則が入っていません', result.stdout)

    def test_last_rule_without_trailing_newline_is_read(self):
        rules = self.root / 'rules'
        rules.write_text(RULES.read_text().rstrip('\n'))
        self.plug(WHILL_AND_IMU, {'whill': 'ttyUSB0'})
        result = self.run_check(WHILL_UDEV_RULES=str(rules))
        self.assertEqual(result.returncode, 1, result.stdout)
        # IMU は最終行の規則。読み落とすと /dev/imu を確かめずに通ってしまう。
        self.assertIn('/dev/imu', result.stdout)

    def test_dangling_alias_is_distinguished_from_missing(self):
        # ttyACM0 が消えた後にリンクだけ残った状態。別名が無い場合と原因の候補が違う。
        self.plug({'ttyUSB0': '067b:2303'}, {'whill': 'ttyUSB0', 'imu': 'ttyACM0'})
        result = self.run_check()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('/dev/imu    リンクはあるが指す先が無い: ttyACM0', result.stdout)
        self.assertNotIn('/dev/imu    無し', result.stdout)

    def test_unreadable_attributes_are_not_called_a_mismatch(self):
        self.plug(WHILL_AND_IMU, {'whill': 'ttyUSB0', 'imu': 'ttyACM1'})
        (self.dev / 'ttyACM1').write_text('')  # 偽の udevadm が知らない tty
        result = self.run_check()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('udevadm で属性を読めません', result.stdout)
        self.assertNotIn('規則は', result.stdout)

    def test_recovery_hints_include_evidence_commands(self):
        self.plug(WHILL_AND_IMU, {'whill': 'ttyUSB0'})
        result = self.run_check()
        for hint in ('ls -l /run/udev/links/', 'journalctl -k --since', 'systemctl is-active ModemManager'):
            self.assertIn(hint, result.stdout)


class NicTests(unittest.TestCase):
    HOST = {'lo': ['127.0.0.1'], NIC: ['192.168.1.100'], 'wlo1': ['10.130.7.163']}

    def test_consistent(self):
        self.assertEqual(find_problems(['lo', NIC], self.HOST, SUBNET), [])

    def test_old_name_is_reported_both_ways(self):
        problems = find_problems(['lo', 'enxold'], self.HOST, SUBNET)
        self.assertTrue(any('実機にありません: enxold' in p for p in problems), problems)
        self.assertTrue(any(f'{NIC} が XML にありません' in p for p in problems), problems)

    def test_lidar_nic_unplugged(self):
        host = {'lo': ['127.0.0.1'], 'wlo1': ['10.130.7.163']}
        problems = find_problems(['lo', NIC], host, SUBNET)
        self.assertEqual(len(problems), 2, problems)

    def test_nic_present_but_no_lidar_address(self):
        host = {'lo': ['127.0.0.1'], NIC: []}
        problems = find_problems(['lo', NIC], host, SUBNET)
        self.assertEqual(len(problems), 1, problems)
        self.assertIn('持つ NIC がありません', problems[0])

    def test_loopback_only_xml(self):
        problems = find_problems(['lo'], self.HOST, SUBNET)
        self.assertTrue(any('lo 以外' in p for p in problems), problems)


if __name__ == '__main__':
    unittest.main()
