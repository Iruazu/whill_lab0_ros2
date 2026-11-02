"""m6r_preflight.sh と check_dds_nic.py を、偽の ros2 と ip で検証する。実機は使わない。"""
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


def executable(path, text):
    path.write_text(text)
    path.chmod(path.stat().st_mode | stat.S_IEXEC)


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        executable(root / 'ros2', FAKE_ROS2)
        links = [{'ifname': 'lo', 'addr_info': [{'local': '127.0.0.1'}]},
                 {'ifname': NIC, 'addr_info': [{'local': '192.168.1.100'}]}]
        executable(root / 'ip', "#!/bin/sh\necho '%s'\n" % json.dumps(links))
        self.root = root
        self.xml = self.write_xml(NIC)

    def write_xml(self, nic):
        path = self.root / 'dds.xml'
        path.write_text('<CycloneDDS xmlns="https://cdds.io/config"><Domain><General><Interfaces>'
                        '<NetworkInterface name="lo"/><NetworkInterface name="%s"/>'
                        '</Interfaces></General></Domain></CycloneDDS>' % nic)
        return path

    def run_preflight(self, silent='', uri=None):
        env = dict(os.environ, PATH=f'{self.root}:{os.environ["PATH"]}', SILENT=silent,
                   PREFLIGHT_INPUT_TIMEOUT_S='1')
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
