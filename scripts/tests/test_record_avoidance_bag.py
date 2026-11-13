"""record_avoidance_bag.sh を、偽の ros2・ip と偽の governor で検証する。実機は使わない。

録画前の確認で止まるべき所で止まり、記録後の確認が欠けた bag でも黙って終わらない
（set -euo pipefail での grep の不一致）ことを確かめる。
"""
import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
SCRIPT = SCRIPTS / 'record_avoidance_bag.sh'
NIC = 'enxcurrent'

# SILENT に挙げたトピックは受信待ちのまま止まる。bag record は出力先を作るだけ。
# bag info は FAKE_BAG_INFO をそのまま返す。
FAKE_ROS2 = r"""#!/usr/bin/env bash
args=" $* "
case "$1 $2" in
  'pkg prefix') exit 0 ;;
  'topic echo')
    for t in $SILENT; do [[ "$args" == *" $t "* ]] && exec sleep 100; done
    exit 0 ;;
  'bag record')
    prev=; for a in "$@"; do [[ "$prev" == -o ]] && out=$a; prev=$a; done
    mkdir -p "$out"; printf '%s\n' "$*" > "$out/cmd"; exit 0 ;;
  'bag info') printf '%s\n' "$FAKE_BAG_INFO"; exit 0 ;;
esac
exit 0
"""

INFO_OK = """Files:             x.mcap
Bag size:          1.0 GiB
Duration:          12.0s
Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 120 | Serialization Format: cdr
Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr"""
INFO_NO_CLOUD = """Files:             x.mcap
Bag size:          1.0 MiB
Duration:          12.0s
Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr"""


def executable(path, text):
    path.write_text(text)
    path.chmod(path.stat().st_mode | stat.S_IEXEC)


class RecordTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        executable(root / 'ros2', FAKE_ROS2)
        links = [{'ifname': 'lo', 'addr_info': [{'local': '127.0.0.1'}]},
                 {'ifname': NIC, 'addr_info': [{'local': '192.168.1.100'}]}]
        executable(root / 'ip', "#!/bin/sh\necho '%s'\n" % json.dumps(links))
        (root / 'gov').write_text('performance\n')
        xml_dir = root / 'configs'
        xml_dir.mkdir()
        self.xml = xml_dir / 'cyclonedds-runtime-min.xml'
        self.xml.write_text(
            '<CycloneDDS xmlns="https://cdds.io/config"><Domain><General><Interfaces>'
            '<NetworkInterface name="lo"/><NetworkInterface name="%s"/>'
            '</Interfaces></General></Domain></CycloneDDS>' % NIC)
        self.bags = root / 'bags'
        self.root = root

    def run_script(self, *args, silent='', info=INFO_OK, **extra):
        env = dict(os.environ, PATH=f'{self.root}:{os.environ["PATH"]}',
                   RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                   CYCLONEDDS_URI=f'file://{self.xml}', WHILL_GOVERNOR_FILE=str(self.root / 'gov'),
                   WHILL_BAGS_ROOT=str(self.bags), SILENT=silent, FAKE_BAG_INFO=info,
                   RECORD_INPUT_TIMEOUT_S='1', PREFLIGHT_INPUT_TIMEOUT_S='1')
        env.update(extra)
        return subprocess.run(['bash', str(SCRIPT), *args], env=env, text=True,
                              capture_output=True, timeout=60)

    def test_static_ok_writes_notes_template(self):
        r = self.run_script('--profile', 'static', '--run', 'r1', 'cone_2m')
        self.assertEqual(r.returncode, 0, r.stderr)
        notes = self.bags / 'r1' / 'notes.csv'
        self.assertTrue(notes.read_text().startswith('bag,object,x_m,height_m,shape,expect'))
        self.assertIn(',empty,', r.stdout)
        bags = [p for p in (self.bags / 'r1').iterdir() if p.is_dir()]
        self.assertEqual(len(bags), 1)
        self.assertTrue(bags[0].name.startswith('cone_2m_'))
        self.assertIn('-s mcap', (bags[0] / 'cmd').read_text())

    def test_missing_cloud_in_bag_warns_instead_of_silent_exit(self):
        r = self.run_script('--profile', 'static', '--run', 'r1', 'x', info=INFO_NO_CLOUD)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('/velodyne_points が記録されていない', r.stderr)
        self.assertIn('notes.csv に次の行', r.stdout)

    def test_low_cloud_count_warns(self):
        r = self.run_script('--profile', 'static', 'x',
                            info=INFO_OK.replace('Count: 120', 'Count: 30'))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('撮り直す', r.stderr)

    def test_required_cloud_silent_fails(self):
        r = self.run_script('--profile', 'static', 'x', silent='/velodyne_points')
        self.assertEqual(r.returncode, 1)
        self.assertIn('/velodyne_points を受信できない', r.stderr)
        self.assertFalse(self.bags.exists())

    def test_static_requires_camera(self):
        r = self.run_script('--profile', 'static', 'x',
                            silent='/camera/camera/depth/image_rect_raw')
        self.assertEqual(r.returncode, 1)
        self.assertIn('realsense:=true', r.stderr)

    def test_drive_camera_optional(self):
        r = self.run_script('--profile', 'drive', 'lap1',
                            silent='/camera/camera/depth/image_rect_raw')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('カメラ無しで記録する', r.stderr)
        self.assertNotIn('notes.csv', r.stdout)

    def test_drive_checks_failsafe_inputs(self):
        r = self.run_script('--profile', 'drive', 'lap1', silent='/pcl_pose')
        self.assertEqual(r.returncode, 1)
        self.assertIn('failsafe の入力が揃わない', r.stderr)

    def test_drive_does_not_require_cmd_vel(self):
        # 停止中は twist_mux が何も出さない。/cmd_vel が無くても録画できること。
        r = self.run_script('--profile', 'drive', 'lap1', silent='/cmd_vel')
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_preflight_rejects_wrong_env(self):
        old = self.root / 'configs' / 'cyclonedds-runtime.xml'
        old.write_text(self.xml.read_text())
        cases = [({'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}, 'RMW_IMPLEMENTATION'),
                 ({'CYCLONEDDS_URI': f'file://{old}'}, 'runtime-min'),
                 ({'WHILL_GOVERNOR_FILE': str(self.root / 'nope')}, 'governor')]
        for env, msg in cases:
            with self.subTest(msg=msg):
                r = self.run_script('x', **env)
                self.assertEqual(r.returncode, 1)
                self.assertIn(msg, r.stderr)

    def test_bad_args(self):
        for args, msg in ((['--profile'], '値が要る'), (['--profile', 'foo', 'x'], 'profile'),
                          (['a,b'], 'タグ'), (['--duration', '0', 'x'], 'duration')):
            with self.subTest(args=args):
                r = self.run_script(*args)
                self.assertNotEqual(r.returncode, 0)
                self.assertIn(msg, r.stderr)


if __name__ == '__main__':
    unittest.main()
