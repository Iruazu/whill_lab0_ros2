"""ROS2 launch for FAST-LIO SAM Velodyne mapping (M5R-3 evaluation).

Why this exists:
  Upstream RightTr/FAST_LIO_SAM ships ROS2 launches only for airy / l2 /
  mid360. The M5R-3 wrapper (scripts/m5r3_run_fastlio_sam.sh) hardcodes
  "mapping_velodyne.launch.py", which exists in neither ROS1 nor ROS2
  trees — the first run (2026-06-21) produced zero trajectory because
  the SLAM node never started. This launch is the missing piece: model
  after mapping_airy.launch.py but point at our local config file
  (m5r3_fastlio_sam_velodyne_config.yaml) for the Velodyne / MPU-9250
  rig.

CLAUDE.md scope:
  src/third_party/ edits are forbidden (legacy/upstream contract), and
  the M5R-3 scope does not justify a full upstream port. This file
  + companion yaml live under scripts/ and are wired into the run via
  the wrapper's SLAM_LAUNCH env override; no upstream files change.

Headless:
  RViz is intentionally omitted. Phase B evaluation is bag-driven,
  measurement is end-to-start trajectory drift + CloudCompare on the
  emitted PCD, neither needs a live viewer. Dropping RViz also lets
  the SLAM node exit cleanly when rosbag2_player ends, avoiding the
  Standard-Viewer-style hang we hit with GLIM run #2.
"""

import os

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, "m5r3_fastlio_sam_velodyne_config.yaml")


def _flatten(d, parent_key="", sep="/"):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(_flatten(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def _load_ros1_yaml_as_params(yaml_file_path):
    # Same logic as upstream mapping_airy.launch.py — FAST-LIO inherited
    # ROS1's nested-yaml style and the launch flattens to /-joined keys.
    with open(yaml_file_path, "r") as f:
        config = yaml.safe_load(f)
    return _flatten(config)


def generate_launch_description():
    yaml_params = _load_ros1_yaml_as_params(CONFIG_PATH)

    # These overrides are copied from mapping_airy.launch.py verbatim.
    # They are not redundant with the yaml: upstream FAST-LIO SAM reads
    # some of them as launch-level params (separate from the lio_sam
    # nested config). Keeping the same set keeps the apples-to-apples
    # comparison with airy.
    params = [
        {"sam_enable": True},
        {"feature_extract_enable": False},
        {"point_filter_num": 4},
        {"max_iteration": 3},
        {"filter_size_surf": 0.5},
        {"filter_size_map": 0.5},
        {"cube_side_length": 1000.0},
        {"runtime_pos_log_enable": False},
        yaml_params,
    ]

    fast_lio_sam = Node(
        package="fast_lio_sam",
        executable="fastlio_mapping",
        output="screen",
        parameters=params,
    )

    return LaunchDescription([fast_lio_sam])
