"""
obstacle_avoidance.launch.py
============================
Launches the depth-based obstacle avoidance node.

AUTONOMOUS mode (default): node publishes to /cmd_vel — do NOT run teleop
or lane_nav at the same time, as they will fight for control.

DRY RUN mode (dry_run:=true): node runs the full depth pipeline and shows
the debug image, but never touches /cmd_vel.  Safe to run alongside teleop
for camera calibration and threshold tuning.

Usage:
  # Autonomous — robot drives and avoids obstacles:
  ros2 launch obst_avoid obstacle_avoidance.launch.py model:=midas

  # Dry run — you drive with teleop, node shows what it detects:
  ros2 launch obst_avoid obstacle_avoidance.launch.py model:=midas dry_run:=true

Optional args:
  variant:=small          model size
  threshold:=0.20         normalised depth trigger threshold
  speed:=0.20             forward driving speed (m/s, autonomous only)
  turn_dur:=3.5           seconds to hold right-turn command (tune for ~90°)
  debug:=true             publish /depth/colorized debug image

Visualise depth and detection zone:
  ros2 run rqt_image_view rqt_image_view /depth/colorized

Emergency stop (autonomous mode):
  ros2 topic pub /cmd_vel geometry_msgs/Twist "{}"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("obst_avoid")
    params_file = os.path.join(pkg, "config", "params.yaml")

    args = [
        DeclareLaunchArgument(
            "model", default_value="midas",
            description="Depth model: midas | depth_anything_v2 | zoe_depth | depth_pro"
        ),
        DeclareLaunchArgument(
            "variant", default_value="small",
            description="Model variant: small | base | large | hybrid | n | k | nk"
        ),
        DeclareLaunchArgument(
            "threshold", default_value="0.20",
            description="Normalised depth trigger threshold [0,1]"
        ),
        DeclareLaunchArgument(
            "speed", default_value="0.20",
            description="Forward driving speed (m/s)"
        ),
        DeclareLaunchArgument(
            "turn_dur", default_value="3.5",
            description="Seconds to hold right-turn (tune for ~90°)"
        ),
        DeclareLaunchArgument(
            "debug", default_value="true",
            description="Publish /depth/colorized debug image"
        ),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="Run detection only — never publish to /cmd_vel (safe with teleop)"
        ),
    ]

    avoidance_node = Node(
        package="obst_avoid",
        executable="obstacle_avoidance_node",
        name="obstacle_avoidance_node",
        parameters=[
            params_file,
            {
                "model_name": LaunchConfiguration("model"),
                "model_variant": LaunchConfiguration("variant"),
                "obstacle_threshold": LaunchConfiguration("threshold"),
                "base_speed": LaunchConfiguration("speed"),
                "turn_duration": LaunchConfiguration("turn_dur"),
                "publish_debug": LaunchConfiguration("debug"),
                "dry_run": LaunchConfiguration("dry_run"),
            },
        ],
        output="screen",
    )

    return LaunchDescription(args + [avoidance_node])
