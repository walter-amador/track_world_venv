"""
depth_benchmark.launch.py
=========================
Launches the passive depth estimation benchmark node.
Drive the robot with teleop while watching /depth/colorized in rqt_image_view.

Usage:
  ros2 launch obst_avoid depth_benchmark.launch.py model:=midas
  ros2 launch obst_avoid depth_benchmark.launch.py model:=depth_anything_v2
  ros2 launch obst_avoid depth_benchmark.launch.py model:=zoe_depth
  ros2 launch obst_avoid depth_benchmark.launch.py model:=depth_pro

Optional args:
  variant:=small          model size (model-specific; see params.yaml)
  threshold:=0.20         normalised depth threshold for zone highlight

Visualise depth:
  ros2 run rqt_image_view rqt_image_view /depth/colorized

Teleop (separate terminal):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
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

    model_arg = DeclareLaunchArgument(
        "model", default_value="midas",
        description="Depth model: midas | depth_anything_v2 | zoe_depth | depth_pro"
    )
    variant_arg = DeclareLaunchArgument(
        "variant", default_value="small",
        description="Model variant: small | base | large | hybrid | n | k | nk"
    )
    threshold_arg = DeclareLaunchArgument(
        "threshold", default_value="0.20",
        description="Obstacle zone normalised depth threshold [0,1]"
    )

    benchmark_node = Node(
        package="obst_avoid",
        executable="depth_benchmark_node",
        name="depth_benchmark_node",
        parameters=[
            params_file,
            {
                "model_name": LaunchConfiguration("model"),
                "model_variant": LaunchConfiguration("variant"),
                "obstacle_threshold": LaunchConfiguration("threshold"),
            },
        ],
        output="screen",
    )

    return LaunchDescription([
        model_arg,
        variant_arg,
        threshold_arg,
        benchmark_node,
    ])
