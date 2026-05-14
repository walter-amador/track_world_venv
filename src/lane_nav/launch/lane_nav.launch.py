"""
lane_nav.launch.py
==================
Launches the three lane-navigation nodes:
  lane_detection_node    —  CV pipeline  (IPM + sliding windows + polynomial fit)
  behavior_manager_node  —  state machine (FOLLOW_LANE / RECOVER / STOP / …)
  lane_controller_node   —  PID → /cmd_vel

Usage:
  # 1. Launch the simulation first (separate terminal):
  #    ros2 launch robot_sim track.launch.py drive_mode:=ackermann
  #
  # 2. Then launch this (new terminal, same environment source):
  #    ros2 launch lane_nav lane_nav.launch.py
  #
  #    Optional: disable debug image stream
  #    ros2 launch lane_nav lane_nav.launch.py debug:=false

To view the debug bird's-eye overlay:
  rqt_image_view           (select /lane/debug_image)

To send commands to the behavior manager:
  ros2 topic pub /behavior/command std_msgs/String "data: STOP"
  ros2 topic pub /behavior/command std_msgs/String "data: START"

To monitor the lateral error in real time:
  ros2 topic echo /lane/lateral_error
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg    = get_package_share_directory('lane_nav')
    params = os.path.join(pkg, 'config', 'params.yaml')

    debug_arg = DeclareLaunchArgument(
        'debug', default_value='true',
        description='Publish /lane/debug_image (disable to reduce CPU load)')

    lane_detection = Node(
        package='lane_nav',
        executable='lane_detection_node',
        name='lane_detection_node',
        parameters=[params, {'publish_debug': LaunchConfiguration('debug')}],
        output='screen',
    )

    behavior_manager = Node(
        package='lane_nav',
        executable='behavior_manager_node',
        name='behavior_manager_node',
        parameters=[params],
        output='screen',
    )

    lane_controller = Node(
        package='lane_nav',
        executable='lane_controller_node',
        name='lane_controller_node',
        parameters=[params],
        output='screen',
    )

    return LaunchDescription([
        debug_arg,
        lane_detection,
        behavior_manager,
        lane_controller,
    ])
