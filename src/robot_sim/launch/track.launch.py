#!/usr/bin/env python3
"""
track.launch.py — Launch the competition track world for robot_sim.

Identical to sim.launch.py except it loads track_world.world instead of
basic_world.world, and spawns the robot on the outer-left road facing north.

Usage:
  ros2 launch robot_sim track.launch.py
  ros2 launch robot_sim track.launch.py drive_mode:=ackermann
  ros2 launch robot_sim track.launch.py use_rviz:=false

Teleop (separate terminal):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_robot_sim = get_package_share_directory('robot_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file      = os.path.join(pkg_robot_sim, 'worlds', 'track_world.world')
    urdf_xacro_file = os.path.join(pkg_robot_sim, 'urdf', 'limo_sim.urdf.xacro')
    rviz_config     = os.path.join(pkg_robot_sim, 'config', 'rviz2.rviz')

    # ── Launch arguments ────────────────────────────────────────────────────
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2')

    declare_verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Verbose Gazebo output')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    declare_drive_mode = DeclareLaunchArgument(
        'drive_mode', default_value='diff',
        choices=['diff', 'ackermann'],
        description="Robot drive mode: 'diff' (4WD skid-steer) or 'ackermann'")

    use_rviz     = LaunchConfiguration('use_rviz')
    verbose      = LaunchConfiguration('verbose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    drive_mode   = LaunchConfiguration('drive_mode')

    # ── Extend GAZEBO_MODEL_PATH ────────────────────────────────────────────
    set_gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_robot_sim, 'models')
    )

    # ── 0. Pre-launch cleanup ───────────────────────────────────────────────
    cleanup = ExecuteProcess(
        cmd=['bash', '-c',
             'killall -9 gzserver 2>/dev/null; '
             'killall -9 gzclient 2>/dev/null; '
             'killall -9 robot_state_publisher 2>/dev/null; '
             'sleep 2'],
        output='screen',
    )

    # ── 1. Gazebo Classic ───────────────────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': verbose,
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # ── 2. robot_state_publisher ────────────────────────────────────────────
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro_file, ' drive_mode:=', drive_mode]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }]
    )

    # ── 3. Spawn robot on the outer-left road, facing north (yaw = π/2) ────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'limo_sim',
            '-topic', 'robot_description',
            '-x', '-4.0',
            '-y', '-0.75',
            '-z',  '0.05',
            '-R',  '0.0',
            '-P',  '0.0',
            '-Y',  '1.5708',   # facing north (+Y)
        ],
        output='screen'
    )

    # ── 4. Pause / unpause physics after spawn (avoids ackermann NaN) ──────
    pause_physics = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=0.5,
                    actions=[ExecuteProcess(
                        cmd=['ros2', 'service', 'call',
                             '/pause_physics', 'std_srvs/srv/Empty', '{}'],
                        output='screen',
                    )],
                ),
                TimerAction(
                    period=2.0,
                    actions=[ExecuteProcess(
                        cmd=['ros2', 'service', 'call',
                             '/unpause_physics', 'std_srvs/srv/Empty', '{}'],
                        output='screen',
                    )],
                ),
            ]
        )
    )

    # ── 5. RViz2 (optional) ─────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # ── Chain: run everything after cleanup finishes ────────────────────────
    start_sim = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cleanup,
            on_exit=[
                gzserver,
                gzclient,
                robot_state_publisher,
                spawn_robot,
                pause_physics,
                rviz2,
            ]
        )
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_verbose,
        declare_use_sim_time,
        declare_drive_mode,
        set_gazebo_model_path,
        cleanup,
        start_sim,
    ])
