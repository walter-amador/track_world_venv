#!/usr/bin/env python3
"""
sim.launch.py — Checkpoint 2 launch file for robot_sim.

Starts:
  1. Pre-launch cleanup (kills any lingering gzserver/gzclient/robot_state_publisher)
  2. Gazebo Classic (gzserver + gzclient) with basic_world.world
  3. robot_state_publisher with the xacro-processed URDF
  4. spawn_entity.py to insert the robot into Gazebo
  5. RViz2 (optional, default on)

Usage:
  ros2 launch robot_sim sim.launch.py
  ros2 launch robot_sim sim.launch.py drive_mode:=ackermann
  ros2 launch robot_sim sim.launch.py use_rviz:=false
  ros2 launch robot_sim sim.launch.py verbose:=true

Drive modes:
  drive_mode:=diff       (default) 4-wheel skid-steer.
                         /cmd_vel → geometry_msgs/Twist (linear.x, angular.z)
  drive_mode:=ackermann  Ackermann front-steering, rear-driven (car-style).
                         /cmd_vel → geometry_msgs/Twist (linear.x, angular.z;
                         the plugin converts angular.z to a steering angle).

Drive the robot (separate terminal):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Camera (always on):
  /camera/image_raw    sensor_msgs/Image  @ 30 FPS  (640×480 RGB)
  /camera/camera_info  sensor_msgs/CameraInfo
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

    world_file      = os.path.join(pkg_robot_sim, 'worlds', 'basic_world.world')
    urdf_xacro_file = os.path.join(pkg_robot_sim, 'urdf', 'limo_sim.urdf.xacro')
    rviz_config     = os.path.join(pkg_robot_sim, 'config', 'rviz2.rviz')

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
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

    # ----------------------------------------------------------------
    # Extend GAZEBO_MODEL_PATH so gzserver finds model://traffic_cone.
    # ----------------------------------------------------------------
    set_gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_robot_sim, 'models')
    )

    # ----------------------------------------------------------------
    # 0. Pre-launch cleanup — kill any lingering simulation processes
    #    so relaunching with a different drive_mode always starts fresh.
    #    A stale gzserver from an ackermann run would otherwise hold the
    #    lock, cause the new gzserver to exit 255, and leave the old
    #    ackermann robot in the world regardless of drive_mode:=diff.
    # ----------------------------------------------------------------
    # killall matches exact process name (not cmdline substring), so it
    # cannot accidentally kill this bash process itself the way pkill -f would.
    cleanup = ExecuteProcess(
        cmd=['bash', '-c',
             'killall -9 gzserver 2>/dev/null; '
             'killall -9 gzclient 2>/dev/null; '
             'killall -9 robot_state_publisher 2>/dev/null; '
             'sleep 2'],
        output='screen',
    )

    # ----------------------------------------------------------------
    # 1. Gazebo Classic server and client
    # ----------------------------------------------------------------
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

    # ----------------------------------------------------------------
    # 2. robot_state_publisher
    #    The xacro `drive_mode` arg selects between diff-drive and
    #    Ackermann URDF branches at parse time.
    # ----------------------------------------------------------------
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

    # ----------------------------------------------------------------
    # 3. Spawn robot into Gazebo
    # ----------------------------------------------------------------
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'limo_sim',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen'
    )

    # ----------------------------------------------------------------
    # 4. After robot spawns: pause physics, wait 1s, unpause.
    #    Prevents the ackermann drive plugin from reading uninitialized
    #    joint velocities (NaN) on the very first physics step.
    # ----------------------------------------------------------------
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

    # ----------------------------------------------------------------
    # 5. RViz2 (optional)
    # ----------------------------------------------------------------
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # ----------------------------------------------------------------
    # Start Gazebo and ROS nodes only after cleanup finishes.
    # ----------------------------------------------------------------
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
