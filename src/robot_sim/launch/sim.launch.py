#!/usr/bin/env python3
"""
sim.launch.py — Checkpoint 1 launch file for robot_sim.

Starts:
  1. Gazebo Classic (gzserver + gzclient) with basic_world.world
  2. robot_state_publisher with the xacro-processed URDF
  3. spawn_entity.py to insert the robot into Gazebo
  4. RViz2 (optional, default on)

Usage:
  ros2 launch robot_sim sim.launch.py
  ros2 launch robot_sim sim.launch.py use_rviz:=false
  ros2 launch robot_sim sim.launch.py verbose:=true

Drive the robot (separate terminal):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
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

    use_rviz     = LaunchConfiguration('use_rviz')
    verbose      = LaunchConfiguration('verbose')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ----------------------------------------------------------------
    # Extend GAZEBO_MODEL_PATH so gzserver finds model://traffic_cone.
    # The package.xml export handles this automatically after install+source,
    # but AppendEnvironmentVariable ensures it works during development too.
    # ----------------------------------------------------------------
    set_gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_robot_sim, 'models')
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
    #    Reads the URDF via xacro at launch time.
    #    Publishes TF for fixed joints; listens to /joint_states for wheels.
    # ----------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', urdf_xacro_file]), value_type=str),
        }]
    )

    # ----------------------------------------------------------------
    # 3. Spawn robot into Gazebo
    #    spawn_entity.py reads from /robot_description and calls Gazebo's
    #    /spawn_entity service (retries for up to 30 s while Gazebo starts).
    #    z=0.01 gives a small drop on physics start so wheels settle on ground.
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
            '-z', '0.01',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen'
    )

    # ----------------------------------------------------------------
    # 4. RViz2 (optional)
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

    return LaunchDescription([
        declare_use_rviz,
        declare_verbose,
        declare_use_sim_time,
        set_gazebo_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        rviz2,
    ])
