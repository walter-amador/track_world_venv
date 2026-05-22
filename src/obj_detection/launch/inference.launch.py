from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("conf", default_value="0.5", description="Confidence threshold"),
        DeclareLaunchArgument("iou",  default_value="0.45", description="IoU threshold"),

        Node(
            package="obj_detection",
            executable="inference_yolo26n",
            name="yolo26n_detection",
            output="screen",
            arguments=[
                "--conf", LaunchConfiguration("conf"),
                "--iou",  LaunchConfiguration("iou"),
            ],
        ),
    ])
