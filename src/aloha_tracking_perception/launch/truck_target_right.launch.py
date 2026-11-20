"""Experimental right follower mapping. Target-only preview is the default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='yolov8m.pt'),
        DeclareLaunchArgument('confidence', default_value='0.25'),
        DeclareLaunchArgument('preview_only', default_value='true', choices=['true', 'false']),
        Node(package='aloha_tracking_perception', executable='detector',
             parameters=[{'model': LaunchConfiguration('model'),
                          'confidence': ParameterValue(LaunchConfiguration('confidence'), value_type=float)}], output='screen'),
        Node(package='aloha_tracking_perception', executable='yolo_to_robot', output='screen'),
        Node(package='aloha_tracking_perception', executable='right_target',
             parameters=[{'preview_only': ParameterValue(LaunchConfiguration('preview_only'), value_type=bool)}],
             output='screen', sigterm_timeout='320', sigkill_timeout='5'),
    ])
