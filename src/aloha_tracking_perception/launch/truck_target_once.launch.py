"""Detection and one-shot grasp pose publication. Hardware bringup stays separate."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='yolov8m.pt'),
        DeclareLaunchArgument('preview_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('target_topic', default_value='/grasp_pose'),
        Node(package='aloha_tracking_perception', executable='detector',
             parameters=[{'model': LaunchConfiguration('model')}], output='screen'),
        Node(package='aloha_tracking_perception', executable='yolo_to_robot', output='screen'),
        Node(package='aloha_tracking_perception', executable='target_once',
             parameters=[{'output_topic': LaunchConfiguration('target_topic'),
                'preview_only': ParameterValue(LaunchConfiguration('preview_only'), value_type=bool)}], output='screen',
             sigterm_timeout='320', sigkill_timeout='5'),
    ])
