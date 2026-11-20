"""RViz only: supply the model locally; never start hardware or move_group."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = MoveItConfigsBuilder(
        "aloha_vx300s", package_name="follower_right_moveit_config"
    ).to_moveit_configs()
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(config.package_path / 'config/trajectory_preview.rviz')),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[config.robot_description,
                        config.robot_description_semantic,
                        config.robot_description_kinematics,
                        config.planning_pipelines, config.joint_limits]),
    ])
