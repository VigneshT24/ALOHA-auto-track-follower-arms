"""Automatic scored planning; hardware execution defaults to disabled."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (MoveItConfigsBuilder('aloha_vx300s', package_name='follower_left_moveit_config')
        .planning_pipelines(pipelines=['ompl'])
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml').to_moveit_configs())
    parameters = config.to_dict()
    parameters.update({
        'planning_pipelines': {'pipeline_names': ['ompl']},
        'plan_request_params': {'planning_attempts': 1, 'planning_pipeline': 'ompl',
            'planning_time': 0.75, 'max_velocity_scaling_factor': 0.1,
            'max_acceleration_scaling_factor': 0.1},
    })
    return LaunchDescription([
        DeclareLaunchArgument('execute_motion', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('compare_orientations', default_value='false', choices=['true', 'false'],
            description='Legacy preview-only flag; all targets now automatically compare orientations.'),
        DeclareLaunchArgument('scoring_config', default_value=PathJoinSubstitution([
            FindPackageShare('aloha_follow_planner'), 'config', 'path_scoring.yaml'])),
        Node(package='follower_left_moveit_config', executable='aloha_grasp_planner.py',
            name='left_moveit_py', output='screen', parameters=[parameters,
                LaunchConfiguration('scoring_config'), {
                'execute_motion': ParameterValue(LaunchConfiguration('execute_motion'), value_type=bool),
                'compare_orientations': ParameterValue(LaunchConfiguration('compare_orientations'), value_type=bool),
            }]),
    ])
