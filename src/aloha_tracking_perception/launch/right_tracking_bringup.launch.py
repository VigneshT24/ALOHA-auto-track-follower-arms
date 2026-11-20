"""Right follower and high camera only. Does not initialize the left arm."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('interbotix_xsarm_descriptions'), 'urdf', 'aloha_vx300s.urdf.xacro']),
        ' robot_name:=follower_right base_link_frame:=base_link use_gripper:=true',
        ' show_ar_tag:=false show_gripper_bar:=true show_gripper_fingers:=true',
        ' use_world_frame:=true hardware_type:=actual',
    ])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'), 'launch', 'xsarm_control.launch.py'])),
            launch_arguments={
                'robot_model': 'aloha_vx300s', 'robot_name': 'follower_right',
                'motor_configs': PathJoinSubstitution([FindPackageShare('interbotix_xsarm_control'), 'config', 'aloha_vx300s.yaml']),
                'mode_configs': PathJoinSubstitution([FindPackageShare('aloha'), 'config', 'follower_modes_right.yaml']),
                'robot_description': description, 'use_rviz': 'false',
                'load_configs': 'false', 'hardware_type': 'actual',
            }.items()),
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             namespace='cam_high', name='camera', output='screen',
             parameters=[{'initial_reset': False}, ParameterFile(
                 PathJoinSubstitution([FindPackageShare('aloha'), 'config', 'rs_cam.yaml']),
                 allow_substs=True)]),
    ])
