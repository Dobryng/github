import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():


    robot_description_config = load_file(
        'jetarm_moveit_config', 'config/jetarm_6dof.urdf.xacro')
    robot_description = {
        'robot_description_semantic': robot_description_config}
    
    robot_description_semantic_config = load_file(
        'jetarm_moveit_config', 'config/jetarm_6dof.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('jetarm_moveit_config', 'config/kinematics.yaml')


    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description=('Set true when using the gazebo simulator.')
    )

    
    picking_node = Node(
        name="tomato_fsm_control",
        package='jetarm_moveit_action',
        executable='tomato_fsm_control',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        picking_node,
    ])
