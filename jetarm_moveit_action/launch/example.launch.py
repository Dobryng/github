import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration
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

    # Create MoveIt configurations for jetarm_6dof robot
    moveit_config = MoveItConfigsBuilder("jetarm_6dof", package_name="jetarm_moveit_config").to_moveit_configs()

    # Static transform publisher node for camera frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_transform_publisher',
        arguments=[
            '-0.048', '0', '0',                # xyz
            '0', '-1.5708', '0',               # rpy (roll, pitch, yaw)
            'link5', 'camera_link'             # frame_id, child_frame_id
        ]
    )

    # Load SRDF and kinematics YAML for jetarm_6dof
    robot_description_semantic_config = load_file(
        'jetarm_moveit_config', 'config/jetarm_6dof.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml('jetarm_moveit_config', 'config/kinematics.yaml')

    # Declare example launch argument
    declare_example_name = DeclareLaunchArgument(
        'example', default_value='pose_groupstate',
        description=('Set an example executable name: '
                     '[gripper_control, pose_groupstate, joint_values,'
                     'pick_and_place, cartesian_path]')
    )

    # Declare use_sim_time argument for simulation
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description=('Set true when using the gazebo simulator.')
    )

    # Example node for executing different example scripts
    example_node = Node(
        name=[LaunchConfiguration('example'), '_node'],
        package='jetarm_moveit_config',
        executable=LaunchConfiguration('example'),
        output='screen',
        parameters=[{'robot_description': moveit_config.robot_description},
                    robot_description_semantic,
                    kinematics_yaml]
    )


    # MoveIt DEMO launch
    demo_launch = generate_demo_launch(moveit_config)

    return LaunchDescription([
        #declare_use_sim_time,
        #SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        declare_example_name,
        #example_node,
        #demo_launch,
        #static_tf
    ])
