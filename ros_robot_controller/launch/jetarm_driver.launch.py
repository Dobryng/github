from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_robot_controller',
            executable='jetarm_driver',
            name='jetarm_driver',
            output='screen'
        ),
    ])