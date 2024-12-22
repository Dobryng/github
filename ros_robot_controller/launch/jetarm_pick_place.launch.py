from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_robot_controller',
            executable='jetarm_pick_place',
            name='jetarm_pick_place_node',
            output='screen'
        ),
    ])
