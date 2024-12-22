# doc/examples/motion_planning_python_api/launch/motion_planning_python_api_tutorial.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_robot_controller',
            executable='jetarm_moveit_command',
            name='jetarm_moveit_command',
            output='screen'
        ),
    ])
