from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    # Build the MoveIt configuration
    moveit_config = MoveItConfigsBuilder("jetarm_6dof", package_name="jetarm_moveit_config").to_moveit_configs()

    # Generate the MoveIt demo launch description
    demo_launch = generate_demo_launch(moveit_config)

    # Add a static transform publisher
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['-0.047', '0', '0', '0', '-1.57', '0', 'link5', 'camera_link']
    )

    # Combine and return the launch description
    return LaunchDescription(demo_launch.entities + [static_tf_node])