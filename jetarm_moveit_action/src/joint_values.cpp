
// Reference:
// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp
//


#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// Logger for joint values control
static const rclcpp::Logger LOGGER = rclcpp::get_logger("joint_values");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Node options for automatically declaring parameters
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // Create a node for MoveIt interface
  auto move_group_node = rclcpp::Node::make_shared("joint_values", node_options);

  // Create a single-threaded executor and add the node
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {executor.spin();}).detach();

  // Initialize MoveIt interface for the arm group
  MoveGroupInterface move_group_arm(move_group_node, "arm_group");

  // Set velocity and acceleration scaling factors (range: 0.0 ~ 1.0)
  move_group_arm.setMaxVelocityScalingFactor(0.5);
  move_group_arm.setMaxAccelerationScalingFactor(0.5);

  // Move the arm to the "vertical" position defined in the SRDF
  move_group_arm.setNamedTarget("vertical");
  move_group_arm.move();

  // Retrieve the current joint values
  auto joint_values = move_group_arm.getCurrentJointValues();

  // Define a target joint value in degrees and convert to radians
  double target_joint_value = angles::from_degrees(-45.0);

  // Apply the target value to each joint sequentially
  for (size_t i = 0; i < joint_values.size(); i++) {
    joint_values[i] = target_joint_value;
    move_group_arm.setJointValueTarget(joint_values);
    move_group_arm.move();
  }

  // Reset the arm to the "vertical" position
  move_group_arm.setNamedTarget("vertical");
  move_group_arm.move();

  rclcpp::shutdown();
  return 0;
}
