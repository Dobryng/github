
// Reference:
// https://github.com/moveit/moveit2/blob/main/moveit_ros/move_group/src/move_group.cpp
//

#include <cmath>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// Logger for gripper control
static const rclcpp::Logger LOGGER = rclcpp::get_logger("gripper_control");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Node options for automatically declaring parameters
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // Create nodes for arm and gripper control
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  // Create a single-threaded executor and add the nodes
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);

  // Run the executor in a separate thread
  std::thread([&executor]() { executor.spin(); }).detach();

  // Initialize the MoveIt2 interface for the arm
  MoveGroupInterface move_group_arm(move_group_arm_node, "arm_group");
  move_group_arm.setMaxVelocityScalingFactor(1.0);  // Set max velocity scaling (0.0 ~ 1.0)
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set max acceleration scaling (0.0 ~ 1.0)

  // Initialize the MoveIt2 interface for the gripper
  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper_group");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set max velocity scaling (0.0 ~ 1.0)
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set max acceleration scaling (0.0 ~ 1.0)

  // Get the current joint values of the gripper
  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();

  // Move the arm to the "home" position defined in the SRDF file
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  // Control the gripper
  // gripper_joint_values[5] corresponds to the gripper joint angle
  gripper_joint_values[5] = angles::from_degrees(180);  // Open gripper to 90 degrees
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // gripper_joint_values[5] = angles::from_degrees(270);  // Close gripper to 0 degrees
  // move_group_gripper.setJointValueTarget(gripper_joint_values);
  // move_group_gripper.move();

  // gripper_joint_values[5] = angles::from_degrees(360);  // Open gripper to 90 degrees again
  // move_group_gripper.setJointValueTarget(gripper_joint_values);
  // move_group_gripper.move();

  // gripper_joint_values[5] = angles::from_degrees(0);  // Close gripper to 0 degrees again
  // move_group_gripper.setJointValueTarget(gripper_joint_values);
  // move_group_gripper.move();

  // Shut down the ROS 2 node
  rclcpp::shutdown();
  return 0;
}

