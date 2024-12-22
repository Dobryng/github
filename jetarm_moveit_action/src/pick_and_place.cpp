
// Reference:
// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp
// 

#include <cmath>
#include <thread>
#include <chrono>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// Logger for pick and place
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create node options and enable automatic parameter declaration
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // Create nodes for arm and gripper control
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  // Single-threaded executor for spinning nodes
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Initialize MoveIt2 interfaces for arm and gripper
  MoveGroupInterface move_group_arm(move_group_arm_node, "arm_group");
  move_group_arm.setMaxVelocityScalingFactor(1.0);  // Set maximum velocity scaling (0.0 ~ 1.0)
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set maximum acceleration scaling (0.0 ~ 1.0)

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper_group");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set maximum velocity scaling (0.0 ~ 1.0)
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set maximum acceleration scaling (0.0 ~ 1.0)

  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();
  double GRIPPER_DEFAULT = 0.0;
  double GRIPPER_OPEN = angles::from_degrees(85);
  double GRIPPER_CLOSE = angles::from_degrees(22);

  // Move arm to the "home" position defined in SRDF
  // move_group_arm.setNamedTarget("home");
  // move_group_arm.move();

  // Open the gripper
  gripper_joint_values[5] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // Set constraints for the arm movement
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "arm_constraints";

  moveit_msgs::msg::JointConstraint joint_constraint;
  joint_constraint.joint_name = "jetarm_lower_arm_fixed_part_joint";
  joint_constraint.position = 0.0;
  joint_constraint.tolerance_above = angles::from_degrees(30);
  joint_constraint.tolerance_below = angles::from_degrees(30);
  joint_constraint.weight = 1.0;
  constraints.joint_constraints.push_back(joint_constraint);

  joint_constraint.joint_name = "jetarm_upper_arm_revolute_part_twist_joint";
  joint_constraint.position = 0.0;
  joint_constraint.tolerance_above = angles::from_degrees(30);
  joint_constraint.tolerance_below = angles::from_degrees(30);
  joint_constraint.weight = 0.8;
  constraints.joint_constraints.push_back(joint_constraint);

  move_group_arm.setPathConstraints(constraints);

  // Move the arm to the target position for picking
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  target_pose.position.x = 0.2;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.13;
  q.setRPY(angles::from_degrees(-180), angles::from_degrees(-180), angles::from_degrees(0));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // Wait for the arm to stabilize
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Close the gripper to grasp the object
  gripper_joint_values[5] = GRIPPER_CLOSE;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // Wait before moving to the next position
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Move the arm to the place position
  target_pose.position.x = 0.12;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.2;
  q.setRPY(angles::from_degrees(0), angles::from_degrees(-180), angles::from_degrees(0));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // Move arm to the "place" position defined in SRDF
  move_group_arm.setNamedTarget("place");
  move_group_arm.move();

  // Open the gripper to release the object
  move_group_gripper.setNamedTarget("open");
  move_group_gripper.move();

  // Clear path constraints
  move_group_arm.clearPathConstraints();

  // Move arm back to the "home" position
  move_group_arm.setNamedTarget("init");
  move_group_arm.move();

  // Reset the gripper to the default position
  gripper_joint_values[5] = GRIPPER_DEFAULT;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // Shut down ROS 2
  rclcpp::shutdown();
  return 0;
}
