
// Reference:
// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp
//

#include <cmath>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("cartesian_path");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // Create nodes for arm and gripper control
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  // Add nodes to the executor for multi-threaded spinning
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Initialize MoveIt2 interfaces for the arm and gripper
  MoveGroupInterface move_group_arm(move_group_arm_node, "arm_group");
  move_group_arm.setMaxVelocityScalingFactor(0.1);  // Set velocity scaling factor (0.0 ~ 1.0)
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set acceleration scaling factor (0.0 ~ 1.0)

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper_group");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set velocity scaling factor (0.0 ~ 1.0)
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set acceleration scaling factor (0.0 ~ 1.0)

  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();

  // Move the arm to the "home" position defined in the SRDF file
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  // Open the gripper to prepare for operation
  gripper_joint_values[0] = angles::from_degrees(90);
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // Plan a circular trajectory around a center point (x=0.3, y=0.0, z=0.1) with a radius of 0.1m
  // The arm will repeat the motion 3 times
  std::vector<geometry_msgs::msg::Pose> waypoints;
  float num_of_waypoints = 30;
  int repeat = 3;
  float radius = 0.1;

  geometry_msgs::msg::Point center_position;
  center_position.x = 0.3;
  center_position.y = 0.0;
  center_position.z = 0.1;

  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  q.setRPY(0, angles::from_degrees(180), 0);  // Set orientation with RPY angles
  target_pose.orientation = tf2::toMsg(q);

  for (int r = 0; r < repeat; r++) {
    for (int i = 0; i < num_of_waypoints; i++) {
      float theta = 2.0 * M_PI * (i / static_cast<float>(num_of_waypoints));
      target_pose.position.x = center_position.x + radius * std::cos(theta);
      target_pose.position.y = center_position.y + radius * std::sin(theta);
      target_pose.position.z = center_position.z;
      waypoints.push_back(target_pose);
    }
  }

  // Compute and execute the Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  move_group_arm.execute(trajectory);

  // Return the arm to the "home" position
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  // Close the gripper
  gripper_joint_values[0] = 0;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  rclcpp::shutdown();
  return 0;
}
