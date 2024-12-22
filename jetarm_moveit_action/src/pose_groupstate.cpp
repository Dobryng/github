
// Reference:
// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/subframes/subframes_tutorial.rst
//


#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pose_groupstate");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_arm_node, "arm_group");
  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper_group");
  move_group_arm.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.25;
  target_pose.position.y = -0.04;
  target_pose.position.z = 0.24;

  // Set planning time and scaling factors
  move_group_arm.setPlanningTime(5.0);
  move_group_arm.setMaxVelocityScalingFactor(0.5);
  move_group_arm.setMaxAccelerationScalingFactor(0.5);

  RCLCPP_INFO(LOGGER, "Setting target pose...");
  if (!move_group_arm.setPoseTarget(target_pose)) {
      RCLCPP_ERROR(LOGGER, "Failed to set pose target");
      return 1;
  }

  RCLCPP_INFO(LOGGER, "Executing motion...");
  bool success = (move_group_arm.move() == moveit::core::MoveItErrorCode::SUCCESS);

  if (!success) {
      RCLCPP_ERROR(LOGGER, "Motion planning or execution failed!");
  }


  // move_group_gripper.setNamedTarget("open");
  // move_group_gripper.move();

  // move_group_arm.setNamedTarget("init");
  // move_group_arm.move();

  // move_group_gripper.setNamedTarget("close");
  // move_group_gripper.move();

  // move_group_arm.setNamedTarget("place");
  // move_group_arm.move();

  // move_group_gripper.setNamedTarget("open");
  // move_group_gripper.move();

  // move_group_arm.setNamedTarget("home");
  // move_group_arm.move();

  // move_group_gripper.setNamedTarget("close");
  // move_group_gripper.move();

  rclcpp::shutdown();
  return 0;
}
