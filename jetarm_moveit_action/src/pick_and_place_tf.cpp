
// Reference:
// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp
// https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
//


#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// Class to implement pick-and-place functionality using MoveIt2 and TF
class PickAndPlaceTf : public rclcpp::Node
{
public:
  PickAndPlaceTf(
    rclcpp::Node::SharedPtr move_group_arm_node,
    rclcpp::Node::SharedPtr move_group_gripper_node)
  : Node("pick_and_place_tf_node")
  {
    using namespace std::placeholders;

    // Initialize MoveIt2 interfaces for arm and gripper
    move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm_group");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper_group");
    move_group_gripper_->setMaxVelocityScalingFactor(0.5);
    move_group_gripper_->setMaxAccelerationScalingFactor(0.5);

    // Move the arm to the "home" position
    // move_group_arm_->setNamedTarget("home");
    // move_group_arm_->move();

    // Set joint constraints for the arm
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

    move_group_arm_->clearPathConstraints();
    move_group_arm_->setPathConstraints(constraints);

    // Move the arm to the initial pose
    init_pose();

    // Initialize TF2 for tracking target positions
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a timer to periodically check for new target positions
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PickAndPlaceTf::on_timer, this));
  }

private:
  void on_timer()
  {
    // Retrieve the transform from the base_link to the target_0 frame
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_->lookupTransform(
        "base_link", "tomatoes_0", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Failed to find transform to target: %s", ex.what());
      return;
    }

    rclcpp::Time now = this->get_clock()->now();
    const std::chrono::nanoseconds FILTERING_TIME = 2s;
    const std::chrono::nanoseconds STOP_TIME_THRESHOLD = 1s;
    const double DISTANCE_THRESHOLD = 0.05;

    tf2::Stamped<tf2::Transform> tf;
    tf2::convert(tf_msg, tf);

    const auto TF_ELAPSED_TIME = now.nanoseconds() - tf.stamp_.time_since_epoch().count();
    const auto TF_STOP_TIME = now.nanoseconds() - tf_past_.stamp_.time_since_epoch().count();
    const double TARGET_Z_MIN_LIMIT = 0.04;

    // Check if the target has been stable for a sufficient time
    if (TF_ELAPSED_TIME < FILTERING_TIME.count()) {
      double tf_diff = (tf_past_.getOrigin() - tf.getOrigin()).length();
      if (tf_diff < DISTANCE_THRESHOLD) {
        if (TF_STOP_TIME > STOP_TIME_THRESHOLD.count()) {
          if (tf.getOrigin().z() < TARGET_Z_MIN_LIMIT) {
            tf.getOrigin().setZ(TARGET_Z_MIN_LIMIT);
          }
          move_group_gripper_->setNamedTarget("open");
          move_group_gripper_->move();
          picking(tf.getOrigin());
        }
      } else {
        tf_past_ = tf;
      }
    }
  }

  void init_pose()
  {
    // Move the arm to the "init" position
    move_group_arm_->setNamedTarget("init");
    move_group_arm_->move();
  }

  void picking(tf2::Vector3 target_position)
  {
    // const double GRIPPER_DEFAULT = 0.0;
    // const double GRIPPER_OPEN = angles::from_degrees(85.0);
    // const double GRIPPER_CLOSE = angles::from_degrees(60.0);

    // Open the gripper and move the arm to the target position
    // control_gripper(GRIPPER_DEFAULT);

    control_arm(target_position.x(), target_position.y(), target_position.z() + 0.1, 0, -180, 0);

    // Close the gripper to grasp the object
    move_group_gripper_->setNamedTarget("close");
    move_group_gripper_->move();
    // control_gripper(GRIPPER_CLOSE);
    init_pose();
    // Move the arm to the "place" position and release the object
    move_group_arm_->setNamedTarget("place");
    move_group_arm_->move();

    // control_gripper(GRIPPER_OPEN);
    move_group_gripper_->setNamedTarget("open");
    move_group_gripper_->move();


    // Reset the arm to the initial position
    init_pose();
  }

  void control_gripper(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[5] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

  void control_arm(
    const double x, const double y, const double z,
    const double roll, const double pitch, const double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    q.setRPY(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));
    target_pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(this->get_logger(), "Moving to pose: x=%f, y=%f, z=%f", x, y, z);
    move_group_arm_->clearPoseTargets();
    move_group_arm_->setPoseTarget(target_pose);
    move_group_arm_->move();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed for target pose!");
      if (!move_group_arm_->move()) {
        RCLCPP_ERROR(this->get_logger(), "Motion execution failed or timed out.");
        return;
      } 
    }
  }

  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  tf2::Stamped<tf2::Transform> tf_past_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto pick_and_place_tf_node = std::make_shared<PickAndPlaceTf>(move_group_arm_node, move_group_gripper_node);
  exec.add_node(pick_and_place_tf_node);
  exec.add_node(move_group_arm_node);
  exec.add_node(move_group_gripper_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

