#include <cmath>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "bounding_box_msg/msg/bounding_box2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/common/centroid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "pcl_ros/transforms.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


class PointCloudSubscriber : public rclcpp::Node
{
public:
  PointCloudSubscriber()
      : Node("point_cloud_detection"), intrinsics_received_(false) 
  {
    // Subscribe to the point cloud data topic
    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/depth/points", 10, std::bind(&PointCloudSubscriber::point_cloud_callback, this, std::placeholders::_1));

    // Subscribe to YOLO bounding box centers
    bounding_box_subscription_ = this->create_subscription<bounding_box_msg::msg::BoundingBox2D>(
        "/center_point", 10, std::bind(&PointCloudSubscriber::bounding_box_callback, this, std::placeholders::_1));

    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/depth/camera_info", 10, std::bind(&PointCloudSubscriber::camera_info_callback, this, std::placeholders::_1));

    // Publisher for 3D detected points
    detection_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/detected_3d_points", 10);

    // Initialize TF2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Subscription<bounding_box_msg::msg::BoundingBox2D>::SharedPtr bounding_box_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr detection_publisher_;

  // TF2 Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; // Point cloud storage

  float fx_, fy_, cx_, cy_;
  bool intrinsics_received_;

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fx_ = msg->k[0]; // Focal length in X
    fy_ = msg->k[4]; // Focal length in Y
    cx_ = msg->k[2]; // Principal point X
    cy_ = msg->k[5]; // Principal point Y

    intrinsics_received_ = true;
    // RCLCPP_INFO(this->get_logger(), "Camera intrinsics received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
  }

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud_);
    // RCLCPP_INFO(this->get_logger(), "Point cloud updated with %ld points.", cloud_->size());
  }

  void bounding_box_callback(const bounding_box_msg::msg::BoundingBox2D::SharedPtr msg)
  {
    if (!intrinsics_received_)
    {
      RCLCPP_WARN(this->get_logger(), "Camera intrinsics not yet received.");
      return;
    }

    if (!cloud_)
    {
      RCLCPP_WARN(this->get_logger(), "Point cloud is not yet available.");
      return;
    }

    // Transform 2D bounding box center to 3D using the point cloud
    geometry_msgs::msg::Point detected_point;
    bool found = project_2d_to_3d(msg->center_x, msg->center_y, detected_point);

    if (found)
    {
      RCLCPP_INFO(this->get_logger(), "Detected object at 3D: [%.2f, %.2f, %.2f]", detected_point.z - 0.17, -detected_point.y + 0.05, -detected_point.x + 0.09);

      // Publish the 3D point
      detection_publisher_->publish(detected_point);

      // Create and broadcast a transform frame
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = this->get_clock()->now();
      transform_stamped.header.frame_id = "base_link";  // Parent frame
      transform_stamped.child_frame_id = "tomatoes_0"; // Unique frame ID
      transform_stamped.transform.translation.x = detected_point.z - 0.17;
      transform_stamped.transform.translation.y = -detected_point.y + 0.05;
      transform_stamped.transform.translation.z = -detected_point.x + 0.09;

      // Identity rotation (no rotation applied)
      transform_stamped.transform.rotation.x = 0.0;
      transform_stamped.transform.rotation.y = 0.0;
      transform_stamped.transform.rotation.z = 0.0;
      transform_stamped.transform.rotation.w = 1.0;

      tf_broadcaster_->sendTransform(transform_stamped);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No valid 3D point found for bounding box center [%.2f, %.2f].", msg->center_x, msg->center_y);
    }
  }

  bool project_2d_to_3d(float center_x, float center_y, geometry_msgs::msg::Point &output_point)
  {
    float closest_distance = std::numeric_limits<float>::max();
    bool point_found = false;

    for (const auto &point : cloud_->points)
    {
      if (point.z <= 0) // Ignore invalid points
        continue;

      // Project 3D point to 2D
      int projected_x = static_cast<int>((fx_ * point.x / point.z) + cx_);
      int projected_y = static_cast<int>((fy_ * point.y / point.z) + cy_);

      // Match with bounding box center
      if (std::abs(projected_x - center_x) < 5 && std::abs(projected_y - center_y) < 5)
      {
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance < closest_distance)
        {
          closest_distance = distance;
          output_point.x = point.x;
          output_point.y = point.y;
          output_point.z = point.z;
          point_found = true;
        }
      }
    }

    return point_found;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriber>());
  rclcpp::shutdown();
  return 0;
}
