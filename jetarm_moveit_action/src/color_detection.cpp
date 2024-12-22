
// Reference:
// https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
// 

#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
using std::placeholders::_1;

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("color_detection")
  {
    // Subscribe to the RGB image topic
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_rect", 10, std::bind(&ImageSubscriber::image_callback, this, _1));

    // Subscribe to the depth image topic
    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/depth/image_raw", 10, std::bind(&ImageSubscriber::depth_callback, this, _1));

    // Subscribe to the camera info topic
    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info_rect", 10, std::bind(&ImageSubscriber::camera_info_callback, this, _1));

    // Publisher for the thresholded image
    image_thresholded_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_thresholded", 10);

    // Transform broadcaster to publish the detected object's position
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_thresholded_publisher_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  sensor_msgs::msg::Image::SharedPtr depth_image_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Ensure camera info and depth image are available before processing
    if (camera_info_ && depth_image_) {
      // Define HSV color range for object detection
      const int LOW_H = 0, HIGH_H = 10;
      const int LOW_S = 150, HIGH_S = 255;
      const int LOW_V = 100, HIGH_V = 255;

      // Convert the incoming image to OpenCV format
      auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);

      // Convert RGB to HSV
      cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_RGB2HSV);

      // Apply thresholding to isolate the target color
      cv::Mat img_thresholded;
      cv::inRange(
        cv_img->image,
        cv::Scalar(LOW_H, LOW_S, LOW_V),
        cv::Scalar(HIGH_H, HIGH_S, HIGH_V),
        img_thresholded);

      // Perform morphological opening to remove small noise
      cv::morphologyEx(
        img_thresholded,
        img_thresholded,
        cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

      // Perform morphological closing to fill small holes
      cv::morphologyEx(
        img_thresholded,
        img_thresholded,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

      // Calculate moments to find the object's center
      cv::Moments moment = moments(img_thresholded);
      double d_m01 = moment.m01;
      double d_m10 = moment.m10;
      double d_area = moment.m00;

      // Ignore objects with very small area
      if (d_area > 10000) {
        // Create a camera model for coordinate transformation
        image_geometry::PinholeCameraModel camera_model;
        camera_model.fromCameraInfo(camera_info_);

        // Calculate the pixel coordinates of the object's center
        const double pixel_x = d_m10 / d_area;
        const double pixel_y = d_m01 / d_area;
        const cv::Point2d point(pixel_x, pixel_y);

        // Rectify the pixel coordinates
        const cv::Point2d rect_point = camera_model.rectifyPoint(point);

        // Project the 2D point to a 3D ray
        const cv::Point3d ray = camera_model.projectPixelTo3dRay(rect_point);

        // Use depth image to compute the actual 3D position of the object
        const double DEPTH_OFFSET = 0.015;
        const auto cv_depth = cv_bridge::toCvShare(depth_image_, depth_image_->encoding);
        const auto front_distance = cv_depth->image.at<ushort>(point) / 1000.0;
        const auto center_distance = front_distance + DEPTH_OFFSET;

        // Check if the detected object is within a valid depth range
        const double DEPTH_MAX = 0.5;
        const double DEPTH_MIN = 0.25;
        if (center_distance < DEPTH_MIN || center_distance > DEPTH_MAX) {
          RCLCPP_INFO_STREAM(this->get_logger(), "Detected object at " << point << " is out of range.");
          return;
        }

        // Compute the object's 3D position
        cv::Point3d object_position(
          ray.x * center_distance,
          ray.y * center_distance,
          ray.z * center_distance);

        // Publish the object's position as a transform
        geometry_msgs::msg::TransformStamped t;
        t.header = msg->header;
        t.child_frame_id = "target_0";
        t.transform.translation.x = object_position.x;
        t.transform.translation.y = object_position.y;
        t.transform.translation.z = object_position.z;
        tf_broadcaster_->sendTransform(t);

        // Publish the thresholded image
        sensor_msgs::msg::Image::SharedPtr img_thresholded_msg =
          cv_bridge::CvImage(msg->header, "mono8", img_thresholded).toImageMsg();
        image_thresholded_publisher_->publish(*img_thresholded_msg);
      }
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info_ = msg;
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    depth_image_ = msg;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
