#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <bounding_box_msg/msg/bounding_box2_d.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>


class PCLProcessor : public rclcpp::Node {
public:
    PCLProcessor()
        : Node("pcl_processor"),
          tf_buffer_(this->get_clock()),  // Provide clock to tf2_ros::Buffer
          tf_listener_(tf_buffer_) {     // Initialize TransformListener

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth/points", 10, std::bind(&PCLProcessor::pointcloudCallback, this, std::placeholders::_1));

        bounding_box_sub_ = this->create_subscription<bounding_box_msg::msg::BoundingBox2D>(
            "/detection_results", 10, std::bind(&PCLProcessor::bounding_box_callback, this, std::placeholders::_1));

        // point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/pcl_output", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
        transformed_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/depth/points", 10);

    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::fromROSMsg(*msg, cloud_);
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        
        geometry_msgs::msg::TransformStamped transform_stamped;

        try {
            transform_stamped = tf_buffer_.lookupTransform(
                "base_link", msg->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to base_link: %s",
                        msg->header.frame_id.c_str(), ex.what());
            return;
        }

        // Use pcl_ros for point cloud transformations
        tf2::doTransform(*msg, transformed_cloud, transform_stamped);

        // Publish the transformed point cloud
        transformed_cloud.header.frame_id = "base_link"; // Set the new frame
        transformed_cloud.header.stamp = this->get_clock()->now();
        transformed_point_cloud_pub_->publish(transformed_cloud);

    }

    void bounding_box_callback(const bounding_box_msg::msg::BoundingBox2D::SharedPtr msg) {
        if (cloud_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Point cloud not received yet!");
            return;
        }

        // RCLCPP_WARN(this->get_logger(), "center_x: %f, center_y: %f", msg->center_x, msg->center_y);
        // RCLCPP_WARN(this->get_logger(), "cloud width: %d, height: %d, points size: %ld", cloud_.width, cloud_.height, cloud_.points.size());
        // Calculate point index based on bbox center
        
        size_t index = static_cast<size_t>(msg->center_y);
        if (index >= cloud_.points.size()) {
            
            RCLCPP_WARN(this->get_logger(), "Bounding box center out of bounds!");
            return;
        }

        pcl::PointXYZ point = cloud_.points[index];
        RCLCPP_INFO(this->get_logger(), "Bounding box center: x=%f, y=%f, z=%f", point.x/10.0, point.y/10.0, point.z/10.0);

        // Broadcast a TF frame at the bounding box center
        // geometry_msgs::msg::TransformStamped transform_stamped;
        // transform_stamped.header.stamp = this->get_clock()->now();
        // transform_stamped.header.frame_id = "camera_depth_optical_frame";  // Parent frame
        // transform_stamped.child_frame_id = "bounding_box_center";  // New frame name

        // transform_stamped.transform.translation.x = -point.x/10.0;
        // transform_stamped.transform.translation.y = point.y/10.0;
        // transform_stamped.transform.translation.z = point.z/10.0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(M_PI / 2, -M_PI / 2, 0);  // Roll, Pitch, Yaw in radians
        // transform_stamped.transform.rotation.x = quaternion.x();
        // transform_stamped.transform.rotation.y = quaternion.y();
        // transform_stamped.transform.rotation.z = quaternion.z();
        // transform_stamped.transform.rotation.w = quaternion.w();

        // tf_broadcaster_->sendTransform(transform_stamped);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_depth_optical_frame";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "bounding_box_center_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = -point.x/10.0;
        marker.pose.position.y = point.y/10.0;
        marker.pose.position.z = point.z/10.0;
        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<bounding_box_msg::msg::BoundingBox2D>::SharedPtr bounding_box_sub_;
    // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_point_cloud_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLProcessor>());
    rclcpp::shutdown();
    return 0;
}
