
// Reference:
// https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html
// https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html
// https://pcl.readthedocs.io/projects/tutorials/en/master/voxel_grid.html
// https://pcl.readthedocs.io/projects/tutorials/en/master/planar_segmentation.html
// https://pcl.readthedocs.io/projects/tutorials/en/master/extract_indices.html
// https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html
//


#include <cmath>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class PointCloudSubscriber : public rclcpp::Node
{
public:
  PointCloudSubscriber()
  : Node("point_cloud_detection")
  {
    // Subscribe to the point cloud data topic
    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/depth/points", 10,
      std::bind(&PointCloudSubscriber::point_cloud_callback, this, std::placeholders::_1));

    // Publisher for classified points
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/classified_points", 10);

    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize transform listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Transform the incoming point cloud to the base_link frame
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Transform lookup failed: %s", ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_transformed;
    pcl_ros::transformPointCloud("base_link", tf_msg, *msg, cloud_transformed);

    // Convert ROS point cloud message to PCL format
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(cloud_transformed, *cloud);

    // Perform preprocessing on the point cloud (filtering, downsampling)
    if (!preprocessing(cloud)) {
      return;
    }

    // Perform clustering on the filtered point cloud
    auto cluster_indices = clustering(cloud);

    // Publish the clustered points and their positions
    auto cloud_output = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    broadcast_cluster_position(cloud, cloud_output, cluster_indices, cloud_transformed.header);

    // Convert the processed point cloud back to ROS message and publish it
    sensor_msgs::msg::PointCloud2 sensor_msg;
    pcl::toROSMsg(*cloud_output, sensor_msg);
    publisher_->publish(sensor_msg);
  }

  bool preprocessing(std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & cloud)
  {
    // Filter points based on the X-axis (range: 0.05 to 0.5 meters)
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.05, 0.5);
    pass.filter(*cloud);

    // Filter points based on the Z-axis (range: 0.03 to 0.5 meters)
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.03, 0.5);
    pass.filter(*cloud);

    // Downsample the point cloud using a voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud);

    // Check if the filtered point cloud is empty
    if (cloud->size() <= 0) {
      RCLCPP_INFO(this->get_logger(), "Filtered point cloud is empty.");
      return false;
    }
    return true;
  }

  std::vector<pcl::PointIndices> clustering(std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & cloud)
  {
    // Perform Euclidean clustering on the point cloud
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);  // Cluster tolerance in meters
    ec.setMinClusterSize(10);     // Minimum number of points per cluster
    ec.setMaxClusterSize(250);    // Maximum number of points per cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    return cluster_indices;
  }

  void broadcast_cluster_position(
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & cloud_input,
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & cloud_output,
    std::vector<pcl::PointIndices> & cluster_indices,
    std_msgs::msg::Header & tf_header)
  {
    int cluster_i = 0;
    const int CLUSTER_MAX = 10;
    const int CLUSTER_COLOR[CLUSTER_MAX][3] = {
      {230, 0, 18}, {243, 152, 18}, {255, 251, 0},
      {143, 195, 31}, {0, 153, 68}, {0, 158, 150},
      {0, 160, 233}, {0, 104, 183}, {29, 32, 136},
      {146, 7, 131}
    };

    for (const auto & point_indices : cluster_indices) {
      auto cloud_cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      for (const auto & point_i : point_indices.indices) {
        cloud_input->points[point_i].r = CLUSTER_COLOR[cluster_i][0];
        cloud_input->points[point_i].g = CLUSTER_COLOR[cluster_i][1];
        cloud_input->points[point_i].b = CLUSTER_COLOR[cluster_i][2];
        cloud_cluster->points.push_back(cloud_input->points[point_i]);
      }

      // Add the cluster to the output cloud
      *cloud_output += *cloud_cluster;

      // Compute the cluster centroid and broadcast its position
      Eigen::Vector4f cluster_centroid;
      pcl::compute3DCentroid(*cloud_cluster, cluster_centroid);
      geometry_msgs::msg::TransformStamped t;
      t.header = tf_header;
      t.child_frame_id = "target_" + std::to_string(cluster_i);
      t.transform.translation.x = cluster_centroid.x();
      t.transform.translation.y = cluster_centroid.y();
      t.transform.translation.z = cluster_centroid.z();
      t.transform.rotation.x = 0.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = 0.0;
      t.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(t);

      RCLCPP_INFO(this->get_logger(), "X: %.2f, Y: %.2f, Z: %.2f", cluster_centroid.x(), cluster_centroid.y(), cluster_centroid.z());

      // Limit the number of clusters to CLUSTER_MAX
      cluster_i++;
      if (cluster_i >= CLUSTER_MAX) {
        break;
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriber>());
  rclcpp::shutdown();
  return 0;
}
