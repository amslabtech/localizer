// Copyright 2023 amsl

#ifndef NDT_LOCALIZER__MAP_MATCHER_HPP_
#define NDT_LOCALIZER__MAP_MATCHER_HPP_

#include <memory>
#include <thread>

#include "Eigen/Dense"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"
#include "pcl/registration/ndt.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pclomp/ndt_omp.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ndt_localizer
{
class MapMatcher : public rclcpp::Node
{
public:
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> CloudType;
  typedef pcl::PointCloud<PointType>::Ptr CloudTypePtr;

  explicit MapMatcher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void map_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  Eigen::Matrix4f get_ndt_transform(const CloudTypePtr & cloud_ptr);
  void apply_voxel_grid_filter(double leaf_size, CloudTypePtr & cloud_ptr);
  void apply_passthrough_filter(
    double range, const CloudTypePtr & cloud_ptr, CloudTypePtr & output_cloud_ptr,
    const Eigen::Vector3f & center = Eigen::Vector3f::Zero());

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  CloudTypePtr map_cloud_ptr_;
  bool is_map_received_;
  nav_msgs::msg::Odometry received_pose_;
  bool is_pose_updated_;

  // ndt params
  double epsilon_;
  double leaf_size_;
  double step_size_;
  double resolution_;
  int max_iterations_;

  double range_;
};

}  // namespace ndt_localizer

#endif  // NDT_LOCALIZER__MAP_MATCHER_HPP_
