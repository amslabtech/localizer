// Copyright 2023 amsl

#include "ndt_localizer/map_matcher.hpp"

#include <algorithm>
#include <memory>

namespace ndt_localizer
{
MapMatcher::MapMatcher(const rclcpp::NodeOptions & options)
: Node("map_matcher", options)
{
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 1);
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud/aligned", 1);
  pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "estimated_pose", 1, std::bind(&MapMatcher::pose_callback, this, std::placeholders::_1));
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "map_cloud", 1, std::bind(&MapMatcher::map_callback, this, std::placeholders::_1));
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "scan_cloud", 1, std::bind(&MapMatcher::cloud_callback, this, std::placeholders::_1));

  epsilon_ = this->declare_parameter<double>("epsilon", 1e-2);
  leaf_size_ = this->declare_parameter<double>("leaf_size", 0.5);
  step_size_ = this->declare_parameter<double>("step_size", 0.1);
  resolution_ = this->declare_parameter<double>("resolution", 1.0);
  max_iterations_ = this->declare_parameter<double>("max_iterations", 30);
  range_ = this->declare_parameter<double>("range", 100.0);

  RCLCPP_INFO(this->get_logger(), "=== map_matcher ===");
  RCLCPP_INFO_STREAM(this->get_logger(), "epsilon: " << epsilon_);
  RCLCPP_INFO_STREAM(this->get_logger(), "leaf_size: " << leaf_size_);
  RCLCPP_INFO_STREAM(this->get_logger(), "step_size: " << step_size_);
  RCLCPP_INFO_STREAM(this->get_logger(), "resolution: " << resolution_);
  RCLCPP_INFO_STREAM(this->get_logger(), "max_iterations: " << max_iterations_);
  RCLCPP_INFO_STREAM(this->get_logger(), "range: " << range_);

  map_cloud_ptr_ = CloudTypePtr(new CloudType);
  is_map_received_ = false;
  is_pose_updated_ = false;

  tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_->setUsingDedicatedThread(true);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
}

void MapMatcher::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  is_pose_updated_ = true;
  received_pose_ = *msg;
}

void MapMatcher::map_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  RCLCPP_INFO(this->get_logger(), "received map");
  pcl::fromROSMsg(*msg, *map_cloud_ptr_);
  is_map_received_ = true;
}

void MapMatcher::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  if (is_map_received_ && is_pose_updated_) {
    CloudTypePtr cloud_ptr(new CloudType);
    pcl::fromROSMsg(*msg, *cloud_ptr);
    cloud_ptr->is_dense = false;
    cloud_ptr->width = cloud_ptr->points.size();
    // RCLCPP_INFO_STREAM(this->get_logger(), "bfr: " << cloud_ptr->points.size());
    apply_voxel_grid_filter(leaf_size_, cloud_ptr);
    // RCLCPP_INFO_STREAM(this->get_logger(), "aft: " << cloud_ptr->points.size());
    const geometry_msgs::msg::TransformStamped sensor_to_base_tf =
      tf_->lookupTransform(received_pose_.child_frame_id, msg->header.frame_id, msg->header.stamp);
    const Eigen::Matrix4f sensor_to_base_mat =
      tf2::transformToEigen(sensor_to_base_tf.transform).matrix().cast<float>();
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, sensor_to_base_mat);
    const Eigen::Matrix4f transform = get_ndt_transform(cloud_ptr);
    if (transform.isZero(1e-6)) {
      return;
    }
    RCLCPP_INFO_STREAM(
      this->get_logger(), "transform:\n"
        << transform);
    geometry_msgs::msg::PoseStamped aligned_pose;
    aligned_pose.header.stamp = msg->header.stamp;
    aligned_pose.header.frame_id = map_cloud_ptr_->header.frame_id;
    aligned_pose.pose.position.x = transform(0, 3);
    aligned_pose.pose.position.y = transform(1, 3);
    aligned_pose.pose.position.z = transform(2, 3);
    Eigen::Quaternionf q(Eigen::Matrix3f(transform.block(0, 0, 3, 3)));
    q.normalize();
    aligned_pose.pose.orientation.w = q.w();
    aligned_pose.pose.orientation.x = q.x();
    aligned_pose.pose.orientation.y = q.y();
    aligned_pose.pose.orientation.z = q.z();
    RCLCPP_INFO_STREAM(
      this->get_logger(), "aligned pose:\n"
        << geometry_msgs::msg::to_yaml(aligned_pose.pose));
    pose_pub_->publish(aligned_pose);
    is_pose_updated_ = false;
  } else {
    if (!is_pose_updated_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "cloud is received but pose has not been updated");
    }
    if (!is_map_received_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "cloud is received but map has not been received");
    }
  }
}

Eigen::Matrix4f MapMatcher::get_ndt_transform(const CloudTypePtr & cloud_ptr)
{
  const Eigen::Vector3f translation_vector = {
    static_cast<float>(received_pose_.pose.pose.position.x),
    static_cast<float>(received_pose_.pose.pose.position.y),
    static_cast<float>(received_pose_.pose.pose.position.z),
  };
  const Eigen::Translation3f init_translation(translation_vector);
  const Eigen::AngleAxisf init_rotation(
    Eigen::Quaternionf(
      received_pose_.pose.pose.orientation.w, received_pose_.pose.pose.orientation.x,
      received_pose_.pose.pose.orientation.y, received_pose_.pose.pose.orientation.z)
    .normalized());
  const Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  RCLCPP_INFO_STREAM(
    this->get_logger(), "init_guess:\n"
      << init_guess);
  CloudTypePtr filtered_map_cloud_ptr(new CloudType);
  apply_passthrough_filter(range_, map_cloud_ptr_, filtered_map_cloud_ptr, translation_vector);
  CloudTypePtr filtered_scan_cloud_ptr(new CloudType);
  apply_passthrough_filter(range_, cloud_ptr, filtered_scan_cloud_ptr);
  RCLCPP_INFO_STREAM(this->get_logger(), "scan size: " << filtered_scan_cloud_ptr->points.size());
  RCLCPP_INFO_STREAM(this->get_logger(), "map size: " << filtered_map_cloud_ptr->points.size());
  // RCLCPP_INFO_STREAM(this->get_logger(), "received_pose:\n" << received_pose_.pose.pose);

  pclomp::NormalDistributionsTransform<PointType, PointType> ndt;
  ndt.setTransformationEpsilon(epsilon_);
  ndt.setStepSize(step_size_);
  ndt.setResolution(resolution_);
  ndt.setMaximumIterations(max_iterations_);
  ndt.setInputSource(filtered_scan_cloud_ptr);
  ndt.setInputTarget(filtered_map_cloud_ptr);
  ndt.setNumThreads(std::thread::hardware_concurrency());
  ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
  CloudTypePtr aligned_cloud_ptr(new CloudType);
  ndt.align(*aligned_cloud_ptr, init_guess);
  if (!ndt.hasConverged()) {
    RCLCPP_ERROR(this->get_logger(), "ndt not converged!");
    return Eigen::Matrix4f::Zero();
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "score: " << ndt.getFitnessScore());
  if (cloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
    aligned_cloud_ptr->header.frame_id = map_cloud_ptr_->header.frame_id;
    pcl::toROSMsg(*aligned_cloud_ptr, aligned_cloud_msg);
    cloud_pub_->publish(aligned_cloud_msg);
  }
  return ndt.getFinalTransformation();
}

void MapMatcher::apply_voxel_grid_filter(double leaf_size, CloudTypePtr & cloud_ptr)
{
  pcl::VoxelGrid<PointType> vg;
  vg.setInputCloud(cloud_ptr);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  CloudTypePtr output_cloud_ptr(new CloudType);
  vg.filter(*output_cloud_ptr);
  // pcl::copyPointCloud(*output_cloud_ptr, *cloud_ptr);
  *cloud_ptr = *output_cloud_ptr;
}

void MapMatcher::apply_passthrough_filter(
  double range, const CloudTypePtr & cloud_ptr, CloudTypePtr & output_cloud_ptr,
  const Eigen::Vector3f & center)
{
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud(cloud_ptr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(center(0) - range, center(0) + range);
  pass.filter(*output_cloud_ptr);
  pass.setInputCloud(output_cloud_ptr);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(center(1) - range, center(1) + range);
  pass.filter(*output_cloud_ptr);
  pass.setInputCloud(output_cloud_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(center(2) - range, center(2) + range);
  pass.filter(*output_cloud_ptr);
}

}  // namespace ndt_localizer
