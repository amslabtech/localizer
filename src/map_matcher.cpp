// Copyright 2023 amsl

#include <algorithm>
#include <memory>

#include "ndt_localizer/map_matcher.h"

namespace ndt_localizer
{
MapMatcher::MapMatcher(void)
  : local_nh_("~")
{
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 1);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud/aligned", 1);
  downsampled_map_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("map_cloud/downsampled", 1, true);
  pose_sub_ =
      nh_.subscribe("estimated_pose", 1, &MapMatcher::pose_callback, this,
                    ros::TransportHints().reliable().tcpNoDelay(true));
  map_sub_ = nh_.subscribe("map_cloud", 1, &MapMatcher::map_callback, this,
                           ros::TransportHints().reliable().tcpNoDelay(true));
  cloud_sub_ = nh_.subscribe("scan_cloud", 1, &MapMatcher::cloud_callback, this,
                             ros::TransportHints().reliable().tcpNoDelay(true));

  local_nh_.param<double>("epsilon", epsilon_, 1e-2);
  local_nh_.param<double>("leaf_size", leaf_size_, 0.5);
  local_nh_.param<double>("step_size", step_size_, 0.1);
  local_nh_.param<double>("resolution", resolution_, 1.0);
  local_nh_.param<int>("max_iterations", max_iterations_, 30);
  local_nh_.param<double>("range", range_, 100);

  ROS_INFO("=== map_matcher ===");
  ROS_INFO_STREAM("epsilon: " << epsilon_);
  ROS_INFO_STREAM("leaf_size: " << leaf_size_);
  ROS_INFO_STREAM("step_size: " << step_size_);
  ROS_INFO_STREAM("resolution: " << resolution_);
  ROS_INFO_STREAM("max_iterations: " << max_iterations_);
  ROS_INFO_STREAM("range: " << range_);

  map_cloud_ptr_ = CloudTypePtr(new CloudType);
  is_map_received_ = false;
  is_pose_updated_ = false;

  tf_ = std::make_shared<tf2_ros::Buffer>();
  tf_->setUsingDedicatedThread(true);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
}

void MapMatcher::pose_callback(const nav_msgs::OdometryConstPtr& msg)
{
  is_pose_updated_ = true;
  received_pose_ = *msg;
}

void MapMatcher::map_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("received map");
  pcl::fromROSMsg(*msg, *map_cloud_ptr_);
  ROS_INFO_STREAM("size: " << map_cloud_ptr_->points.size());
  apply_voxel_grid_filter(leaf_size_, map_cloud_ptr_);
  ROS_INFO_STREAM("downsampled size: " << map_cloud_ptr_->points.size());
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*map_cloud_ptr_, output);
  downsampled_map_pub_.publish(output);
  is_map_received_ = true;
}

void MapMatcher::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (is_map_received_ && is_pose_updated_)
  {
    CloudTypePtr cloud_ptr(new CloudType);
    pcl::fromROSMsg(*msg, *cloud_ptr);
    cloud_ptr->is_dense = false;
    cloud_ptr->width = cloud_ptr->points.size();
    // ROS_INFO_STREAM("bfr: " << cloud_ptr->points.size());
    apply_voxel_grid_filter(leaf_size_, cloud_ptr);
    // ROS_INFO_STREAM("aft: " << cloud_ptr->points.size());
    const geometry_msgs::TransformStamped sensor_to_base_tf =
        tf_->lookupTransform(received_pose_.child_frame_id,
                             msg->header.frame_id, ros::Time(0));
    const Eigen::Matrix4f sensor_to_base_mat =
        tf2::transformToEigen(sensor_to_base_tf.transform)
            .matrix()
            .cast<float>();
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, sensor_to_base_mat);
    const Eigen::Matrix4f transform = get_ndt_transform(cloud_ptr);
    if (transform.isZero(1e-6))
    {
      return;
    }
    ROS_INFO_STREAM("transform:\n"
                    << transform);
    geometry_msgs::PoseStamped aligned_pose;
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
    ROS_INFO_STREAM("aligned pose:\n"
                    << aligned_pose.pose);
    pose_pub_.publish(aligned_pose);
    is_pose_updated_ = false;
  }
  else
  {
    if (!is_pose_updated_)
    {
      ROS_WARN_THROTTLE(1.0, "cloud is received but pose has not been updated");
    }
    if (!is_map_received_)
    {
      ROS_WARN_THROTTLE(1.0, "cloud is received but map has not been received");
    }
  }
}

Eigen::Matrix4f MapMatcher::get_ndt_transform(const CloudTypePtr& cloud_ptr)
{
  const Eigen::Vector3f translation_vector =
      {
          static_cast<float>(received_pose_.pose.pose.position.x),
          static_cast<float>(received_pose_.pose.pose.position.y),
          static_cast<float>(received_pose_.pose.pose.position.z),
      };
  const Eigen::Translation3f init_translation(translation_vector);
  const Eigen::AngleAxisf init_rotation(
      Eigen::Quaternionf(received_pose_.pose.pose.orientation.w,
                         received_pose_.pose.pose.orientation.x,
                         received_pose_.pose.pose.orientation.y,
                         received_pose_.pose.pose.orientation.z)
          .normalized());
  const Eigen::Matrix4f init_guess =
      (init_translation * init_rotation).matrix();
  ROS_INFO_STREAM("init_guess:\n"
                  << init_guess);
  CloudTypePtr filtered_map_cloud_ptr(new CloudType);
  apply_passthrough_filter(range_, map_cloud_ptr_, filtered_map_cloud_ptr,
                           translation_vector);
  CloudTypePtr filtered_scan_cloud_ptr(new CloudType);
  apply_passthrough_filter(range_, cloud_ptr, filtered_scan_cloud_ptr);
  ROS_INFO_STREAM("scan size: " << filtered_scan_cloud_ptr->points.size());
  ROS_INFO_STREAM("map size: " << filtered_map_cloud_ptr->points.size());
  // ROS_INFO_STREAM("received_pose:\n" << received_pose_.pose.pose);

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
  if (!ndt.hasConverged())
  {
    ROS_ERROR("ndt not converged!");
    return Eigen::Matrix4f::Zero();
  }
  ROS_INFO_STREAM("score: " << ndt.getFitnessScore());
  if (cloud_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 aligned_cloud_msg;
    aligned_cloud_ptr->header.frame_id = map_cloud_ptr_->header.frame_id;
    pcl::toROSMsg(*aligned_cloud_ptr, aligned_cloud_msg);
    cloud_pub_.publish(aligned_cloud_msg);
  }
  return ndt.getFinalTransformation();
}

void MapMatcher::apply_voxel_grid_filter(double leaf_size,
                                         CloudTypePtr& cloud_ptr)
{
  pcl::VoxelGrid<PointType> vg;
  vg.setInputCloud(cloud_ptr);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  CloudTypePtr output_cloud_ptr(new CloudType);
  vg.filter(*output_cloud_ptr);
  // pcl::copyPointCloud(*output_cloud_ptr, *cloud_ptr);
  *cloud_ptr = *output_cloud_ptr;
}

void MapMatcher::apply_passthrough_filter(double range,
                                          const CloudTypePtr& cloud_ptr,
                                          CloudTypePtr& output_cloud_ptr,
                                          const Eigen::Vector3f& center)
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

void MapMatcher::process(void)
{
  ros::spin();
}

}  // namespace ndt_localizer
