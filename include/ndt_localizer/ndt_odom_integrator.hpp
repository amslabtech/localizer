// Copyright 2023 amsl

#ifndef NDT_LOCALIZER__NDT_ODOM_INTEGRATOR_HPP_
#define NDT_LOCALIZER__NDT_ODOM_INTEGRATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace ndt_localizer
{
class NDTOdomIntegrator : public rclcpp::Node
{
public:
  explicit NDTOdomIntegrator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void ndt_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void map_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void init_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  void initialize_state(double x, double y, double z, double roll, double pitch, double yaw);
  geometry_msgs::msg::PoseWithCovariance get_pose_msg_from_state(void);
  void predict_by_odom(const Eigen::Vector3d & dp);
  void predict_by_imu(const Eigen::Vector3d & dr);
  void predict_between_timestamps(const rclcpp::Time begin_stamp, const rclcpp::Time end_stamp);
  void update_by_ndt_pose(const Eigen::VectorXd & pose);
  Eigen::Matrix3d get_rotation_matrix(double roll, double pitch, double yaw);
  bool is_mahalanobis_distance_gate(
    const double mahalanobis_distance_threshold, const Eigen::VectorXd & ndt_pose,
    const Eigen::VectorXd & last_pose, const Eigen::MatrixXd & cov);
  bool is_covariance_large(
    const double pose_covariance_threshold, const double direction_covariance_threshold,
    const Eigen::MatrixXd & covariance);
  void publish_map_to_odom_tf(const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose);

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;

  double init_sigma_position_;
  double init_sigma_orientation_;
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string robot_frame_id_;
  double sigma_odom_;
  double sigma_imu_;
  double sigma_ndt_;
  bool enable_odom_tf_;
  bool enable_tf_;
  double mahalanobis_distance_threshold_;
  double pose_covariance_threshold_;
  double direction_covariance_threshold_;

  unsigned int state_dim_;
  unsigned int position_dim_;
  unsigned int orientation_dim_;
  int queue_capacity_;
  std::vector<nav_msgs::msg::Odometry> odom_queue_;
  std::vector<sensor_msgs::msg::Imu> imu_queue_;
  Eigen::VectorXd last_pose_;
  Eigen::MatrixXd last_covariance_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd p_;
  Eigen::MatrixXd q_odom_;
  Eigen::MatrixXd q_imu_;
  Eigen::MatrixXd r_;
  rclcpp::Time last_odom_stamp_;
  rclcpp::Time last_imu_stamp_;
  rclcpp::Time last_pose_stamp_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace ndt_localizer

#endif  // NDT_LOCALIZER__NDT_ODOM_INTEGRATOR_HPP_
