// Copyright 2023 amsl
#include "ndt_localizer/ndt_odom_integrator.hpp"

#include <iterator>
#include <memory>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ndt_localizer
{
NDTOdomIntegrator::NDTOdomIntegrator(const rclcpp::NodeOptions& options)
  : Node("ndt_odom_integrator", options)
{
  RCLCPP_INFO(this->get_logger(), "=== ndt_odom_integrator ===");
  estimated_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("estimated_pose", 1);
  ndt_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "ndt_pose", 1, std::bind(&NDTOdomIntegrator::ndt_pose_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, std::bind(&NDTOdomIntegrator::odom_callback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 1, std::bind(&NDTOdomIntegrator::imu_callback, this, std::placeholders::_1));
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "map_cloud", 1, std::bind(&NDTOdomIntegrator::map_callback, this, std::placeholders::_1));
  init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1,
      std::bind(&NDTOdomIntegrator::init_pose_callback, this, std::placeholders::_1));

  init_sigma_position_ = this->declare_parameter<double>("init_sigma_position", 10.0);
  init_sigma_orientation_ = this->declare_parameter<double>("init_sigma_orientation", M_PI);
  const double init_x = this->declare_parameter<double>("init_x", 0.0);
  const double init_y = this->declare_parameter<double>("init_y", 0.0);
  const double init_z = this->declare_parameter<double>("init_z", 0.0);
  const double init_roll = this->declare_parameter<double>("init_roll", 0.0);
  const double init_pitch = this->declare_parameter<double>("init_pitch", 0.0);
  const double init_yaw = this->declare_parameter<double>("init_yaw", 0.0);
  sigma_odom_ = this->declare_parameter<double>("sigma_odom", 1e-4);
  sigma_imu_ = this->declare_parameter<double>("sigma_imu", 1e-4);
  sigma_ndt_ = this->declare_parameter<double>("sigma_ndt", 1e-2);
  enable_odom_tf_ = this->declare_parameter<bool>("enable_odom_tf", true);
  enable_tf_ = this->declare_parameter<bool>("enable_tf", true);
  queue_capacity_ = this->declare_parameter<int>("queue_capacity", 1000);
  mahalanobis_distance_threshold_ = this->declare_parameter<double>("mahalanobis_distance_threshold", 1.5);
  pose_covariance_threshold_ = this->declare_parameter<double>("pose_covariance_threshold", 1.0);
  direction_covariance_threshold_ = this->declare_parameter<double>("direction_covariance_threshold", 1.0);

  RCLCPP_INFO_STREAM(this->get_logger(), "init_sigma_position: " << init_sigma_position_);
  RCLCPP_INFO_STREAM(this->get_logger(), "init_sigma_orientation: " << init_sigma_orientation_);
  RCLCPP_INFO_STREAM(this->get_logger(), "init_x: " << init_x);
  RCLCPP_INFO_STREAM(this->get_logger(), "init_y: " << init_y);
  RCLCPP_INFO_STREAM(this->get_logger(), "init_z: " << init_z);
  RCLCPP_INFO_STREAM(this->get_logger(), "init_roll: " << init_roll);
  RCLCPP_INFO_STREAM(this->get_logger(), "init_pitch: " << init_pitch);
  RCLCPP_INFO_STREAM(this->get_logger(), "init_yaw: " << init_yaw);
  RCLCPP_INFO_STREAM(this->get_logger(), "sigma_odom: " << sigma_odom_);
  RCLCPP_INFO_STREAM(this->get_logger(), "sigma_imu: " << sigma_imu_);
  RCLCPP_INFO_STREAM(this->get_logger(), "sigma_ndt: " << sigma_ndt_);
  RCLCPP_INFO_STREAM(this->get_logger(), "enable_odom_tf: " << enable_odom_tf_);
  RCLCPP_INFO_STREAM(this->get_logger(), "enable_tf: " << enable_tf_);
  RCLCPP_INFO_STREAM(this->get_logger(), "queue_capacity: " << queue_capacity_);
  RCLCPP_INFO_STREAM(
      this->get_logger(), "mahalanobis_distance_threshold: " << mahalanobis_distance_threshold_);
  RCLCPP_INFO_STREAM(
      this->get_logger(), "pose_covariance_threshold: " << pose_covariance_threshold_);
  RCLCPP_INFO_STREAM(
      this->get_logger(), "direction_covariance_threshold: " << direction_covariance_threshold_);

  tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_->setUsingDedicatedThread(true);
  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  state_dim_ = 6;
  position_dim_ = 3;
  orientation_dim_ = 3;

  initialize_state(init_x, init_y, init_z, init_roll, init_pitch, init_yaw);

  q_odom_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
  q_odom_.block(0, 0, position_dim_, position_dim_) =
      sigma_odom_ * Eigen::MatrixXd::Identity(position_dim_, position_dim_);
  q_imu_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
  q_imu_.block(position_dim_, position_dim_, orientation_dim_, orientation_dim_) =
      sigma_imu_ * Eigen::MatrixXd::Identity(orientation_dim_, orientation_dim_);
  r_ = sigma_ndt_ * Eigen::MatrixXd::Identity(state_dim_, state_dim_);

  last_odom_stamp_ = rclcpp::Time(0);
  last_imu_stamp_ = rclcpp::Time(0);
  map_frame_id_ = "";
  odom_frame_id_ = "";
  robot_frame_id_ = "";
}

void NDTOdomIntegrator::ndt_pose_callback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
  if (map_frame_id_.empty())
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "frame_id is empty");
    return;
  }
  double roll, pitch, yaw;
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.orientation, q);
  tf2::Matrix3x3 rot(q);
  rot.getRPY(roll, pitch, yaw);
  Eigen::VectorXd received_pose = Eigen::VectorXd::Zero(state_dim_);
  received_pose << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, roll, pitch,
      yaw;

  // Predict update from last pose
  x_ = last_pose_;
  p_ = last_covariance_;
  const rclcpp::Time received_pose_stamp = msg->header.stamp;
  if (last_pose_stamp_ <= received_pose_stamp)
  {
    predict_between_timestamps(last_pose_stamp_, received_pose_stamp);

    if (
        is_mahalanobis_distance_gate(mahalanobis_distance_threshold_, received_pose, x_, p_) ||
        is_covariance_large(pose_covariance_threshold_, direction_covariance_threshold_, p_))
    {
      update_by_ndt_pose(received_pose);
    }

    last_pose_ = x_;
    last_covariance_ = p_;
    last_pose_stamp_ = msg->header.stamp;
    predict_between_timestamps(received_pose_stamp, this->get_clock()->now());
  }
  else
  {
    // After initialize
    predict_between_timestamps(last_pose_stamp_, this->get_clock()->now());
  }

  const geometry_msgs::msg::PoseWithCovariance p = get_pose_msg_from_state();
  nav_msgs::msg::Odometry estimated_pose;
  estimated_pose.header.frame_id = map_frame_id_;
  estimated_pose.header.stamp = msg->header.stamp;
  estimated_pose.child_frame_id = robot_frame_id_;
  estimated_pose.pose = p;
  estimated_pose_pub_->publish(estimated_pose);

  // Erase queue before received pose
  std::vector<nav_msgs::msg::Odometry>::iterator itr_odom = odom_queue_.begin();
  std::vector<sensor_msgs::msg::Imu>::iterator itr_imu = imu_queue_.begin();
  while (itr_odom != odom_queue_.end())
  {
    if (rclcpp::Time(itr_odom->header.stamp) < received_pose_stamp)
    {
      odom_queue_.erase(itr_odom);
    }
    else
    {
      itr_odom++;
    }
  }
  while (itr_imu != imu_queue_.end())
  {
    if (rclcpp::Time(itr_imu->header.stamp) < received_pose_stamp)
    {
      imu_queue_.erase(itr_imu);
    }
    else
    {
      itr_imu++;
    }
  }
}

void NDTOdomIntegrator::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
  if (enable_odom_tf_)
  {
    geometry_msgs::msg::TransformStamped odom_to_robot_tf;
    odom_to_robot_tf.header = msg->header;
    odom_to_robot_tf.child_frame_id = msg->child_frame_id;
    odom_to_robot_tf.transform.translation.x = msg->pose.pose.position.x;
    odom_to_robot_tf.transform.translation.y = msg->pose.pose.position.y;
    odom_to_robot_tf.transform.translation.z = msg->pose.pose.position.z;
    odom_to_robot_tf.transform.rotation = msg->pose.pose.orientation;
    tfb_->sendTransform(odom_to_robot_tf);
  }

  if (map_frame_id_.empty())
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "frame_id is empty");
    return;
  }
  const rclcpp::Time stamp = msg->header.stamp;
  odom_frame_id_ = msg->header.frame_id;
  robot_frame_id_ = msg->child_frame_id;
  if (last_odom_stamp_ != rclcpp::Time(0))
  {
    const double dt = (stamp - last_odom_stamp_).seconds();
    const Eigen::Vector3d dp = {
        dt * msg->twist.twist.linear.x,
        dt * msg->twist.twist.linear.y,
        dt * msg->twist.twist.linear.z,
    };
    predict_by_odom(dp);
    const geometry_msgs::msg::PoseWithCovariance p = get_pose_msg_from_state();
    nav_msgs::msg::Odometry estimated_pose;
    estimated_pose.header.frame_id = map_frame_id_;
    estimated_pose.header.stamp = msg->header.stamp;
    estimated_pose.child_frame_id = robot_frame_id_;
    estimated_pose.pose = p;
    estimated_pose_pub_->publish(estimated_pose);

    if (enable_tf_)
    {
      publish_map_to_odom_tf(estimated_pose.header.stamp, estimated_pose.pose.pose);
    }
  }
  else
  {
    // first callback
  }
  last_odom_stamp_ = stamp;
  odom_queue_.push_back(*msg);
}

void NDTOdomIntegrator::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
  if (map_frame_id_.empty())
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "frame_id is empty");
    return;
  }
  const rclcpp::Time stamp = msg->header.stamp;
  if (last_imu_stamp_ != rclcpp::Time(0))
  {
    const double dt = (stamp - last_imu_stamp_).seconds();
    const Eigen::Vector3d dr = {
        dt * msg->angular_velocity.x,
        dt * msg->angular_velocity.y,
        dt * msg->angular_velocity.z,
    };
    predict_by_imu(dr);
  }
  else
  {
    // first callback
  }
  last_imu_stamp_ = stamp;
  imu_queue_.push_back(*msg);
}

void NDTOdomIntegrator::map_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  map_frame_id_ = msg->header.frame_id;
  RCLCPP_INFO_STREAM(this->get_logger(), "frame_id is set as " << map_frame_id_);
}

void NDTOdomIntegrator::init_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
{
  RCLCPP_INFO(this->get_logger(), "received init pose");
  if (map_frame_id_.empty())
  {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000, "map has not been received");
    return;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped pose_in_map;
  try
  {
    tf_->transform(*msg, pose_in_map, map_frame_id_, tf2::durationFromSec(0.1));
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000, ex.what());
    return;
  }
  double roll, pitch, yaw;
  tf2::Quaternion q;
  tf2::fromMsg(pose_in_map.pose.pose.orientation, q);
  tf2::Matrix3x3 rot(q);
  rot.getRPY(roll, pitch, yaw);
  initialize_state(
      pose_in_map.pose.pose.position.x, pose_in_map.pose.pose.position.y,
      pose_in_map.pose.pose.position.z, roll, pitch, yaw);
  last_pose_stamp_ = msg->header.stamp;
  RCLCPP_INFO_STREAM(this->get_logger(), "pose: " << x_.transpose());
}

void NDTOdomIntegrator::initialize_state(
    double x, double y, double z, double roll, double pitch, double yaw)
{
  x_ = Eigen::VectorXd::Zero(state_dim_);
  x_ << x, y, z, roll, pitch, yaw;
  p_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  p_.block(0, 0, position_dim_, position_dim_) =
      init_sigma_position_ * p_.block(0, 0, position_dim_, position_dim_);
  p_.block(position_dim_, position_dim_, orientation_dim_, orientation_dim_) =
      init_sigma_orientation_ *
      p_.block(position_dim_, position_dim_, orientation_dim_, orientation_dim_);

  const geometry_msgs::msg::PoseWithCovariance p = get_pose_msg_from_state();
  nav_msgs::msg::Odometry estimated_pose;
  estimated_pose.header.frame_id = map_frame_id_;
  // estimated_pose.header.stamp = rclcpp::Time::now();
  estimated_pose.header.stamp = this->get_clock()->now();
  estimated_pose.child_frame_id = robot_frame_id_;
  estimated_pose.pose = p;
  estimated_pose_pub_->publish(estimated_pose);

  last_pose_ = x_;
  last_covariance_ = p_;
  odom_queue_.clear();
  imu_queue_.clear();
  odom_queue_.reserve(queue_capacity_);
  imu_queue_.reserve(queue_capacity_);
}

geometry_msgs::msg::PoseWithCovariance NDTOdomIntegrator::get_pose_msg_from_state(void)
{
  geometry_msgs::msg::PoseWithCovariance p;
  p.pose.position.x = x_(0);
  p.pose.position.y = x_(1);
  p.pose.position.z = x_(2);
  tf2::Quaternion q;
  q.setRPY(x_(3), x_(4), x_(5));
  p.pose.orientation = tf2::toMsg(q);
  for (unsigned int i = 0; i < state_dim_; ++i)
  {
    for (unsigned int j = 0; j < state_dim_; ++j)
    {
      p.covariance[i * state_dim_ + j] = p_(i, j);
    }
  }
  return p;
}

void NDTOdomIntegrator::predict_by_odom(const Eigen::Vector3d& dp)
{
  const double rx = x_(3);
  const double ry = x_(4);
  const double rz = x_(5);
  const double crx = cos(rx);
  const double srx = sin(rx);
  const double cry = cos(ry);
  const double sry = sin(ry);
  const double crz = cos(rz);
  const double srz = sin(rz);

  // RCLCPP_INFO(this->get_logger(), "predict by odom");
  // RCLCPP_INFO_STREAM(this->get_logger(), "x: " << x_.transpose());
  // RCLCPP_INFO_STREAM(this->get_logger(), "p:\n" << p_);
  // f(x)
  Eigen::VectorXd f(state_dim_);
  f.segment(0, position_dim_) = x_.segment(0, position_dim_) + get_rotation_matrix(rx, ry, rz) * dp;
  f.segment(position_dim_, orientation_dim_) = x_.segment(position_dim_, orientation_dim_);

  // jf(x)
  Eigen::MatrixXd jf = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
  jf.block(0, 0, position_dim_, position_dim_) = Eigen::Matrix3d::Identity();
  jf(0, 3) = dp(1) * (crx * sry * crz) + dp(2) * (-srx * sry * crz + crx * srz);
  jf(0, 4) =
      dp(0) * (-sry * crz) + dp(1) * (srx * cry + crz) + dp(2) * (-srx * sry * crz + crx * crz);
  jf(0, 5) = dp(0) * (-crx * srz) + dp(1) * (-srx * sry * srz - crx * crz) +
             dp(2) * (-crx * sry * srz + srx * crz);
  jf(1, 3) = dp(1) * (crx * sry * srz - srx * crz) + dp(2) * (-srx * sry * srz - crx * crz);
  jf(1, 4) = dp(0) * (-sry * srz) + dp(1) * (srx * cry * srz) + dp(2) * (crx * cry * srz);
  jf(1, 5) = dp(0) * (cry * crz) + dp(1) * (srx * sry * crz - crx * srz) +
             dp(2) * (crx * sry * crz + srx * srz);
  jf(2, 3) = dp(1) * (crx * cry) + dp(2) * (-srx * cry);
  jf(2, 4) = dp(0) * (-cry) + dp(1) * (-srx * sry) + dp(2) * (-crx * sry);
  jf(2, 5) = 0.0;
  jf.block(position_dim_, position_dim_, orientation_dim_, orientation_dim_) =
      Eigen::Matrix3d::Identity();

  x_ = f;
  p_ = jf * p_ * jf.transpose() + q_odom_;
  // RCLCPP_INFO_STREAM(this->get_logger(), "x: " << x_.transpose());
  // RCLCPP_INFO_STREAM(this->get_logger(), "p:\n" << p_);
}

void NDTOdomIntegrator::predict_by_imu(const Eigen::Vector3d& dr)
{
  const double rx = x_(3);
  const double ry = x_(4);
  // const double rz = x_(5);
  const double crx = cos(rx);
  const double srx = sin(rx);
  // const double t_rx = tan(rx);
  const double cry = cos(ry);
  const double sry = sin(ry);
  const double t_ry = tan(ry);
  // const double crz = cos(rz);
  // const double srz = sin(rz);
  // const double t_rz = tan(rz);

  // RCLCPP_INFO(this->get_logger(), "predict by imu");
  // RCLCPP_INFO_STREAM(this->get_logger(), "x: " << x_.transpose());
  // RCLCPP_INFO_STREAM(this->get_logger(), "p:\n" << p_);
  Eigen::Matrix3d rot;
  rot << 1.0, srx * t_ry, crx * t_ry, 0.0, crx, -srx, 0.0, srx / cry, crx / cry;

  // f(x)
  Eigen::VectorXd f(state_dim_);
  f.segment(0, position_dim_) = x_.segment(0, position_dim_);
  f.segment(position_dim_, orientation_dim_) =
      x_.segment(position_dim_, orientation_dim_) + rot * dr;
  for (unsigned int i = position_dim_; i < state_dim_; ++i)
  {
    f(i) = atan2(sin(f(i)), cos(f(i)));
  }

  // jf(x)
  Eigen::MatrixXd jf = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
  jf.block(0, 0, position_dim_, position_dim_) = Eigen::Matrix3d::Identity();
  jf(3, 3) = 1.0 + dr(1) * crx * t_ry - dr(2) * srx * t_ry;
  jf(3, 4) = dr(1) * srx / (cry * cry) + dr(2) * crx / (cry * cry);
  jf(3, 5) = 0.0;
  jf(4, 3) = -dr(1) * srx - dr(2) * crx;
  jf(4, 4) = 1.0;
  jf(4, 5) = 0.0;
  jf(5, 3) = dr(1) * crx / cry - dr(2) * srx / cry;
  jf(5, 4) = dr(1) * srx * sry / (cry * cry) + dr(2) * crx * sry / (cry * cry);
  jf(5, 5) = 1.0;

  x_ = f;
  p_ = jf * p_ * jf.transpose() + q_imu_;
  // RCLCPP_INFO_STREAM(this->get_logger(), "x: " << x_.transpose());
  // RCLCPP_INFO_STREAM(this->get_logger(), "p:\n" << p_);
}

void NDTOdomIntegrator::predict_between_timestamps(
    const rclcpp::Time begin_stamp, const rclcpp::Time end_stamp)
{
  std::vector<nav_msgs::msg::Odometry>::iterator itr_odom = odom_queue_.begin();
  std::vector<sensor_msgs::msg::Imu>::iterator itr_imu = imu_queue_.begin();
  if (odom_queue_.size() >= 2)
  {
    while ((itr_odom + 1) != odom_queue_.end())
    {
      if (
          begin_stamp <= rclcpp::Time(itr_odom->header.stamp) &&
          rclcpp::Time(itr_odom->header.stamp) < end_stamp)
      {
        const double dt =
            (rclcpp::Time((itr_odom + 1)->header.stamp) - rclcpp::Time(itr_odom->header.stamp))
                .seconds();
        const Eigen::Vector3d dp = {
            dt * itr_odom->twist.twist.linear.x,
            dt * itr_odom->twist.twist.linear.y,
            dt * itr_odom->twist.twist.linear.z,
        };
        predict_by_odom(dp);
      }
      itr_odom++;
    }
  }
  if (imu_queue_.size() >= 2)
  {
    while ((itr_imu + 1) != imu_queue_.end())
    {
      if (
          begin_stamp <= rclcpp::Time(itr_imu->header.stamp) &&
          rclcpp::Time(itr_imu->header.stamp) < end_stamp)
      {
        const double dt =
            (rclcpp::Time((itr_imu + 1)->header.stamp) - rclcpp::Time(itr_imu->header.stamp))
                .seconds();
        const Eigen::Vector3d dr = {
            dt * itr_imu->angular_velocity.x,
            dt * itr_imu->angular_velocity.y,
            dt * itr_imu->angular_velocity.z,
        };
        predict_by_imu(dr);
      }
      itr_imu++;
    }
  }
}

void NDTOdomIntegrator::update_by_ndt_pose(const Eigen::VectorXd& pose)
{
  RCLCPP_INFO(this->get_logger(), "update by ndt");
  RCLCPP_INFO_STREAM(this->get_logger(), "x: " << x_.transpose());
  RCLCPP_INFO_STREAM(this->get_logger(), "p:\n"
                                             << p_);
  const Eigen::VectorXd z = pose;
  RCLCPP_INFO_STREAM(this->get_logger(), "z:\n"
                                             << z.transpose());
  const Eigen::MatrixXd jh = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  Eigen::VectorXd y = z - x_;
  for (unsigned int i = position_dim_; i < state_dim_; ++i)
  {
    y(i) = atan2(sin(y(i)), cos(y(i)));
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "y:\n"
                                             << y.transpose());
  const Eigen::MatrixXd s = jh * p_ * jh.transpose() + r_;
  RCLCPP_INFO_STREAM(this->get_logger(), "s:\n"
                                             << s);
  const Eigen::MatrixXd k = p_ * jh.transpose() * s.inverse();
  RCLCPP_INFO_STREAM(this->get_logger(), "k:\n"
                                             << k);
  x_ = x_ + k * y;
  for (unsigned int i = position_dim_; i < state_dim_; ++i)
  {
    x_(i) = atan2(sin(x_(i)), cos(x_(i)));
  }
  p_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - k * jh) * p_;
  RCLCPP_INFO_STREAM(this->get_logger(), "x: " << x_.transpose());
  RCLCPP_INFO_STREAM(this->get_logger(), "p:\n"
                                             << p_);
}

Eigen::Matrix3d NDTOdomIntegrator::get_rotation_matrix(double roll, double pitch, double yaw)
{
  const Eigen::Matrix3d rot = Eigen::Quaterniond(
                                  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                                  .matrix();
  return rot;
}

bool NDTOdomIntegrator::is_mahalanobis_distance_gate(
    const double mahalanobis_distance_threshold, const Eigen::VectorXd& ndt_pose,
    const Eigen::VectorXd& last_pose, const Eigen::MatrixXd& cov)
{
  const double mahalanobis_distance =
      std::sqrt((ndt_pose - last_pose).transpose() * cov.inverse() * (ndt_pose - last_pose));

  if (mahalanobis_distance > mahalanobis_distance_threshold)
  {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Mahalanobis_distance distance is over the threshold: " << mahalanobis_distance);
    return false;
  }
  else
  {
    RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Mahalanobis_distance distance is under the threshold: " << mahalanobis_distance);
    return true;
  }
}

bool NDTOdomIntegrator::is_covariance_large(
    const double pose_covariance_threshold, const double direction_covariance_threshold,
    const Eigen::MatrixXd& /*covariance*/)
{
  const double variance_x = p_(0, 0);
  const double covariance_xy = p_(0, 1);
  const double variance_y = p_(1, 1);
  const double variance_yaw = p_(5, 5);

  if (
      (variance_x > pose_covariance_threshold) || (covariance_xy > pose_covariance_threshold) ||
      (variance_y > pose_covariance_threshold) || (variance_yaw > direction_covariance_threshold))
  {
    if (variance_x > pose_covariance_threshold)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "variance_x is over the threshold: " << variance_x);
    }
    if (covariance_xy > pose_covariance_threshold)
    {
      RCLCPP_ERROR_STREAM(
          this->get_logger(), "covariance_xy is over the threshold: " << covariance_xy);
    }
    if (variance_y > pose_covariance_threshold)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "variance_y is over the threshold: " << variance_y);
    }
    if (variance_yaw > direction_covariance_threshold)
    {
      RCLCPP_ERROR_STREAM(
          this->get_logger(), "variance_yaw is over the threshold: " << variance_yaw);
    }

    return true;
  }
  else
  {
    // RCLCPP_WARN_STREAM(this->get_logger(), "Covariance is under the threshold.");
    return false;
  }
}

void NDTOdomIntegrator::publish_map_to_odom_tf(
    const rclcpp::Time& stamp, const geometry_msgs::msg::Pose& pose)
{
  tf2::Transform map_to_robot_tf;
  tf2::convert(pose, map_to_robot_tf);
  geometry_msgs::msg::PoseStamped robot_to_map_pose;
  robot_to_map_pose.header.frame_id = robot_frame_id_;
  robot_to_map_pose.header.stamp = stamp;
  tf2::toMsg(map_to_robot_tf.inverse(), robot_to_map_pose.pose);
  geometry_msgs::msg::PoseStamped odom_to_map_pose;
  try
  {
    tf_->transform(robot_to_map_pose, odom_to_map_pose, odom_frame_id_, tf2::durationFromSec(0.1));
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000, ex.what());
    return;
  }
  tf2::Transform odom_to_map_tf;
  tf2::convert(odom_to_map_pose.pose, odom_to_map_tf);
  geometry_msgs::msg::TransformStamped map_to_odom_tf;
  map_to_odom_tf.header.stamp = stamp;
  map_to_odom_tf.header.frame_id = map_frame_id_;
  map_to_odom_tf.child_frame_id = odom_frame_id_;
  tf2::convert(odom_to_map_tf.inverse(), map_to_odom_tf.transform);
  tfb_->sendTransform(map_to_odom_tf);
}

}  // namespace ndt_localizer
