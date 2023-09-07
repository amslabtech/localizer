// Copyright 2023 amsl

#ifndef NDT_LOCALIZER_NDT_ODOM_INTEGRATOR_H
#define NDT_LOCALIZER_NDT_ODOM_INTEGRATOR_H

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

namespace ndt_localizer
{
class NDTOdomIntegrator
{
public:
  NDTOdomIntegrator(void);
  void ndt_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
  void odom_callback(const nav_msgs::OdometryConstPtr& msg);
  void imu_callback(const sensor_msgs::ImuConstPtr& msg);
  void map_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void init_pose_callback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void initialize_state(double x, double y, double z, double roll, double pitch,
                        double yaw);
  geometry_msgs::PoseWithCovariance get_pose_msg_from_state(void);
  void predict_by_odom(const Eigen::Vector3d& dp);
  void predict_by_imu(const Eigen::Vector3d& dr);
  void predict_between_timestamps(const ros::Time begin_stamp, const ros::Time end_stamp);
  void update_by_ndt_pose(const Eigen::VectorXd& pose);
  Eigen::Matrix3d get_rotation_matrix(double roll, double pitch, double yaw);
  void publish_map_to_odom_tf(const ros::Time& stamp,
                              const geometry_msgs::Pose& pose);
  void process(void);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle local_nh_;
  ros::Publisher estimated_pose_pub_;
  ros::Subscriber ndt_pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber init_pose_sub_;

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

  unsigned int state_dim_;
  unsigned int position_dim_;
  unsigned int orientation_dim_;
  int queue_size_;
  std::vector<nav_msgs::Odometry> odom_queue_;
  std::vector<sensor_msgs::Imu> imu_queue_;
  Eigen::VectorXd last_pose_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd p_;
  Eigen::MatrixXd q_odom_;
  Eigen::MatrixXd q_imu_;
  Eigen::MatrixXd r_;
  ros::Time last_odom_stamp_;
  ros::Time last_imu_stamp_;
  ros::Time last_pose_stamp_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace ndt_localizer

#endif  // NDT_LOCALIZER_NDT_ODOM_INTEGRATOR_H
