#ifndef __NDT_ODOM_INTEGRATOR_H
#define __NDT_ODOM_INTEGRATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf2/utils.h>

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
    void initialize_state(double x, double y, double z, double roll, double pitch, double yaw);
    geometry_msgs::PoseWithCovariance get_pose_msg_from_state(void);
    void predict_by_odom(const Eigen::Vector3d& dp);
    void predict_by_imu(const Eigen::Vector3d& dr);
    void update_by_ndt_pose(const Eigen::VectorXd& pose);
    Eigen::Matrix3d get_rotation_matrix(double roll, double pitch, double yaw);
    void process(void);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher estimated_pose_pub_;
    ros::Subscriber ndt_pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber map_sub_;

    double init_sigma_position_;
    double init_sigma_orientation_;
    std::string frame_id_;// will be overwrited by frame_id of the map 
    double sigma_odom_;
    double sigma_imu_;
    double sigma_ndt_;

    unsigned int state_dim_;
    unsigned int position_dim_;
    unsigned int orientation_dim_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd p_;
    Eigen::MatrixXd q_odom_;
    Eigen::MatrixXd q_imu_;
    Eigen::MatrixXd r_;
    ros::Time last_odom_stamp_;
    ros::Time last_imu_stamp_;
};

}

#endif// __NDT_ODOM_INTEGRATOR_H