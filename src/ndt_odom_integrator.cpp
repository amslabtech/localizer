#include "ndt_localizer/ndt_odom_integrator.h"

namespace ndt_localizer
{
NDTOdomIntegrator::NDTOdomIntegrator(void)
: local_nh_("~")
{
    ROS_INFO("=== ndt_odom_integrator ===");
    estimated_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("estimated_pose", 1);
    ndt_pose_sub_ = nh_.subscribe("ndt_pose", 1, &NDTOdomIntegrator::ndt_pose_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    odom_sub_ = nh_.subscribe("odom", 1, &NDTOdomIntegrator::odom_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    imu_sub_ = nh_.subscribe("imu/data", 1, &NDTOdomIntegrator::imu_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    map_sub_ = nh_.subscribe("map_cloud", 1, &NDTOdomIntegrator::map_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    init_pose_sub_ = nh_.subscribe("initialpose", 1, &NDTOdomIntegrator::init_pose_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));

    local_nh_.param<double>("init_sigma_position", init_sigma_position_, 10);
    local_nh_.param<double>("init_sigma_orientation", init_sigma_orientation_, M_PI);
    double init_x, init_y, init_z, init_roll, init_pitch, init_yaw;
    local_nh_.param<double>("init_x", init_x, 0);
    local_nh_.param<double>("init_y", init_y, 0);
    local_nh_.param<double>("init_z", init_z, 0);
    local_nh_.param<double>("init_roll", init_roll, 0);
    local_nh_.param<double>("init_pitch", init_pitch, 0);
    local_nh_.param<double>("init_yaw", init_yaw, 0);
    local_nh_.param<double>("sigma_odom", sigma_odom_, 1e-4);
    local_nh_.param<double>("sigma_imu", sigma_imu_, 1e-4);
    local_nh_.param<double>("sigma_ndt", sigma_ndt_, 1e-2);
    local_nh_.param<bool>("enable_odom_tf", enable_odom_tf_, true);
    local_nh_.param<bool>("enable_tf", enable_tf_, true);

    ROS_INFO_STREAM("init_sigma_position: " << init_sigma_position_);
    ROS_INFO_STREAM("init_sigma_orientation: " << init_sigma_orientation_);
    ROS_INFO_STREAM("init_x: " << init_x);
    ROS_INFO_STREAM("init_y: " << init_y);
    ROS_INFO_STREAM("init_z: " << init_z);
    ROS_INFO_STREAM("init_roll: " << init_roll);
    ROS_INFO_STREAM("init_pitch: " << init_pitch);
    ROS_INFO_STREAM("init_yaw: " << init_yaw);
    ROS_INFO_STREAM("sigma_odom: " << sigma_odom_);
    ROS_INFO_STREAM("sigma_imu: " << sigma_imu_);
    ROS_INFO_STREAM("sigma_ndt: " << sigma_ndt_);
    ROS_INFO_STREAM("enable_odom_tf: " << enable_odom_tf_);
    ROS_INFO_STREAM("enable_tf: " << enable_tf_);

    tf_ = std::make_shared<tf2_ros::Buffer>();
    tf_->setUsingDedicatedThread(true);
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    state_dim_ = 6;
    position_dim_ = 3;
    orientation_dim_ = 3;

    initialize_state(init_x, init_y, init_z, init_roll, init_pitch, init_yaw);

    q_odom_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    q_odom_.block(0, 0, position_dim_, position_dim_) = sigma_odom_ * Eigen::MatrixXd::Identity(position_dim_, position_dim_);
    q_imu_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    q_imu_.block(0, 0, position_dim_, position_dim_) = sigma_odom_ * Eigen::MatrixXd::Identity(position_dim_, position_dim_);
    r_ = sigma_ndt_ * Eigen::MatrixXd::Identity(state_dim_, state_dim_);

    last_odom_stamp_ = ros::Time(0);
    last_imu_stamp_ = ros::Time(0);
    map_frame_id_ = "";
    odom_frame_id_ = "";
    robot_frame_id_ = "";
}

void NDTOdomIntegrator::ndt_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(map_frame_id_.empty()){
        ROS_ERROR_THROTTLE(3.0, "frame_id is empty");
        return;
    }
    double roll, pitch, yaw;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    tf2::Matrix3x3 rot(q);
    rot.getRPY(roll, pitch, yaw);
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(state_dim_);
    pose << msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            roll,
            pitch,
            yaw;
    update_by_ndt_pose(pose);
    const geometry_msgs::PoseWithCovariance p = get_pose_msg_from_state();
    nav_msgs::Odometry estimated_pose;
    estimated_pose.header.frame_id = map_frame_id_;
    estimated_pose.header.stamp = msg->header.stamp;
    estimated_pose.child_frame_id = robot_frame_id_;
    estimated_pose.pose = p;
    estimated_pose_pub_.publish(estimated_pose);
    if(enable_tf_){
        publish_map_to_odom_tf(estimated_pose.header.stamp, estimated_pose.pose.pose);
    }
}

void NDTOdomIntegrator::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    if(enable_odom_tf_){
        geometry_msgs::TransformStamped odom_to_robot_tf;
        odom_to_robot_tf.header = msg->header;
        odom_to_robot_tf.child_frame_id = msg->child_frame_id;
        odom_to_robot_tf.transform.translation.x = msg->pose.pose.position.x;
        odom_to_robot_tf.transform.translation.y = msg->pose.pose.position.y;
        odom_to_robot_tf.transform.translation.z = msg->pose.pose.position.z;
        odom_to_robot_tf.transform.rotation = msg->pose.pose.orientation;
        tfb_->sendTransform(odom_to_robot_tf);
    }

    if(map_frame_id_.empty()){
        ROS_ERROR_THROTTLE(3.0, "frame_id is empty");
        return;
    }
    const ros::Time stamp = msg->header.stamp;
    odom_frame_id_ = msg->header.frame_id;
    robot_frame_id_ = msg->child_frame_id;
    if(last_odom_stamp_ != ros::Time(0)){
        const double dt = (stamp - last_odom_stamp_).toSec();
        const Eigen::Vector3d dp = {dt * msg->twist.twist.linear.x, dt * msg->twist.twist.linear.y, dt * msg->twist.twist.linear.z};
        predict_by_odom(dp);
        const geometry_msgs::PoseWithCovariance p = get_pose_msg_from_state();
        nav_msgs::Odometry estimated_pose;
        estimated_pose.header.frame_id = map_frame_id_;
        estimated_pose.header.stamp = msg->header.stamp;
        estimated_pose.child_frame_id = robot_frame_id_;
        estimated_pose.pose = p;
        estimated_pose_pub_.publish(estimated_pose);
        if(enable_tf_){
            publish_map_to_odom_tf(estimated_pose.header.stamp, estimated_pose.pose.pose);
        }
    }else{
        // first callback
    }
    last_odom_stamp_ = stamp;
}

void NDTOdomIntegrator::imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
    if(map_frame_id_.empty()){
        ROS_ERROR_THROTTLE(3.0, "frame_id is empty");
        return;
    }
    const ros::Time stamp = msg->header.stamp;
    if(last_imu_stamp_ != ros::Time(0)){
        const double dt = (stamp - last_imu_stamp_).toSec();
        const Eigen::Vector3d dr = {dt * msg->angular_velocity.x, dt * msg->angular_velocity.y, dt * msg->angular_velocity.z};
        predict_by_imu(dr);
        const geometry_msgs::PoseWithCovariance p = get_pose_msg_from_state();
        nav_msgs::Odometry estimated_pose;
        estimated_pose.header.frame_id = map_frame_id_;
        estimated_pose.header.stamp = msg->header.stamp;
        estimated_pose.child_frame_id = robot_frame_id_;
        estimated_pose.pose = p;
        estimated_pose_pub_.publish(estimated_pose);
    }else{
        // first callback
    }
    last_imu_stamp_ = stamp;
}

void NDTOdomIntegrator::map_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    map_frame_id_ = msg->header.frame_id;
    ROS_INFO_STREAM("frame_id is set as " << map_frame_id_);
}

void NDTOdomIntegrator::init_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    ROS_INFO("received init pose");
    if(map_frame_id_.empty()){
        ROS_ERROR_THROTTLE(3.0, "map has not been received");
        return;
    }
    geometry_msgs::PoseWithCovarianceStamped pose_in_map;
    try{
        tf_->transform(*msg, pose_in_map, map_frame_id_, ros::Duration(0.1));
    }catch(tf2::TransformException& ex){
        ROS_WARN_STREAM_THROTTLE(3.0, ex.what());
        return;
    }
    double roll, pitch, yaw;
    tf2::Quaternion q;
    tf2::fromMsg(pose_in_map.pose.pose.orientation, q);
    tf2::Matrix3x3 rot(q);
    rot.getRPY(roll, pitch, yaw);
    initialize_state(pose_in_map.pose.pose.position.x,
                     pose_in_map.pose.pose.position.y,
                     pose_in_map.pose.pose.position.z,
                     roll,
                     pitch,
                     yaw);
    ROS_INFO_STREAM("pose: " << x_.transpose());
}

void NDTOdomIntegrator::initialize_state(double x, double y, double z, double roll, double pitch, double yaw)
{
    x_ = Eigen::VectorXd::Zero(state_dim_);
    x_ << x, y, z, roll, pitch, yaw;
    p_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    p_.block(0, 0, position_dim_, position_dim_) = init_sigma_position_ * p_.block(0, 0, position_dim_, position_dim_);
    p_.block(position_dim_, position_dim_, orientation_dim_, orientation_dim_) =
        init_sigma_orientation_ * p_.block(position_dim_, position_dim_, orientation_dim_, orientation_dim_);

    const geometry_msgs::PoseWithCovariance p = get_pose_msg_from_state();
    nav_msgs::Odometry estimated_pose;
    estimated_pose.header.frame_id = map_frame_id_;
    estimated_pose.header.stamp = ros::Time::now();
    estimated_pose.child_frame_id = robot_frame_id_;
    estimated_pose.pose = p;
    estimated_pose_pub_.publish(estimated_pose);
}

geometry_msgs::PoseWithCovariance NDTOdomIntegrator::get_pose_msg_from_state(void)
{
    geometry_msgs::PoseWithCovariance p;
    p.pose.position.x = x_(0);
    p.pose.position.y = x_(1);
    p.pose.position.z = x_(2);
    tf2::Quaternion q;
    q.setRPY(x_(3), x_(4), x_(5));
    p.pose.orientation = tf2::toMsg(q);
    for(unsigned int i=0;i<state_dim_;++i){
        for(unsigned int j=0;j<state_dim_;++j){
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
    
    // f(x)
    Eigen::VectorXd f(state_dim_);
    f.segment(0, position_dim_) = x_.segment(0, position_dim_) + get_rotation_matrix(rx, ry, rz) * dp;
    f.segment(position_dim_, orientation_dim_) = x_.segment(position_dim_, orientation_dim_);

    // jf(x)
    Eigen::MatrixXd jf = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    jf.block(0, 0, position_dim_, position_dim_) = Eigen::Matrix3d::Identity();
    jf(0, 3) = dp(1) * (crx * sry * crz) + dp(2) * (-srx * sry * crz + crx * srz);
    jf(0, 4) = dp(0) * (-sry * crz) + dp(1) * (srx * cry + crz) + dp(2) * (-srx * sry * crz + crx * crz);
    jf(0, 5) = dp(0) * (-crx * srz) + dp(1) * (-srx * sry * srz - crx * crz) + dp(2) * (-crx * sry * srz + srx * crz);
    jf(1, 3) = dp(1) * (crx * sry * srz - srx * crz) + dp(2) * (-srx * sry * srz - crx * crz);
    jf(1, 4) = dp(0) * (-sry * srz) + dp(1) * (srx * cry * srz) + dp(2) * (crx * cry * srz);
    jf(1, 5) = dp(0) * (cry * crz) + dp(1) * (srx * sry * crz - crx * srz) + dp(2) * (crx * sry * crz + srx * srz);
    jf(2, 3) = dp(1) * (crx * cry) + dp(2) * (-srx * cry);
    jf(2, 4) = dp(0) * (-cry) + dp(1) * (-srx * sry) + dp(2) * (-crx * sry);
    jf(2, 5) = 0.0;
    jf.block(position_dim_, position_dim_, orientation_dim_, orientation_dim_) = Eigen::Matrix3d::Identity();

    x_ = f;
    p_ = jf * p_ * jf.transpose() + q_odom_;
}

void NDTOdomIntegrator::predict_by_imu(const Eigen::Vector3d& dr)
{
    const double rx = x_(3);
    const double ry = x_(4);
    const double rz = x_(5);
    const double crx = cos(rx);
    const double srx = sin(rx);
    const double t_rx = tan(rx);
    const double cry = cos(ry);
    const double sry = sin(ry);
    const double t_ry = tan(ry);
    const double crz = cos(rz);
    const double srz = sin(rz);
    const double t_rz = tan(rz);

    Eigen::Matrix3d rot;
    rot << 1.0, srx * t_ry, crx * t_ry,
           0.0,        crx,       -srx,
           0.0,  srx / cry,  crx / cry;

    // f(x)
    Eigen::VectorXd f(state_dim_);
    f.segment(0, position_dim_) = x_.segment(0, position_dim_);
    f.segment(position_dim_, orientation_dim_) = x_.segment(position_dim_, orientation_dim_) + rot * dr;
    for(unsigned int i=position_dim_;i<state_dim_;++i){
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
}

void NDTOdomIntegrator::update_by_ndt_pose(const Eigen::VectorXd& pose)
{
    const Eigen::VectorXd z = pose;
    const Eigen::MatrixXd jh = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    Eigen::VectorXd y = z - x_;
    for(unsigned int i=position_dim_;i<state_dim_;++i){
        y(i) = atan2(sin(y(i)), cos(y(i)));
    }
    const Eigen::MatrixXd s = jh * p_ * jh.transpose() + r_;
    const Eigen::MatrixXd k = p_ * jh.transpose() * s.inverse();
    x_ = x_ + k * y;
    for(unsigned int i=position_dim_;i<state_dim_;++i){
        x_(i) = atan2(sin(x_(i)), cos(x_(i)));
    }
    p_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - k * jh) * p_;
}

Eigen::Matrix3d NDTOdomIntegrator::get_rotation_matrix(double roll, double pitch, double yaw)
{
    const Eigen::Matrix3d rot = Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                                                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                                   Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).matrix();
    return rot;
}

void NDTOdomIntegrator::publish_map_to_odom_tf(const ros::Time& stamp, const geometry_msgs::Pose& pose)
{
    tf2::Transform map_to_robot_tf;
    tf2::convert(pose, map_to_robot_tf);
    geometry_msgs::PoseStamped robot_to_map_pose;
    robot_to_map_pose.header.frame_id = robot_frame_id_;
    robot_to_map_pose.header.stamp = stamp;
    tf2::toMsg(map_to_robot_tf.inverse(), robot_to_map_pose.pose);
    geometry_msgs::PoseStamped odom_to_map_pose;
    try{
        tf_->transform(robot_to_map_pose, odom_to_map_pose, odom_frame_id_, ros::Duration(0.1));
    }catch(tf2::TransformException& ex){
        ROS_WARN_STREAM_THROTTLE(3.0, ex.what());
        return;
    }
    tf2::Transform odom_to_map_tf;
    tf2::convert(odom_to_map_pose.pose, odom_to_map_tf);
    geometry_msgs::TransformStamped map_to_odom_tf;
    map_to_odom_tf.header.stamp = stamp;
    map_to_odom_tf.header.frame_id = map_frame_id_;
    map_to_odom_tf.child_frame_id = odom_frame_id_;
    tf2::convert(odom_to_map_tf.inverse(), map_to_odom_tf.transform);
    tfb_->sendTransform(map_to_odom_tf);
}

void NDTOdomIntegrator::process(void)
{
    ros::spin();
}

}
