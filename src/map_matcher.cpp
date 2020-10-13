#include "ndt_localizer/map_matcher.h"

namespace ndt_localizer
{
MapMatcher::MapMatcher(void)
: local_nh_("~")
{
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 1);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud/aligned", 1);
    pose_sub_ = nh_.subscribe("estimated_pose", 1, &MapMatcher::pose_callback, this, ros::TransportHints().reliable().tcpNoDelay(true)); 
    map_sub_ = nh_.subscribe("map_cloud", 1, &MapMatcher::map_callback, this, ros::TransportHints().reliable().tcpNoDelay(true)); 
    cloud_sub_ = nh_.subscribe("scan_cloud", 1, &MapMatcher::cloud_callback, this, ros::TransportHints().reliable().tcpNoDelay(true)); 

    local_nh_.param<double>("epsilon", epsilon_, 1e-8);
    local_nh_.param<double>("leaf_size", leaf_size_, 0.1);
    local_nh_.param<double>("step_size", step_size_, 0.1);
    local_nh_.param<double>("resolution", resolution_, 0.1);
    local_nh_.param<int>("max_iterations", max_iterations_, 100);

    ROS_INFO("=== map_matcher ===");
    ROS_INFO_STREAM("epsilon: " << epsilon_);
    ROS_INFO_STREAM("leaf_size: " << leaf_size_);
    ROS_INFO_STREAM("step_size: " << step_size_);
    ROS_INFO_STREAM("resolution: " << resolution_);
    ROS_INFO_STREAM("max_iterations: " << max_iterations_);

    is_map_received_ = false;
    is_pose_updated_ = false;
}

void MapMatcher::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    is_pose_updated_ = true;
    received_pose_ = *msg;
}

void MapMatcher::map_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    ROS_INFO("received map");
    pcl::fromROSMsg(*msg, *map_cloud_ptr_);
    is_map_received_ = true;
}

void MapMatcher::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(is_map_received_ && is_pose_updated_){
        CloudTypePtr cloud_ptr(new CloudType);
        pcl::fromROSMsg(*msg, *cloud_ptr);
        const Eigen::Matrix4f transform = get_ndt_transform(cloud_ptr);
        if(transform.isZero(1e-6)){
            return;
        }
        geometry_msgs::PoseStamped aligned_pose;
        aligned_pose.header = msg->header;
        aligned_pose.pose.position.x = transform(3, 0);
        aligned_pose.pose.position.y = transform(3, 1);
        aligned_pose.pose.position.z = transform(3, 2);
        Eigen::Quaternionf q(Eigen::Matrix3f(transform.block(0, 0, 3, 3)));
        q.normalize();
        aligned_pose.pose.orientation.w = q.w();
        aligned_pose.pose.orientation.x = q.x();
        aligned_pose.pose.orientation.y = q.y();
        aligned_pose.pose.orientation.z = q.z();
        pose_pub_.publish(aligned_pose);
        is_pose_updated_ = false;
    }else{
        if(!is_pose_updated_){
            ROS_WARN_THROTTLE(1.0, "cloud is received but pose has not been udpated");
        }
        if(!is_map_received_){
            ROS_WARN_THROTTLE(1.0, "cloud is received but map has not been received");
        }
    }
}

Eigen::Matrix4f MapMatcher::get_ndt_transform(const CloudTypePtr& cloud_ptr)
{
    const Eigen::Translation3f init_translation(received_pose_.pose.pose.position.x,
                                                received_pose_.pose.pose.position.y,
                                                received_pose_.pose.pose.position.z);
    const Eigen::AngleAxisf init_rotation(Eigen::Quaternionf(received_pose_.pose.pose.orientation.w,
                                                             received_pose_.pose.pose.orientation.x,
                                                             received_pose_.pose.pose.orientation.y,
                                                             received_pose_.pose.pose.orientation.z
                                                             ).normalized());
    const Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(epsilon_);
    ndt.setStepSize(step_size_);
    ndt.setResolution(resolution_);
    ndt.setMaximumIterations(max_iterations_);
    ndt.setInputSource(cloud_ptr);
    ndt.setInputTarget(map_cloud_ptr_);
    CloudTypePtr aligned_cloud_ptr(new CloudType);
    ndt.align(*aligned_cloud_ptr, init_guess);
    if(!ndt.hasConverged()){
        ROS_ERROR("ndt not converged!");
        return Eigen::Matrix4f::Zero();
    }
    if(cloud_pub_.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 aligned_cloud_msg;
        pcl::toROSMsg(*aligned_cloud_ptr, aligned_cloud_msg);
        cloud_pub_.publish(aligned_cloud_msg);
    }
    return ndt.getFinalTransformation();
}

void MapMatcher::process(void)
{
    ros::spin();
}

}