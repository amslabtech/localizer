#ifndef __MAP_MATCHER_H
#define __MAP_MATCHER_H

#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pclomp/ndt_omp.h"

#include <Eigen/Dense>

namespace ndt_localizer
{
class MapMatcher 
{
public:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> CloudType;
    typedef pcl::PointCloud<PointType>::Ptr CloudTypePtr;
    MapMatcher(void);
    void pose_callback(const nav_msgs::OdometryConstPtr& msg);
    void map_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    Eigen::Matrix4f get_ndt_transform(const CloudTypePtr& cloud_ptr);
    void apply_voxel_grid_filter(double leaf_size, CloudTypePtr& cloud_ptr);
    void apply_passthrough_filter(double range, const CloudTypePtr& cloud_ptr, CloudTypePtr& output_cloud_ptr, const Eigen::Vector3f& center = Eigen::Vector3f::Zero());
    void process(void);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher downsampled_map_pub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber cloud_sub_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf_;

    CloudTypePtr map_cloud_ptr_;
    bool is_map_received_;
    nav_msgs::Odometry received_pose_;
    bool is_pose_updated_;

    // ndt params
    double epsilon_;
    double leaf_size_;
    double step_size_;
    double resolution_;
    int max_iterations_;

    double range_;
};

}

#endif// __MAP_MATCHER_H