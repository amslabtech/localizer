#ifndef __MAP_MATCHER_H
#define __MAP_MATCHER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

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
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void map_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    Eigen::Matrix4f get_ndt_transform(const CloudTypePtr& cloud_ptr);
    void apply_voxel_grid_filter(double leaf_size, CloudTypePtr& cloud_ptr);
    void process(void);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher cloud_pub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber cloud_sub_;

    CloudTypePtr map_cloud_ptr_;
    bool is_map_received_;
    geometry_msgs::PoseWithCovarianceStamped received_pose_;
    bool is_pose_updated_;

    // ndt params
    double epsilon_;
    double leaf_size_;
    double step_size_;
    double resolution_;
    int max_iterations_;
};

}

#endif// __MAP_MATCHER_H