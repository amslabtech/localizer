/* map_match.cpp
 *
 * 2019.08.25
 *
 * author : R.Kusakari
 *
*/

#include"map_match_omp.hpp"

Matcher::Matcher(ros::NodeHandle n,ros::NodeHandle private_nh_) :
    local_lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>),     //範囲狭めたレーザの点群
    map_cloud(new pcl::PointCloud<pcl::PointXYZI>),     //mapの点群
    local_map_cloud(new pcl::PointCloud<pcl::PointXYZI>),//自分付近のmapの点群
    ndt_cloud(new pcl::PointCloud<pcl::PointXYZI>),
    is_start(false)
{
    pc_pub = n.advertise<sensor_msgs::PointCloud2>("/vis/ndt", 10);
    map_pub = n.advertise<sensor_msgs::PointCloud2>("/vis/map", 1, true);
    odom_pub = n.advertise<nav_msgs::Odometry>("/NDT/result", 10);

    pc_sub = n.subscribe("/velodyne_points", 1, &Matcher::lidarcallback, this);
    odom_sub = n.subscribe("/EKF/result", 1, &Matcher::odomcallback, this);

    private_nh_.param("PARENT_FRAME", PARENT_FRAME, {"/map"});
    /* n.param("CHILD_FRAME", CHILD_FRAME, {"/matching_base_link"}); */
    private_nh_.param("VOXEL_SIZE",VOXEL_SIZE ,{0.3});
    private_nh_.param("LIMIT_RANGE",LIMIT_RANGE, {20.0});
    private_nh_.param("MATCHING_SCORE_THRESHOLD", MATCHING_SCORE_THRESHOLD, {0.1});
    private_nh_.param("CLOUD_MAP_OFFSET_X", CLOUD_MAP_OFFSET_X, {0.0});
    private_nh_.param("CLOUD_MAP_OFFSET_Y", CLOUD_MAP_OFFSET_Y, {0.0});
    private_nh_.param("CLOUD_MAP_OFFSET_Z", CLOUD_MAP_OFFSET_Z, {0.0});
    private_nh_.param("CLOUD_MAP_OFFSET_ROLL", CLOUD_MAP_OFFSET_ROLL, {0.0});
    private_nh_.param("CLOUD_MAP_OFFSET_PITCH", CLOUD_MAP_OFFSET_PITCH, {0.0});
    private_nh_.param("CLOUD_MAP_OFFSET_YAW", CLOUD_MAP_OFFSET_YAW, {0.0});
    private_nh_.param("RESOLUTION", RESOLUTION, {0.5});

    std::cout<<"PARENT_FRAME : "<<PARENT_FRAME<<std::endl;
    /* std::cout<<"CHILD_FRAME : "<<CHILD_FRAME<<std::endl; */
    std::cout<<"VOXEL_SIZE: "<<VOXEL_SIZE<<std::endl;
    std::cout<<"LIMIT_RANGE : "<<LIMIT_RANGE<<std::endl;
    std::cout<<"CLOUD_MAP_OFFSET_X : "<< CLOUD_MAP_OFFSET_X <<std::endl;
    std::cout<<"CLOUD_MAP_OFFSET_Y : "<< CLOUD_MAP_OFFSET_Y <<std::endl;
    std::cout<<"CLOUD_MAP_OFFSET_Z : "<< CLOUD_MAP_OFFSET_Z <<std::endl;
    std::cout<<"CLOUD_MAP_OFFSET_ROLL : "<< CLOUD_MAP_OFFSET_ROLL <<std::endl;
    std::cout<<"CLOUD_MAP_OFFSET_PITCH : "<< CLOUD_MAP_OFFSET_PITCH <<std::endl;
    std::cout<<"CLOUD_MAP_OFFSET_YAW : "<< CLOUD_MAP_OFFSET_YAW <<std::endl;
    std::cout<<"RESOLUTION : "<< RESOLUTION <<std::endl;

    // buffer_odom.header.frame_id = PARENT_FRAME;
    // buffer_odom.child_frame_id = CHILD_FRAME;

    ndt.setNumThreads(omp_get_max_threads());
    ndt.setNeighborhoodSearchMethod(pclomp::KDTREE);
    ndt.setTransformationEpsilon(0.001);
    ndt.setStepSize(0.1);
    ndt.setResolution(RESOLUTION);//1.0 change 05/09
    ndt.setMaximumIterations(35);
}


void
Matcher::map_read(std::string filename){

    pcl::PointCloud<pcl::PointXYZI>::Ptr low_map_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    std::cout << "loading map..." << std::endl;
    if(pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *low_map_cloud) == -1){
        PCL_ERROR ("事前地図ないよ \n");
        exit(-1);
    }
    low_map_cloud->header.frame_id = PARENT_FRAME;
    std::cout<< "\x1b[32m" << "map has been loaded from : "<< filename << "\x1b[m\r" <<std::endl;
    std::cout << "raw map points: " << low_map_cloud->points.size() << std::endl;

    pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxel_filter;
    // pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

    voxel_filter.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    voxel_filter.setInputCloud (low_map_cloud);
    voxel_filter.filter (*map_cloud);

    std::cout << "downsampled map points: " << map_cloud->points.size() << std::endl;

    Eigen::Matrix3d offset_rotation;
    offset_rotation = Eigen::AngleAxisd(CLOUD_MAP_OFFSET_ROLL, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(CLOUD_MAP_OFFSET_PITCH, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(CLOUD_MAP_OFFSET_YAW, Eigen::Vector3d::UnitZ());
    Eigen::Translation3d offset_translation(CLOUD_MAP_OFFSET_X, CLOUD_MAP_OFFSET_Y, CLOUD_MAP_OFFSET_Z);

    Eigen::Affine3d cloud_map_offset = offset_rotation * offset_translation;
    pcl::transformPointCloud(*map_cloud, *map_cloud, cloud_map_offset);
    std::cout << "cloud origin in " << map_cloud->header.frame_id << ": \n" << cloud_map_offset.matrix() << std::endl;

    sensor_msgs::PointCloud2 vis_map;
    pcl::toROSMsg(*map_cloud , vis_map);

    vis_map.header.stamp = ros::Time(0); //laserのframe_id
    // vis_map.header.stamp = ros::Time(0); //laserのframe_id
    vis_map.header.frame_id = PARENT_FRAME;

    map_pub.publish(vis_map);
    // sleep(1.0);
}


void
Matcher::lidarcallback(const sensor_msgs::PointCloud2::Ptr msg){

    buffer_time  = msg->header.stamp;
    buffer_pc  = *msg;

}

void
Matcher::odomcallback(const nav_msgs::OdometryConstPtr& msg){
    is_start = true;
    buffer_odom = *msg;
}

Eigen::Matrix4f
Matcher::ndt_matching(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt,
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, nav_msgs::Odometry odo){

    std::cout << "--- ndt start ---" << std::endl;
    double start_time = ros::Time::now().toSec();
    std::cout << "source cloud: " << cloud_src->points.size() << std::endl;
    std::cout << "target cloud: " << cloud_tgt->points.size() << std::endl;
    /*------ Voxel Grid ------*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_src (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_tgt (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    vg.setInputCloud(cloud_src);
    vg.filter(*filtered_cloud_src);
    vg.setInputCloud(cloud_tgt);
    vg.filter(*filtered_cloud_tgt);
    std::cout << "downsampled source cloud: " << filtered_cloud_src->points.size() << std::endl;
    std::cout << "downsampled target cloud: " << filtered_cloud_tgt->points.size() << std::endl;

    Eigen::AngleAxisf init_rotation (tf::getYaw(odo.pose.pose.orientation) , Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (odo.pose.pose.position.x, odo.pose.pose.position.y, odo.pose.pose.position.z);

    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
    std::cout << "init guess:\n" << init_guess << std::endl;

    ndt.setInputTarget(filtered_cloud_tgt);
    ndt.setInputSource(filtered_cloud_src);
    std::cout << "cloud set" << std::endl;
    ndt.align (*cloud, init_guess);

    std::cout << "ndt has converged: " << ndt.hasConverged() << std::endl;
    std::cout << "ndt score: " << ndt.getFitnessScore() << std::endl;
    Eigen::Matrix4f result = ndt.getFinalTransformation();
    std::cout << "ndt result: \n" << result << std::endl;
    std::cout << "ndt time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
    return result;
}

void
Matcher::local_pc(
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &output_cloud,
        double x_now,double y_now)
{
    output_cloud->points.clear();

    for(const auto& temp_point : input_cloud->points){

        if((LIMIT_RANGE * (-1) + x_now <= temp_point.x && temp_point.x  <= LIMIT_RANGE+ x_now) && (LIMIT_RANGE *(-1) + y_now <= temp_point.y && temp_point.y <= LIMIT_RANGE + y_now) ){

            output_cloud->points.push_back(temp_point);
        }
    }
}


void
Matcher::calc_rpy(Eigen::Matrix4f ans, double &yaw){
    double roll, pitch;
    tf::Matrix3x3 mat_l;
    mat_l.setValue(static_cast<double>(ans(0, 0)), static_cast<double>(ans(0, 1)), static_cast<double>(ans(0, 2)),
            static_cast<double>(ans(1, 0)), static_cast<double>(ans(1, 1)), static_cast<double>(ans(1, 2)),
            static_cast<double>(ans(2, 0)), static_cast<double>(ans(2, 1)), static_cast<double>(ans(2, 2)));

    mat_l.getRPY(roll, pitch, yaw, 1);
}


void
Matcher::process(){
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(buffer_pc,*lidar_cloud);

    local_pc(lidar_cloud, local_lidar_cloud, 0.0, 0.0);

    local_pc(map_cloud, local_map_cloud, buffer_odom.pose.pose.position.x, buffer_odom.pose.pose.position.y);

    Eigen::Matrix4f answer = ndt_matching(local_map_cloud,local_lidar_cloud, ndt_cloud,buffer_odom);

    if(ndt.getFitnessScore() < MATCHING_SCORE_THRESHOLD){
        double ans_yaw;

        calc_rpy(answer,ans_yaw);

        buffer_odom.pose.pose.position.x =  answer(0, 3);
        buffer_odom.pose.pose.position.y =  answer(1, 3);
        buffer_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(ans_yaw);

        odom_pub.publish(buffer_odom);


        sensor_msgs::PointCloud2 vis_pc;
        pcl::toROSMsg(*ndt_cloud , vis_pc);

        vis_pc.header.stamp = buffer_time;
        vis_pc.header.frame_id = PARENT_FRAME;

        pc_pub.publish(vis_pc);
    }else{
        std::cout << "\033[31mmathcing result is not used due to high sum of squared distance between clouds\033[0m" << std::endl;
    }
}

