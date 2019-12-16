/* map_match_node.cpp
 *
 * 2019.08.25
 *
 * author : R.Kusakari
 *
*/
#include<ros/ros.h>
#include"map_match_omp.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "map_match_omp");
    ros::NodeHandle n;
    ros::NodeHandle priv_nh("~");
    ros::Rate loop(20);

    ROS_INFO("\033[1;32m---->\033[0m map_match Started.");

    Matcher matcher(n,priv_nh);

    std::string map_file;
    /* map_file = argv[1]; */
    priv_nh.param("MAP_FILE", map_file, std::string("$(find localizer)/example_data/d_kan_indoor.pcd"));
    matcher.map_read(map_file);

    std::cout << "waiting for data ..." << std::endl;
    while(ros::ok()){

        if(matcher.is_start) matcher.process();
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}








