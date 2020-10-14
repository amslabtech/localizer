#include "ndt_localizer/ndt_odom_integrator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_odom_integrator");
    ndt_localizer::NDTOdomIntegrator ndt_odom_integrator;
    ndt_odom_integrator.process();
    return 0;
}