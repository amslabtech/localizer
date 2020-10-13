#include "ndt_localizer/map_matcher.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_matcher");
    ndt_localizer::MapMatcher map_matcher;
    map_matcher.process();
    return 0;
}