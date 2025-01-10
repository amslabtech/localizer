// Copyright 2023 amsl

#include "ndt_localizer/map_matcher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<ndt_localizer::MapMatcher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
