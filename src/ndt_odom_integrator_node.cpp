// Copyright 2023 amsl

#include "ndt_localizer/ndt_odom_integrator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<ndt_localizer::NDTOdomIntegrator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
