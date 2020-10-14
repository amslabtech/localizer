cmake_minimum_required(VERSION 3.5)
project(ndt_localizer)

add_compile_options(-std=c++14 -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

find_package(PCL 1.8.0 REQUIRED)
find_package(Eigen 3 REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ndt_localizer
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(map_matcher src/map_matcher_node.cpp src/map_matcher.cpp)
add_executable(ndt_odom_integrator src/ndt_odom_integrator_node.cpp src/ndt_odom_integrator.cpp)

add_dependencies(map_matcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ndt_odom_integrator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(map_matcher
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(ndt_odom_integrator
    ${catkin_LIBRARIES}
)