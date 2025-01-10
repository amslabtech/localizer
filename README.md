# localizer

## Requirement
- ros (humble or later)

## Runtime requirements
- tf from robot base frame (e.g. base_link) to sensor frame (e.g. velodyne)

## Nodes
### map_matcher
#### Published topics
- /ndt_pose (geometry_msgs/PoseStamped)
  - aligned pose
- /cloud/aligned (sensor_msgs/PointCloud2)
  - scan_cloud aligned by NDT (for debug)
- /map_cloud/downsampled (sensor_msgs/PointCloud2)
  - map_cloud downsamped by VoxelGridFilter (for debug)
#### Subscribed topics
- /scan_cloud (sensor_msgs/PointCloud2)
  - sensor data
- /map_cloud (sensor_msgs/PointCloud2)
  - map data
- /estimated_pose (geometry_msgs/PoseWithCovarianceStamped)
  - used to set `init_guess` for NDT

### ndt_odom_integrator
#### Published topics
- /estimated_pose (geometry_msgs/PoseWithCovarianceStamped)
  - the output of this localizer
#### Subscribed topics
- /imu/data (sensor_msgs/Imu)
- /odom (nav_msgs/Odometry)
- /ndt_pose (geometry_msgs/PoseStamped)
  - from `map_matcher` node
- /map_cloud (sensor_msgs/PointCloud2)
  - used to get `frame_id`
- /initialpose (geometry_msgs/PoseWithCovarianceStamped)
  - used to set initial pose from `rviz`
