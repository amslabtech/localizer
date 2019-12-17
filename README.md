# localizer

## Requirement
- ros (kinetic)
- PCL 1.8
- [ndt_omp](https://github.com/koide3/ndt_omp)(optional)

## Runtime requirements
- tf from /base_link to /velodyne

### Subscrived topics
- /odom (nav_msgs/Odometry)
- /imu/data (sensor_msgs/Imu)
- /velodyne_points(sensor_msgs/PointCloud2)
- /move_base_simple/goal( geometry_msgs/PoseStamped)
  - deprecated

## How to use
- [download](https://drive.google.com/file/d/1BaPeG6ogi5xXnTieIbWilvUuJZT4bIzt/view?usp=sharing)
- give initial robot's position as below:
<p align="center"><img src="example_data/init_pose.gif" width=600></p>

- run
```
~$  ./run.sh
```
