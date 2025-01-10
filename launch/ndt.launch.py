from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="ndt_localizer",
                executable="map_matcher",
                name="map_matcher",
                output="screen",
                parameters=[
                    {
                        "leaf_size": 0.5,
                        "epsilon": 1e-2,
                        "max_iteration": 30,
                        "range": 50.0,
                    },
                ],
                remappings=[
                    ("scan_cloud", "velodyne_points"),
                    ("map_cloud", "cloud_pcd"),
                ],
            ),
            Node(
                package="ndt_localizer",
                executable="ndt_odom_integrator",
                name="ndt_odom_integrator",
                output="screen",
                parameters=[
                    {
                        "sigma_odom": 0.01,
                        "sigma_imu": 0.01,
                        "sigma_ndt": 0.1,
                    },
                ],
                remappings=[
                    ("odom", "odom"),
                    ("imu/data", "imu/data"),
                    ("map_cloud", "cloud_pcd"),
                ],
            ),
        ]
    )
