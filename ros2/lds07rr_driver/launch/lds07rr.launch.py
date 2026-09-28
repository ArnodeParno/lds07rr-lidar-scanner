"""Start the LDS07RR driver plus a static transform base_link -> laser."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    height = LaunchConfiguration("laser_height")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyAMA3"),
        DeclareLaunchArgument("laser_height", default_value="0.15",
                              description="height of the lidar above base_link in metres"),
        Node(package="lds07rr_driver", executable="lds07rr_node", name="lds07rr",
             parameters=[{"port": port}], output="screen", on_exit=Shutdown()),
        Node(package="tf2_ros", executable="static_transform_publisher", name="laser_tf",
             arguments=["--x", "0", "--y", "0", "--z", height,
                        "--frame-id", "base_link", "--child-frame-id", "laser"], on_exit=Shutdown()),
    ])
