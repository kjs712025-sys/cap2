import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("omnibot_nav2")
    backend_url = LaunchConfiguration("backend_url")
    api_token = LaunchConfiguration("api_token")
    enable_bridge = LaunchConfiguration("enable_bridge")
    enable_slam = LaunchConfiguration("enable_slam")
    enable_nav2 = LaunchConfiguration("enable_nav2")
    nav2_params = os.path.join(package_share, "config", "nav2_params.yaml")
    slam_params = os.path.join(package_share, "config", "slam_toolbox.yaml")
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend_url",
                default_value="http://127.0.0.1:8000",
                description="URL of the existing OmniBot backend",
            ),
            DeclareLaunchArgument(
                "api_token",
                default_value=EnvironmentVariable("ROBOT_API_TOKEN", default_value=""),
                description="Backend API token, if authentication is enabled",
            ),
            DeclareLaunchArgument(
                "enable_bridge",
                default_value="true",
                description="Start the backend bridge node",
            ),
            DeclareLaunchArgument(
                "enable_slam",
                default_value="true",
                description="Start online SLAM Toolbox mapping",
            ),
            DeclareLaunchArgument(
                "enable_nav2",
                default_value="true",
                description="Start the Nav2 navigation stack",
            ),
            Node(
                package="omnibot_nav2",
                executable="backend_bridge",
                name="omnibot_backend_bridge",
                condition=IfCondition(enable_bridge),
                output="screen",
                parameters=[
                    {
                        "backend_url": backend_url,
                        "api_token": api_token,
                        "scan_poll_period": 0.05,
                        "lidar_x": 0.0,
                        "lidar_y": 0.0,
                        "lidar_z": 0.16,
                        "lidar_yaw": 0.0,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                condition=IfCondition(enable_slam),
                launch_arguments={
                    "slam_params_file": slam_params,
                    "use_sim_time": "false",
                    "autostart": "true",
                    "use_lifecycle_manager": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                condition=IfCondition(enable_nav2),
                launch_arguments={
                    "params_file": nav2_params,
                    "use_sim_time": "false",
                    "autostart": "true",
                    "use_composition": "False",
                    "use_respawn": "False",
                }.items(),
            ),
        ]
    )
