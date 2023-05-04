# pylint: disable=E0401,E0611
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    PARAM_DIR = "params"
    API_SERVER_YAML = "api_server_oanda.yaml"
    API_SERVER_PARAMS = "api_server_oanda_params"

    # Get the launch directory
    current_dir = get_package_share_directory("trade_bringup")

    # LaunchConfiguration
    api_server_yaml = LaunchConfiguration(API_SERVER_PARAMS)

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            API_SERVER_PARAMS,
            default_value=os.path.join(current_dir, PARAM_DIR, API_SERVER_YAML),
            description="",
        )
    )

    ld.add_action(
        Node(
            package="api_server_oanda",
            executable="pricing_publisher",
            name="pricing_publisher",
            parameters=[api_server_yaml],
            remappings=None,
            output="screen",
        )
    )

    ld.add_action(
        Node(
            package="api_server_oanda",
            executable="api_server",
            name="api_server",
            parameters=[api_server_yaml],
            remappings=None,
            output="screen",
        )
    )

    return ld
