# pylint: disable=E0401,E0611
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_node_app_bb(inst: str, gran: str, param_yaml: LaunchConfiguration) -> Node:

    pricing = "pricing_" + inst
    latest_candle = inst + "_" + gran + "_latest_candle"

    return Node(
        package="trade_app",
        executable="app_bb",
        name="app_bb",
        namespace=inst + "/" + gran,
        parameters=[param_yaml],
        remappings=[
            ("order_register", "/order_register"),
            ("trade_close_request", "/trade_close_request"),
            ("candles_by_length", "/candles_by_length"),
            (pricing, "/" + pricing),
            ("heart_beat", "/heart_beat"),
            (latest_candle, "/" + latest_candle),
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:

    PARAM_DIR = "params"
    TRADE_APP_BB_YAML = "trade_app_bb.yaml"
    TRADE_MANAGER_YAML = "trade_manager.yaml"
    TRADE_APP_BB_PARAMS = "trade_app_bb_params"
    TRADE_MANAGER_PARAMS = "trade_manager_params"

    # Get the launch directory
    current_dir = get_package_share_directory("trade_bringup")

    # LaunchConfiguration
    trade_app_bb_yaml = LaunchConfiguration(TRADE_APP_BB_PARAMS)
    trade_manager_yaml = LaunchConfiguration(TRADE_MANAGER_PARAMS)

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            TRADE_APP_BB_PARAMS,
            default_value=os.path.join(current_dir, PARAM_DIR, TRADE_APP_BB_YAML),
            description="",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            TRADE_MANAGER_PARAMS,
            default_value=os.path.join(current_dir, PARAM_DIR, TRADE_MANAGER_YAML),
            description="",
        )
    )

    ld.add_action(create_node_app_bb("usdjpy", "m1", trade_app_bb_yaml))
    ld.add_action(create_node_app_bb("eurusd", "h1", trade_app_bb_yaml))

    ld.add_action(
        Node(
            package="trade_manager",
            executable="candles_store",
            name="candles_store",
            parameters=[trade_manager_yaml],
            remappings=None,
            output="screen",
        )
    )

    ld.add_action(
        Node(
            package="trade_manager",
            executable="order_scheduler",
            name="order_scheduler",
            parameters=[trade_manager_yaml],
            remappings=None,
            output="screen",
        )
    )

    return ld
