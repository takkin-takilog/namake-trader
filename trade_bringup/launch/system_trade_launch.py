import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def create_node_app_bb(
    inst: str,
    gran: str,
    param_yaml: LaunchConfiguration,
    log_level: LaunchConfiguration,
) -> Node:
    namespace = inst + "/" + gran
    pricing = "pricing_" + inst
    latest_candle = inst + "_" + gran + "_latest_candle"

    return Node(
        package="trade_app",
        executable="app_bb",
        name="app_bb",
        namespace=namespace,
        parameters=[param_yaml],
        remappings=[
            ("order_register", "/order_register"),
            ("trade_close_request", "/trade_close_request"),
            ("candles_by_length", "/candles_by_length"),
            (pricing, "/" + pricing),
            ("heart_beat", "/heart_beat"),
            (latest_candle, "/" + latest_candle),
        ],
        ros_arguments=[
            "--log-level",
            [namespace.replace("/", "."), ".app_bb:=", log_level],
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    # Directory
    PARAM_DIR = "params"
    # YAML file
    YAML_TRADE_APP_BB = "trade_app_bb.yaml"
    YAML_TRADE_MANAGER = "trade_manager.yaml"
    # Variable name
    VN_USE_SIM_TIME = "use_sim_time"
    VN_LOG_LEVEL = "log_level"
    VN_TRADE_APP_BB_PARAMS = "trade_app_bb_params"
    VN_TRADE_MANAGER_PARAMS = "trade_manager_params"

    # Get the launch directory
    current_dir = get_package_share_directory("trade_bringup")

    # Create the launch configuration variables
    lc_use_sim_time = LaunchConfiguration(VN_USE_SIM_TIME)
    lc_log_level = LaunchConfiguration(VN_LOG_LEVEL)
    lc_trade_app_bb_yaml = LaunchConfiguration(VN_TRADE_APP_BB_PARAMS)
    lc_trade_manager_yaml = LaunchConfiguration(VN_TRADE_MANAGER_PARAMS)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        VN_USE_SIM_TIME,
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        VN_LOG_LEVEL,
        default_value="info",
        description="Log level",
    )

    declare_trade_app_bb_params_cmd = DeclareLaunchArgument(
        VN_TRADE_APP_BB_PARAMS,
        default_value=os.path.join(current_dir, PARAM_DIR, YAML_TRADE_APP_BB),
        description="Trade-App bollinger-band parameter",
    )

    declare_trade_manager_params_cmd = DeclareLaunchArgument(
        VN_TRADE_MANAGER_PARAMS,
        default_value=os.path.join(current_dir, PARAM_DIR, YAML_TRADE_MANAGER),
        description="Trade-Manager package parameter",
    )

    load_nodes = GroupAction(
        actions=[
            SetParameter(VN_USE_SIM_TIME, lc_use_sim_time),
            Node(
                package="trade_manager",
                executable="candles_store",
                name="candles_store",
                parameters=[lc_trade_manager_yaml],
                remappings=None,
                ros_arguments=[
                    "--log-level",
                    ["candles_store:=", lc_log_level],
                ],
                output="screen",
            ),
            Node(
                package="trade_manager",
                executable="order_scheduler",
                name="order_scheduler",
                parameters=[lc_trade_manager_yaml],
                remappings=None,
                ros_arguments=[
                    "--log-level",
                    ["order_scheduler:=", lc_log_level],
                ],
                output="screen",
            ),
            create_node_app_bb("usdjpy", "m1", lc_trade_app_bb_yaml, lc_log_level),
            create_node_app_bb("eurusd", "h1", lc_trade_app_bb_yaml, lc_log_level),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_trade_app_bb_params_cmd)
    ld.add_action(declare_trade_manager_params_cmd)

    # Add the actions to launch all of the nodes
    ld.add_action(load_nodes)

    return ld
