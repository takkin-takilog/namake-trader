# pylint: disable=E0401,E0611
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:

    # ---------- Your OANDA Account Number and Access Token ----------
    # Demo Account
    PRAC_ACCOUNT_NUMBER = "***-***-*******-***"
    PRAC_ACCESS_TOKEN = "*************************-*************************"
    # Live Account
    LIVE_ACCOUNT_NUMBER = "***-***-*******-***"
    LIVE_ACCESS_TOKEN = "*************************-*************************"

    # Directory
    PARAM_DIR = "params"
    # YAML file
    YAML_API_SERVER = "api_server_oanda.yaml"
    # Variable name
    VN_USE_SIM_TIME = "use_sim_time"
    VN_LOG_LEVEL = "log_level"
    VN_USE_ENV_LIVE = "use_env_live"
    VN_ENV_PRAC_ACCOUNT_NUMBER = "env_practice.account_number"
    VN_ENV_PRAC_ACCESS_TOKEN = "env_practice.access_token"
    VN_ENV_LIVE_ACCOUNT_NUMBER = "env_live.account_number"
    VN_ENV_LIVE_ACCESS_TOKEN = "env_live.access_token"
    VN_CONNECTION_TIMEOUT = "connection_timeout"
    VN_API_SERVER_PARAMS = "api_server_oanda_params"

    # Get the launch directory
    current_dir = get_package_share_directory("trade_bringup")

    # Create the launch configuration variables
    lc_use_sim_time = LaunchConfiguration(VN_USE_SIM_TIME)
    lc_log_level = LaunchConfiguration(VN_LOG_LEVEL)
    lc_use_env_live = LaunchConfiguration(VN_USE_ENV_LIVE)
    lc_env_prac_account_number = LaunchConfiguration(VN_ENV_PRAC_ACCOUNT_NUMBER)
    lc_env_prac_access_token = LaunchConfiguration(VN_ENV_PRAC_ACCESS_TOKEN)
    lc_env_live_account_number = LaunchConfiguration(VN_ENV_LIVE_ACCOUNT_NUMBER)
    lc_env_live_access_token = LaunchConfiguration(VN_ENV_LIVE_ACCESS_TOKEN)
    lc_connection_timeout = LaunchConfiguration(VN_CONNECTION_TIMEOUT)
    lc_api_server_yaml = LaunchConfiguration(VN_API_SERVER_PARAMS)

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

    declare_use_env_live_cmd = DeclareLaunchArgument(
        VN_USE_ENV_LIVE,
        default_value="false",
        description="Whether to use Live environment",
        choices=["true", "false"],
    )

    declare_env_practice_account_number_cmd = DeclareLaunchArgument(
        VN_ENV_PRAC_ACCOUNT_NUMBER,
        default_value=PRAC_ACCOUNT_NUMBER,
        description="Your demo account number of Oanda",
    )

    declare_env_practice_access_token_cmd = DeclareLaunchArgument(
        VN_ENV_PRAC_ACCESS_TOKEN,
        default_value=PRAC_ACCESS_TOKEN,
        description="Your demo access token of Oanda",
    )

    declare_env_live_account_number_cmd = DeclareLaunchArgument(
        VN_ENV_LIVE_ACCOUNT_NUMBER,
        default_value=LIVE_ACCOUNT_NUMBER,
        description="Your live account number of Oanda",
    )

    declare_env_live_access_token_cmd = DeclareLaunchArgument(
        VN_ENV_LIVE_ACCESS_TOKEN,
        default_value=LIVE_ACCESS_TOKEN,
        description="Your live access token of Oanda",
    )

    declare_connection_timeout_cmd = DeclareLaunchArgument(
        VN_CONNECTION_TIMEOUT,
        default_value="10",
        description="Connection timeout time [sec]",
    )

    declare_api_server_params_cmd = DeclareLaunchArgument(
        VN_API_SERVER_PARAMS,
        default_value=os.path.join(current_dir, PARAM_DIR, YAML_API_SERVER),
        description="",
    )

    load_nodes = GroupAction(
        actions=[
            SetParameter(VN_USE_SIM_TIME, lc_use_sim_time),
            Node(
                package="api_server_oanda",
                executable="pricing_publisher",
                name="pricing_publisher",
                parameters=[
                    {VN_USE_ENV_LIVE: lc_use_env_live},
                    {VN_ENV_PRAC_ACCOUNT_NUMBER: lc_env_prac_account_number},
                    {VN_ENV_PRAC_ACCESS_TOKEN: lc_env_prac_access_token},
                    {VN_ENV_LIVE_ACCOUNT_NUMBER: lc_env_live_account_number},
                    {VN_ENV_LIVE_ACCESS_TOKEN: lc_env_live_access_token},
                    {VN_CONNECTION_TIMEOUT: lc_connection_timeout},
                    lc_api_server_yaml,
                ],
                remappings=None,
                ros_arguments=[
                    "--log-level",
                    ["pricing_publisher:=", lc_log_level],
                ],
                output="screen",
            ),
            Node(
                package="api_server_oanda",
                executable="api_server",
                name="api_server",
                parameters=[
                    {VN_USE_ENV_LIVE: lc_use_env_live},
                    {VN_ENV_PRAC_ACCOUNT_NUMBER: lc_env_prac_account_number},
                    {VN_ENV_PRAC_ACCESS_TOKEN: lc_env_prac_access_token},
                    {VN_ENV_LIVE_ACCOUNT_NUMBER: lc_env_live_account_number},
                    {VN_ENV_LIVE_ACCESS_TOKEN: lc_env_live_access_token},
                    {VN_CONNECTION_TIMEOUT: lc_connection_timeout},
                    lc_api_server_yaml,
                ],
                remappings=None,
                ros_arguments=[
                    "--log-level",
                    ["api_server:=", lc_log_level],
                ],
                output="screen",
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_env_live_cmd)
    ld.add_action(declare_env_practice_account_number_cmd)
    ld.add_action(declare_env_practice_access_token_cmd)
    ld.add_action(declare_env_live_account_number_cmd)
    ld.add_action(declare_env_live_access_token_cmd)
    ld.add_action(declare_connection_timeout_cmd)
    ld.add_action(declare_api_server_params_cmd)

    # Add the actions to launch all of the nodes
    ld.add_action(load_nodes)

    return ld
