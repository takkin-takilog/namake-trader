# pylint: disable=E0401,E0611
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:

    # Use Live account environment if true
    # Use Demo account environment if false
    use_env_live = False

    # Variable name
    VN_LOG_LEVEL = "log_level"
    VN_USE_ENV_LIVE = "use_env_live"

    # Get the launch directory
    current_dir = get_package_share_directory("trade_bringup")
    launch_dir = os.path.join(current_dir, "launch")

    # Create the launch configuration variables
    lc_log_level = LaunchConfiguration(VN_LOG_LEVEL)
    lc_use_env_live = LaunchConfiguration(VN_USE_ENV_LIVE)

    declare_log_level_cmd = DeclareLaunchArgument(
        VN_LOG_LEVEL,
        default_value="debug",
        description="Log level",
    )

    declare_use_env_live_cmd = DeclareLaunchArgument(
        VN_USE_ENV_LIVE,
        default_value="true" if use_env_live else "false",
        description="Whether to use Live environment",
        choices=["true", "false"],
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "api_server_oanda_launch.py")
                ),
                launch_arguments={
                    VN_LOG_LEVEL: lc_log_level,
                    VN_USE_ENV_LIVE: lc_use_env_live,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "system_trade_launch.py")
                ),
                launch_arguments={
                    VN_LOG_LEVEL: lc_log_level,
                    VN_USE_ENV_LIVE: lc_use_env_live,
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_env_live_cmd)

    # Add the actions to launch all of nodes
    ld.add_action(bringup_cmd_group)

    return ld
