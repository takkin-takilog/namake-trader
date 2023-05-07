# pylint: disable=E0401,E0611
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:

    USE_SIM_TIME = "use_sim_time"

    # Get the launch directory
    current_dir = get_package_share_directory("trade_bringup")
    launch_dir = os.path.join(current_dir, "launch")

    # LaunchConfiguration
    use_sim_time = LaunchConfiguration(USE_SIM_TIME)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        USE_SIM_TIME, default_value="false", description="Use simulation clock if true"
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "api_server_oanda_launch.py")
                ),
                launch_arguments={USE_SIM_TIME: use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "system_trade_launch.py")
                ),
                launch_arguments={USE_SIM_TIME: use_sim_time}.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to launch all of nodes
    ld.add_action(bringup_cmd_group)

    return ld
