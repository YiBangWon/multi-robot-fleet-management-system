"""Launch only the Stage simulator and optional multi-robot controller composition."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    simulation_dir = get_package_share_directory("fms_simulation")
    use_multi_robot_controller = "false"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_dir, "launch", "stage.launch.py")
            ),
            launch_arguments={
                "filename_stage_world": os.path.join(
                    simulation_dir, "cfg", "map", "kiva", "stage.world"
                ),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_dir, "launch", "multi_robot_controller.launch.py")
            ),
            launch_arguments={
                "nr_of_robots": "50",
                "run_stage": "true",
            }.items(),
            condition=IfCondition(use_multi_robot_controller),
        ),
    ])
