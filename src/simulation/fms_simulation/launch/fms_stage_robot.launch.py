"""Launch the single-robot controller helper used in Stage-based experiments."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import NotSubstitution


def generate_launch_description():
    simulation_dir = get_package_share_directory("fms_simulation")
    use_multi_robot_controller = "false"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_dir, "launch", "single_robot_controller.launch.py")
            ),
            launch_arguments={
                "max_v": "0.26",
                "max_w": "1.82",
                "run_stage": "true",
            }.items(),
            condition=IfCondition(NotSubstitution(use_multi_robot_controller)),
        ),
    ])
