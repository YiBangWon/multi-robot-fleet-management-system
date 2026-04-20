"""Compose RViz and the fleet manager into a single server-side launch file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _declare_argument(name: str, default_value: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default_value)


def generate_launch_description():
    server_dir = get_package_share_directory("fms_server")
    argument_defaults = {
        "filename_rviz": "",
        "filename_topolomap": "",
        "filename_task_list": "",
        "filename_task_table": "",
        "filename_robot_table": "",
        "nr_of_robots": "4",
        "run_RHCR": "false",
        "run_stage": "false",
        "directed_graph": "false",
        "remove_stations": "false",
        "load_state_tables": "false",
        "dist_bw_nodes": "1.5",
        "display_scale": "1.0",
        "test_mode": "-1",
        "filename_result": "/",
        "fms_mode": "1",
        "optimizer": "1",
        "launch_rviz": "true",
        "rviz_use_sim_time": "false",
    }
    launch_arguments = {
        name: LaunchConfiguration(name) for name in argument_defaults
    }

    return LaunchDescription([
        *[_declare_argument(name, default) for name, default in argument_defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(server_dir, "launch", "rviz.launch.py")),
            launch_arguments={
                "filename_rviz": launch_arguments["filename_rviz"],
                "use_sim_time": launch_arguments["rviz_use_sim_time"],
            }.items(),
            condition=IfCondition(launch_arguments["launch_rviz"]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(server_dir, "launch", "FMS.launch.py")),
            launch_arguments={
                "filename_topolomap": launch_arguments["filename_topolomap"],
                "filename_task_list": launch_arguments["filename_task_list"],
                "filename_task_table": launch_arguments["filename_task_table"],
                "filename_robot_table": launch_arguments["filename_robot_table"],
                "nr_of_robots": launch_arguments["nr_of_robots"],
                "run_RHCR": launch_arguments["run_RHCR"],
                "directed_graph": launch_arguments["directed_graph"],
                "remove_stations": launch_arguments["remove_stations"],
                "load_state_tables": launch_arguments["load_state_tables"],
                "dist_bw_nodes": launch_arguments["dist_bw_nodes"],
                "display_scale": launch_arguments["display_scale"],
                "test_mode": launch_arguments["test_mode"],
                "filename_result": launch_arguments["filename_result"],
                "fms_mode": launch_arguments["fms_mode"],
                "optimizer": launch_arguments["optimizer"],
            }.items(),
        ),
    ])
