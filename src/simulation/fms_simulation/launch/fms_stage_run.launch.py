"""Launch the fleet manager for Stage-based experiments without controller bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    simulation_dir = get_package_share_directory("fms_simulation")
    server_dir = get_package_share_directory("fms_server")

    room = "kiva"
    robot_count = "51"

    rviz_config = os.path.join(server_dir, "cfg", "rviz", f"{room}.rviz")
    topological_map = os.path.join(server_dir, "cfg", "graph", room, "topological_map.yaml")
    task_list = os.path.join(server_dir, "cfg", "task_list", room, "task_list.csv")
    task_table = os.path.join(server_dir, "cfg", "state_table", room, "task_table.yaml")
    robot_table = os.path.join(server_dir, "cfg", "state_table", room, "robot_table.yaml")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(server_dir, "launch", "fleet_manager.launch.py")
            ),
            launch_arguments={
                "filename_rviz": rviz_config,
                "filename_topolomap": topological_map,
                "filename_task_list": task_list,
                "filename_task_table": task_table,
                "filename_robot_table": robot_table,
                "run_RHCR": "false",
                "run_stage": "true",
                "directed_graph": "false",
                "remove_stations": "false",
                "nr_of_robots": robot_count,
                "load_state_tables": "false",
                "dist_bw_nodes": "0.8",
                "display_scale": "0.3",
                "test_mode": "1",
                "filename_result": os.path.join(simulation_dir, "result"),
                "fms_mode": "0",
                "optimizer": "0",
            }.items(),
        ),
    ])
