"""Launch the fleet manager together with the robot-side controller stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import NotSubstitution


def generate_launch_description():
    robot_manager_dir = get_package_share_directory("robot_manager")
    server_dir = get_package_share_directory("fms_server")
    navigation_dir = get_package_share_directory("ns_turtlebot3_navigation2")

    room = "testroom_demo"
    robot_count = "4"
    run_rhcr = "false"
    directed_graph = "false"
    remove_stations = "false"
    load_state_tables = "false"
    distance_between_nodes = "0.8"
    display_scale = "0.3"
    use_multi_robot_controller = "false"

    rviz_config = os.path.join(server_dir, "cfg", "rviz", f"{room}.rviz")
    topological_map = os.path.join(server_dir, "cfg", "graph", room, "topological_map.yaml")
    task_list = os.path.join(server_dir, "cfg", "task_list", room, "task_list.csv")
    task_table = os.path.join(server_dir, "cfg", "state_table", room, "task_table.yaml")
    robot_table = os.path.join(server_dir, "cfg", "state_table", room, "robot_table.yaml")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_manager_dir, "launch", "multi_robot_controller.launch.py")
            ),
            launch_arguments={
                "nr_of_robots": robot_count,
            }.items(),
            condition=IfCondition(use_multi_robot_controller),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_manager_dir, "launch", "single_robot_controller.launch.py")
            ),
            launch_arguments={
                "max_v": "0.3",
                "max_w": "0.2",
            }.items(),
            condition=IfCondition(NotSubstitution(use_multi_robot_controller)),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_dir, "launch", "ns_turtlebot3_navigation2.launch.py")
            ),
        ),
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
                "run_RHCR": run_rhcr,
                "directed_graph": directed_graph,
                "remove_stations": remove_stations,
                "nr_of_robots": robot_count,
                "load_state_tables": load_state_tables,
                "dist_bw_nodes": distance_between_nodes,
                "display_scale": display_scale,
            }.items(),
        ),
    ])
