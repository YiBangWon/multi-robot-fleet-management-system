"""Launch the Gazebo-based multi-robot logistics demo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution


def generate_launch_description():
    simulation_dir = get_package_share_directory("fms_simulation")
    server_dir = get_package_share_directory("fms_server")

    room = "ssh"
    robot_count = "8"
    run_rhcr = "false"
    directed_graph = "false"
    remove_stations = "false"
    load_state_tables = "false"
    distance_between_nodes = "0.8"
    display_scale = "0.2"
    use_multi_robot_controller = LaunchConfiguration("use_multi_robot_controller")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_gzclient = LaunchConfiguration("enable_gzclient")
    launch_rviz = LaunchConfiguration("launch_rviz")
    spawn_delay = LaunchConfiguration("spawn_delay")

    rviz_config = os.path.join(server_dir, "cfg", "rviz", f"{room}.rviz")
    topological_map = os.path.join(server_dir, "cfg", "graph", room, "topological_map.yaml")
    task_list = os.path.join(server_dir, "cfg", "task_list", room, "task_list.csv")
    task_table = os.path.join(server_dir, "cfg", "state_table", room, "task_table.yaml")
    robot_table = os.path.join(server_dir, "cfg", "state_table", room, "robot_table.yaml")

    result_directory = os.path.join(simulation_dir, "result")
    test_mode = "1"
    fms_mode = "0"
    optimizer = "0"

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_multi_robot_controller",
            default_value="false",
            description="Launch the multi-robot controller instead of one controller per robot.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation time throughout the Gazebo demo.",
        ),
        DeclareLaunchArgument(
            "enable_gzclient",
            default_value="true",
            description="Launch the Gazebo client UI. Disable this for headless servers.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch the server-side RViz dashboard.",
        ),
        DeclareLaunchArgument(
            "spawn_delay",
            default_value="5.0",
            description="Seconds to wait before spawning the first robot in Gazebo.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_dir, "cfg", "map", room, "multi_nav.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "enable_gzclient": enable_gzclient,
                "spawn_delay": spawn_delay,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_dir, "launch", "multi_robot_controller.launch.py")
            ),
            launch_arguments={
                "nr_of_robots": robot_count,
            }.items(),
            condition=IfCondition(use_multi_robot_controller),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_dir, "launch", "single_robot_controller.launch.py")
            ),
            launch_arguments={
                "nr_of_robots": robot_count,
                "max_v": "0.2",
                "max_w": "0.2",
            }.items(),
            condition=IfCondition(NotSubstitution(use_multi_robot_controller)),
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
                "test_mode": test_mode,
                "filename_result": result_directory,
                "fms_mode": fms_mode,
                "optimizer": optimizer,
                "launch_rviz": launch_rviz,
                "rviz_use_sim_time": use_sim_time,
            }.items(),
        ),
    ])
