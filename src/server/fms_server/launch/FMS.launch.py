"""Launch the core fleet manager node with explicit runtime parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _declare_argument(name: str, default_value: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default_value)


def generate_launch_description():
    argument_defaults = {
        "filename_topolomap": "",
        "filename_task_list": "",
        "filename_task_table": "",
        "filename_robot_table": "",
        "nr_of_robots": "4",
        "run_RHCR": "false",
        "directed_graph": "false",
        "remove_stations": "false",
        "load_state_tables": "false",
        "dist_bw_nodes": "1.5",
        "display_scale": "1.0",
        "test_mode": "-1",
        "filename_result": "",
        "fms_mode": "1",
        "optimizer": "1",
    }
    launch_arguments = {
        name: LaunchConfiguration(name) for name in argument_defaults
    }

    return LaunchDescription([
        *[_declare_argument(name, default) for name, default in argument_defaults.items()],
        Node(
            package="fms_core",
            executable="fms_core_node",
            name="fms",
            output="both",
            parameters=[launch_arguments],
        ),
    ])
