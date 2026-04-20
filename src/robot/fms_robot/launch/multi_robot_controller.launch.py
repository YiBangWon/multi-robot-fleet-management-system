"""Launch configuration for multi robot controller.launch."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    launch_dir = get_package_share_directory('fms_server')

    nr_of_robots = LaunchConfiguration('nr_of_robots')
    nr_of_robots_arg = DeclareLaunchArgument('nr_of_robots', default_value='4')

    is_pure_rotation = LaunchConfiguration('is_pure_rotation')
    is_pure_rotation_arg = DeclareLaunchArgument('is_pure_rotation', default_value='true')

    max_v = LaunchConfiguration('max_v')
    max_v_arg = DeclareLaunchArgument('max_v', default_value='1.0')

    max_w = LaunchConfiguration('max_w')
    max_w_arg = DeclareLaunchArgument('max_w', default_value='0.5')

    robot_model = LaunchConfiguration('robot_model')
    robot_model_arg = DeclareLaunchArgument('robot_model', default_value='robot_model_0')

    param_file = LaunchConfiguration('param_file')  ##########
    param_file_arg = DeclareLaunchArgument('param_file', default_value='1.5') ##########

    node_name = LaunchConfiguration('node_name')
    node_name_arg = DeclareLaunchArgument('node_name', default_value='multi_robot_controller')

    run_stage = LaunchConfiguration('run_stage')
    run_stage_arg = DeclareLaunchArgument('run_stage', default_value='false')

    return LaunchDescription([
        nr_of_robots_arg,
        is_pure_rotation_arg,
        max_v_arg,
        max_w_arg,
        robot_model_arg,
        param_file_arg,
        node_name_arg,
        run_stage_arg,
        Node(
            package='robot_manager',
            executable='fms_ctrl_node',
            name=LaunchConfiguration('node_name'),
            output='both',
            parameters=[{'nr_of_robots': nr_of_robots,
                        'is_pure_rotation': is_pure_rotation,
                        'max_v': max_v,
                        'max_w': max_w,
                        'run_stage': run_stage,
                        }],
        ),
    ])
