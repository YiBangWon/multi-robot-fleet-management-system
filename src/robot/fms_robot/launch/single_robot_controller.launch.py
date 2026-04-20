"""Launch configuration for single robot controller.launch."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    print("single_robot_controller")

    launch_dir = get_package_share_directory('fms_server')

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

    run_stage = LaunchConfiguration('run_stage')
    run_stage_arg = DeclareLaunchArgument('run_stage', default_value='false')

    return LaunchDescription([
        is_pure_rotation_arg, max_v_arg, max_w_arg,run_stage_arg,
        # Node(
        #     package='fms_ctrl', executable='fms_single_ctrl_node', name='single_robot_controller0',  output='both',
        #     parameters=[{'robot_name': 'robot_0', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w,}],
        # ),
        Node(
            package='robot_manager', executable='fms_single_ctrl_node', name='robot_manager',  output='both',
            parameters=[{'robot_name': 'carter1', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w,'run_stage': run_stage,}],
        ),
        # Node(
        #     package='fms_ctrl', executable='fms_single_ctrl_node', name='single_robot_controller2',  output='both',
        #     parameters=[{'robot_name': 'carter2', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w,}],
        # ),
        # Node(
        #     package='fms_ctrl', executable='fms_single_ctrl_node', name='single_robot_controller3',  output='both',
        #     parameters=[{'robot_name': 'robot_3', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w,}],
        # ),
        # Node(
        #     package='fms_ctrl', executable='fms_single_ctrl_node', name='single_robot_controller4',  output='both',
        #     parameters=[{'robot_name': 'robot_4', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w,}],
        # ),
        # Node(
        #     package='fms_ctrl', executable='fms_single_ctrl_node', name='single_robot_controller5',  output='both',
        #     parameters=[{'robot_name': 'robot_5', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w,}],
        # ),
        # Node(
        #     package='fms_ctrl', executable='fms_single_ctrl_node', name='single_robot_controller6',  output='both',
        #     parameters=[{'robot_name': 'robot_6', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w,}],
        # ),
        # Node(
        #     package='fms_ctrl', executable='fms_single_ctrl_node', name='single_robot_controller7',  output='both',
        #     parameters=[{'robot_name': 'robot_7', 'is_pure_rotation': is_pure_rotation, 'max_v': max_v, 'max_w': max_w,}],
        # ),
    ])
