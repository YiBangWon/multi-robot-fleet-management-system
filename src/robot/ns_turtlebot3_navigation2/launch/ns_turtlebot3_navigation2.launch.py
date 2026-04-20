#!/usr/bin/env python3
"""Launch configuration for ns turtlebot3 navigation2.launch."""

#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Arshad Mehmood

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()

    TURTLEBOT3_MODEL = 'burger'
    namespace = 'robot_3'

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    ns_turtlebot3_navigation2 = get_package_share_directory('ns_turtlebot3_navigation2')

    package_dir = get_package_share_directory('ns_turtlebot3_navigation2')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    urdf = os.path.join(
        ns_turtlebot3_navigation2, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')


    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map_test.yaml'),
                     },],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])

    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    turtlebot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                        'publish_frequency': 10.0}],
        remappings=remappings,
        arguments=[urdf],
    )

    bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'bringup_launch.py')),
                launch_arguments={
                                'slam': 'False',
                                'namespace': namespace,
                                'use_namespace': 'True',
                                'map': '',
                                'map_server': 'False',
                                'params_file': params_file,
                                'default_bt_xml_filename': os.path.join(
                                    get_package_share_directory('nav2_bt_navigator'),
                                    'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                'autostart': 'true',
                                'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                                )

    ld.add_action(turtlebot_state_publisher)
    ld.add_action(bringup_cmd)

    message = '\'{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: -3.3325, y: -0.6925, z: 0.0}, orientation: {w: 0.1}}, } }\''

    initial_pose_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', namespace + '/initialpose', 'geometry_msgs/PoseWithCovarianceStamped', message],
        output='screen'
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'rviz_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': namespace,
                              'use_namespace': 'True',
                              'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                               condition=IfCondition(enable_rviz)
                                )

    #ld.add_action(initial_pose_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_params_file_cmd)

    return ld
