#!/usr/bin/env python3
"""Source file for multi nav.launch."""

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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()
    robot_spawn_interval = 4.0
    bringup_offset = 1.5

    # Names and poses of the robots
    robots = [
        {'name': 'carter1', 'x_pose': '-0.59625', 'y_pose': '-5.4481', 'z_pose': 0.01},
        {'name': 'carter2', 'x_pose': '0.79125', 'y_pose': '-5.4294', 'z_pose': 0.01},
        {'name': 'carter3', 'x_pose': '1.9538', 'y_pose': '-5.4294', 'z_pose': 0.01},
        {'name': 'carter4', 'x_pose': '3.2475', 'y_pose': '-5.4106', 'z_pose': 0.01},
        {'name': 'carter5', 'x_pose': '4.5225', 'y_pose': '-5.3356', 'z_pose': 0.01},
        {'name': 'carter6', 'x_pose': '5.8538', 'y_pose': '-5.3731', 'z_pose': 0.01},
        {'name': 'carter7', 'x_pose': '7.0913', 'y_pose': '-5.2981', 'z_pose': 0.01},
        {'name': 'carter8', 'x_pose': '8.3475', 'y_pose': '-5.2419', 'z_pose': 0.01},
        # {'name': 'carter5', 'x_pose': '-5.3', 'y_pose': '-8.7344', 'z_pose': 0.01},
        # {'name': 'carter6', 'x_pose': '-3.65', 'y_pose': '-9.8781', 'z_pose': 0.01},
        # ...
        # ...
    ]
    initial_pose_offset = len(robots) * robot_spawn_interval + 8.0

    TURTLEBOT3_MODEL = 'waffle'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='false')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    enable_gzclient = LaunchConfiguration('enable_gzclient', default='true')
    declare_enable_gzclient = DeclareLaunchArgument(
        name='enable_gzclient',
        default_value=enable_gzclient,
        description='Enable the Gazebo client UI.'
    )

    spawn_delay = LaunchConfiguration('spawn_delay', default='5.0')
    declare_spawn_delay = DeclareLaunchArgument(
        name='spawn_delay',
        default_value=spawn_delay,
        description='Seconds to wait before spawning the first robot.'
    )

    package_dir = get_package_share_directory('fms_gazebo')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    urdf = os.path.join(
        package_dir, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf')

    world = os.path.join(
        package_dir, 'worlds', 'ssh.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(enable_gzclient),
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_enable_gzclient)
    ld.add_action(declare_spawn_delay)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    map_server = Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('fms_simulation'), 'cfg/map/ssh','map_image.yaml'),
                     },],
        remappings=remappings)

    map_server_lifecycle = Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])


    ld.add_action(map_server)
    ld.add_action(map_server_lifecycle)

    ######################

    # Remapping is required for state publisher otherwise /tf and /tf_static
    # will get be published on root '/' namespace
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    # Stagger the robot bringup so a slow /spawn_entity response does not block the rest.
    for index, robot in enumerate(robots):

        namespace = '/' + robot['name']
        spawn_time = PythonExpression([spawn_delay, ' + ', str(index * robot_spawn_interval)])
        bringup_time = PythonExpression([spawn_delay, ' + ', str(index * robot_spawn_interval + bringup_offset)])

        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                            'publish_frequency': 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )

        # Create spawn call
        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(package_dir, 'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', '0.01', '-Y', '0.0',
                '-unpause',
            ],
            output='screen',
        )

        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_dir, 'launch', 'nav2_bringup', 'bringup_launch.py')),
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

        ld.add_action(
            TimerAction(
                period=spawn_time,
                actions=[turtlebot_state_publisher, spawn_turtlebot3_burger],
            )
        )
        ld.add_action(
            TimerAction(
                period=bringup_time,
                actions=[bringup_cmd],
            )
        )
    ######################

    ######################
    # Publish initial poses after the Nav2 stacks have had time to come up.
    for index, robot in enumerate(robots):

        namespace = '/' + robot['name']
        initial_pose_time = PythonExpression(
            [spawn_delay, ' + ', str(initial_pose_offset + index * 0.5)]
        )

        # Create a initial pose topic publish call
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'nav2_bringup', 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time,
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )

        drive_turtlebot3_burger = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
        )

        ld.add_action(
            TimerAction(
                period=initial_pose_time,
                actions=[initial_pose_cmd, rviz_cmd, drive_turtlebot3_burger],
            )
        )
    ######################
    return ld
