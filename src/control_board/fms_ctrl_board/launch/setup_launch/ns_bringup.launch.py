"""Launch configuration for ns bringup.launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    ns_nav_launch_dir = os.path.join(get_package_share_directory('ns_turtlebot3_navigation2'), 'launch',)
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ns_nav_launch_dir, 'ns_turtlebot3_navigation2.launch.py')),
        ),
    ])