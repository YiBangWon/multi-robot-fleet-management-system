"""Launch configuration for fms ctrl board.launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    fms_demo_launch_dir = os.path.join(get_package_share_directory('fms_ctrl_board'), 'launch', 'setup_launch')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fms_demo_launch_dir, 'ns_bringup.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fms_demo_launch_dir, 'follow_poses.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fms_demo_launch_dir, 'clear_costmap.launch.py')),
        ),
    ])