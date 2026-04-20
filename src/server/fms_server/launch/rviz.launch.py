"""Launch RViz with an explicit config file and time source."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    filename_rviz = LaunchConfiguration('filename_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    filename_rviz_arg = DeclareLaunchArgument('filename_rviz', default_value='')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    return LaunchDescription([
        filename_rviz_arg,
        use_sim_time_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', [filename_rviz]],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
