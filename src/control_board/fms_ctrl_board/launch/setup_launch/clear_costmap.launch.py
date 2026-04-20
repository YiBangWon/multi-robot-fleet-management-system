"""Launch costmap-clearing helpers for the active control-board namespaces."""

from launch import LaunchDescription
from launch_ros.actions import Node


ROBOT_NAMESPACES = ["robot_0"]


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="clear_costmap",
                executable="clear_costmap",
                name=f"{namespace}_clear_costmap",
                namespace=namespace,
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
            for namespace in ROBOT_NAMESPACES
        ]
    )
