import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("highlevel_planning_ros2"),
        "predicate_learning.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="highlevel_planning_ros2",
                executable="run_server",
                name="simserver_ros",
                # parameters=[config],
                output={"stdout": "screen", "stderr": "screen"},
            ),
            Node(
                package="highlevel_planning_ros2",
                executable="run_gui",
                name="predicate_gui",
                output={"stdout": "screen", "stderr": "screen"},
            ),
        ]
    )
