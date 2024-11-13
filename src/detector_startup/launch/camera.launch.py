from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_launch_node = Node(
        package="camera_node",
        executable='camera_node'
    )
    launch_description = LaunchDescription(
        [camera_launch_node]
    )
    return launch_description