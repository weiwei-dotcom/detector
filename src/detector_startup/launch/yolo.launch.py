from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    yolo_launch_node = Node(
        package="yolo_detector",
        executable='yolo_detector'
    )
    launch_description = LaunchDescription(
        [yolo_launch_node]
    )
    return launch_description