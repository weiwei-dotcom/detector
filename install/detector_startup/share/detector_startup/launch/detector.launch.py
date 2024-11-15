from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_launch_node = Node(
        package="camera_node",
        executable='camera_node'
    )
    yolo_launch_node = Node(
        package="yolo_detector",
        executable='yolo_detector'
    )
    launch_description = LaunchDescription(
        [camera_launch_node, yolo_launch_node]
    )
    return launch_description