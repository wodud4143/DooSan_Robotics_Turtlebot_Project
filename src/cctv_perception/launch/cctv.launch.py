"""CCTV 인지 노드 실행 (PC1).

    ros2 launch cctv_perception cctv.launch.py camera_index:=2 model_path:=yolo11x-pose.pt
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path", default_value="yolo11x-pose.pt",
            description="YOLO pose 가중치 경로",
        ),
        DeclareLaunchArgument(
            "camera_index", default_value="2",
            description="OpenCV 카메라 index(CCTV 웹캠)",
        ),
        Node(
            package="cctv_perception",
            executable="cctv_detector_node",
            output="screen",
            parameters=[{
                "model_path": ParameterValue(LaunchConfiguration("model_path"), value_type=str),
                "camera_index": ParameterValue(LaunchConfiguration("camera_index"), value_type=int),
            }],
        ),
    ])
