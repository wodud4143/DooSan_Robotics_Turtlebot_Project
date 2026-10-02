"""robot1 미션 FSM 실행 (PC3): /robot1 네임스페이스, AED 탑재.

    ros2 launch robot1_control robot1.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "aed_loaded", default_value="true",
            description="AED 탑재 여부(true인 로봇만 응급 배차 대상)",
        ),
        Node(
            package="robot1_control",
            executable="robot1_mission_fsm_node",
            namespace="robot1",
            output="screen",
            parameters=[{
                "aed_loaded": ParameterValue(LaunchConfiguration("aed_loaded"), value_type=bool),
            }],
        ),
    ])
