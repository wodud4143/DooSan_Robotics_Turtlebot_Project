"""robot3 주행·미션 노드 실행 (PC4).

    ros2 launch robot3_control robot3.launch.py

순찰 노드와 미션 FSM은 상대 토픽(navigate_to_pose, amcl_pose, mission_cmd 등)을 쓰므로
/robot3 네임스페이스로 실행한다. 나머지 노드는 절대 토픽만 사용한다.
비전 노드는 robot3_vision.launch.py로 따로 실행한다.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="robot3_control", executable="guide_motion_node", output="screen"),
        Node(package="robot3_control", executable="luggage_follower_node", output="screen"),
        Node(
            package="robot3_control",
            executable="robot3_patrol_node",
            namespace="robot3",
            output="screen",
        ),
        Node(
            package="robot3_control",
            executable="robot3_mission_fsm_node",
            namespace="robot3",
            output="screen",
            parameters=[{"aed_loaded": False}],
        ),
    ])
