"""robot3 비전 노드 실행 (PC4): 후방 가이드 추적(웹캠 + UniDepthV2), 전방 짐 추종 추적(OAK-D).

    ros2 launch robot3_control robot3_vision.launch.py

모델 경로, 추론 장치, OpenCV 창 표시는 환경변수로 바꾼다(각 노드 상단 주석 참고).
  AURA_GUIDE_YOLO_MODEL, AURA_GUIDE_YOLO_DEVICE, AURA_UNIDEPTH_MODEL_ID,
  AURA_LUGGAGE_YOLO_MODEL, AURA_LUGGAGE_YOLO_DEVICE, AURA_ENABLE_GUI
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="robot3_control", executable="guide_rear_tracker_node", output="screen"),
        Node(package="robot3_control", executable="luggage_rgbd_tracker_node", output="screen"),
    ])
