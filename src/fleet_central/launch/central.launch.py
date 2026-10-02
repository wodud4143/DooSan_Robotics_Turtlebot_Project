"""중앙 계층 실행 (PC2): 미션 배차, DB 기록, 혼잡구역 keepout 마스크.

    ros2 launch fleet_central central.launch.py

DB 경로는 환경변수 AURA_DB_PATH로 지정한다(admin_ui와 같은 값을 써야 대시보드에 반영된다).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="fleet_central", executable="fleet_dispatcher_node", output="screen"),
        Node(package="fleet_central", executable="db_manager_node", output="screen"),
        Node(package="fleet_central", executable="crowd_keepout_mask_node", output="screen"),
    ])
