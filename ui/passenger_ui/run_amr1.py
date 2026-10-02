"""AMR1 승객 UI 실행: python3 run_amr1.py

AMR1은 카메라 정렬 없이 수동 정렬(manual)을 사용한다. 중앙과는 String(JSON)으로,
로봇1 즉시 제어(/rb1_standby, /rb1_service_end, /emergency_end)는 Bool(True)로 직접 통신한다.
"""
from launcher import launch_ui


if __name__ == "__main__":
    launch_ui(
        robot_id="AMR1",
        port=5001,
        alignment_input_mode="manual",
    )
