# AURA 아키텍처

AURA는 공항형 환경에서 TurtleBot4 2대(**robot1**, **robot3**)를 중앙 노드로 조율하여
**안내 / 짐 운반 / 순찰 / 응급 대응** 역할을 수행하는 멀티 AMR 시스템이다.

시스템은 4개 계층과 1개 공용 인터페이스 패키지로 구성된다.

```
① UI 계층        admin_ui(7000) · passenger_ui(5001/5003)   ← Flask
② 중앙 계층      fleet_dispatcher · db_manager · crowd_keepout_mask
③ 로봇 계층      robot1_control · robot3_control
④ 인지 계층      cctv_perception (쓰러짐·혼잡·인원수)
공용 인터페이스  fleet_interfaces (msg / srv / action)
```

---

## 공용 인터페이스 — `fleet_interfaces`

| 종류 | 이름 | 요약 |
|---|---|---|
| msg | `RobotStatus` | robot_id, mode(7종), battery_pct, pose, `aed_loaded`, available |
| action | `ExecuteMission` | (mode, dest_x, dest_y, priority) → (success, final_state) / feedback(current_mode, progress_pct) |
| srv | `EmergencyStop` | reason → success |

`RobotStatus.mode`: `IDLE(0) · GUIDE(1) · LUGGAGE_ASSIST(2) · EMERGENCY_DISPATCH(3) · ERROR(4) · CHARGING(5) · HOTPLACE_DISPATCH(6)`

---

## ② 중앙 계층 — `fleet_central`

- **`fleet_dispatcher_node`** — 미션 배차.
  - 승객 UI 이벤트(`/aura/service_request`, `/aura/service_end`, `/aura/robot_select`)를 받아
    GUIDE / LUGGAGE_ASSIST goal을 보낸다. AMR1→robot1, AMR3→robot3 고정 매핑이고,
    목적지는 `goal_id`(goal1_1, goal1_2, goal_2)를 `GOAL_TABLE` 좌표로 바꿔 전달한다.
  - `/fall_detection`과 `/fall_detection_point`가 모두 들어오면 응급 이벤트를 1회 생성하고,
    `RobotStatus.aed_loaded==True`인 로봇(가장 먼저 상태를 보고한 로봇)에 EMERGENCY_DISPATCH goal을 보낸다.
  - 우선순위: EMERGENCY 10, GUIDE 5, LUGGAGE_ASSIST 5, HOTPLACE 3. 선점 여부는 로봇 측 `goal_cb`가 판단한다.
  - robot3 GUIDE는 승객 정렬 후 180° 회전이 끝난(`/turn_complete=True`) 다음에 goal을 보낸다.
  - 미션 결과를 `/aura/arrival_status`(JSON)로 UI에 알린다.
- **`db_manager_node`** — 주요 토픽 16개를 구독해 SQLite(WAL, 7개 테이블)에 기록한다.
  쓰러짐 확정 이미지는 파일로 저장하고 경로를 `event_images`에 남긴다. admin UI가 이 DB를 읽는다.
- **`crowd_keepout_mask_node`** — `/hot_place`와 `/robot1/map`으로 Nav2 Keepout Filter 마스크를 만든다.
  혼잡구역 중심 반경 0.4 m 셀을 100(진입 금지)으로 채우고, 빈 배열이 오면 마스크를 비운다(robot1 전용).

---

## ③ 로봇 계층

### robot1 — `robot1_control` (응급/AED 대응)

단일 노드 `robot1_mission_fsm_node`(`/robot1` 네임스페이스)가 11개 상태의 FSM을 담당한다.

- `ExecuteMission` 액션 서버, `EmergencyStop` 서비스 서버, `RobotStatus` 1 Hz 발행.
- 6개 좌표 무한 순찰 → GUIDE / HOTPLACE_DISPATCH / EMERGENCY_DISPATCH 처리 → 순찰 복귀.
- 응급: 이동 중 4초마다 경고음 → 도착 후 경고음 + 제자리 회전(4초) → `/rgbd_fall_person_point` 최종 접근
  (10초 내 미수신 시 현장 대기) → `/emergency_end` 수신 시 순찰 복귀.
- 승객 UI와 `/rb1_standby`, `/rb1_service_end`, `/emergency_end`(Bool)로 직접 연결된다.
- Nav2 keepout filter를 `/robot1/keepout_costmap_filter_info`로 설정하면 혼잡구역을 우회한다.

### robot3 — `robot3_control` (안내/짐 운반)

`robot3_mission_fsm_node`(`/robot3` 네임스페이스)가 `ExecuteMission` 액션 서버와 `EmergencyStop` 서비스를 제공하고,
GUIDE / LUGGAGE_ASSIST의 실제 주행은 로컬 인지·모션 노드에 위임한다.

| 노드 | 역할 | 핵심 토픽 |
|---|---|---|
| `guide_rear_tracker_node` | 안내용 **후방 웹캠** 사람 추적 (YOLO11s + DeepOCSORT, UniDepthV2 단안 깊이) | sub `/image_raw/compressed`, `/turn_complete` → pub `/tracking_web`, `/tracking_web_center_pixel`, `/target_depth` |
| `guide_motion_node` | 안내 주행: 180° 회전, 추적 조건부 Nav2 이동 | sub `/robot3/mission_cmd`, `/turn_around`, 추적 3종 → pub `/turn_complete`, `/robot3/mission_complete`, `/robot3/cmd_vel` |
| `luggage_rgbd_tracker_node` | 짐 운반용 **전방 OAK-D RGB-D** 사람 추적 (YOLO11s-pose, 발목 keypoint depth) | sub `/carrying`, `/service_end` → pub `/tracking_rgbd`, `/tracking_rgbd_center_pixel`, `/target_depth_rgbd` |
| `luggage_follower_node` | 짐 운반 추종 주행 (P 제어 회전 + 거리 기반 속도) | sub 추적 3종, `/carrying` → pub `/robot3/cmd_vel`, `/front_start`, `/robot3/mission_complete` |
| `robot3_patrol_node` | 6개 좌표 순찰, 혼잡구역 접근, 대기 핸드셰이크 | sub `/hot_place`, `/start_patrol`, `/rb3_standby` → pub `/rb3_standby_done` |
| `robot3_mission_fsm_node` | 중앙 goal 처리, GUIDE 위임, HOTPLACE·응급 이동 | ExecuteMission 서버, pub `/robot3/mission_cmd`, `/rb3_standby`, `/service_cancel` |

> 방향 규칙: **후방 카메라 = 안내**(로봇이 앞장서고 승객이 뒤따름 → `tracking_web`),
> **전방 카메라 = 짐 운반**(승객이 앞장서고 로봇이 따라감 → `tracking_rgbd`).

제어권은 토픽으로 넘긴다. 순찰은 `/rb3_standby`로 멈추고 `/start_patrol`로 다시 시작하며,
안내 주행은 `/turn_complete` 이후에만 움직이고, 짐 추종이 시작되면(`/front_start=1`) 후방 추적과 안내 주행이 잠긴다.

---

## ④ 인지 계층 — `cctv_perception`

고정 CCTV 노드 `cctv_detector_node` (YOLO11x-pose + DeepOCSORT):

- 쓰러짐 감지 → `/fall_detection`(Bool, 매 프레임) + `/fall_detection_point`(PointStamped, 확정 동안)
- 인원수 → `/population`(Int32)
- 혼잡구역 → `/hot_place`(PoseArray, 1.0 m 이내로 이어진 2명 이상 무리의 중심)
- 영상 → `/cctv/image_raw/compressed`(오버레이 없는 원본 JPEG, 최대 10 FPS),
  확정 순간 오버레이 이미지 1회 → `/cctv/fall_detection_image`, `/cctv/hot_place_image`(sensor_msgs/Image)

픽셀 좌표는 바닥 6점으로 맞춘 Homography로 map 좌표로 변환한다. 이벤트는 2초 연속 유지되면 확정한다.

---

## ① UI 계층

### admin_ui (포트 7000)

관제 대시보드. `camera_bridge`가 CCTV 압축 이미지와 `/population`을 구독해 MJPEG·SSE로 내보내고,
`database`가 `db_manager`의 SQLite를 읽기 전용으로 조회한다. ROS 없이 DB 전용 모드로도 실행된다.

### passenger_ui (AMR1 포트 5001 / AMR3 포트 5003)

승객 키오스크. Flask app-factory 구조이며 로봇별 프로파일로 통신 방식이 다르다.

| | AMR1 (robot1) | AMR3 (robot3) |
|---|---|---|
| 포트 | 5001 | 5003 |
| 정렬 모드 | `manual` (카메라 정렬 없음) | `tracking_int32` (비전 노드 Int32 사용) |
| 중앙 통신 | `/aura/*` String(JSON) | `/aura/*` String(JSON) |
| 로봇 직접 명령 | `/rb1_standby`·`/rb1_service_end`·`/emergency_end` Bool | `/rb3_standby`·`/service_end`·`/carrying`·`/turn_around` Bool |

`run_all_ui.py`로 두 UI를 동시에 실행한다. 상세 토픽은 [topics.md](topics.md)를 참고한다.
