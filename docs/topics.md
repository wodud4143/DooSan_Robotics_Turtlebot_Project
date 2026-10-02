# 토픽 레퍼런스

소스의 `create_publisher` / `create_subscription` / `ActionClient` / `ActionServer` / `create_service` 호출을 기준으로 정리했다.
네임스페이스로 실행하는 노드(robot1·robot3 FSM, robot3 순찰)의 상대 이름은 실제 이름(`/robot1/...`, `/robot3/...`)으로 적었다.

## CCTV 인지 → 중앙·UI

| 토픽 | 타입 | 발행 | 구독 | 비고 |
|---|---|---|---|---|
| `/fall_detection` | `std_msgs/Bool` | cctv_detector | fleet_dispatcher, db_manager, passenger_ui(AMR1) | 매 프레임 발행 |
| `/fall_detection_point` | `geometry_msgs/PointStamped` | cctv_detector | fleet_dispatcher, db_manager | 쓰러짐 확정 동안 매 프레임 (map 좌표) |
| `/population` | `std_msgs/Int32` | cctv_detector | admin_ui, fleet_dispatcher, db_manager | ROI 안 인원수 |
| `/hot_place` | `geometry_msgs/PoseArray` | cctv_detector | crowd_keepout_mask, robot3_patrol, fleet_dispatcher, db_manager | 혼잡구역 중심 (없으면 빈 배열) |
| `/cctv/image_raw/compressed` | `sensor_msgs/CompressedImage` | cctv_detector | admin_ui | 오버레이 없는 원본 JPEG, 최대 10 FPS |
| `/cctv/fall_detection_image` | `sensor_msgs/Image` | cctv_detector | db_manager | 쓰러짐 확정 순간 1회 (오버레이 포함) |
| `/cctv/hot_place_image` | `sensor_msgs/Image` | cctv_detector | – | 혼잡 확정 순간 1회 (오버레이 포함) |

## 승객 UI ↔ 중앙

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/aura/robot_select` | `std_msgs/String`(JSON) | passenger_ui(AMR3) | fleet_dispatcher(로그), db_manager |
| `/aura/service_request` | `std_msgs/String`(JSON) | passenger_ui | fleet_dispatcher, db_manager |
| `/aura/service_end` | `std_msgs/String`(JSON) | passenger_ui | fleet_dispatcher, db_manager |
| `/aura/arrival_status` | `std_msgs/String`(JSON) | fleet_dispatcher | passenger_ui, db_manager |
| `/aura/luggage_load_confirm` | `std_msgs/String`(JSON) | passenger_ui(AMR3) | – |

## 중앙 ↔ 로봇 (robot1, robot3 공통)

| 인터페이스 | 타입 | 서버 / 발행 | 클라이언트 / 구독 |
|---|---|---|---|
| `/robotN/execute_mission` | `fleet_interfaces/action/ExecuteMission` | robotN 미션 FSM | fleet_dispatcher |
| `/robotN/emergency_stop` | `fleet_interfaces/srv/EmergencyStop` | robotN 미션 FSM | fleet_dispatcher |
| `/robotN/status` | `fleet_interfaces/msg/RobotStatus` (1 Hz) | robotN 미션 FSM | fleet_dispatcher, db_manager |
| `/turn_complete` | `std_msgs/Bool` | guide_motion | fleet_dispatcher, guide_motion(자기 수신), guide_rear_tracker, passenger_ui(AMR3) |
| `/service_cancel` | `std_msgs/Bool` | fleet_dispatcher, robot3 미션 FSM | db_manager (로봇 측 구독 없음) |
| `/robot1/population` | `std_msgs/Int32` | fleet_dispatcher (`/population` 중계) | – |

## 혼잡구역 Keepout (robot1 Nav2)

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/robot1/map` | `nav_msgs/OccupancyGrid` | map_server | crowd_keepout_mask |
| `/robot1/keepout_filter_mask` | `nav_msgs/OccupancyGrid` (TRANSIENT_LOCAL) | crowd_keepout_mask | robot1 Nav2 keepout filter |
| `/robot1/keepout_costmap_filter_info` | `nav2_msgs/CostmapFilterInfo` (TRANSIENT_LOCAL) | crowd_keepout_mask | robot1 Nav2 keepout filter |

## robot1

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/rb1_standby` | `std_msgs/Bool` | passenger_ui(AMR1) | robot1 미션 FSM |
| `/rb1_service_end` | `std_msgs/Bool` | passenger_ui(AMR1) | robot1 미션 FSM, db_manager |
| `/emergency_end` | `std_msgs/Bool` | passenger_ui(AMR1) | robot1 미션 FSM |
| `/rgbd_fall_person_point` | `geometry_msgs/PointStamped` | (저장소 안에는 발행 노드 없음) | robot1·robot3 미션 FSM |
| `/robot1/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 | robot1 미션 FSM |
| `/robot1/cmd_vel` | `geometry_msgs/Twist` | robot1 미션 FSM (경고 회전, ESTOP 0 명령), Nav2 | TurtleBot4 |
| `/robot1/cmd_audio` | `irobot_create_msgs/AudioNoteVector` | robot1 미션 FSM | Create3 |

## robot3 — 가이드 (후방 웹캠)

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | 후방 웹캠 드라이버(저장소 외부) | guide_rear_tracker |
| `/tracking_web` | `std_msgs/Int32` (0 없음 / 1 보임 / 2 소실) | guide_rear_tracker | guide_motion, passenger_ui(AMR3), db_manager |
| `/tracking_web_center_pixel` | `std_msgs/Float32` | guide_rear_tracker | guide_motion |
| `/target_depth` | `std_msgs/Float32` (m, 무효 100.0) | guide_rear_tracker | guide_motion |
| `/retarget_web` | `std_msgs/Bool` | guide_rear_tracker(자동 재탐색 알림) | guide_rear_tracker |
| `/rear_yolo_ocsort/image_raw/compressed` | `sensor_msgs/CompressedImage` | guide_rear_tracker | – (시각화) |
| `/turn_around` | `std_msgs/Bool` | passenger_ui(AMR3) | guide_motion, db_manager |
| `/robot3/mission_cmd` | `std_msgs/String` (목적지 이름) | robot3 미션 FSM | guide_motion |
| `/robot3/odom` | `nav_msgs/Odometry` | TurtleBot4 | guide_motion (180° 회전 판정) |

## robot3 — 짐 운반 (전방 OAK-D)

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/robot3/oakd/rgb/image_raw/compressed` | `sensor_msgs/CompressedImage` | OAK-D 드라이버 | luggage_rgbd_tracker |
| `/robot3/oakd/stereo/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` (16UC1 PNG) | OAK-D 드라이버 | luggage_rgbd_tracker |
| `/carrying` | `std_msgs/Bool` | passenger_ui(AMR3) | luggage_rgbd_tracker, luggage_follower |
| `/tracking_rgbd` | `std_msgs/Int32` (0 / 1 / 2) | luggage_rgbd_tracker | luggage_follower, passenger_ui(AMR3), db_manager |
| `/tracking_rgbd_center_pixel` | `std_msgs/Float32` | luggage_rgbd_tracker | luggage_follower |
| `/target_depth_rgbd` | `std_msgs/Float32` (m, 무효 100.0) | luggage_rgbd_tracker | luggage_follower |
| `/front_start` | `std_msgs/Int32` (0 잠금 / 1 추종 중, 10 Hz) | luggage_follower | guide_motion, guide_rear_tracker |
| `/service_end` | `std_msgs/Bool` | passenger_ui(AMR3) | luggage_rgbd_tracker, luggage_follower, guide_motion |

## robot3 — 순찰·미션 FSM

| 토픽 / 인터페이스 | 타입 | 발행 / 서버 | 구독 / 클라이언트 |
|---|---|---|---|
| `/robot3/mission_complete` | `std_msgs/Bool` | guide_motion(도착), luggage_follower(서비스 종료) | robot3 미션 FSM |
| `/rb3_standby` | `std_msgs/Bool` | passenger_ui(AMR3), robot3 미션 FSM | robot3_patrol |
| `/rb3_standby_done` | `std_msgs/Bool` | robot3_patrol | robot3 미션 FSM |
| `/start_patrol` | `std_msgs/Bool` | guide_motion, luggage_follower, robot3 미션 FSM | robot3_patrol |
| `/start_front` | `std_msgs/Int32` | (저장소 안에는 발행 노드 없음) | guide_motion |
| `/robot3/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL | robot3_patrol |
| `/robot3/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 | guide_motion, robot3_patrol, robot3 미션 FSM |
| `/robot3/cmd_vel` | `geometry_msgs/Twist` | guide_motion, luggage_follower, robot3 미션 FSM(ESTOP 0 명령), Nav2 | TurtleBot4 |
| `/robot3/cmd_audio` | `irobot_create_msgs/AudioNoteVector` | robot3 미션 FSM | Create3 |

## 노드 이름

| 실행 파일 (`ros2 run`) | 노드 이름 | 네임스페이스 |
|---|---|---|
| `cctv_perception cctv_detector_node` | `cctv_fall_crowd_node` | – |
| `fleet_central fleet_dispatcher_node` | `fleet_dispatcher_node` | – |
| `fleet_central db_manager_node` | `aura_db_manager_node` | – |
| `fleet_central crowd_keepout_mask_node` | `crowd_keepout_mask_publisher` | – |
| `robot1_control robot1_mission_fsm_node` | `robot_mission_fsm` | `/robot1` (필수) |
| `robot3_control robot3_mission_fsm_node` | `robot_mission_fsm` | `/robot3` (필수) |
| `robot3_control robot3_patrol_node` | `waypoint_patrol_node` | `/robot3` (필수) |
| `robot3_control guide_motion_node` | `guarded_nav_to_goal` | – |
| `robot3_control guide_rear_tracker_node` | `rear_person_vision_node` | – |
| `robot3_control luggage_rgbd_tracker_node` | `rgbd_person_tracking_node` | – |
| `robot3_control luggage_follower_node` | `front_move` | – |
