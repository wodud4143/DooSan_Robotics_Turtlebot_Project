# AURA 관리자 대시보드 (admin_ui)

`db_manager_node`가 기록한 SQLite DB와 CCTV 영상·인원수 토픽을 한 화면에서 보는 **읽기 전용** 관제 대시보드입니다.
로봇 제어 명령은 발행하지 않습니다.

## 화면 구성

- 상단 상태: DB 연결, CCTV 수신, 인원수 수신
- 요약 카드: 공항 인원수, 등록 로봇 수, 응급 누적, 완료 서비스
- AMR 상태 카드: 모드, 배터리, AED 탑재, 위치, 마지막 수신 시각(5초 이내면 ONLINE)
- LIVE CCTV: `/cctv/image_raw/compressed`를 MJPEG(`/video/cctv`)로 중계
- 응급 현황, 낙상 스냅샷(`event_images` 테이블의 이미지 파일), 시스템 로그

## 동작 방식

| 데이터 | 경로 | 갱신 |
|---|---|---|
| 로봇·응급·스냅샷·로그 | SQLite (`mode=ro`로 열어 읽기만 함) | `/api/dashboard` 1초 폴링 |
| 인원수 | `/population` (`std_msgs/Int32`) 구독 | SSE(`/api/population/stream`)로 즉시 전달, 10초 heartbeat |
| CCTV 영상 | `/cctv/image_raw/compressed` (`sensor_msgs/CompressedImage`, JPEG) 구독 | MJPEG, 재인코딩 없이 전달 |

rclpy를 불러올 수 없으면 DB 전용 모드로 뜹니다(영상·인원수 영역만 비활성).

## 실행

```bash
pip install -r requirements.txt          # Flask 3.x

source /opt/ros/humble/setup.bash
source ~/aura_ws/install/setup.bash
export ROS_DOMAIN_ID=<도메인 ID>
export AURA_DB_PATH=$HOME/aura_data/amr_system.db   # db_manager_node와 같은 값

cd <repo>/ui/admin_ui
python3 app.py            # 또는 ./run.sh
```

브라우저에서 `http://<중앙 PC IP>:7000`으로 접속합니다.

> `db_manager_node`와 admin_ui는 DB 기본 경로가 다릅니다
> (db_manager: 설치된 모듈 옆 `data/amr_system.db`, admin_ui: `~/Downloads/data/amr_system.db`).
> 두 프로세스에 **같은 `AURA_DB_PATH`** 를 지정해야 대시보드에 기록이 나타납니다.

## 환경변수

| 변수 | 기본값 | 설명 |
|---|---|---|
| `AURA_DB_PATH` | `~/Downloads/data/amr_system.db` | 읽을 SQLite 파일 |
| `AURA_ADMIN_CCTV_TOPIC` | `/cctv/image_raw/compressed` | CCTV 영상 토픽 |
| `AURA_ADMIN_POPULATION_TOPIC` | `/population` | 인원수 토픽 |
| `AURA_ADMIN_HOST` | `0.0.0.0` | 바인드 주소 (같은 네트워크 전체에 열림, 인증 없음) |
| `AURA_ADMIN_PORT` | `7000` | 웹 포트 |
| `AURA_ROS_SETUP`, `AURA_WS_SETUP` | `/opt/ros/humble/setup.bash`, `~/aura_ws/install/setup.bash` | `run.sh`가 source할 경로 |

`run.sh`는 ROS setup 파일을 먼저 source하므로 ROS가 없는 PC에서는 `python3 app.py`로 실행합니다.

## 확인 명령

```bash
ros2 topic hz /cctv/image_raw/compressed     # 최대 10 Hz
ros2 topic echo /population
ros2 topic pub -r 2 /population std_msgs/msg/Int32 "{data: 127}"   # 인원수 카드 갱신 테스트
```
