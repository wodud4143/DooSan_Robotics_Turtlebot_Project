#!/usr/bin/env bash
set -e

# ROS2·워크스페이스 setup 경로는 환경변수로 바꿀 수 있다.
AURA_ROS_SETUP="${AURA_ROS_SETUP:-/opt/ros/humble/setup.bash}"
AURA_WS_SETUP="${AURA_WS_SETUP:-$HOME/aura_ws/install/setup.bash}"

source "$AURA_ROS_SETUP"
if [ -f "$AURA_WS_SETUP" ]; then
  source "$AURA_WS_SETUP"
fi

export AURA_DB_PATH="${AURA_DB_PATH:-$HOME/Downloads/data/amr_system.db}"
export AURA_ADMIN_CCTV_TOPIC="${AURA_ADMIN_CCTV_TOPIC:-/cctv/image_raw/compressed}"
export AURA_ADMIN_PORT="${AURA_ADMIN_PORT:-7000}"

cd "$(dirname "$0")"
exec python3 app.py
