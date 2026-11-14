#!/usr/bin/env bash
# 실험 1회 자동 실행: 월드 새로 띄우기 → 로봇이 설 때까지 대기 → 시험 → 월드 종료
# 사용: bash run_exp.sh <태그> "<무엇을 바꿨는지 메모>" [반복횟수=3]
#   예: bash run_exp.sh mu1.0 "발 마찰 0.6->1.0"
set -u
TAG=${1:?태그를 입력하세요 (예: baseline)}; NOTE=${2:-}; REP=${3:-3}
WS=${WS:-$HOME/ros2_hw}
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

cleanup() {
  kill -INT "${LPID:-0}" 2>/dev/null; sleep 3
  pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null; pkill -f spawn_entity 2>/dev/null
}
trap cleanup EXIT

pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null; pkill -f spawn_entity 2>/dev/null; sleep 2
echo "[1/3] 월드 실행 (로그: /tmp/gait_launch.log)"
ros2 launch a1_champ gait_test.launch.py spawn:=flat gui:=false > /tmp/gait_launch.log 2>&1 &
LPID=$!
echo "[2/3] 로봇이 설 때까지 35초 대기"
sleep 35
echo "[3/3] 시험 실행"
ros2 run a1_champ gait_eval.py --tag "$TAG" --note "$NOTE" --repeat "$REP" --tests walk,spin_left,spin_right
