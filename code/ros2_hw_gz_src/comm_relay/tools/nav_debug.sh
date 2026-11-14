#!/usr/bin/env bash
# Nav2(DWB) 정지 원인 진단 (Gazebo, 화면 없음, 탐색·comm_relay 없이). 약 5분.
#   sim + SLAM + Nav2(지형 층 켬) + 지형 노드 -> nav_debug.py (a1_1 에 목표 3 개, 명령·odom·DWB 평가 기록)
source "$(dirname "$0")/common.sh"
trap cleanup_all EXIT
cleanup_all
start_sim
ros2 launch a1_exploration dual_slam.launch.py > "$OUT/slam.log" 2>&1 &
sleep 5
ros2 launch a1_exploration nav2_planners.launch.py > "$OUT/planner.log" 2>&1 &
ros2 launch a1_exploration nav2_controllers.launch.py > "$OUT/controller.log" 2>&1 &
ros2 pkg prefix terrain_perception >/dev/null 2>&1 && ros2 launch terrain_perception terrain.launch.py > "$OUT/terrain.log" 2>&1 &
wait_nav2
sleep 8
echo "--- DWB 평가 토픽: $(timeout 10 ros2 topic list 2>/dev/null | grep -E 'a1_1/.*evaluation' | tr '\n' ' ')"
python3 -u "$T/nav_debug.py" 2>&1 | tee "$OUT/nav_debug.log"
echo "--- controller 경고·오류 (a1_1)"
grep a1_1 "$OUT/controller.log" | grep -E "WARN|ERROR" | sed -E 's/^\[[^]]*\] //; s/\[[0-9]+\.[0-9]+\] //' | sort | uniq -c | sort -rn | head -8 | cut -c1-200
errors_summary
