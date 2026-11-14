#!/usr/bin/env bash
# Nav2 계획 실패 진단 (약 5분): 시뮬 + SLAM + Nav2 만 띄우고(comm_relay 없음) a1_2 기준으로
#   기동 후 5 s / 25 s / 50 s 에 지도·비용지도·계획 결과를 찍고, a1_2 를 0.6 m 앞으로 걷게 한 뒤 한 번 더.
source "$(dirname "$0")/common.sh"
trap cleanup_all EXIT
cleanup_all; start_sim
ros2 launch a1_exploration dual_slam.launch.py > "$OUT/slam.log" 2>&1 &
sleep 5
ros2 launch a1_exploration nav2_planners.launch.py > "$OUT/planner.log" 2>&1 &
ros2 launch a1_exploration nav2_controllers.launch.py > "$OUT/controller.log" 2>&1 &
for k in $(seq 1 60); do sleep 1; ok=0
  for ns in a1_1 a1_2; do for n in planner_server controller_server; do
    timeout 5 ros2 lifecycle get /$ns/$n 2>/dev/null | grep -q active && ok=$((ok+1)); done; done
  [ $ok = 4 ] && break; done
echo "Nav2 서버 active $ok/4"
echo "--- SLAM 설정: $(grep -hE 'map_update_interval|minimum_travel_distance|max_laser_range|resolution' $R/src/a1_exploration/config/slam_params.yaml | tr -s ' ' | tr '\n' ' ')"
for w in 5 20 25; do sleep $w; python3 -u "$T/nav_probe.py" a1_2 "(Nav2 active 뒤 누적 대기)"; done
echo "--- a1_2 를 0.6 m 앞으로 (0.15 m/s 4 s)"
timeout 5 ros2 topic pub -r 20 /a1_2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}" >/dev/null 2>&1
ros2 topic pub --once /a1_2/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
sleep 8
python3 -u "$T/nav_probe.py" a1_2 "(0.6 m 걸은 뒤)"
echo "--- 플래너 경고"; grep -iE "warn|error" "$OUT/planner.log" | grep -v "Timed out waiting for transform" | cut -c1-180 | tail -12
cleanup_all; errors_summary
