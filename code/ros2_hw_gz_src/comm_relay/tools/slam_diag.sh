#!/usr/bin/env bash
# SLAM 지도가 안 커지는 원인 진단 (Gazebo, 화면 없음, comm_relay/Nav2 없이). 소요 약 3분.
#   sim + dual_slam -> 지도 그림 -> slam_diag.py a1_2 (관찰 10 s, 360° 회전, 1 m 전진) -> 지도 그림 -> SLAM 로그 경고 집계
source "$(dirname "$0")/common.sh"
trap cleanup_all EXIT
cleanup_all
start_sim
ros2 launch a1_exploration dual_slam.launch.py > "$OUT/slam.log" 2>&1 &
sleep 10
python3 "$T/map_snapshot.py" a1_2 "$OUT/map_before.png" "움직이기 전" 2>&1 | grep -v "^$"
python3 -u "$T/slam_diag.py" a1_2 2>&1 | tee "$OUT/slam_diag.log"
python3 "$T/map_snapshot.py" a1_2 "$OUT/map_after.png" "회전·전진 뒤" 2>&1 | grep -v "^$"
echo "--- SLAM/relay 로그 경고·오류 (같은 문장 묶음, 숫자는 #)"
grep -E "WARN|ERROR|Dropped|Failed|dropping|queue" "$OUT/slam.log" | sed -E 's/^\[[^]]*\] //; s/\[[0-9]+\.[0-9]+\]//g; s/[0-9]+(\.[0-9]+)?/#/g' | \
  sort | uniq -c | sort -rn | head -25 | cut -c1-220
echo "--- SLAM 시작 메시지"
grep -E "a1_2" "$OUT/slam.log" | grep -viE "warn|error" | sed -E 's/^\[[^]]*\] //' | head -12 | cut -c1-200
errors_summary
