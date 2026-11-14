#!/usr/bin/env bash
# 깊이 카메라 지형 위험 확인 (Gazebo, 화면 없음). 소요 약 3분. terrain_setup.sh apply 뒤에.
#   sim + dual_slam + Nav2 + terrain_hazards -> terrain_check.py (점구름 프레임, 경사로 옆 가장자리 위험, 비용지도)
source "$(dirname "$0")/common.sh"
trap cleanup_all EXIT
cleanup_all
ros2 pkg prefix terrain_perception >/dev/null 2>&1 || { echo "terrain_perception 패키지 없음 (빌드 확인)"; exit 1; }
start_sim
ros2 launch a1_exploration dual_slam.launch.py > "$OUT/slam.log" 2>&1 &
sleep 5
ros2 launch a1_exploration nav2_planners.launch.py > "$OUT/planner.log" 2>&1 &
ros2 launch a1_exploration nav2_controllers.launch.py > "$OUT/controller.log" 2>&1 &
ros2 launch terrain_perception terrain.launch.py > "$OUT/terrain.log" 2>&1 &
wait_nav2; dup_nodes
grep -h "terrain_layer\|terrain_patch" "$OUT/planner.log" "$OUT/controller.log" | head -4
sleep 5
python3 -u "$T/terrain_check.py" 2>&1 | tee "$OUT/terrain_check.log"
echo "--- terrain_hazards 노드 기록"
grep -E "처리|Traceback|Error" "$OUT/terrain.log" | sed -E 's/^\[[^]]*\] //' | tail -4 | cut -c1-200
python3 "$T/map_snapshot.py" a1_1 "$OUT/terrain_a1_1_map.png" "지형 확인 뒤" 2>/dev/null | head -2
errors_summary
