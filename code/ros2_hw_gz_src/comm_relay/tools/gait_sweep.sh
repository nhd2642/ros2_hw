#!/usr/bin/env bash
# A1 보행 속도 응답 측정 (Gazebo Sim, 화면 없음): 시뮬만 띄우고 a1_2 를 도로(평지) 따라 여러 속도로 걷게 해 정답 위치로 잰다.
#   bash gait_sweep.sh ["0.2 0.0" "0.3 0.0" ...]     각 항목 = "vx wz", 기본은 아래 목록 (직진 합 12.6 m: a1_2 앞 도로가 비어 있어야 함).
#   NS=a1_1 이면 a1_1 로 (오른쪽 선회는 a1_1, 왼쪽 선회는 a1_2 쪽이 트여 있음). 결과: $OUT/gait.txt
T=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
R=$(cd "$T/../../.." && pwd)
[ -f "$R/install/setup.bash" ] || R=$(cd "$R/../ws" 2>/dev/null && pwd)   # code/ 쪽 실제 경로로 불러도 작업 공간(<저장소>/ws)을 찾게
OUT=${OUT:-$(cd "$R/.." && pwd)/results/$(date +%Y%m%d_%H%M%S)_gait}
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$R/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((20 + RANDOM % 60))}
trap 'bash "$T/run_mission.sh" clean' EXIT
bash "$T/run_mission.sh" clean
ros2 launch disaster_sim sim.launch.py gui:=false > "$OUT/sim.log" 2>&1 &
until [ "$(grep -c 'activated.*joint_group_effort_controller' "$OUT/sim.log")" -ge 2 ]; do sleep 1; done
sleep 10
# 항목 = "vx wz [vy]". 옆걸음(vy)·저속·제자리 회전 (2026-11-19, 좁은 곳 강건성)은 건물이 없는 출발 자리에서 먼저 (좌우가 서로 상쇄)
[ $# -eq 0 ] && set -- "0.0 0.0 0.1" "0.0 0.0 -0.1" "0.0 0.0 0.2" "0.0 0.0 -0.2" "0.0 0.3" "0.0 -0.3" \
  "0.05 0.0" "0.1 0.0" "0.2 0.0" "0.3 0.0" "0.4 0.0" "0.5 0.0" "0.0 0.5" "0.0 -0.5" "0.25 0.3" "0.25 -0.3"
for c in "$@"; do
  set -- $c
  python3 -u "$T/walk_check.py" ${NS:-a1_2} "$1" "$2" 8 "${3:-0}" 2>&1 | tail -1 | tee -a "$OUT/gait.txt"
  python3 "$T/sim_sleep.py" 2
done
echo "결과: $OUT/gait.txt"
