#!/usr/bin/env bash
# 임무 전체를 순서대로 띄우고 지켜본다 (시뮬 -> SLAM -> Nav2 -> 지형 -> 임무 계층 -> start).
#   bash run_mission.sh [start_set]      start_set: 0 = 처음부터, k = 세트 k 부터 (부분 시험)
# 환경 변수: GUI=1 (Gazebo 화면), SETS=n (n 세트만 하고 끝, 기본 전부), DUR=시뮬 초 한도 (기본 6000),
#           START=0 (start 를 자동으로 보내지 않음 -> 사용자가 husky_mission 터미널이나 /husky/cmd 로),
#           COMM_OFF_AT / COMM_OFF_FOR = 탐색 시작 뒤 시뮬 초에 통신 0, 그 뒤 몇 초 뒤 1 (통신 시험)
# 결과: $OUT (기본 <저장소>/results/<시각>_set<k>) 에 로그와 timeline.csv, summary.txt
T=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
R=$(cd "$T/../../.." && pwd)
[ -f "$R/install/setup.bash" ] || R=$(cd "$R/../ws" 2>/dev/null && pwd)   # code/ 쪽 실제 경로로 불러도 작업 공간(<저장소>/ws)을 찾게
K=${1:-0}
OUT=${OUT:-$(cd "$R/.." && pwd)/results/$(date +%Y%m%d_%H%M%S)_set$K}   # R = 작업 공간(<저장소>/ws) -> 저장소 최상위의 results/
source /opt/ros/humble/setup.bash
source "$R/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((20 + RANDOM % 60))}

kill_all() {   # $1 신호, $2 무늬. 이 스크립트의 조상(부른 셸)은 건너뜀 — 부른 명령줄에 무늬 글자가 들어 있어도 안 죽게
  local anc=" $$ " p=$$ pid
  while [ "${p:-1}" -gt 1 ]; do p=$(ps -o ppid= -p "$p" | tr -d ' '); anc="$anc$p "; done
  for pid in $(pgrep -f "$2"); do
    case "$anc" in *" $pid "*) ;; *) kill "$1" "$pid" 2>/dev/null ;; esac
  done
}
cleanup() {
  kill_all -INT "sim.launch.py|mission.launch.py|dual_slam.launch|nav2_(planners|controllers).launch|terrain.launch"
  sleep 2
  kill_all -9 "ign gazebo|ruby.*ign |quadruped_controller_node|state_estimation_node|ekf_node|robot_state_publisher|parameter_bridge"
  kill_all -9 "planner_server|controller_server|lifecycle_manager|async_slam_toolbox|scan_ready_relay|terrain_hazards"
  kill_all -9 "lib/comm_relay/|a1_exploration/scripts/|tools/mission_watch.py"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  ros2 daemon stop >/dev/null 2>&1
}
wait_log() {   # $1 파일들(glob), $2 무늬, $3 필요한 줄 수, $4 한도(벽시계 s), $5 설명
  local t0=$SECONDS
  while [ $((SECONDS - t0)) -lt "$4" ]; do
    [ "$(cat $1 2>/dev/null | grep -cE "$2")" -ge "$3" ] && { echo "$5: $((SECONDS - t0)) s"; return 0; }
    sleep 1
  done
  echo "실패: $5 ($4 s 안에 안 됨)"; return 1
}
[ "${1:-}" = clean ] && { cleanup; exit 0; }
mkdir -p "$OUT"
trap cleanup EXIT
cleanup
echo "=== 임무 실행: start_set $K, ROS_DOMAIN_ID $ROS_DOMAIN_ID, 결과 $OUT"
bringup() {   # 시뮬 -> SLAM -> Nav2 -> 지형
  ros2 launch disaster_sim sim.launch.py gui:=$([ "${GUI:-0}" = 1 ] && echo true || echo false) start_set:=$K > "$OUT/sim.log" 2>&1 &
  wait_log "$OUT/sim.log" 'activated.*joint_group_effort_controller' 2 180 "두 A1 컨트롤러 활성" || return 1
  sleep 8                                   # soft start 3 s + 기립 안정
  ros2 launch a1_exploration dual_slam.launch.py > "$OUT/slam.log" 2>&1 &
  sleep 5
  ros2 launch a1_exploration nav2_planners.launch.py > "$OUT/planner.log" 2>&1 &
  ros2 launch a1_exploration nav2_controllers.launch.py > "$OUT/controller.log" 2>&1 &
  ros2 launch terrain_perception terrain.launch.py > "$OUT/terrain.log" 2>&1 &
  wait_log "$OUT/planner.log $OUT/controller.log" 'Managed nodes are active' 4 120 "Nav2 서버 4 개 active"
}
# Nav2 가 TF 를 못 받아 안 켜지는 기동 불량이 가끔 있다 (2026-11-12 1/4 회, 이전 기록 약 2/25 회) -> 정리하고 한 번 다시
bringup || { echo "-> 정리하고 기동 다시 (1 회)"; cleanup; bringup || exit 1; }
ros2 launch comm_relay mission.launch.py start_set:=$K > "$OUT/mission.log" 2>&1 &
sleep 10
if [ "${START:-1}" = 1 ]; then
  for i in 1 2 3; do ros2 topic pub --once /husky/cmd std_msgs/msg/String "{data: start}" > /dev/null 2>&1; done
  echo "start 보냄"
fi
python3 -u "$T/mission_watch.py" "$OUT" "$K" "${SETS:-99}" "${DUR:-6000}" "${COMM_OFF_AT:-0}" "${COMM_OFF_FOR:-60}" 2>&1 | tee "$OUT/summary.txt"
grep -E "\[MISSION\]|\[COMM\]|\[HUSKY\]" "$OUT/mission.log" | sed -E 's/^\[[^]]*\] //' | cut -c1-200 > "$OUT/mission_events.txt"
echo "결과 폴더: $OUT"
