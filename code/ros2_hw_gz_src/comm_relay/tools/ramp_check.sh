#!/usr/bin/env bash
# 경사로 시험 (Gazebo Sim, 화면 없음): 시뮬 + SLAM 만 띄우고 a1_1 을 세트1 남쪽 방 경사로 앞 (6.4, -2.4) 에 남쪽 보게 놓은 뒤
# ramp_check.py 순서대로 걷고 돌게 해서 미끄러짐·SLAM 오차를 잰다. 한 번 약 10 분.
#   LOOPS=2 TAG=base bash ramp_check.sh     결과: $OUT (기본 <저장소>/results/<시각>_ramp_<TAG>) 의 ramp.txt, ramp.csv
T=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
R=$(cd "$T/../../.." && pwd)
[ -f "$R/install/setup.bash" ] || R=$(cd "$R/../ws" 2>/dev/null && pwd)
OUT=${OUT:-$(cd "$R/.." && pwd)/results/$(date +%Y%m%d_%H%M%S)_ramp_${TAG:-base}}
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$R/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((20 + RANDOM % 60))}
trap 'bash "$T/run_mission.sh" clean' EXIT
bash "$T/run_mission.sh" clean
ros2 launch disaster_sim sim.launch.py gui:=false a1_1_pose:=6.4,-2.4,-1.5708 > "$OUT/sim.log" 2>&1 &
t0=$SECONDS
until [ "$(grep -c 'activated.*joint_group_effort_controller' "$OUT/sim.log")" -ge 2 ]; do
  sleep 1; [ $((SECONDS - t0)) -gt 240 ] && { echo "실패: A1 컨트롤러가 240 s 안에 안 켜짐"; exit 1; }
done
sleep 8                                   # soft start 3 s + 기립 안정 (run_mission.sh 와 같게)
ros2 launch a1_exploration dual_slam.launch.py > "$OUT/slam.log" 2>&1 &
python3 "$T/sim_sleep.py" 8               # SLAM 첫 지도
python3 -u "$T/ramp_check.py" "$OUT" "${LOOPS:-2}" 2>&1 | tee "$OUT/ramp.txt"
grep 'scan_ready_relay_a1_1' "$OUT/slam.log" | grep -o '지운 스캔 누적 [0-9]*' | tail -1 | sed 's/^/relay /' | tee -a "$OUT/ramp.txt"
echo "결과: $OUT"
