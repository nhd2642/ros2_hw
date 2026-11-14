# Gazebo 시험 공통 (source 해서 씀). 사용자 PC(WSL, ~/ros2_hw) 기준.
# set -u 는 쓰지 않는다: /opt/ros/humble/setup.bash 가 AMENT_TRACE_SETUP_FILES 같은 미정의 변수를 읽어 중단됨
T=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
R=${R:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}   # 작업 공간 = 이 스크립트(src/<pkg>/tools) 3 단계 위
S=$R/src/a1_exploration/scripts
OUT=${OUT:-$HOME/comm_relay_tests/$(date +%Y%m%d_%H%M%S)_$(basename "$0" .sh)}
mkdir -p "$OUT"
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash   # 다른 설치(conda 등)는 미리 환경을 켜 두면 됨
source "$R/install/setup.bash"
# Fast DDS UDP 만 쓰기는 기본 끔 (record8 에서 켰더니 a1_1 controller_server 가 inactive 로 남아 기동 실패, 2026-11-09).
# 공유 메모리 잠금 충돌은 실행마다 ROS_DOMAIN_ID 를 바꿔 피함 (gazebo_test_mission.sh). 시험용으로 켜려면 A1_DDS_UDP=1
if [ "${A1_DDS_UDP:-0}" = 1 ]; then export FASTRTPS_DEFAULT_PROFILES_FILE="$T/fastdds_udp.xml"; fi

cleanup_all() { bash "$T/run_mission.sh" clean; }   # 정리 무늬는 run_mission.sh 한 곳에만 둔다

wait_nav2() {   # Nav2 서버 4 개 active 대기 (전체 90 s). lifecycle_manager 로그 "Managed nodes are active" 로 판단
  # (ros2 lifecycle get CLI 는 daemon 상태에 따라 Traceback 을 내서 active 인데도 0/4 로 셀 수 있음: 2026-11-08)
  local t0=$SECONDS ok
  while [ $((SECONDS - t0)) -lt 90 ]; do
    # 관리자 4 개(로봇 2 x planner/controller)가 각각 active 를 냈는지 이름으로 셈. 이전엔 줄 수만 세서 a1_2 planner 가
    # 'Configuring' 에서 멈췄는데도 4/4 로 통과 -> 목표가 전부 'planner 거절' (2026-11-09 record10 1 회차)
    ok=0
    for mgr in a1_1.lifecycle_manager_planner a1_2.lifecycle_manager_planner a1_1.lifecycle_manager_controller a1_2.lifecycle_manager_controller; do
      cat "$OUT"/*planner.log "$OUT"/*controller.log 2>/dev/null | grep "$mgr" | grep -q "Managed nodes are active" && ok=$((ok + 1))
    done
    [ "$ok" -ge 4 ] && { echo "Nav2 서버 active 4/4 ($((SECONDS - t0)) s)"; return 0; }
    if [ $(((SECONDS - t0) % 10)) -eq 9 ]; then     # 보조: CLI 로도 (로그 문구가 다를 때 대비)
      local c=0 ns n
      for ns in a1_1 a1_2; do for n in planner_server controller_server; do
        timeout 8 ros2 lifecycle get /$ns/$n 2>/dev/null | grep -q '^active' && c=$((c+1)); done; done
      [ $c = 4 ] && { echo "Nav2 서버 active 4/4 ($((SECONDS - t0)) s, CLI)"; return 0; }
    fi
    sleep 1
  done
  echo "--- ros2 lifecycle get 직접 (Traceback 이면 마지막 줄):"
  timeout 8 ros2 lifecycle get /a1_1/controller_server 2>&1 | tail -2 | cut -c1-200
  echo "Nav2 서버 active ${ok:-0}/4 — 90 s 안에 안 됨. lifecycle/TF 기록:"
  grep -hE "lifecycle_manager|Failed|Aborting|Timed out waiting for transform" "$OUT"/*planner.log "$OUT"/*controller.log 2>/dev/null \
    | sed -E 's/^\[[^]]*\] //' | tail -6 | cut -c1-200
  [ "${NAV2_RETURN:-0}" = 1 ] && return 1
  exit 1
}

dup_nodes() {   # 같은 이름 노드가 둘 이상이면 출력 (옛 프로세스가 남은 경우)
  local d; d=$(timeout 10 ros2 node list 2>/dev/null | sort | uniq -d)
  [ -n "$d" ] && echo "경고: 같은 이름 노드 중복 -> 옛 프로세스가 남아 있음: $(echo $d)"
  return 0
}

start_sim() {   # 두 로봇 컨트롤러 활성까지 대기 (최대 150 s)
  ros2 daemon stop >/dev/null 2>&1
  ros2 launch disaster_sim sim.launch.py gui:=false > "$OUT/sim.log" 2>&1 &
  local k
  for k in $(seq 1 150); do sleep 1
    [ "$(grep -c 'activated.*joint_group_effort_controller' "$OUT/sim.log")" -ge 2 ] && \
    [ "$(grep -c 'activated.*joint_states_controller' "$OUT/sim.log")" -ge 2 ] && break
    if grep -qE 'process has died.*(ign gazebo|gz sim|ruby)' "$OUT/sim.log"; then   # 150 s 기다리지 않고 바로 원인 출력
      echo "Gazebo Sim 서버가 죽음 (${k} s). 원인:"; grep -E '\[ERROR\]|terminate called' "$OUT/sim.log" | head -4 | cut -c1-220
      exit 1
    fi
  done
  echo "두 로봇 컨트롤러 활성: ${k} s"; sleep 8    # soft start 3 s + 기립 안정
  start_vis
}

start_vis() {   # 시각화 (2026-11-10 사용자 요청; 기본 끔 — 켜면 RTF 0.6 -> 0.2 대, VIS=1 로 켬): Gazebo 화면 + 로봇마다 RViz (SLAM 맵·라이다·앞 카메라·경로·사람 표시)
  # Gazebo 화면은 서버와 다른 프로세스(ign gazebo -g)라 화면이 죽어도 시뮬은 계속됨. WSL 은 ogre2 가 죽어 화면도 ogre(VIS_ENGINE)
  [ "${VIS:-0}" = 1 ] || return 0
  command -v rviz2 >/dev/null || echo "주의: rviz2 없음 -> sudo apt install ros-humble-rviz2 (Gazebo 화면만 띄움)"
  ign gazebo -g --render-engine-gui "${VIS_ENGINE:-ogre}" > "$OUT/gz_gui.log" 2>&1 &
  local ns x=0
  for ns in a1_1 a1_2; do   # 로봇마다 /tf 가 따로라 RViz 도 로봇마다 하나 (/<ns>/tf 로 바꿔 받음)
    sed "s#/NS/#/$ns/#g; s#XPOS#$x#" "$T/vis.rviz" > "$OUT/vis_$ns.rviz"
    rviz2 -d "$OUT/vis_$ns.rviz" --ros-args -r __node:=rviz_$ns -r /tf:=/$ns/tf -r /tf_static:=/$ns/tf_static \
      -p use_sim_time:=true > "$OUT/rviz_$ns.log" 2>&1 &
    x=900
  done
  echo "시각화: Gazebo 화면 + RViz 2 개 (a1_1 왼쪽, a1_2 오른쪽). 기본은 끔"
}

errors_summary() {   # Traceback 은 예외 종류별로: 시험이 끝날 때 프로세스를 죽여서 생긴 것(KeyboardInterrupt 등)은 따로 센다
  echo "--- 오류 (Traceback / ERROR)"
  for f in "$OUT"/*.log; do
    n=$(grep -c Traceback "$f"); e=$(grep -c '\[ERROR\]' "$f"); k=$(grep -c 'exit code -9\|exit code -2\|exit code -15' "$f")
    [ "$n" = 0 ] && [ "$e" = 0 ] && continue
    kinds=$(sed -E 's/^\[[^]]*\] //' "$f" | awk '/Traceback/{t=1;next} t && /^[A-Za-z_.0-9]*(Error|Exception|Interrupt)/{sub(/:.*/,"");print;t=0}' | sort | uniq -c | tr '\n' ' ')   # launch 접두어 [python3-3] 제거 후 분류
    [ "$n" = 0 ] && kinds="없음"; echo "$(basename "$f"): Traceback $n [${kinds:-종류 미상}], ERROR $e (그중 정리 단계 강제종료 $k)"
    [ "$e" -gt "$k" ] && grep '\[ERROR\]' "$f" | grep -v 'exit code -9\|exit code -2\|exit code -15' | \
      sed -E 's/^\[[^]]*\] //; s/\[[0-9]+\.[0-9]+\] //; s/[0-9]+\.[0-9]+|[0-9]{2,}/#/g' | sort | uniq -c | sort -rn | head -5 | cut -c1-200 | sed 's/^/      /'   # 숫자(시각 등)는 # 로 묶어 셈
  done
  echo "(로그: $OUT)"
}

sim_start_ok() {   # 시작 점검 (2026-11-09 record11b: Husky 스폰 실패, a1_2 가 스폰 직후 주저앉음 -> 결과 못 믿음)
  local n bad=""
  for n in husky a1_1 a1_2; do
    [ "$(grep -c "OK creation of entity" "$OUT/sim.log")" -ge 3 ] || bad="$bad 스폰3개미만"; break   # Gazebo Sim: ros_gz_sim create 로그 (이름은 안 찍힘)
  done
  for n in a1_1 a1_2; do
    local z; z=$(timeout 15 ros2 topic echo --once --field pose.pose.position.z /$n/odom/ground_truth 2>/dev/null | head -1)
    if [ -z "$z" ]; then echo "주의: $n 위치(ground_truth)를 못 읽음 - z 점검 건너뜀"
    else awk -v z="$z" 'BEGIN{exit !(z >= 0.15)}' || bad="$bad $n:z=$z"; fi
  done
  [ -z "$bad" ] && return 0
  echo "시작 불량:$bad"; return 1
}

start_sim_checked() {   # start_sim + 시작 점검, 불량이면 정리하고 한 번만 다시
  start_sim
  sim_start_ok && return 0
  echo "-> 정리하고 시뮬 다시 (1 회)"; cleanup_all; start_sim
  sim_start_ok || { echo "시작 불량이 반복됨 -> 중단"; exit 1; }
}
