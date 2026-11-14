#!/usr/bin/env bash
# B3: 저속 보행 확인 (gait_test 평지, a1_1 한 대, 화면 없음). 소요 약 3분.
source "$(dirname "$0")/common.sh"
trap cleanup_all EXIT
cleanup_all
ros2 daemon stop >/dev/null 2>&1
ros2 launch a1_champ gait_test.launch.py spawn:=flat gui:=false > "$OUT/gait_test.log" 2>&1 &
for k in $(seq 1 90); do sleep 1
  grep -q 'activated.*joint_group_effort_controller' "$OUT/gait_test.log" && break; done
echo "컨트롤러 활성: ${k} s"; sleep 8
timeout 300 python3 -u "$T/lowspeed_check.py" | tee "$OUT/lowspeed.log"
cleanup_all
errors_summary
