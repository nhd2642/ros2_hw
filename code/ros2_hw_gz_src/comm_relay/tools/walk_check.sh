#!/usr/bin/env bash
# Gazebo Sim 보행·odom 점검: 시뮬(헤드리스) -> 기립 높이 -> a1_2 평지 직진 0.2 m/s 8 s, 제자리 회전 0.5 rad/s 6 s -> 정리.
# 왜: 물리 엔진이 ODE(Classic) -> DART(Gazebo Sim) 로 바뀜. 클라우드 시험(RTF 0.2)에서 정답 대비 직진 86~101 %, odom/raw 94 %,
#     EKF odom 은 36 % 로 낮게 나옴 -> RTF 가 높은 사용자 PC 에서 같은 값을 재서 원인(부하 vs 설정)을 가림.
source "$(dirname "$0")/common.sh"
cleanup_all
start_sim_checked
python3 -u "$T/walk_check.py" a1_2 0.2 0.0 8 | tee "$OUT/walk.txt"
python3 -u "$T/walk_check.py" a1_2 0.0 0.5 6 | tee -a "$OUT/walk.txt"
python3 -u "$T/walk_check.py" a1_1 0.2 0.0 8 | tee -a "$OUT/walk.txt"   # a1_1 은 경사로 쪽
RES=${CR_RESULTS:-$([ -d /mnt/c/Users/fllno ] && echo /mnt/c/Users/fllno/comm_relay_results || echo "$HOME/comm_relay_results")}
mkdir -p "$RES" && cp "$OUT/walk.txt" "$RES/walk_check_$(date +%Y%m%d_%H%M%S).txt"
cleanup_all
echo "결과: $OUT/walk.txt (결과 폴더에도 복사)"
