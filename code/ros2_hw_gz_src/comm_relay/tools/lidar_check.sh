#!/usr/bin/env bash
# 라이다 값 점검 (2026-11-10): 사용자 PC 에서 스캔이 전부 range_min(0.15 m) 으로 나온 원인 가르기. 시뮬을 세 가지 방식으로 차례로 띄워 스캔을 읽음.
#   hw_novis : 그냥 (화면 없음)        -> 이게 FAIL 이면 시각화 때문이 아님
#   hw_vis   : 그냥 + Gazebo 화면·RViz  -> hw_novis 는 PASS 인데 이게 FAIL 이면 시각화 때문
#   sw_novis : LIBGL_ALWAYS_SOFTWARE=1 (화면 없음, 소프트웨어 렌더링) -> 이것만 PASS 면 WSL GPU 렌더링 문제
# 사용: bash lidar_check.sh   (세 가지 약 6~10 분). 일부만: MODES="hw_novis sw_novis" bash lidar_check.sh
source "$(dirname "$0")/common.sh"
RES=${CR_RESULTS:-$([ -d /mnt/c/Users/fllno ] && echo /mnt/c/Users/fllno/comm_relay_results || echo "$HOME/comm_relay_results")}
mkdir -p "$RES"
{ echo "== 렌더러 (glxinfo)"; glxinfo -B 2>/dev/null | grep -E "OpenGL (renderer|version)|Device|Vendor" || echo "glxinfo 없음 (sudo apt install mesa-utils 하면 볼 수 있음)"; } | tee "$OUT/lidar_check.txt"
summary=""
for mode in ${MODES:-hw_novis hw_vis sw_novis}; do
  cleanup_all
  case $mode in
    hw_novis) export VIS=0; unset LIBGL_ALWAYS_SOFTWARE ;;
    hw_vis)   export VIS=1; unset LIBGL_ALWAYS_SOFTWARE ;;
    sw_novis) export VIS=0; export LIBGL_ALWAYS_SOFTWARE=1 ;;
  esac
  echo "=== $mode" | tee -a "$OUT/lidar_check.txt"
  ( start_sim_checked && sleep 5 && python3 -u "$T/lidar_check.py" ) 2>&1 | tee -a "$OUT/lidar_check.txt"
  cp "$OUT/sim.log" "$OUT/sim_$mode.log" 2>/dev/null
  r=$(awk -v m="=== $mode" '$0==m{f=1} f&&/^RESULT/{print $2; exit}' "$OUT/lidar_check.txt")
  summary="$summary $mode=${r:-시작실패}"
done
cleanup_all
echo "=== 요약:$summary" | tee -a "$OUT/lidar_check.txt"
cp "$OUT/lidar_check.txt" "$RES/lidar_check_$(date +%Y%m%d_%H%M%S).txt"
echo "결과: $OUT/lidar_check.txt (결과 폴더에도 복사)"
