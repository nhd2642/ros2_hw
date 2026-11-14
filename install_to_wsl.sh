#!/usr/bin/env bash
# 이 묶음을 ~/disaster_develop 에 풀고, 최종 코드(~/ros2_hw_gz/src)와 묶음 zip·누적표를 함께 복사한다.
# 사용 (WSL): bash /mnt/c/Users/fllno/comm_relay_results/disaster_develop_pkg/install_to_wsl.sh
# 이미 있는 파일은 덮어쓰고, ~/disaster_develop 에 사용자가 만든 다른 파일은 지우지 않는다.
set -eo pipefail
PKG=$(cd "$(dirname "$0")" && pwd)
DST=${DST:-$HOME/disaster_develop}
SRC=${SRC:-$HOME/ros2_hw_gz/src}
RES=/mnt/c/Users/fllno/comm_relay_results
mkdir -p "$DST"
cp -r "$PKG/." "$DST/"
echo "문서·도구 -> $DST"
if [ -d "$SRC" ]; then
  mkdir -p "$DST/code/ros2_hw_gz_src"
  if command -v rsync >/dev/null; then
    rsync -a --delete --exclude '__pycache__' --exclude '*.pyc' --exclude '.pytest_cache' "$SRC/" "$DST/code/ros2_hw_gz_src/"
  else
    cp -r "$SRC/." "$DST/code/ros2_hw_gz_src/"
  fi
  echo "최종 코드 $SRC -> $DST/code/ros2_hw_gz_src ($(find "$DST/code/ros2_hw_gz_src" -type f | wc -l) 파일)"
else
  echo "!! $SRC 없음 -> 코드 복사 안 함"
fi
if [ -d "$RES/bundles" ]; then
  mkdir -p "$DST/history/bundles"
  cp "$RES"/bundles/*.zip "$DST/history/bundles/" 2>/dev/null || true
  cp "$RES"/bundles/*.sh "$DST/history/bundles/" 2>/dev/null || true
  echo "묶음 zip $(ls "$DST/history/bundles" | wc -l) 개 -> $DST/history/bundles"
fi
[ -f "$RES/runs.csv" ] && cp "$RES/runs.csv" "$DST/history/results/runs.csv"
echo "결과 폴더(실행별 기록)는 크므로 복사하지 않음: $RES"
echo "끝. Claude Code 는 cd $DST && claude 로 시작 (CLAUDE.md, HANDOFF.md 를 먼저 읽게 됨)."
