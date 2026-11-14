#!/usr/bin/env bash
# A1 메시(.dae)를 unitree_ros에서 받아 이 패키지의 meshes/에 복사
set -e
HERE=$(cd "$(dirname "$0")/.." && pwd)
TMP=$(mktemp -d)
git clone --depth 1 https://github.com/chvmp/unitree_ros "$TMP/unitree_ros"
cp "$TMP"/unitree_ros/robots/a1_description/meshes/*.dae "$HERE/meshes/"
cp "$TMP"/unitree_ros/robots/a1_description/urdf/a1.urdf "$HERE/scripts/a1_original.urdf"
rm -rf "$TMP"
echo "메시 복사 완료: $(ls "$HERE/meshes" | wc -l)개"
