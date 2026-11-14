#!/usr/bin/env bash
# 깊이 카메라 지형 위험을 쓰도록 ros2_hw 의 3개 파일을 고침 (각각 *.bak_terrain 백업). 사용: bash terrain_setup.sh [apply|revert|status]
#  1) a1_champ/urdf/a1.urdf.xacro : terrain_camera (Gazebo Sim: 160x120 빔 gpu_lidar 로 깊이 카메라를 대신함 — 화각·해상도·거리 같음. depth_camera 는 ogre1 에서 두 대째에
#     'ItemIdentityException depth_points_vs.glsl' 로 죽고(WSL 은 ogre2 가 안 돼 ogre1 필요), gpu_lidar 는 두 엔진 모두 됨.
#     점 구름 gz 토픽 .../depth/points -> a1.launch.py 브리지 -> /<ns>/terrain_camera/points, 좌표 = 센서 링크(x 앞))
#     위치 (0.29, 0, 0.04): 예전 (0.16, 0, 0.06) 은 몸통 메시(trunk.dae 앞머리 x 0.281 m) 바로 위라 깊이가 거의 전부 0.1 m 미만(-inf)이었음 (2026-11-10 Gazebo Sim 확인) (깊이, 160x120, 87°, 10 Hz, 몸통 앞 위 40° 아래) + 광학 프레임
#     (URDF 안에는 주석 없이 ASCII 만 넣음: 주석 속 ':' 가 gazebo_ros2_control 을 죽임)
#     기존 front_camera(RGB, 마네킹 인식용)는 그대로
#  2) a1_exploration/launch/nav2_controllers.launch.py : 국소 비용지도에 terrain_layer, 라이다 obstacle_min_range 0.2,
#     진행 검사 시간 10 -> 20 s (A1 제자리 회전이 느려 뒤쪽 목표에서 'Failed to make progress')
#  3) a1_exploration/launch/nav2_planners.launch.py    : 전역 비용지도에 terrain_layer
#  지형 층을 잠깐 끄려면 파일을 되돌리지 않고 A1_TERRAIN=0 으로 nav2 launch 를 띄우면 됨
set -e
R=${R:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}   # 작업 공간 = 이 스크립트(src/<pkg>/tools) 3 단계 위
U=$R/src/a1_champ/urdf/a1.urdf.xacro
C=$R/src/a1_exploration/launch/nav2_controllers.launch.py
P=$R/src/a1_exploration/launch/nav2_planners.launch.py
mode=${1:-apply}
case "$mode" in
  apply)
    for f in "$U" "$C" "$P"; do [ -f "$f.bak_terrain" ] || cp -a "$f" "$f.bak_terrain"; done
    python3 - "$U" "$C" "$P" <<'PY'
import sys
U, C, P = sys.argv[1:4]
CAM = '''  <link name="terrain_camera_link">
    <inertial><mass value="0.02"/><inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>
    <visual><geometry><box size="0.02 0.06 0.02"/></geometry></visual>
  </link>
  <joint name="terrain_camera_joint" type="fixed">
    <parent link="trunk"/><child link="terrain_camera_link"/>
    <origin xyz="0.29 0.0 0.04" rpy="0 0.6981 0"/>
  </joint>
  <link name="terrain_camera_optical"/>
  <joint name="terrain_camera_optical_joint" type="fixed">
    <parent link="terrain_camera_link"/><child link="terrain_camera_optical"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>
  <gazebo reference="terrain_camera_link">
    <sensor name="terrain_camera" type="gpu_lidar">
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <topic>/$(arg robot_ns)/terrain_camera/depth</topic>
      <ignition_frame_id>terrain_camera_link</ignition_frame_id>
      <ray>
        <scan>
          <horizontal><samples>160</samples><resolution>1</resolution><min_angle>-0.759</min_angle><max_angle>0.759</max_angle></horizontal>
          <vertical><samples>120</samples><resolution>1</resolution><min_angle>-0.61</min_angle><max_angle>0.61</max_angle></vertical>
        </scan>
        <range><min>0.1</min><max>4.0</max><resolution>0.01</resolution></range>
      </ray>
    </sensor>
  </gazebo>
'''
# URDF 에는 주석을 넣지 않고 ASCII 만: gazebo_ros2_control 이 robot_description 을 명령행 인자
# ('--param robot_description:=<xml>')로 넘기는데 rcl 이 값을 YAML 로 읽어서, 주석 속 ':' 나 '#' 이 있으면
# 'Couldn't parse parameter override rule' 로 gzserver 가 죽음 (gazebo_ros2_control issue #295).
# 1차 블록의 주석 '되돌리기: revert' 의 ':' 때문에 2026-11-08 기동 실패. 원본 a1.urdf.xacro 는 주석 0 개.
assert CAM.isascii() and '<!--' not in CAM
import re
s = open(U, encoding='utf-8').read()
s = re.sub(r'  <!-- >>> terrain_camera.*?<!-- <<< terrain_camera -->\n', '', s, flags=re.S)        # 1차 블록 제거
s = re.sub(r'  <link name="terrain_camera_link">.*?<gazebo reference="terrain_camera_link">.*?</gazebo>\n', '',
           s, flags=re.S)                                                                          # 지금 블록 (다시 넣기)
i = s.rindex('</robot>')
s = s[:i] + CAM + s[i:]
open(U, 'w', encoding='utf-8').write(s)
if '<!--' in s and ':' in ''.join(re.findall(r'<!--(.*?)-->', s, re.S)):
    sys.exit('URDF 주석에 ":" -> gazebo_ros2_control 이 못 읽음')
bad = [n + 1 for n, line in enumerate(s.splitlines()) if not line.isascii()]
if bad:
    sys.exit(f'URDF 에 비ASCII 문자 줄 {bad[:5]} -> gazebo_ros2_control 이 못 읽음')

def block(var, fn, key, extra=''):
    return f'''        # >>> terrain_patch (terrain_perception terrain_setup.sh, 되돌리기: revert, 지형 층만 끄기: A1_TERRAIN=0)
        try:
            from terrain_perception import nav2_patch as _np
            _np.{fn}({var}[f'/{{ns}}/{key}/{key}']['ros__parameters'], ns){extra}
        except ImportError as _e:
            print(f'[terrain_patch] terrain_perception 없음 -> 보정 없이 실행: {{_e}}')
        # <<< terrain_patch
'''
TUNE = "\n            _np.tune_controller(params[f'/{ns}/controller_server']['ros__parameters'])"
TUNE_P = "\n            if hasattr(_np, 'tune_planner'):\n                _np.tune_planner(parameters[f'/{ns}/planner_server']['ros__parameters'])"
for path, var, fn, key, extra in ((C, 'params', 'add_local', 'local_costmap', TUNE),
                                  (P, 'parameters', 'add_global', 'global_costmap', TUNE_P)):
    s = open(path).read()
    s = re.sub(r'        # >>> terrain_patch.*?        # <<< terrain_patch\n\n?', '', s, flags=re.S)   # 이전 블록 교체
    anchor = '        with tempfile.NamedTemporaryFile('
    assert s.count(anchor) == 1, path
    s = s.replace(anchor, block(var, fn, key, extra) + '\n' + anchor)
    open(path, 'w').write(s)
PY
    ;;
  revert)
    for f in "$U" "$C" "$P"; do [ -f "$f.bak_terrain" ] && cp -a "$f.bak_terrain" "$f"; done
    ;;
  status) ;;
  *) echo "사용: $0 [apply|revert|status]"; exit 1 ;;
esac
echo "URDF terrain_camera: $(grep -c 'terrain_camera_link' "$U") 줄 (비ASCII 줄 $(LC_ALL=C grep -cP '[^\x00-\x7F]' "$U"), 주석 $(grep -c '<!--' "$U")), nav2 지형 층: controllers $(grep -c terrain_patch "$C") / planners $(grep -c terrain_patch "$P")"
[ "$mode" = status ] && exit 0
source /opt/ros/humble/setup.bash; cd "$R"
colcon build --symlink-install --packages-select a1_champ a1_exploration terrain_perception 2>&1 | tail -1
