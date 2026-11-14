# Gazebo Sim (Fortress) 이전 + record17 (2026-11-10)

사용자 결정: Fortress + Humble, 소스는 결과 폴더 압축(`ros2_hw_src.tgz`), Classic 판은 따로 남김(작업 공간 `~/ros2_hw` 그대로, 새 판은 `~/ros2_hw_gz`). 카메라 이동·목표 보정·인식 1 s 모두 수락, "다른 맵에서도 통하는 일반적 방향" 요구.

## 묶음
- `comm_relay_results\bundles\record17_gz.zip` -> `bash record17_gz/gz_setup.sh` (apt ros-humble-ros-gz ros-humble-ign-ros2-control, src 복사, 모델 복사, 빌드, 단위 시험). 상세 변경표는 `src/comm_relay/GZ_MIGRATION.md`. (수정 1 반영본으로 다시 만듦)
- `bundles\record17_gz_fix1.zip` -> `bash record17_gz_fix1/apply.sh`: 이미 설치한 `~/ros2_hw_gz` 에 수정 1 파일만 덮고 빌드.
- 클라우드 작업 사본: `/home/claude/gz/ws` (RoboStack conda `humble` 환경: `/home/claude/gz/env.sh`, Xvfb :99 — 세션 재시작 때 꺼지니 다시 켤 것, `OGRE_RESOURCE_PATH=$CONDA_PREFIX/lib/OGRE`, 시뮬 `/home/claude/gz/startsim.sh`, 정리 `/home/claude/gz/killall.sh` — pkill 패턴이 호출한 셸 명령줄과 겹치면 셸이 죽으니 따로 호출).

## 이전 내용 요약
- world: 시스템 플러그인(Physics DART, UserCommands, SceneBroadcaster, Sensors, Imu), 재질 스크립트 -> 색, person_standing 모델 패키지에 포함. 렌더 엔진은 월드에 박지 않고 world.launch.py `render_engine`(기본 ogre).
- URDF: ign_ros2_control(namespace), OdometryPublisher(정답), gpu_lidar(라이다), camera, 지형 센서 gpu_lidar 160x120 빔, imu(ENU). 주석·비ASCII 없음 유지.
- launch: ros_gz_sim gz_sim.launch.py(`--render-engine`) / create, ros_gz_bridge(/clock + 로봇별 config_file). 토픽 이름 Classic 과 같게.
- 도구: common.sh 정리 패턴, 스폰 로그 'OK creation of entity', 작업 공간 경로를 설치 위치에서 계산(고정 ~/ros2_hw 제거).

## 클라우드 시험 결과 (Humble+Fortress 6.16, CPU 2, GPU 없음, Xvfb)
- 월드 로드, /clock 약 970 Hz. 세 개체 스폰, 두 로봇 컨트롤러 활성, 서기 z 0.218.
- 센서: scan 360 유효(몸 맞은 점 없음, 최소 0.78 m), imu, 640x480 rgb8 카메라 fx 253.9(=320/tan 0.9), 지형 점 구름 frame terrain_camera_link(x 앞).
- IMU 기본이 처음 방향 기준이라 a1_2 map 이 90° 돌아감 -> ENU 지정 후 imu/odom/정답 yaw 일치.
- 앞 카메라·지형 카메라가 몸통 메시(trunk.dae 앞머리 x 0.281 m) 안 -> (0.29,0,0.0), (0.29,0,0.04)로 이동: 카메라 'H' 가림 사라짐, 지형 깊이 유효 7 -> 19197/19200 점(ogre2), 18920/19200(ogre1 gpu_lidar).
- 보행(DART, 재조정 없음): 평지 직진 0.2 m/s 에 97~101 %, 회전 0.5 rad/s 약 87 %, 경사로 오름.
- odom: CHAMP odom/raw 94~95 %, EKF /odom 은 RTF 0.2 에서 36 %, RTF 0.33 에서 77 % -> 느린 시뮬 탓으로 봄(미확정) -> 사용자 PC walk_check.sh.
- 전체 미션(ogre2, RTF 0.06): 모든 노드 기동·오류 없음, Nav2 4/4, 목표 배정·계획·추종·도착 1, 지형 점 처리, 사람 인식 장 저장. 120 s '이동 부족' 무효 -> 탐색 성능 평가는 사용자 PC.
- 단위 시험: comm_relay 66, terrain_perception 17 통과.

## 수정 1 (사용자 PC 첫 실행 실패, 2026-11-10 10:08)
- 증상: walk_check.sh 23 s 에 `ign gazebo` 죽음, `terminate ... Ogre::UnimplementedException`. WSLg(D3D12)에서 ogre2 가 GL3PlusTextureGpu::copyTo 미구현으로 죽는 알려진 문제(gz-rendering #662, gz-sim #920). 우회: ogre1 또는 LIBGL_ALWAYS_SOFTWARE=1(느림).
- 조치: 렌더 엔진 기본 ogre(ogre1). 클라우드에서 ogre1 시험 중 depth_camera 두 대째에 `ItemIdentityException depth_points_vs.glsl` 로 죽음 -> 지형 센서를 같은 화각(1.518 x 1.22 rad)·해상도(160x120)·거리(0.1~4 m) gpu_lidar 로 바꿈 (점 구름 좌표 동일 x 앞, 브리지 gz 토픽 동일). 깊이 영상·info 브리지 항목 삭제(쓰는 곳 없음).
- 결과(클라우드 ogre1): 서버 안 죽음, 컨트롤러 4, 모든 센서, 카메라 가림 없음, RTF 0.33(ogre2 0.26), 직진 97 %, odom/raw 95 %, /odom 77 %.

## record17 기능 (일반화 재검토 반영)
1. 앞 카메라 몸 밖으로 (로봇 모델 수정이라 맵 무관). survivor cam_dx 0.29.
2. Husky 금지 반경 대신 **전역 비용지도 목표 보정**(nav_executor + mission_logic.refine_goal): 목표 칸 비용 > 127(모름 255 제외)이면 0.5 m 안 가장 가까운 여유 칸으로, 없으면 블랙리스트. 파라미터 goal_refine, goal_max_cost, goal_refine_radius.
3. 사람 인식 period 1 s (파라미터).
4. 통신 시험 명령 3 번 보내기 + 스위치가 받은 off/on 횟수 출력.

## 남은 '이 맵 전용' 요소 (Gazebo Sim 판이 Classic 수준 확인 뒤 일반화)
- 남북 경계 y=0 고정 -> Husky 위치·두 시작점으로 경계선 계산.
- comm_zones.yaml 건물 범위 숫자 -> 월드별 설정 파일/끄기.
- 시작 위치·마네킹 정답 -> launch 인자·평가 설정.
- 모니터·실행기의 벽시계 기준 시간 한도(TURNING 30 s, 멈춤 30 s, 120 s 무효 판정) -> 시뮬 시간 기준 후보.

## 사용자 PC 다음 단계
1. (이미 설치함) `bash record17_gz_fix1/apply.sh`
2. `bash ~/ros2_hw_gz/src/comm_relay/tools/walk_check.sh`
3. `bash ~/ros2_hw_gz/src/comm_relay/tools/gazebo_test_mission.sh mission_commoff`, `DUR=600 ... mission`
판정: Classic record15/16 수준(구역 ≥ 95 %, 낙상 0, 통신 복귀·재개, 면적 64~107 m²), 사람 발견 위치 오차, 목표 보정 로그.
