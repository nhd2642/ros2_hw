# 패키지

모든 패키지는 `code/ros2_hw_gz_src/` 에 있습니다 (`ws/src` 가 이곳을 가리킴).

## comm_relay (ament_python)

임무 계층. 이 프로젝트에서 새로 만든 코드 대부분.

| 경로 | 내용 |
| --- | --- |
| `comm_relay/husky_mission.py` | 임무 단계, 사용자 명령, 하트비트, 사람 집계 |
| `comm_relay/husky_driver.py` | Husky 주행·위치 추정 |
| `comm_relay/a1_follower.py` | A1 추종, Husky 기준 자세 보정 |
| `comm_relay/nav_executor.py` | Nav2 계획·추종·실패 처리·완료·홈 복귀 |
| `comm_relay/return_relay.py` | 통신 끊김 복귀 |
| `comm_relay/cmd_vel_arbiter.py` | 속도 명령 중재 |
| `comm_relay/frontier_comm_filter.py` | 구역·블랙리스트 프런티어 필터 |
| `comm_relay/survivor_detector.py` | 사람 인식·위치·보고 |
| `comm_relay/clock_throttle.py` | `/clock` → `/clock_slow` |
| `comm_relay/*_logic.py` | ROS 없는 판단 로직 (`drive`, `mission`, `return`, `nudge`, `survivor`, `arbiter`) |
| `launch/mission.launch.py` | 임무 계층 전체 |
| `tools/` | `run_mission.sh`, `mission_watch.py`, 진단 도구들 |
| `test/` | 단위 시험 |

## a1_exploration (ament_cmake)

SLAM·Nav2 launch 와 탐색 스크립트 (팀원 코드 기반).

| 경로 | 내용 |
| --- | --- |
| `launch/dual_slam.launch.py` | 로봇별 slam_toolbox + scan_ready_relay |
| `launch/slam.launch.py` | 로봇 한 대 SLAM |
| `launch/nav2_planners.launch.py` | 로봇별 planner_server + 전역 비용지도 |
| `launch/nav2_controllers.launch.py` | 로봇별 controller_server (DWB) + 국소 비용지도 |
| `config/slam_params.yaml` | slam_toolbox 설정 |
| `scripts/nav_goal_bridge.py` | 지도 병합(`shared_map_with_robots` 상속), 좌표 변환 |
| `scripts/frontier_detector.py` | 프런티어 추출 |
| `scripts/goal_assigner.py` | 두 로봇 목표 배정 |
| `scripts/scan_ready_relay.py` | 스캔 전처리 (기울기·다른 로봇) |
| `scripts/shared_map_merger.py`, `shared_map_with_robots.py` | 병합 기반 클래스 |
| `scripts/preview_nav2_paths.py`, `test_follow_paths.py` | 개발용 |

## terrain_perception (ament_python)

| 경로 | 내용 |
| --- | --- |
| `terrain_perception/terrain_node.py` | 깊이 점구름 → 위험 칸 / 빈 칸 |
| `terrain_perception/grid.py` | 위험 판정·기억 (ROS 없음) |
| `terrain_perception/nav2_patch.py` | Nav2 launch 에 지형 층 패치 |
| `launch/terrain.launch.py` | `max_step`, `cloud_frame` 인자 |
| `tools/terrain_setup.sh`, `tools/fake_depth.py` | 패치 적용, 가짜 깊이 입력 |

## disaster_sim (ament_cmake)

| 경로 | 내용 |
| --- | --- |
| `scripts/make_world.py` | **배치 원본** → `worlds/mission.world`, `config/mission.yaml` |
| `launch/sim.launch.py` | 월드 + Husky + A1 두 대 (`gui`, `start_set`) |
| `launch/world.launch.py` | 월드만 (`world`, `gui`, `render_engine`, `software_rendering` …) |
| `models/husky/` | Husky A200 (DiffDrive, 2D 라이다, IMU) |
| `models/person_standing/` | 사람 모델 |
| `worlds/disaster.world` | 이전 구조의 원본 맵 |

## a1_champ (ament_cmake)

| 경로 | 내용 |
| --- | --- |
| `urdf/a1.urdf.xacro` | A1 모델 + 센서 + ros2_control |
| `config/gait.yaml`, `joints.yaml`, `links.yaml`, `ros_control.yaml` | CHAMP 설정 |
| `launch/a1.launch.py` | 로봇 한 대 (`robot_ns`, `x`, `y`, `z`, `yaw`) |
| `launch/gait_test.launch.py`, `worlds/gait_test.world` | 평지 보행 시험 |
| `scripts/gait_eval.py`, `joint_tracking.py`, `run_exp.sh` | 보행 평가 |

## 그 밖

| 패키지 | 내용 |
| --- | --- |
| `champ/` | CHAMP 보행 제어기 (`champ`, `champ_base`, `champ_msgs`, `champ_bringup`, `champ_description`, `champ_gazebo`, `champ_config`, `champ_navigation`) |
| `champ_teleop` | 키보드·조이스틱 조종 |
| `a1_rl` | 강화학습 보행 실험 (지금 임무에서는 안 씀) |
