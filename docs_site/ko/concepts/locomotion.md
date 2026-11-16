# A1 보행 (CHAMP)

## 구성

A1 은 [CHAMP](https://github.com/chvmp/champ) 보행 제어기로 걷습니다. CHAMP 는 `cmd_vel`(vx, vy, wz)을 받아 발 궤적을 만들고, 역기구학으로 관절 각도를 계산합니다. 관절 명령은 `ign_ros2_control` 을 거쳐 Gazebo 관절에 들어갑니다.

| 구성 | 패키지 | 노드 |
| --- | --- | --- |
| 로봇 모델 | `a1_champ` (`urdf/a1.urdf.xacro`, `scripts/make_a1_urdf.py`) | — |
| 보행 제어 | `champ_base` | `quadruped_controller_node` |
| 상태 추정 | `champ_base` + `robot_localization` | `state_estimation_node`, `ekf_node` |
| 관절 제어 | `ign_ros2_control` | `joint_group_effort_controller` |
| launch | `a1_champ/launch/a1.launch.py` | `robot_ns`, `x`, `y`, `z`, `yaw` 인자 |

`sim.launch.py` 는 `a1_1` 을 6 s, `a1_2` 를 20 s 뒤에 차례로 스폰합니다. 기동 뒤 soft start 3 s + 기립 안정 시간이 필요해 `run_mission.sh` 는 컨트롤러 활성 뒤 8 s 를 더 기다립니다.

## 센서

| 센서 | 종류 | 설정 | 토픽 |
| --- | --- | --- | --- |
| 평면 라이다 | gpu_lidar | 360 점, 360°, 0.15~10 m, 10 Hz, 높이 약 0.28 m | `/<ns>/scan` |
| 앞 카메라 | camera | 수평 FOV 1.8 rad (약 103°), 640 × 480, 2 Hz | `/<ns>/front_camera/image_raw` |
| 지형 깊이 | gpu_lidar (깊이 센서처럼) | 160 × 120, 수평 ±43°, 수직 ±35°, 0.1~4 m, 5 Hz | `/<ns>/terrain_camera/points` |
| IMU | imu | 100 Hz | `/<ns>/imu/data` |

:::{note}
시뮬 서버는 **소프트웨어 렌더링**으로 돌립니다. WSL 에서 GPU 렌더링을 쓰면 라이다 거리값이 전부 0.15 m(range_min)로 나오는 문제가 있었습니다.
:::

## 보행 설정 (a1_champ/config/gait.yaml)

| 값 | 설정 | 뜻 |
| --- | --- | --- |
| `max_linear_velocity_x` | 0.5 m/s | 앞뒤 최대 |
| `max_linear_velocity_y` | 0.25 m/s | 옆 최대 (실제로는 옆걸음이 거의 안 나감) |
| `max_angular_velocity_z` | 1.0 rad/s | 회전 최대 |
| `swing_height` | 0.04 m | 발 드는 높이 → 지형 인식 `max_step` 과 같게 |
| `stance_duration` | 0.25 s | 디딤 시간 |
| `nominal_height` | 0.2 m | 몸 높이 |
| `odom_scaler` | 1.125 | 다리 odom 눈금 (2026-11-12 에 1.11 → 1.125) |
| `com_x_translation` | −0.015 m | 무게중심 보정 |

측정 결과는 {doc}`../how_to/measure_gait`.

## 알아 둘 성질

- **옆걸음은 거의 안 나갑니다.** 옆걸음 위주로 움직이면 실제/odom 이동 비율이 0.05~0.24 였습니다(다리 odom 만 움직였다고 셈). 그래서 벗어나기·조향은 모두 앞뒤 축으로만 하고, 필요하면 먼저 제자리 회전합니다.
- **경사 위 제자리 회전은 미끄럽니다.** 10° 경사로 위에서 6 s 회전에 0.5 m 미끄러져 넘어진 적이 있습니다. 기울기 6° 넘으면 제자리 회전을 하지 않습니다.
- **막혀도 다리 odom 은 전진합니다.** 끼여서 제자리걸음을 하면 odom 이 계속 늘어 SLAM 위치가 흐릅니다.
- **cmd_vel 타임아웃이 없습니다.** 마지막 명령을 계속 따르므로 `cmd_vel_arbiter` 가 입력이 없을 때 0 명령을 냅니다.

## 원본 CHAMP 와 다른 점

이 프로젝트는 Gazebo Classic(ODE)에서 Gazebo Sim(DART)으로 옮기면서 보행·odom 을 다시 맞췄습니다. 그 과정은 `docs/project_docs/champ_fix_log.md`, `gz_migration.md`, `gait_experiment_log.md` 에 있습니다. 강화학습 보행 시도는 `a1_rl` 패키지와 `rl_sar_integration.md` 에 남아 있지만 지금 임무에는 쓰지 않습니다.
