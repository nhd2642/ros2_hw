# launch 와 스크립트

## run_mission.sh

```bash
bash comm_relay/tools/run_mission.sh [start_set | clean]
```

| 인자 / 환경 변수 | 기본 | 뜻 |
| --- | --- | --- |
| 첫 인자 `start_set` | 0 | 0 = 처음부터, k = 세트 k 직전 위치에서 시작 |
| 첫 인자 `clean` | | 남은 프로세스 정리만 하고 끝 |
| `GUI` | 0 | 1 = Gazebo 화면 |
| `START` | 1 | 0 = `start` 를 자동으로 보내지 않음 |
| `SETS` | 99 (전부) | n 세트만 하고 끝 |
| `DUR` | 6000 | 시뮬 초 한도 |
| `COMM_OFF_AT` | 0 (안 함) | 탐색 시작 뒤 이 시뮬 초에 통신 0 |
| `COMM_OFF_FOR` | 60 | 그 뒤 이 시뮬 초 뒤 통신 1 |
| `OUT` | `<저장소>/results/<시각>_set<k>` | 결과 폴더 |
| `ROS_DOMAIN_ID` | 20~79 무작위 | 겹치지 않게 |

하는 일: 정리 → 시뮬 → (컨트롤러 활성 + 8 s) → SLAM → Nav2 + 지형 → (서버 4 개 active, 실패하면 한 번 다시) → 임무 계층 → `start` 3 번 → `mission_watch.py` → `mission_events.txt` 뽑기 → 종료 시 정리.

## launch 파일

### disaster_sim/sim.launch.py

월드 + Husky + A1 두 대. Husky 4 s, `a1_1` 6 s, `a1_2` 20 s 뒤 스폰.

| 인자 | 기본 | 뜻 |
| --- | --- | --- |
| `gui` | true | false = 서버만 |
| `start_set` | 0 | 1 이상이면 그 앞 세트 주차점·추종 자리에 놓음 |

### disaster_sim/world.launch.py

| 인자 | 기본 | 뜻 |
| --- | --- | --- |
| `world` | `mission.world` | `worlds/` 안 파일 (원본 맵 `disaster.world`) |
| `gui` | true | |
| `render_engine` | `ogre` | `ogre`(WSL 용) / `ogre2` |
| `software_rendering` | true | 시뮬 서버만 소프트웨어 GL (라이다 거리값 문제 회피) |
| `headless_rendering` | false | |
| `verbose` | false | |

### a1_exploration

| launch | 내용 |
| --- | --- |
| `dual_slam.launch.py` | 두 로봇 slam_toolbox + scan_ready_relay |
| `nav2_planners.launch.py` | 두 로봇 planner_server + 전역 비용지도 + lifecycle_manager |
| `nav2_controllers.launch.py` | 두 로봇 controller_server + 국소 비용지도. 환경 변수 `A1_DWB_DIAG=1` 이면 DWB 평가 발행 |

### terrain_perception/terrain.launch.py

| 인자 | 기본 |
| --- | --- |
| `max_step` | 0.04 |
| `cloud_frame` | `auto` |

### comm_relay/mission.launch.py

| 인자 | 기본 | 뜻 |
| --- | --- | --- |
| `start_set` | 0 | sim.launch.py 와 같은 값 |
| `husky` | true | false = `husky_mission` 은 사용자가 따로 띄움 |

띄우는 것: `husky_driver`, `a1_follower`, `return_relay`, `cmd_vel_arbiter`, `frontier_comm_filter`, `nav_executor`, `survivor_detector`, (`husky_mission`), 그리고 `a1_exploration/scripts` 의 `nav_goal_bridge.py`, `frontier_detector.py`, `goal_assigner.py`. 파이썬 노드는 전부 `/clock:=/clock_slow`, `use_sim_time:=true`.

:::{note}
`clock_throttle` 은 `world.launch.py`(그래서 `sim.launch.py`)에서 월드와 함께 뜹니다. 임무 계층보다 먼저 `/clock_slow` 가 있어야 합니다.
:::

## 도구 스크립트 (comm_relay/tools)

| 스크립트 | 용도 |
| --- | --- |
| `run_mission.sh` | 임무 전체 |
| `mission_watch.py` | 관찰 기록·판정 (`<폴더> <start_set> <세트 수> <한도> [통신 끔 시각] [끈 시간]`) |
| `gait_sweep.sh` | 보행 속도 응답 |
| `walk_check.sh`/`.py` | 보행·odom 점검 |
| `lowspeed_check.sh`/`.py` | 저속 명령 추종 |
| `slam_diag.sh`/`.py` | SLAM 지도 진단 |
| `nav_probe.sh`/`.py` | Nav2 계획 실패 진단 |
| `nav_debug.sh`/`.py` | DWB 정지 진단 |
| `terrain_check.sh`/`.py` | 지형 위험 확인 |
| `lidar_check.sh`/`.py` | 라이다 값 점검 |
| `costmap_check.py`, `costmap_diag.py`, `tf_check.py`, `dwb_diag.py` | 비용지도·TF·DWB 진단 |
| `map_snapshot.py` | 지도 + 스캔 PNG |
| `sim_sleep.py` | 시뮬 시간으로 기다림 |
| `common.sh` | 도구 공통 함수 (정리, 시각화 `VIS=1`) |
| `fastdds_udp.xml` | Fast DDS UDP 전용 설정 |
| `vis.rviz` | 로봇별 RViz 템플릿 (`/NS/`) |
