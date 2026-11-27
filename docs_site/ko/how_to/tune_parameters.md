# 파라미터 조정하기

이 페이지는 **자주 바꾸는 값이 어디 있는지**와 바꾸는 방법을 모았습니다. 모든 파라미터 목록은 {doc}`../reference/nodes` 에 있습니다.

## 바꾸는 세 가지 방법

1. **launch 에서 넘기기** — `mission.launch.py` 의 `node(...)` 호출에 키워드로 추가 (예: `node('nav_executor', idle_done=120.0)`). 다시 빌드할 필요 없음(`--symlink-install`).
2. **실행 중 바꾸기** — `ros2 param set /nav_executor done_after 45.0`. 다만 대부분의 노드는 시작할 때 한 번만 읽으므로 다시 띄워야 반영됩니다.
3. **설정 파일** — SLAM(`a1_exploration/config/slam_params.yaml`), Nav2(`a1_exploration/launch/nav2_*.launch.py` 안 사전), 보행(`a1_champ/config/gait.yaml`).

:::{warning}
`a1_exploration/launch/nav2_*.launch.py` 에는 `terrain_setup.sh` 같은 스크립트가 넣은 **패치 블록**이 있습니다. 파일을 통째로 덮지 말고 필요한 줄만 고치세요.
:::

## 임무 (husky_mission)

| 파라미터 | 기본 | 뜻 |
| --- | --- | --- |
| `explore_timeout` | 1500 s | 탐색이 이보다 길면 REGROUP 으로 A1 을 부름 |
| `regroup_timeout` | 300 s | REGROUP 에서 이만큼 기다리면 다음 세트로 |
| `arrive_tol` | 0.6 m | A1 이 자기 추종 자리에서 이 안이면 '도착' |
| `arrive_wait` | 60 s | Husky 도착 뒤 A1 을 최대 이만큼 기다림 |
| `lag_stop` / `lag_go` | 2.5 / 1.5 m | A1 이 이만큼 처지면 Husky 정지 / 이 안으로 오면 다시 출발 |

## 속도

| 대상 | 어디 | 값 |
| --- | --- | --- |
| Husky 주행 최대 | `husky_driver` `v_max` | 0.25 m/s |
| A1 추종 최대 | `a1_follower` `v_max` | 0.35 m/s |
| A1 탐색 (Nav2 MPPI) | `nav2_controllers.launch.py` `vx_max` | 0.35 m/s |
| 복귀 직접 주행 | `return_relay` `vx` | 0.15 m/s |
| CHAMP 상한 | `gait.yaml` `max_linear_velocity_x` | 0.5 m/s |

속도를 올리기 전에 {doc}`measure_gait` 로 그 속도의 응답과 기울기를 확인하세요. 탐색은 0.2 m/s(DWB·MPPI)와 0.35 m/s(MPPI)만 시험했습니다.

## 탐색 실행기 (nav_executor)

| 파라미터 | 기본 | 뜻 |
| --- | --- | --- |
| `done_after` | 30 s | 갈 프런티어 0 개가 이만큼 이어지면 탐색 완료 → 홈 |
| `idle_done` | 90 s | 탐색 중 추종을 한 번도 못 시작한 채 이만큼 지나면 완료 처리 |
| `goal_timeout` | 90 s | 목표 하나를 따르는 최대 시간 |
| `min_dwell` | 8 s | 목표를 바꾸기 전 최소로 따를 시간 |
| `commit_radius` | 1.0 m | 지금 목표 이 안에 프런티어가 남았으면 목표 유지 |
| `turn_first` / `turn_done` | 0.785 / 0.35 rad | 경로가 45° 넘게 틀어지면 먼저 제자리 회전, 20° 안이면 멈춤 |
| `turn_max_tilt` | 6° | 이보다 기울어 있으면 제자리 회전 안 함 (경사로 미끄럼 방지) |
| `fall_tilt` | 45° | 이보다 기울면 넘어짐(FALLEN) |
| `home_retries` / `home_timeout` | 8 / 300 s | 홈 복귀 재시도 횟수 / 시간 한도 |
| `fail_area_n` | 3 | 같은 자리 실패가 세트당 이 횟수면 그곳 포기 |
| `nudge_after` / `nudge_dist` | 3 / 0.3 m | 같은 자리 계획 실패 횟수 / 옮겨 보기 거리 |

## 목표 배정 (goal_assigner)

| 파라미터 | 기본 | 뜻 |
| --- | --- | --- |
| `goal_spread` | 3.0 m | 두 로봇의 목표가 이보다 가까우면 벌점 (공동 건물에서 갈라지게) |
| `goal_clearance` | 0.40 m | 목표 칸은 장애물에서 이 이상 떨어져야 함 |
| `robot_blacklist_radius` / `_ttl` | 0.9 m / 60 s | 실패한 목표 둘레를 그 로봇에게 이만큼·이 시간 동안 안 줌 |
| `zone_goal_margin` | 0.3 m | 구역 가장자리 여유 |

## 사람 인식 (survivor_detector)

| 파라미터 | 기본 | 뜻 |
| --- | --- | --- |
| `conf` | 0.5 | 사람 점수 기준. 실제 사람은 0.89~1.00, 오탐(기둥 옆)은 0.41 였음 |
| `period` | 1.0 s | 로봇마다 추론 간격 |
| `model_dir` | `<ws>/person_ai_models/yolov4-tiny` | 모델 파일 폴더 |

## Nav2 비용지도

| 값 | 어디 | 현재 |
| --- | --- | --- |
| 전역 팽창 반경 | `nav2_planners.launch.py` `inflation_radius` | 0.70 m (fix18 에서 0.35 → 0.70) |
| 국소 팽창 반경 | `nav2_controllers.launch.py` `inflation_radius` | 0.35 m |
| footprint | 두 파일 모두 | 0.60 × 0.32 m 사각형 |
| 계획기 | `nav2_planners.launch.py` | NavFn 다익스트라 (`use_astar: False`), tolerance 0.25 m |
| 도착 판정 | `nav2_controllers.launch.py` | xy 0.20 m, 방향 무관 (yaw 3.14) |

:::{tip}
값을 하나 바꾸면 **같은 설정으로 두 번 이상** 돌려 비교하세요. 같은 코드로도 세트 탐색 시간이 180~810 s 로 달라집니다.
:::
