# 시스템 구조

이 페이지는 패키지와 노드가 어떻게 계층을 이루고, 어떤 토픽으로 이어지며, 어떤 순서로 뜨는지 설명합니다.

## 계층

```{graphviz} ../../diagrams/architecture.dot
```

| 계층 | 패키지 | 이 프로젝트에서 한 일 |
| --- | --- | --- |
| 임무 | `comm_relay` | 거의 전부 새로 작성: 임무 단계, Husky 주행, A1 추종, Nav2 실행기, 통신 복귀, 사람 인식, 중재 |
| 탐색 | `a1_exploration` | 팀원 코드(프런티어·목표 배정·지도 병합) 위에 구역·블랙리스트·자세 보정 등을 덧붙임 |
| 항법·인식 | `slam_toolbox`, Nav2, `terrain_perception` | SLAM·Nav2 설정, 지형 층 새로 작성 |
| 시뮬레이션 | `disaster_sim`, `a1_champ`, `champ` | 임무 v2 월드·Husky 모델 새로 작성, Gazebo Sim 으로 옮김 |

설계 원칙 몇 가지가 전체에 걸쳐 있습니다.

- **좌표는 설정 파일 하나에서.** 노드는 위치를 코드에 두지 않고 `mission.yaml` 만 읽습니다 ({doc}`world_layout`).
- **판단과 ROS 를 나눔.** `*_logic.py` 는 ROS 없이 시험할 수 있는 순수 로직, 노드 파일은 메시지 변환만 ({doc}`../how_to/run_unit_tests`).
- **CHAMP 에 쓰는 쪽은 하나.** 다섯 가지 속도 명령원은 모두 `cmd_vel_arbiter` 를 거칩니다 ({doc}`communication_return`).
- **Nav2 는 계획·추종 서버만.** `bt_navigator`·복구 동작 없이, 계획 → 추종 → 실패 처리를 `nav_executor` 가 직접 합니다 ({doc}`exploration`).

## 단계별로 쓰는 노드

같은 A1 이라도 임무 단계에 따라 몰고 가는 노드가 다릅니다.

| 단계 | A1 을 모는 노드 | 속도 명령 토픽 | 중재 우선순위 |
| --- | --- | --- | --- |
| TRANSIT (도로 이동) | `a1_follower` | `/<ns>/cmd_vel_follow` | 0 |
| EXPLORE / REGROUP | `nav_executor` → Nav2 controller | `/<ns>/cmd_vel_nav_test` | 1 |
| 통신 끊김 | `return_relay` | `/<ns>/cmd_vel_return` | 2 |
| 사람 발견 (1 초) | `survivor_detector` | `/<ns>/cmd_vel_hold` | 3 |
| 사람이 조종 | teleop | `/<ns>/cmd_vel_teleop` | 4 |

## 주요 토픽 흐름

**임무 상태 방송** — `husky_mission` 이 1 Hz 로 `/mission/state` (`MissionState`, {doc}`../reference/interfaces`)를 냅니다. 여기에 단계, 세트 번호, 로봇별 구역 사각형, 통신 상태가 들어 있고 `a1_follower`, `nav_executor`, `return_relay`, `frontier_detector`, `frontier_comm_filter`, `goal_assigner`, `terrain_node` 가 이것을 보고 동작을 바꿉니다. 사용자는 action `/husky/run_mission` 으로 임무를 요청하고 service `/husky/set_comm`·`/husky/get_survivors` 로 통신을 끄고 켜거나 생존자를 조회합니다.

**탐색 목표** — SLAM 지도 → `nav_goal_bridge`(공유 지도) → `frontier_detector` → `frontier_comm_filter` → `goal_assigner` → `nav_goal_bridge`(로봇 map 좌표로) → `nav_executor` → Nav2. 자세한 내용은 {doc}`exploration`.

**사람 보고** — `survivor_detector` → `/husky/survivor_report` (`SurvivorReport`) → `husky_mission` 이 같은 사람을 묶어 `/husky/survivors` (`SurvivorList`) 로 집계.

**세트 완료** — `nav_executor` 가 홈에 도착하면 `/<ns>/set_done` (latched) → `husky_mission`.

전체 목록은 {doc}`../reference/topics`.

## 기동 순서

```{graphviz} ../../diagrams/launch_order.dot
```

순서가 중요한 이유:

1. **시뮬이 먼저.** `/clock` 이 없으면 시뮬 시간을 쓰는 모든 노드가 멈춥니다. A1 은 관절 컨트롤러가 켜지고 서서 안정될 때까지(약 8 초) 기다려야 SLAM 이 기울어진 스캔을 받지 않습니다.
2. **SLAM → Nav2.** Nav2 전역 비용지도는 SLAM 지도(`map` 프레임)를 static 층으로 씁니다.
3. **임무 계층이 마지막.** `nav_executor` 는 Nav2 액션 서버가 준비돼야 상태 하트비트를 냅니다.

## 시간과 CPU

- 모든 판정·한도는 **시뮬 시간** 기준입니다. RTF 가 0.3 이면 시뮬 900 초 = 실제 약 50 분.
- CHAMP 보행은 1 kHz `/clock` 이 필요하지만, rclpy 노드는 `/clock` 을 받기만 해도 노드마다 CPU 약 20 % 를 썼습니다(파이썬 노드 14 개, RTF 0.36). 그래서 `clock_throttle` 이 50 Hz `/clock_slow` 를 만들고, 파이썬 노드는 모두 `/clock:=/clock_slow` 로 리맵합니다 ({doc}`coordinates_time`).
- 예외: 하트비트 끊김 판정(`return_relay`)은 벽시계 기준입니다.

## 이전 구조와의 차이

임무 v2 (2026-11-12~) 이전에는 건물 한 채, Husky 고정, 구역 경계 y = 0 고정 구조였습니다. 그때의 노드 `husky_comm_switch`, `husky_zone_planner`, `husky_survivor_board` 는 `husky_mission` 하나로 합쳐졌고, `los_checker`, `zones.py`, `comm_zones.yaml` 은 `/mission/state` 의 구역 사각형으로 바뀌었습니다. 이전 구조의 그림과 기록은 {doc}`../results/history`.

```{screenshot} rqt_graph_v2.png
임무 실행 중 rqt_graph (Nodes/Topics (active), 네임스페이스 a1_1 만 펼친 상태).
```
