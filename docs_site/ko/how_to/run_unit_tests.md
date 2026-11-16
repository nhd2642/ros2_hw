# 단위 시험 실행하기

판단 로직은 ROS 없이 시험할 수 있게 별도 모듈로 나뉘어 있습니다. 시뮬을 띄우기 전에 항상 먼저 돌리세요 (1 분 안).

## 실행

```bash
source /opt/ros/humble/setup.bash          # test_turn 이 nav_executor(rclpy) 를 불러옴
cd ~/ros2_hw/ws/src/comm_relay && python3 -m pytest -q test
cd ~/ros2_hw/ws/src/terrain_perception && python3 -m pytest -q test
```

기대 결과 (2026-11-17 확인):

```text
64 passed in 0.79s      # comm_relay
18 passed in 3.42s      # terrain_perception
```

특정 파일·시험만:

```bash
python3 -m pytest -q test/test_drive.py
python3 -m pytest -q test/test_logic.py -k return
```

## 무엇을 시험하나

| 시험 파일 | 대상 모듈 | 시험 수 | 확인하는 것 |
| --- | --- | --- | --- |
| `comm_relay/test/test_drive.py` | `drive_logic` | 10 | 장애물 회피 조향, Husky 몸 찾기·상자 맞춤 |
| `comm_relay/test/test_logic.py` | `return_logic`, `arbiter_logic` | 20 | 복귀 상태 전이, breadcrumb 되짚기, 우선순위 중재 |
| `comm_relay/test/test_mission_logic.py` | `mission_logic` | 7 | 탐색 완료 판정, 구역·경로 검사, 사람 집계, 시작 자세, 목표 보정 |
| `comm_relay/test/test_nudge.py` | `nudge_logic` | 11 | 계획 실패 연속 판정, '옮겨 보기' 방향, 0 명령 감시 |
| `comm_relay/test/test_survivor.py` | `survivor_logic` | 9 | 사람 위치 계산, 같은 사람 묶기, 보고 대기열 |
| `comm_relay/test/test_turn.py` | `nav_executor` | 3 | 경로 앞쪽 방향 계산, 각도 차 (먼저 돌기 판단에 씀) |
| `comm_relay/test/test_costmap_diag.py` | `tools/costmap_diag` | 4 | 비용지도 진단 도구 |
| `terrain_perception/test/test_grid.py` | `grid` | 10 | 단차·경사 위험 칸 판정, 기억 |
| `terrain_perception/test/test_nav2_patch.py` | `nav2_patch` | 6 | Nav2 launch 에 지형 층을 넣는 패치 |
| `terrain_perception/test/test_on_ramp.py` | `grid` | 2 | 경사로 위에서 지면을 위험으로 안 보는지 |

## 새 로직을 넣을 때

ROS 노드(`*_node.py`, `husky_mission.py` …)에 판단을 직접 쓰기보다, `*_logic.py` 에 순수 함수·클래스로 두고 노드는 메시지 변환만 하는 지금 방식을 따르세요. 그러면 위처럼 시뮬 없이 시험할 수 있습니다.
