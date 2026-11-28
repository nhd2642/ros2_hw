# comm_relay — Husky 임무·주행, A1 추종·탐색 실행·통신 복귀·사람 보고

전체 설계와 실행법: 저장소 `docs/mission_design.md`. 이전 구조의 기록: `docs/system_structure.md`, `docs/project_docs/`.

| 노드 | 하는 일 |
|---|---|
| `husky_mission` | 사용자 요청(action `RunMission`, service `SetComm`·`GetSurvivors`; `mission_interfaces`), 단계(TRANSIT → EXPLORE → REGROUP), 구역·홈 지정, 통신 하트비트, 사람 집계 |
| `husky_driver` | Husky 목표점 주행 + 2D 라이다 회피, 위치 추정 |
| `a1_follower` | 이동 단계에 Husky 뒤 자기 자리 추종 + 라이다 회피 |
| `nav_executor` | 탐색 단계에 Nav2 계획·추종·실패 처리, 완료 판정, 탐색 시작 위치 복귀 |
| `return_relay` | 통신이 끊기면 지나온 길을 되짚어 탐색 시작 위치로 |
| `cmd_vel_arbiter` | 조종 > 사람 발견 정지 > 통신 복귀 > Nav2 > 추종, 입력이 끊기면 0 명령 |
| `frontier_comm_filter` | 지금 구역 밖·실패 목표 주변 프런티어 제거 |
| `survivor_detector` | 카메라 사람 인식 → 위치 → Husky 보고 (끊기면 모았다가 보냄) |
| `clock_throttle` | `/clock` 1 kHz → `/clock_slow` 50 Hz (파이썬 노드용) |

ROS 없는 로직(단위 시험 `test/`): `drive_logic`(조향), `mission_logic`(구역·완료·사람 집계·목표 보정), `return_logic`, `nudge_logic`,
`survivor_logic`, `arbiter_logic`.

```
python3 -m pytest -q test                      # 단위 시험
bash tools/run_mission.sh                      # 임무 전체 (시뮬부터), 결과 <저장소>/results/
SETS=1 bash tools/run_mission.sh 4             # 공동 건물만
bash tools/gait_sweep.sh                       # A1 보행 속도 응답
bash tools/run_mission.sh clean                # 남은 프로세스 정리
```
