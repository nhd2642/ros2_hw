# 새 세션 인수인계 (record14 -> record16 결과, 2026-11-10)

새 세션은 이 문서와 `claude/comm_relay_plan_part2.md`(§54~§58)를 먼저 읽고 시작한다. 사용자는 한국어로 대화한다.

## 1. 프로젝트 목표 (사용자 확정, 2026-11-09)
1. 각 A1이 자기 쪽을 자율 탐색하며 자체 맵 생성
2. 사람(마네킹) 발견 시 Husky에 신호 + 자체 맵에 표시. 사용자 결정: 통신 끊겨 있으면 모았다가 복구 때 보냄 + 지도 표시, 발견 시 1초 정지 후 탐색 계속 (record15 구현)
3. 자기 맵을 다 탐색하면 원위치(스폰) 복귀
4. Husky 통신 불량은 거리 기반이 아니라 사용자 조작(키보드 on/off). 현재는 `/husky/comm_cmd` 토픽만 있음 (키보드 도구 없음)
5. SLAM frontier + Nav2, Husky는 구동하지 않음
6. Husky를 맵 가운데 두어 각 A1이 자기 쪽(남 a1_1 / 북 a1_2, split_y=0)만 탐색. 구역 지정은 시작 때 Husky 노드가 `/zone_assignment`로 내림 = 남/북 경계선(y=0)이지 Husky 둘레 접근 금지 구역은 아님 (2026-11-10 사용자에게 설명함)

## 2. 현재 상태
- GitHub `nhd2642/ros2_hw` `main` = record14 (PR merge). 이전 상태 커밋 `b82f894`.
- **CHAMP 누락 확인됨 (2026-11-10)**: 수정 스크립트 `comm_relay_results\bundles\champ_include.sh`(브랜치 `fix-champ-include`, 커밋까지). 사용자: "github는 이따 할게" -> 아직 안 함.
- 작업 기준선: **record16** (`bundles\record16.zip`) + 작업 사본 `/home/claude/work/record16`에 통신 시험 명령 3번 보내기 수정(아직 번들 안 함).
- 사람 인식 모델(23 MB)은 `~/ros2_hw/person_ai_models`라 GitHub에 안 올라감 -> 팀 공유 드라이브로.

## 3. record14 결과 요약
- 통신 on/off 복귀·재개, 구역 96~100 %, 낙상 0 = 통과. 600 s에서 a1_2 서쪽 방에서 모든 계획 실패 -> 블랙리스트로 프런티어 0 -> 거짓 DONE (record15 수정).

## 4. record15 결과 (RTF 0.37~0.48)
| 실행 | 결과 폴더 | 요약 |
|---|---|---|
| mission_commoff 300 s | `20261109_234345_mission_commoff` | 64.5 m². 구역 100 %. 낙상 0. 통신 복귀·재개 정상. a1_2 완료 오판(249 s, Husky 북동 모서리 (0.64,0.47) 끼임: 출발 칸이 Husky 팽창 영역, 주변 89 m² 트임). 사람 0. |
| mission 600 s | `20261109_235342_mission` | 106.6 m², 북쪽 방 27.9 m², a1_2 y 5.45. 구역 100 %. 낙상 0. 완료 오판 없음. 사람 1/3: person_standing 1.41 m, 점수 0.45, 오차 0.34 m, Husky 지연 0.1 s, 1초 정지 동작. |

## 5. record16 내용과 결과 (2026-11-10, RTF 0.45~0.46)
내용: (1) assigner BFS 닿는 넓이 < 3 m²면 frontier_options -1, (2) `/<ns>/person_score` + 결과 폴더 `frames/`(본 장 절반 크기) + 모니터 `pscore`/`options` 기록.

| 실행 | 결과 폴더 | 요약 |
|---|---|---|
| mission_commoff 300 s | `20261110_003151_mission_commoff` | **통신 시험 무효**: Husky 스위치가 'a1_1 on'만 받고 'off'를 못 받음(`ros2 topic pub --once`가 연결 전에 보내고 끝난 것으로 추정) -> 복귀 0. 74.8 m²(북쪽 방 23.6). 구역 두 대 100 %. 낙상 0. 사람 0. |
| mission 600 s | `20261110_003934_mission` | 71.4 m². 구역 두 대 100 %. 낙상 0. 완료 오판 없음. 사람 0. a1_1: 도착 0, 실패 23, 블랙리스트 20, 경사로 남쪽 끝 (0.4~1.2, -4.0) 지형 위험 근처에서 359~584 s 멈춤 반복. a1_2: Husky 북쪽 (-0.6~0.5, 0.68)에서 295~497 s 멈춤 여러 번. |

- 주머니(-1) 수정: 두 실행 모두 시작 직후 1번(지도 작을 때) 말고 -1·0 없음 -> 그 상황이 다시 안 생겨 검증 안 됨.
- 계획 실패 진단: 600 s a1_1 256 s 로봇 칸 253·0.3 m 안 100/109 막힘(장애물 붙음), 371·381 s 로봇 칸 0·막힘 0·닿는 넓이 83.7 -> 시작은 멀쩡, 목표 쪽 문제(벽 너머 등).
- **사람 인식 핵심 발견**: frames 모든 장에서 화면 가운데~아래(약 60 %)를 같은 위치의 검은 'H' 모양 물체가 가림. 팀원 원래 이미지(지형 카메라 없음)에도 같음 -> A1 몸통 메시(앞머리) 안이나 바로 뒤에 카메라가 있어 자기 몸이 보이는 것으로 판단(카메라 camera_link = trunk 기준 (0.14, 0, 0.03), clip near 0.1, trunk 충돌 상자 0.267 길이). 사람이 가운데 막대에 잘려 MobileNet이 못 잡음. record15의 '가까우면 다리만' 추정은 정정: 가림이 주원인으로 봄. 두 실행 모두 마네킹이 카메라 시야(±45°, 4 m, 시야선)에 든 장 0이라 점수로는 확인 못 함. 모든 장 최고 점수 0.0(모델 confidence_threshold 0.25 미만은 0으로 나옴).
- 수정안(사용자 결정 필요, 공용 URDF): camera_link를 몸 밖으로(예 x 0.25~0.30 또는 위로 z +0.08~0.10) 옮기고 첫 장면으로 가림 없어졌는지 확인.

## 6. 실험 실행법
```
cp /mnt/c/Users/fllno/comm_relay_results/bundles/recordNN.zip ~/ && cd ~ && unzip -o recordNN.zip && bash ~/recordNN/install.sh
cd ~/ros2_hw && source install/setup.bash
bash src/comm_relay/tools/gazebo_test_mission.sh mission_commoff
DUR=600 bash src/comm_relay/tools/gazebo_test_mission.sh mission
```
- 결과 폴더 `C:\Users\fllno\comm_relay_results`. "끝"이면 직접 읽기 (console.txt, events/goals/timeseries/summary.csv, runs.csv, run_map.png, costmap_diag/, frames/, analysis/zone_cov.py). 구역 비율은 timeseries y. 사람 노출: 정답 위치 + map.npz 시야선 + yaw ±45°, 4 m (pscore 사건 시각과 timeseries 맞춤). 통신 시험은 mission.log.gz의 `[COMM] 사용자 명령` 확인.

## 7. 작업 규칙
- 프로젝트 CLAUDE.md 준수. 읽은 숫자만, 추측은 추측이라고.
- GitHub: 사용자가 직접 push. 세션은 저장소 접근·collaborator 안 함. 번들은 `bundles\`. 명령은 한 줄씩.

## 8. 주요 구성
- `src/comm_relay`(husky_zone_planner, husky_comm_switch, return_relay, cmd_vel_arbiter[teleop 4 > hold 3 > return 2 > nav 1], frontier_comm_filter, nav_executor, mission_logic, survivor_detector, husky_survivor_board), `src/terrain_perception`.
- 패치된 팀 파일: goal_assigner.py(assigner_setup.sh), scan_ready_relay.py, nav2 launch 2개, a1.urdf.xacro(지형 카메라), slam_params.yaml, husky model.sdf, CHAMP odom 수정.
- 마네킹 정답(월드, 평가 전용): person_standing (2.905,-4.823), person_standing_0 (-2.347,-5.514), person_standing_1 (-3.766,5.407).

## 9. 다음 할 일 (사용자 결정 대기)
1. 카메라 위치 옮기기(URDF) — 사람 인식의 주원인 후보.
2. Husky 둘레 목표 금지 반경(assigner) — a1_2가 Husky 옆에서 계속 멈춤.
3. 인식 주기 2 s -> 1 s (사용자 "이따").
4. 통신 시험 명령 3번 보내기(작업 사본에 반영됨), 경사로 남쪽 끝·북서 모서리 정체, GitHub CHAMP.
