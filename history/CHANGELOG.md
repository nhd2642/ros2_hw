# 작업 이력 (2026-11-03 ~ 11-12)

자세한 근거·숫자는 `docs/project_docs/` 의 각 문서. 여기는 시간 순 요약과 각 변경의 판정이다. 묶음 zip 은 `history/bundles/`.

## 1단계: A1 보행 (Gazebo Classic, 11-03 ~ 11-04)
문서: gait_experiment_log, gait_alternatives_research, champ_fix_log, bigexp_results, bigexp_p6_results, diag6_results, diag6_ramp_offline_results, rl_sar_integration
- CHAMP(effort + JTC PD) A1 이 후진은 정상(0.97배)인데 전진이 0.2배. update_rate 1000 Hz, p 360 채택. 파라미터 34 조합·대규모 실험(P0~P6)·다리별 계측·시작 램프 시험으로도 전진/후진 비대칭 원인 못 찾음 (com_x 를 바꾸면 전진/후진이 맞바뀜).
- rl_sar(사전학습 A1 정책) 평지 시험: 직진 0.3 m/s 0.92배, 작은 명령 무반응 구간 있음.
- 이후 사용자 PC `~/walk_parameters` 에서 보행을 다시 맞춤 (/clock 1000 Hz, JTC open_loop + d 3.6, gait com_x -0.015, soft start 등) -> CHAMP 그대로 진행.

## 2단계: 통신·복귀·Nav2·지형 통합 (Classic, 11-06 ~ 11-07)
문서: comm_relay_integration_plan (1~40절)
- 팀원 comm_relay(role2-comm) 구성만 참고해 `src/comm_relay` 다시 작성: los_checker, return_relay(지나온 길 직접 주행 복귀, 길 위 장애물 정지), cmd_vel_arbiter, frontier_comm_filter, nav_executor.
- CHAMP odom 고장 발견·수정: Gazebo 에서 foot_contacts 가 안 나와 다리 odom 이 스폰에 고정 -> 계획 접촉(stance 위상) 사용, odom_scaler 1.11. 원본 반영.
- SLAM: slam_toolbox Humble 은 회전만으로 지도를 안 늘림 (0.45 m 이동 조건) -> minimum_travel 0.2.
- Husky 몸 높이 0.25 < 라이다 평면 0.30 -> 0.39 로 (env_fix).
- 경사로 옆 가장자리 넘어짐 -> 새 패키지 terrain_perception (깊이 센서로 단차·경사 위험 칸 -> Nav2 비용지도 층). 가림 너머 낙차 규칙, 라이다 벽 억제 등 여러 번 수정 (terrain1~7).

## 3단계: Nav2 추종 문제와 기록 체계 (Classic, 11-08)
문서: comm_relay_integration_plan (29~40절)
- navfix1: DWB RotateToGoal 창(0.25)과 목표 판정(0.20) 불일치 -> xy_goal_tolerance 0.20.
- navfix2~4: 경로가 몸 뒤쪽이면 DWB 가 서 있음 -> 회전 셰임(TF 지연에 약함) -> 실행기 '먼저 돌기(TURNING)'.
- navfix5: 플래너 allow_unknown True (a1_2 계획 실패 24 -> 0).
- record1~4: 결과를 표·그림으로 남기는 체계(run_tables, run_report, runs.csv), 목표 유지, 경사 위 먼저 돌기 금지, 기울 때 바닥 맞는 라이다 빔만 지우기(scan_ready_relay 패치), 지형 층 sensor_frame 수정.

## 4단계: 구역 분담·완료·통신·사람 (Classic, 11-09 ~ 11-10)
문서: comm_relay_plan_part2 (41~58절), session_handoff_record14
- record6: 지형 위험 중 라이다가 본 벽 근처 억제. record7: goal_assigner 여유 0.25 -> 0.15 (BFS 끊김).
- record8~9: 같은 자리 계획 실패 3 번 -> '옮겨 보기', 옆걸음 없이(CHAMP 옆걸음은 거의 안 나감).
- record10: **국소 비용지도 라이다 층이 아무것도 안 찍던 원인** = max_obstacle_height 기본 0 -> 2.0. 도착 5 -> 16.
- record11~12: 사용자 설계 확인(Husky 는 일부러 막은 경계) -> goal_assigner 구역 분담, Husky 노드가 구역 지정(/zone_assignment).
- record13: 0 명령 감시(Nav2 명령 0 이 8 s -> 실패 처리). record14: 탐색 완료 -> 스폰 복귀, husky_comm_switch(사용자 on/off), 경로 구역 검사.
- record15~16: 사람 인식(survivor_detector, MobileNet-SSD) + Husky 게시판, 통신 끊기면 모았다 보냄, 1 s 정지. 카메라가 몸통 메시 안이라 'H' 가림 발견.

## 5단계: Gazebo Sim(Fortress) 이전 (11-10)
문서: gz_migration, test_plan_record17_gz, review_fix6_20261110
- record17_gz: 월드·URDF·launch·브리지 이전, `~/ros2_hw_gz`. 카메라 몸 밖으로, 전역 비용지도 목표 보정, 인식 1 s.
- fix1: ogre2 가 WSLg 에서 죽음 -> ogre1, 지형 센서 depth_camera -> gpu_lidar.
- fix2: 시각화 기본 켬 -> RTF 급락 (내 실수, fix 뒤 VIS 기본 0).
- fix3~4: GPU 라이다 거리값 전부 0.15 -> 시뮬 서버만 소프트웨어 렌더링.
- fix5: 카메라 갱신 주기 낮춤 (RTF).
- fix6: 모든 시간 한도를 시뮬 시간으로 (RTF 0.1~0.6 에서 벽시계 한도가 3~7 배 엄격), 사람 인식 조각 추론(SAHI) — 클라우드 사람 장 11 중 1 -> 5, 오탐 0/421.
- fix7: Nav2 footprint 원(0.23) -> 사각형 0.6x0.32 (Husky 옆 0.5 m 틈), 최대 속도 0.15 -> 0.2. 처음 리뷰에서 병목을 구역 경계로 잘못 짚음.
- fix8: DWB 0 명령 진단 도구(dwb_diag, DWB_DIAG=1). 동작 변화 없음.

## 6단계: 정지·갇힘 (11-10 ~ 11-11)
문서: fix7_results, fix9_fix10_results, fix11_fix12_results, fix12_900s_results, review_fix7_to_fix15
| 묶음 | 바꾼 것 | 판정 |
|---|---|---|
| fix9 | 경로 끝 0.25 m 안이면 바로 도착 처리 | 해로움 (a1_2 120 s 정지) -> fix12 에서 제거 |
| fix10 | DWB RotateToGoal 빼고 도착 방향 무관(yaw 3.14) | 채택: 도착 직후 정렬 정지 9/15 제거, 0 명령 정지 0 |
| fix11 | 기울기 가드 25° 1 s | 작동 기록 0, 넘어짐 못 막음 (사실상 안 쓰임) |
| fix12 | fix9 제거 + DWB min_speed_xy 0.03·min_speed_theta 0.05 (정지 궤적 제외) | 채택: 정지 0. 대신 좁은 곳 제자리 회전으로 갇힘이 옮겨감 |
| fix13 | 홈 계획 실패 시 지나온 길 역추종 + 목표 기준 반복 실패 차단 | 역추종은 둠, 반복 실패 차단은 목표 기준이라 거의 무효 (설계 실수) |
| fix14 | 같은 자리 추종 실패 2 번 -> 폐경로로 닫고 1 m 물러남, 닫힌 자리 지나는 목표 안 감 | 경로 출발점을 기록하는 버그 |
| fix15 | 닫는 자리를 실패한 순간 로봇 위치로 | 채택 (기준선). 폐경로 거부 과다는 남음 |

## 7단계: 실험 규칙 확정 후 (11-11 ~ 11-12)
문서: review_fix7_to_fix15, experiment_plan_20261111
- 재검토: 탐색 면적은 실행 간 ±20 m² 로 차이를 못 가림, 한 번 실행으로 결론, 한 묶음 여러 변경, 클라우드 확인 없이 전달 등 실수 정리 -> 규칙(한 변경·2 회·600 s·같은 시점·정답 면적) 확정.
- 정답 면적 지표(compare_runs.py): 남 58.7 / 북 58.8 m². 지난 실행 재계산 -> 600 s 에 대부분 90 % 근처. 남쪽은 경사로 너머를 봤나에 따라 두 갈래.
| 묶음 | 바꾼 것 | 판정 |
|---|---|---|
| fix16 | 지형 위험 칸 0.15 m 넓힘 | 기각: 경사로 입구를 좁혀 남쪽 76·88 % (기준 94·96) |
| fix17 | 이미 도착한 자리 실행 내내 제외 | 미채택: 남쪽 88·62 %, 작동은 1~3 번뿐 |
| fix18 | 전역 비용지도 팽창 0.35 -> 0.70 | 조건부 채택: 경사로 가장자리 멈춤 131·1 s -> 0·0 s, 북쪽 평균 4 포인트 낮음, 계획 실패 11·4 |
| fix19 | 옮겨 보기를 비용 낮은 곳까지(최대 0.6 m) | 판정 불가: 두 실행 모두 한 번도 작동 안 함. 코드는 둔 상태 |

## 지금 남은 문제 (HANDOFF.md 6 절)
1. 경사로 옆 가장자리 넘어짐 (지형 카메라가 앞만 봐서 옆 낭떠러지를 못 봄) 2. 폐경로 거부 과다 3. 팽창 영역 안 시작 함정 4. 기동 실패 5. 홈 복귀 거의 없음 6. 북쪽 변동 7. 사람 위치 오차 8. RTF 0.2~0.3

## 8단계: 임무 v2 — 세트 3개 + 공동 건물, Husky 자율주행 (11-12, Claude Code)
문서: docs/mission_design.md
- 월드 생성기(make_world.py) + mission.yaml, Husky A200 실제 모델(DiffDrive, 2D 라이다, IMU), 새 노드 husky_mission·husky_driver·a1_follower·clock_throttle, 공용 조향 drive_logic.
- A1 스택은 구역을 사각형으로, 홈을 세트별 탐색 시작 위치로, 탐색을 임무 단계(EXPLORE·REGROUP)에만 하도록 고침. husky_comm_switch·husky_zone_planner·husky_survivor_board·los_checker·zones 삭제.
- 시험하며 고친 것 (각각 실행 기록은 results/):
| 증상 | 원인 | 조치 |
|---|---|---|
| 추종 중 a1_2 가 11° 비스듬히 감 | 공유 좌표 정렬이 스폰 yaw 를 믿음 (기립 때 몸이 10° 돎) | 정렬 회전 0 (map 은 IMU ENU 로 이미 월드 방향) |
| 건물 안 후보가 없음 | 프런티어 상위 15 개를 도로 쪽이 차지 | frontier_detector 가 구역 안에서만 찾음 |
| a1_2 지도 번짐·위치 2.3 m 어긋남 | 움직이는 Husky·상대 A1 이 SLAM 지도에 자취로 남음 | scan_ready_relay 가 다른 로봇에 맞은 빔을 지움 |
| 난간 끝 옆에서 못 움직임 | 경사로 입구가 문·벽에 너무 가까움 | 배치 규칙(통로 1.6 m, 장애물은 벽에 붙이거나 1.2 m 비움) |
| 좁은 방에서 위치 2.3 m 튐 | slam_toolbox 거짓 루프 클로징 (탐색 창 8 m) | do_loop_closing false |
| 남은 목표 전부 '폐경로' 거부로 멈춤 | 닫힌 자리 반경 0.8 m 가 문 하나를 통째로 막음 | 반경 0.4 m + 목표 없이 90 s 면 완료 처리 |
| RTF 낮음 | 파이썬 노드마다 /clock 1 kHz 수신 | /clock_slow 50 Hz 리맵, TF 발행 200 -> 50 Hz |
| 경사로 난간 끝에 끼임, 제자리걸음 중 SLAM 2 m 흐름 | 난간 달린 폭 1.9 m 경사로 | 벽에서 벽까지 덮는 8° 둔덕 |
| 공동 건물에서 목표 0 | 문에서 4 m: 문 너머 지도가 줄무늬 | 구역 밖일 때 접근 목표 |
| 사람 발판에 걸려 복귀 실패 | 목표 칸 여유 0.15 m | 목표 칸은 장애물에서 0.4 m 이상 |
| 40 m 뒤 옆 건물 문으로 들어감 | SLAM 지도 1 m 흐름 | Husky 기준 상대 추종 + 세트마다 정렬 보정 + odom_scaler 1.125 |
| 사람 보고 14 건 / 일치 7 명 | 가장 가까운 라이다 점에 위치를 찍음 | 상자 높이로 본 거리와 맞는 점, 범위 7 m |
- 결과: 전체 임무 1 회 PASS (results/20261113_035152_set0: 1375 s, 사람 9/9, 넘어짐 0, 한도 초과 0). 자세한 표와 남은 한계는 docs/mission_design.md 5 절.
- 재현성 실행 (11-13, 통신 0/1 포함 4 회)에서 더 찾은 것: NavFn A* 의 0 점 경로(-> 다익스트라), 한 번 실패로 세트가 끝남(-> 실패 목표 60 s 뒤 재시도, 같은 곳 3 번까지),
  벽 끝을 돌아야 하는 1.6 m 입구에 갇힘(-> 2.4 m), 안 지워지는 프런티어 되풀이 방문(-> 같은 자리 3 번 제한), 세트가 바뀔 때 '목표 없음' 타이머가 안 지워짐(-> 세트마다 초기화).
  결과와 남은 한계는 docs/mission_design.md 5 절 '재현성'.
- 그 뒤 (11-14): Husky 중심 추정을 상자 맞춤으로 바꿈 (세트 도착 자세 보정이 0.47 m 틀리던 것). 지금 코드의 전체 임무 results/20261114_062151_set0 은 PASS
  (1907 s, 통신 0/1 포함, 사람 9/9·보고 9 건, 넘어짐 0, 한도 초과 0). 첫 PASS 부터 세어 전체 7 번 중 끝까지 4 번, PASS 3 번
  (2026-11-19: 홈 실패를 세는 기준으로는 PASS 2 번 — 011715 는 공동 건물 a1_1 홈 실패).
