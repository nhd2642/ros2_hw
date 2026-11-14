# 통신·복귀(comm_relay) 통합 점검과 계획 (2026-11-06)

선행 기록: 보행 3차 결과와 SLAM 병합은 사용자 PC `~/walk_parameters/RESULTS.md`, 통합 대기열은 `NEXT_INTEGRATION.md`. 이 문서는 그 다음 단계(통신·복귀 → Nav2 실제 연결)의 기준 문서다. **현재 상태는 6절.**

## 1. 진행 상태 점검 (ros2_hw.zip, walk_parameters.zip 대조)
- `apply_to_ros2_hw/src` 14개 파일이 원본 `ros2_hw/src` 와 바이트 동일. 변경 13개 파일 모두 `*.bak_walkparam` 백업 있음(백업은 walk_parameters 반영 전 원래 상태) + 새 파일 `gazebo_clock1000.yaml`.
- 반영 내용이 RESULTS.md 와 일치: /clock 1000 Hz(gait_test, disaster world.launch), JTC open_loop_control true + d 3.6, gait.yaml com_x -0.015 / com_y 0.0018, CHAMP com_y_translation(헤더 2, cpp 2), soft start(gazebo 일 때만), CHAMP 를 effort 컨트롤러 활성 뒤 시작, a1.launch.py OpaqueFunction(robot_ns 확정) + spawner namespace, state_estimation frame_prefix(launch 에서 False), URDF IMU/라이다/카메라 `__ns` remap, 경사로 z 0.1065 → 0.0671 (두 월드).
- a1_exploration 8개 스크립트 + test_follow_paths.py 존재. 스크립트는 설치되지 않고 src 경로로 실행하는 구조(CMake 가 launch/config 만 설치).
- 마지막 빌드 2026-11-06 01:36 (champ, champ_base, a1_champ, disaster_sim) 성공.
- 주의: `apply_to_ros2_hw.sh revert` 는 13개를 한꺼번에 반영 전 상태로 되돌린다 → SLAM 수정(frame_prefix, __ns)까지 되돌아가 SLAM 이 다시 깨진다.
- 주의: 로컬 src/disaster_sim/models 에 person_standing 폴더가 없음(팀 저장소에는 있음). comm_relay apply.sh 가 모델 경로를 점검하고 못 찾으면 팀 저장소 사본을 넣는다.
- 로컬 CHAMP = 049b33c(ros2_hw.repos 고정 커밋), 백업 파일은 그 커밋 원본과 동일.

## 2. 팀원 comm_relay (참고만 함)
- 위치: 팀 저장소 `github.com/nhd2642/ros2_hw` 브랜치 `role2-comm`, 커밋 c0fcb05 (junyj-Star, 2026-11-06 00:01). main 에는 없음. 나머지 패키지는 11-03 초기 버전. 팀원 가이드의 `cd src/comm_relay && git pull origin main` 은 그대로는 동작하지 않음.
- los_checker: `/a1_N/odom` 과 `/a1_N/odom/ground_truth` 를 같은 콜백으로 받아 고정 구역으로 판정(Husky 위치/벽 무시). 5 Hz, 연결일 때만 `/a1_N/comm_status`(Bool True) 발행. 구역: 외곽 |x|,|y| > 5.5, 중앙 |x|,|y| <= 1.0, y > 3.0 → 단절.
- return_relay: comm_status 를 1 s 못 받으면 단절. 연결 중 0.1 m 간격 breadcrumb, 단절 시 역순 추종(제자리 회전 ±0.3, 아니면 vx 0.2), `/a1_N/cmd_vel` 직접 발행. 경로 없으면 home (0,0). 벽시계 기준.

## 3. 빈틈 / 충돌 (중요도 순, 점검 시점)
1. [치명] 위치 프레임 혼합: EKF `/odom`(스폰 원점) 과 ground_truth(월드) 를 같은 콜백에. 팀원 환경은 EKF 가 출력을 못 내던 상태(frame_prefix 수정 전)라 문제가 안 보였을 가능성이 큼.
2. [치명] 중앙 단절 구역 = Husky 자리 = 남북 날개를 잇는 유일한 통로. 기본 home (0,0) 도 그 안.
3. [높음] CHAMP 에 cmd_vel 타임아웃 없음 → 복귀 끝/통신 복구 뒤 마지막 명령으로 계속 움직임.
4. [높음, 임무 정책] person_1 은 y>3 단절 구역 안. 외곽 ±5.5 가 남쪽 복도를 자름.
5. [높음, Nav2 단계] cmd_vel 쓰는 쪽이 둘. goal_assigner 는 통신 구역을 모름 → 경계 왕복.
6. [중간] 기동 순서(los_checker 없으면 1 s 뒤 복귀), shared_map 정렬 전 이동 위험.
7. [중간] 장애물 회피 없는 직선 되짚기, 순간이동 시험은 벽을 가로지르는 경로, 경사로 북쪽 방향 미확인.
8. [중간, 팀] CHAMP 수정 5개 파일이 팀 저장소에 없음 → 팀원 PC 에서 SLAM 프레임 문제 재발. 공용 파일 [공용] PR 필요.
9. [낮음] Nav2 = planner/controller 서버뿐(복구·재계획 없음), 목표 3 s 마다 재배정, 복귀 속도와 Nav2 상한 불일치, 저속 보행 미측정.

## 4. 계획 (요약)
A. 통신·복귀: 백업 → 패키지 반영 → 빌드 → 원본 문제 재현 → 수정 → Gazebo 시험(정상/단절-복귀-정지/los_checker 종료) → 기록.
B. Nav2 연결: 명령 중재 → 목표 실행기(유지/교체/블랙리스트/복귀) → cmd_vel_nav_test 연결 → 저속 보행 확인 → 두 로봇 탐색.

## 5. 사용자 결정 (2026-11-06 12:09)
- 통신 모델: 팀원 고정 구역 방식. 단절 시: 즉시 복귀. comm_relay: 로컬에서 수정 (팀원 코드는 구성만 참고, 내 ros2_hw 기준으로 합침). 빈틈은 전부 해결 방안대로.

## 6. 구현과 검증 (2026-11-06 12시대)
팀원 코드는 구성만 참고하고 내 ros2_hw 기준으로 `src/comm_relay` 를 다시 짬 (다른 패키지 변경 없음, a1_exploration 은 실행 시 리맵만).
- 노드: los_checker(ground truth 로 판정, 하트비트 /<ns>/comm_status + 모니터용 /<ns>/comm_link), return_relay(odom 프레임 breadcrumb, WAIT/NORMAL/RETURNING/HOLD/STOPPING, 0.15 m/s · 0.4 rad/s), cmd_vel_arbiter(teleop 3 > return 2 > nav(cmd_vel_nav_test) 1, 0.5 s 끊기면 0 명령, 20 Hz), frontier_comm_filter(단절 구역+0.3 m, 실패 목표 0.6 m 제외 → /frontier_candidates_comm), nav_executor(ComputePathToPose → FollowPath, 같은 목표 0.5 m 무시, 8 s 최소 유지, 실패/90 s 초과 → /exploration_blacklist, 목표가 구역 밖+0.2 m 면 거부, 복귀 시 지나온 길(odom 프레임)을 FollowPath, 두 번 실패하면 return_relay 가 직접 주행).
- launch: comm.launch.py(A), mission.launch.py(B: comm + filter + executor + 팀원 nav_goal_bridge/frontier_detector/goal_assigner(입력 리맵)). SLAM·Nav2 서버는 팀원 launch 로 먼저 띄움.
- 구역(config/comm_zones.yaml): 건물 외곽 x ±4.93 / y ±6.585, 북쪽 방 y > 3.0, 히스테리시스 0.15 m. 중앙 구역 삭제. 홈 = 스폰 지점(odom 원점).
- 빈틈 8번(CHAMP 팀 공유)은 패치 파일(원본 049b33c 에 적용 시 로컬 파일과 바이트 동일 확인) + setup_env.sh 적용 줄 + [공용] PR 목록까지 준비, 실제 push/PR 은 저장소 소유자 몫. 순간이동 시험은 지원 안 함(odom 이 안 움직임), person_1 방은 즉시 복귀 정책상 탐색 제외.
- 실제 월드 주의: 북쪽 방 입구는 x=0 정북이 꺾인 벽(Wall_91, y 약 3.1)으로 막혀 있어 서쪽(x < -1.05) 또는 동쪽(x > 3.0)으로 들어간다. Gazebo 시험은 (-2.0, 2.3) → (-2.0, 3.6) 경로로 조종.
- 검증(클라우드, RoboStack ROS 2 Humble, Gazebo 없음): 단위 시험 13/13, 가상 로봇(CHAMP 처럼 마지막 명령 유지)·가상 Nav2 시나리오 4/4 (startup 7/7, return 9/9, checker_loss 5/5, nav 17/17). goal_assigner 리맵(/frontier_candidates_comm 구독) 확인. 팀원 원본을 같은 가상 로봇에 붙이면 기동 3 s 동안 a1_2 에 0 아닌 명령 24건(0.44 m 이동), 북쪽 방에서 놓으면 복귀하지 못하고 y 5.16 까지 간 뒤 제자리 회전(마지막 명령 wz 0.3 유지) — 빈틈 1·3·6 재현.
- 사용자 PC 반영 묶음 comm_relay_bundle.zip: apply.sh(스냅샷 tar → 복사 → 빌드 → 단위·가상 시험(별도 ROS 도메인) → 마네킹 모델 점검 → NEXT_INTEGRATION.md 한 줄), revert.sh, Gazebo 시험 gazebo_test_comm.sh(A4), lowspeed_check.sh(B3), gazebo_test_mission.sh(B: nav 시나리오 + 전체 탐색 DUR 초). 가짜 HOME 에서 apply/revert 전 과정 확인.
- 아직 안 된 것: Gazebo 실측 전부(사용자 PC 에서 실행 필요). 특히 실제 보행으로 서쪽 입구 경유 진입·복귀, Nav2 FollowPath 로 odom 프레임 복귀 경로 추종, 0.05~0.15 m/s 저속 보행, 두 로봇 전체 탐색.

## 7. Gazebo 1차 결과와 odom 문제 (2026-11-06 13시)
- gazebo_test_comm.sh: startup 7/7, checker_loss 5/5 통과. return 실패: NORMAL -> RETURNING -> HOLD 가 1 ms 안에 (odom 상 이미 홈).
- 원인 확정: foot_contacts 발행자 0. CHAMP state_estimation 은 joint_states + foot_contacts 동기화로만 다리 위치를 갱신 ->
  Gazebo 모드에서 quadruped_controller 는 foot_contacts 를 안 내고 champ_gazebo contact_sensor 도 안 띄움 -> 다리 속도 0 ->
  EKF /odom 위치가 스폰에 고정(방향만 IMU). SLAM 은 map->odom 이 이동을 다 떠안아 멀쩡해 보였음. Nav2(로컬 비용지도·DWB 가 odom 기준)도 같은 문제.
- 사용자 결정: 1번(다리 odom 살리기). 실제 접촉 센서 대신 CHAMP 계획 접촉(stance 위상)을 씀: libchamp 다리 odom 이 딛는 다리 2개를 전제(합/2)라서.
  champ_gazebo contact_sensor 는 링크 이름만으로 구분해 두 로봇이 섞여 못 씀.
- 수정 4개: quadruped_controller.h/.cpp `foot_contacts_from_gait`(기본 false), libchamp odometry.h 회전 속도 /2(원래 2배), a1.launch.py 설정.
  state_estimation 의 vel_dt 단위 버그는 odom/raw 위치에만 영향(EKF 는 속도만 씀)이라 그대로 둠.
- 클라우드 확인: 패치 CHAMP 빌드 성공. 운동학 고리 시험 수정 전 속도 0 / 수정 후 (0.15,0)->0.155, (0,0.4)->0.411, (0.15,0.3)->(0.155,0.310).
- 사용자 PC: odom_fix_bundle.zip — verify_copy.sh(오버레이 사본에서 odom 정확도·SLAM 보정량·복귀), apply_orig.sh(*.bak_odomfix), revert_orig.sh, 팀 CHAMP 패치 갱신판.

## 8. 사본 검증 1차 결과와 전체 검토 (2026-11-06 13:30~14:00)
- 시험 A(재난 월드, a1_2 2.59 m): 실제 변위 1.98 m vs odom 1.72 m, 위치 오차 0.258 m(허용 0.21) FAIL. 오차는 거의 길이(-13%), 방향은 맞음. SLAM map->odom 보정 0.18 m(수정 전이면 이동 전부). 회전 76° (명령 90°, 0.4 rad/s 에서 0.85배 — 보행 특성).
- 시험 B(복귀): 9/9 PASS. 실제 odom 으로 지나온 길을 되짚어 9.4 s 만에 복구, 정지.
- 검토에서 찾은 것: (1) 방향 검사가 IMU 절대 yaw(EKF)라 다리 odom 검증이 아니었음 -> 다리 odom 회전 적분으로 교체. (2) foot_contacts 발행자 a1_2 0 은 daemon 탐색 타이밍 문제(odom/raw 속도가 0 아님) -> 메시지 수신으로 교체. (3) Traceback/ERROR 집계가 정리 단계 강제종료까지 셈 -> 예외 종류별 집계. (4) nav_executor 버그: 두 번째 단절 때 이전 복귀 경로를 최대 5 s 재사용 -> 수정, 동쪽 입구 두 번째 단절 시나리오 추가(이전 코드 5.50 m 떨어진 경로로 FAIL, 수정 후 0.32 m PASS, 가상 4/4). (5) odom/raw 위치 1.1e27 의 원인은 vel_dt 가 아니라 x_pos_ 미초기화(수정 전부터) -> 초기화 + vel_dt 단위 수정 추가(EKF 는 위치 안 씀, /odom 무관). (6) 부수 효과: 몸통 높이 추정 0.40 m 고정 -> 실제 다리 기준(운동학 0.20 m), RESULTS 의 라이다 높이 0.49 m 문제 원인. TF base_footprint->base_link 가 바뀌므로 SLAM/Nav2 재확인 필요.
- 패치 v2 = 6개 파일(+gait.yaml odom_scaler 선택), 팀 CHAMP 패치 7개 파일. odom_calib.py(평지 구간별 실제/다리odom 비율, 높이/기울기 추정, 제안 odom_scaler). 운동학+가상 GT 로 도구 확인(비율 0.98).
- 사용자 PC 현재 상태: 원본에 comm_relay 만 추가(다른 패키지 변경 없음). odom 수정은 사본(~/walk_parameters/odom_ws)에만. 원본에서는 odom 이 멈춰 있어 comm 복귀는 HOLD 로 빠지는 상태(기동/통신노드 종료 시험은 통과).

## 9. 1.11 재검증과 판정 기준 수정 (2026-11-06 17:20)
- ODOM_SCALER=1.11 재난 월드: EKF 위치 오차 0.033 m (비율 0.99, 방향 차 1°), EKF 방향 71.1° = 실제 71.1°, SLAM 보정 0.030 m, 복귀 9/9. FAIL 은 '다리 odom 회전 15°' 항목 하나(91.1° vs 71.1°).
- 해석: 다리 odom 회전 = 명령값(90° -> 91.1°, 평지 183° -> 184°). 실제 회전은 바닥 미끄럼에 따라 평지 1.10배, 재난 월드 0.79배 -> 다리 odom 회전은 판정에 부적합. 방향은 EKF 가 IMU 절대 yaw 로 잡음. odom_scaler 는 직선 속도에만 곱해짐(libchamp 확인).
- 사용자 결정: B(기준 고쳐 재검증 후 반영, --force 안 씀). 시험 A v3 판정: EKF 위치 <= max(0.15, 8 %), EKF 방향 <= 5°, SLAM 보정 <= max(0.3, 10 %), 서 있음, 1.5 m. 다리 odom 회전은 참고 출력.
- 새로 안 것: 재난 월드에서 회전 0.79배 -> 저속 확인을 평지(gait_test)만이 아니라 재난 월드에서도 할 것.

## 10. 사본 검증 통과와 RTF 측정 문제 (2026-11-06 17:30)
- v3 기준 사본 검증 PASS 2/0: EKF 위치 0.043 m(0.98), EKF 방향 0°, SLAM 보정 0.05 m, 복귀 9/9 (사본 3회 연속). -> VERIFIED, apply_orig.sh 가능.
- 시험 도구 문제 발견: 명령 구간과 속도 계산을 벽시계로 함. 재난 월드 RTF 약 0.7 이라 "재난 월드 회전 0.79배"(71°, 81°로 들쭉날쭉)는 미끄럼이 아니라 RTF 효과일 가능성이 큼. 위치·EKF 방향·SLAM·복귀 판정은 시간 무관 비교라 유효. odom_scaler 1.11 은 시뮬 시간 기준 EKF 비교(보정 전 0.87 x 1.11 = 0.97 vs 실측 0.98~0.99)로도 맞음.
- 수정: lowspeed_check / odom_calib / odom_accuracy 의 속도·적분을 GT/odom_raw 헤더 시뮬 시간으로, RTF 출력. 노드 5개 종료 신호 시 RCLError Traceback 정리(rclpy.ok() 일 때 나는 오류는 그대로). 가상 4/4.
- 다음 순서: apply_orig.sh -> update_comm_relay.sh -> gazebo_test_comm.sh(원본) -> lowspeed_check.sh(시뮬 시간) -> gazebo_test_mission.sh nav -> mission.

## 11. 원본 반영 완료 (2026-11-06 17:45)
- apply_orig.sh: 7개 파일 사본과 일치, odom_scaler 1.11, champ/champ_base/a1_champ 빌드 성공. update_comm_relay.sh 적용.
- 원본 gazebo_test_comm.sh: startup 7/7, return 9/9 (처음으로 원본 통과, 9.6 s 만에 복구, y 2.81), checker_loss 5/5. Traceback 0 (sim.log ERROR 11 은 정리 단계 강제종료).
- 메모: checker_loss '제자리 유지'가 명목 스폰 기준이라 기립 자리 차이(0.043 m)까지 셈 -> 죽이기 직전 위치 기준으로 고침(다음 묶음에 포함).
- 다음: lowspeed_check.sh (시뮬 시간) -> gazebo_test_mission.sh nav -> mission.

## 12. 저속·Nav2 1부 결과와 복귀 방식 결정 (2026-11-06 18:40)
- lowspeed 9/9 (시뮬 시간: 전진 0.96~1.07, 회전 1.11~1.13). nav 시나리오 21/25: 시험용 목표(1.0,2.3)/(-1.0,2.3)가 SLAM 미탐색 칸 -> planner 거부(allow_unknown false). nav_probe: 시작 지도 알려진 칸 1441(반경 약 1 m), 0.6 m 명령(실제 0.26 m, RTF) 뒤에도 그대로 -> SLAM 이 첫 스캔만 넣은 상태로 보임(minimum_travel 0.5 m / 0.5 rad). 원인 확정은 map_snapshot 그림으로.
- Nav2 복귀: 서쪽 정상, 동쪽은 뒤돌기에서 DWB "Failed to make progress" -> 43 s 뒤 직접 주행 대체.
- 사용자 결정: 복귀는 직접 주행(지나온 길 되짚기) 기본, Nav2 는 옵션(return_drive:=auto).
- 길 위 장애물 정지 설계(오검출 검토 후): 직선 띠가 아니라 앞으로 되짚을 breadcrumb 꺾은선(앞 0.5 m, 반폭 0.15 m = A1 발 끝 폭)에 라이다 점 3 개 이상이 연속 2 스캔. 근거: 그 띠는 몸이 실제로 지나간 자리라 그때 비어 있었음 -> 지금 점은 새 장애물. 옆 벽·꺾인 길 정면 벽은 띠 밖(직선 띠는 꺾이는 곳 정면 벽을 오검출, 시험으로 확인). 자기 몸 안 보임(라이다 몸통 위 0.09, 몸통 윗면 0.057, range_min 0.15), 땅은 30° 넘게 기울어야 0.5 m 안에 맞음. 제자리 회전은 안 막음. 스캔 끊기면 경고 후 검사 없이.
  - 못 보는 것: 라이다 평면(약 0.3 m)보다 낮은 잔해. 위험: odom 왕복 누적 오차가 반폭 여유를 넘으면(긴 복귀, 좁은 문) 벽 오검출 -> 그때는 멈춤(안전 쪽).
  - 10 s 넘게 막히면 nav_executor 가 살아 있을 때 Nav2 에 넘김, Nav2 도 실패하면 그 자리 대기. 기각: "통신 중 본 점은 무시" 방식 — 통신 중에 생긴 장애물을 지도처럼 기억해 무시하게 됨.
- 검증(클라우드): 단위 23/23(꺾인 길·정면 벽 오검출 없음 포함), 가상 6/6 — startup 7, return 10, checker_loss 5, obstacle 12(멈추기 전 0.18 m 걸음, 상자 중심 최소 0.75 m, 치우자 1.6 s 만에 재개, 홈 HOLD), nav(direct) 27, nav_auto 25. 음성 대조: obstacle_check:=false 면 상자 중심 0.10 m 까지 통과 -> FAIL 8/12.
- 시험 도구: nav 목표를 전역 비용지도의 연결된 알려진 빈 칸(0.6~1.6 m, 주변 0.15 m, 구역 안)에서 동·서 끝으로 고름, 부족하면 제자리 한 바퀴 뒤 다시. 모든 복귀 확인에 "장애물 오검출 없음"(/<ns>/return_events) 추가 -> Gazebo 입구 벽 옆에서 실제 오검출 확인. Gazebo obstacle 시나리오(/spawn_entity 0.3x0.3x0.8 상자). map_snapshot.py(지도+스캔 PNG, 라이다 roll/pitch).
- 묶음: return_direct.zip (install.sh: 패키지 통째 보관 -> 덮어쓰기 -> 빌드 -> 단위 시험). 다음: gazebo_test_comm.sh(4개) -> gazebo_test_mission.sh nav (지도 그림 확인) -> mission.

## 13. Gazebo 결과: 통신 4/4, nav 는 SLAM 지도 문제로 막힘 (2026-11-06 18:50)
- gazebo_test_comm.sh 4/4: startup 7/7, return 10/10(입구 옆 벽 오검출 없음), checker_loss 5/5, obstacle 12/12(상자 앞 0.17 m 걸어와 정지, 상자 중심 최소 0.75 m, 치우자 1.7 s 만에 재개, 홈 HOLD).
- 단, 상자 치운 뒤 모퉁이 (0,2.3) 근처에서 0.2 s 짜리 정지 2 번(연속 2 스캔만 잡히고 풀림) -> 원인 미확인. 사건 문장에 로봇 위치·띠 안 점 수·가장 가까운 점 거리/방위를 붙이고, "치운 뒤 추가 정지 없음" 확인 항목 추가.
- nav: 목표 후보 부족으로 FAIL. 시작 지도 알려진 칸 1422, 제자리 한 바퀴 뒤에도 1422 그대로(지도 stamp 는 60 -> 95 s 로 갱신) -> SLAM 노드는 살아 있으나 새 스캔을 지도에 안 넣음. 스캔 자체는 정상(유효 360/360, 중앙 2.25 m, 최대 7.9 m). 1 m 안 37점(최소 0.95 m) 정체 미확인(바닥 링 또는 주변 물체).
  - 의심: slam_params transform_timeout 0.0 + EKF TF 지연, odom 수정 뒤 base_footprint->base_link 변화, 첫 스캔이 기립 중(기울어진 상태)에 들어감.
  - map_snapshot 이 TF 를 못 읽음: 로봇별 TF 가 /<ns>/tf 라서 -> 리맵으로 수정.
- 진단 묶음 diag2.zip: slam_diag.sh(sim+dual_slam만, scan/scan_ready 수, TF 쌍, 스캔-TF 시각차, base_link 기울기, 360° 회전·1 m 전진 전후 지도 칸, SLAM 로그 경고 집계), nav 목표는 가장 먼 두 후보(>= 0.7 m).

## 14. SLAM 진단 결과: 고장 아님, 회전으로는 스캔을 안 넣음 (2026-11-07 21:15)
- slam_diag: 라이다 수평(pitch 0°, z 0.29 m, base z 0.196), 스캔 정상, relay 53/59 전달. 제자리 362° 회전 뒤 알려진 칸 1445 -> 1445, 1.02 m 전진 뒤 -> 4054.
- 원인: slam_toolbox(Humble) shouldProcessScan 은 직선 이동(0.8 x 0.5² -> 약 0.45 m)만 보고 minimum_travel_heading 은 안 본다 (slam_toolbox PR #709 에서 지적). 회전만으로는 지도가 안 늚. nav_probe 의 0.6 m 명령은 실제 0.26 m(RTF) 라 역시 미달.
- 시작 지도가 작은 이유: a1_2 스폰 자리가 좌우 벽 1.0 m(폭 약 2 m) 사이, 북쪽 벽 약 2 m. 스캔 1 장 + 1° 해상도라 먼 곳은 부채꼴로 듬성. "1 m 안 37점" 은 바닥이 아니라 이 좌우 벽.
- 참고: 스캔 stamp 가 최신 odom TF 보다 중앙 222 ms / 최대 322 ms 앞섬(EKF TF 지연). Nav2 transform_tolerance 0.4/0.6 이라 지금은 안 걸림, 추후 경고 나면 여기부터.
- 수정: nav 시나리오 지도 보충을 회전 -> 0.6 m 전진, slam_diag /clock QoS(best effort)·종료 Traceback 정리. 묶음 nav3.zip.

## 15. Nav2 1부 통과, 오정지 원인 = 라이다 0.15 m 점 (2026-11-07 21:36)
- gazebo_test_mission.sh nav 28/28: 알려진 빈 칸에서 목표 2개(간격 1.49 m) 계획·추종·보류 목표 도착, 직접 복귀 서쪽(12.9 s)·동쪽(13.7 s), 실행기 PAUSED, 입구 벽 오검출 없음. 끝 지도 알려진 칸 16624.
- obstacle 12/13: 상자 정지·재개·HOLD 는 통과, "치운 뒤 추가 정지 없음" FAIL. 오정지 4번 모두 가장 가까운 점 0.15 m(= 라이다 range_min), 방위 +-28~40°, 걸을 때만(모퉁이 회전 중, 상자 접근 직전 0.4 s 짜리도 같은 원인). 서 있을 때 스캔 최소 1.00 m -> 몸통·다리가 걸을 때 라이다 평면에 걸리거나 광선 시작점(0.15 m)이 몸 안에 들어가는 것으로 추정(state_estimation TF 는 pitch 0 고정이라 실제 기울기는 모름 -> 다음 시험에 GT roll/pitch 출력).
- 수정: 장애물 검사에서 range_min(+1 cm) 값과 몸 상자(앞 0.25 / 뒤 0.30 / 좌우 0.17 m) 안 점 제외. 대가: 0.25 m 안에 갑자기 나타난 것은 못 봄(0.5 m 앞에서 먼저 잡히므로 정상 접근은 영향 없음).
- 가상 로봇에 같은 점(걸을 때 40 % 스캔, +-30~40°, 0.15 m) 재현. 대조: ignore_self:=false 면 obstacle 12/13·return 9/10 FAIL(Gazebo 와 같은 증상), 수정 후 6/6. 단위 24/24. errors_summary 가 정리 단계 아닌 ERROR 문장을 보여줌. 묶음 selfhit.zip.

## 16. 중간 점검: 복잡도·실수·실환경 위험 (2026-11-07 22:10)
- 완료: comm 4종(obstacle 오정지 수정 재시험 중), odom 수정 원본 반영, 저속 9/9, Nav2 1부 28/28, SLAM 회전 특성 확인. 남음: obstacle 재시험, 두 로봇 전체 탐색(mission), 팀 공유.
- 복잡도: 실행 노드 5개 약 1000줄. 가장 복잡한 부분은 Nav2 복귀 경로(nav_executor RETURNING·return_path·재전송·막힘 넘김)인데 지금은 auto 옵션/10 s 막힘 넘김에서만 쓰임. 막힘 넘김은 Gazebo 미검증이고 같은 막힌 길을 FollowPath 하므로 이득이 작음 -> 기본 끄기 제안. 시험·진단 도구(scenario 570줄 등)는 설치 안 됨, 안정되면 정리.
- 실수 기록: vel_dt 설명 오류(실제 x_pos_ 미초기화), 판정 기준(IMU 방향·다리 yaw), 벽시계 측정("0.79배"), nav_executor 이전 복귀 경로 재사용, map_snapshot TF 토픽, SLAM 회전 가정(한 바퀴 보충 무효), 동/서 1 m 목표 조건, 장애물 검사의 라이다 0.15 m 점 미고려.
- 실환경 위험: (1) comm_relay 시간 판정이 전부 벽시계 -> mission 부하로 RTF 가 크게 떨어지면 하트비트 지연으로 가짜 복귀 가능 -> mission_monitor 에 RTF·"연결 구역인데 복귀" 검사 추가(다음 묶음). (2) SLAM 은 0.45 m 이동마다만 갱신 -> 스폰 근처 듬성한 칸으로 목표 -> 계획 실패·블랙리스트 증가 가능. (3) odom TF 가 스캔보다 0.22~0.32 s 늦음(Nav2 허용 0.4/0.6). (4) 라이다 0.3 m 아래 장애물 못 봄. (5) los_checker 는 ground truth 기반(시뮬 전용). (6) CHAMP odom 수정·comm_relay 는 로컬에만 -> 팀원 PC 에서는 복귀가 HOLD 로 빠짐. (7) return_path stamp 를 벽시계로 넣던 것 -> 0(최신 TF)으로(다음 묶음, 지금은 같은 프레임이라 영향 없음).

## 17. 장애물 오정지 수정 확인, mission 준비 (2026-11-07 22:20)
- Gazebo obstacle 13/13 (치운 뒤 추가 정지 없음, 상자 중심 최소 0.74 m, 14.7 s 정지 후 재개), return 10/10 (입구 벽 오검출 없음).
- 치운 뒤 복귀 중 몸통 최대 |roll| 3.8°, |pitch| 2.4°, z 0.213 m (GT) -> 몸통 기울기만으로는 라이다(몸통 윗면보다 3.3 cm 위) 평면에 안 닿음(0.15 m 거리에서 1 cm). 0.15 m 점의 정체는 다리(스윙) 또는 광선 시작점이 다른 충돌체 안에 있는 경우로 남음. 필터로 해결됐으므로 추적 보류.
- 사용자 결정: 막힘 시 Nav2 넘김 기본 끔(block_handover 0). 묶음 mission1.zip: 넘김 끔, return_path stamp 0, mission_monitor RTF(30 s 구간)·"연결 구역인데 복귀"·장애물 정지 집계, mission 끝에 두 로봇 지도 그림, comm 시험 grep 소음 제거. 단위 25/25, 가상 6/6.

## 18. 첫 두 로봇 탐색 결과와 원인 3가지 (2026-11-07 22:40)
- mission 300 s: RTF 0.53~0.57, 연결 구역인데 복귀 0회, 넘어짐 없음, 장애물 정지 0. a1_2 3.5 m 이동·계획 9회, a1_1 0.4 m 이동·계획 56회(FAIL). 끝 지도 a1_1 1357칸(첫 스캔 그대로), a1_2 4012칸.
- 원인 1 (a1_1 제자리): goal_assigner 는 프런티어 0.65 m 안의 '가장 가까운 안전 칸'을 목표로 줌 -> 첫 스캔 지도(반경 약 1 m)에서는 목표가 0.3~0.6 m 앞. 0.4 m 걸어 목표 0.18 m 안(xy 허용 0.20)에서 '즉시 도착'만 반복. 0.45 m 를 못 걸어 SLAM 이 지도를 안 갱신 -> 프런티어 그대로 -> 같은 목표 무한 반복.
- 원인 2 (a1_2 진행 실패): Husky 모델 몸체 높이 0.25 m < 라이다 평면 약 0.30 m, 안테나는 visual 뿐 -> SLAM·Nav2 가 Husky 를 빈 칸으로 봄. 블랙리스트 (-0.52, 0.23) 은 Husky 모서리(x +-0.495, y +-0.335), 'Failed to make progress' 7번.
- 원인 3 (같은 실패 목표 4번): frontier_comm_filter 블랙리스트 반경 0.6 m 인데 goal_assigner 목표는 프런티어에서 최대 0.65 m 떨어진 칸 -> 실패 목표를 블랙리스트해도 프런티어가 안 지워져 같은 칸이 다시 배정. (내 통합의 빈틈)
- 프로세스 종료 ERROR(팀원 스크립트 4개 exit code 1)는 종류가 RCLError/ExternalShutdownException 뿐 -> 정리 단계(이중 shutdown).
- 수정(mission2.zip): nav_executor 도착 목표 기억 -> 8 s 안 재전송 무시, 8~60 s 사이 재전송은 블랙리스트. 필터 반경 0.9 m(0.65 + 여유). env_fix.sh(백업 *.bak_envfix, revert): slam minimum_travel_distance 0.2, Husky 몸체 0.99x0.67x0.39(실제 A200 높이). 단위 25/25, 가상 6/6(nav 에 '도착 목표 재전송' 확인 추가). env_fix 는 팀 공용 파일(a1_exploration, disaster_sim) -> [공용] 후보.

## 19. mission2 결과: 지도는 커졌고, 넘어짐·Nav2 시작 실패 (2026-11-07 23:15)
- 좋아진 것: 끝 지도 a1_1 12459칸 / a1_2 12562칸(이전 1357/4012), a1_2 9.5 m, 연결 구역인데 복귀 0회(RTF 0.35 까지 떨어져도), 장애물 정지 0.
- a1_1 넘어짐: (-0.96, -2.38), 기울기 51° -> 180°(뒤집힘), 60 s 이후 이동 없음. 위치 = ramp_south_a(중심 (0, -2.49), 1.9x1.0x0.04 판, 10° 기울기) 서쪽 옆 가장자리. 서쪽(x < -1.07)은 트여 있어 로봇이 경사로 옆으로 내려가다 넘어진 것으로 봄. 경사로는 최고 약 0.19 m 로 라이다 평면(0.30 m)보다 낮아 SLAM·Nav2 에 안 보임 -> 옆 가장자리를 피할 방법이 없음.
- 넘어진 a1_1 이 계속 목표를 받아 계획 실패 블랙리스트 13건 -> 공용 프런티어 15 -> 3 개 -> a1_2 도 180 s 이후 정지. => nav_executor 에 넘어짐 처리 추가(IMU 기울기 > 45° 면 FALLEN, 목표 무시·블랙리스트 안 함, 20° 아래 3 s 면 복귀). 가상 IMU 로 확인.
- nav 시나리오 5개 FAIL: 계획은 3번 성공했는데 FOLLOWING 이 0.5 s 안에 끝나고 로봇이 안 움직임(nav_controller.log 에 ERROR 없음). Husky 가 보이게 된 뒤 처음 -> 로그 확인 필요.
- 결정 필요: 경사로 옆 가장자리 처리 (라이다에 보이는 난간 추가 / 깊이 카메라를 비용지도에 / 경사로 구역 제외).

## 20. 깊이 카메라 지형 위험 (사용자 결정 2번, 2026-11-07 23:45)
- 결정: 경사로 옆 가장자리는 월드 표시가 아니라 센서로 해결 (여러 월드에서 쓸 범용 시스템). 새 패키지 terrain_perception.
- 센서: A1 에 terrain_camera 추가 (Gazebo depth, 160x120, HFOV 87°, 10 Hz, 0.1~4 m, 몸통 앞 x 0.16 / z 0.06, 40° 아래, 광학 프레임 terrain_camera_optical). 기존 front_camera(RGB, 마네킹 인식)는 그대로. 지면 약 0.07~3 m 를 봄.
- 판정(grid.py, 한 프레임 안에서만): 5 cm 칸 높이 = 최대 z. 위험 = 칸 안 높이 폭 > max_step, 8 이웃 높이 차 > max_step, 2 칸 거리 경사 > 25°. max_step 0.04 = CHAMP swing_height. 10° 경사로 표면 통과, 옆 가장자리(0.04 m 넘는 곳)·턱·상자 앞면 위험. 한 프레임 안에서만 비교하는 이유: state_estimation TF 에 몸통 roll/pitch 가 없어(0 고정) 프레임마다 지면 전체가 몇 도씩 기울어 보임 -> 다른 시각 칸끼리 비교하면 가짜 턱. 결과는 odom 격자에 최근 관측 우선으로 30 s 기억. 몸 상자 안 점(자기 다리) 버림. TF 지연(0.2~0.3 s) 때문에 점구름을 잠깐 쌓았다 처리.
- Nav2: 국소·전역 비용지도에 terrain_layer(ObstacleLayer, 라이다 층과 분리): terrain_hazards marking, terrain_free clearing(광선 시작 = 깊이 카메라). 전역은 updateWithMax 라 SLAM 벽을 못 지움. 라이다 obstacle_min_range 0.2 (몸 걸림 0.15 m 점 제외). 팀원 nav2 launch 에는 import 한 줄짜리 블록만 넣음(terrain_setup.sh, *.bak_terrain, A1_TERRAIN=0 으로 끔).
- 확인(클라우드): 단위 7개(평지·3° 기울어진 평지 무위험, 경사로 표면 빈 칸·옆 가장자리 위험, 3 cm 턱 통과, 상자 앞면·35° 경사 위험, 격자 이동·잊기). 가짜 깊이 카메라(광학 프레임·TF 포함)로 노드 끝까지: 옆 가장자리 위험 132칸, 경사로 표면 0(높은 끝 낭떠러지만 위험). nav2 launch 패치 후 파라미터 생성 확인.
- Gazebo 확인 도구 terrain_check.sh: 점구름 Hz/프레임(앞 바닥 높이 |z| < 0.03), 경사로 옆 가장자리 위험·가운데 없음, 국소/전역 비용지도 치명 칸, RTF.
- 같이: nav_executor 넘어짐 처리, cleanup 에서 남은 Nav2/SLAM 강제 종료 + 같은 이름 노드 중복 경고(nav 시나리오 FollowPath 가 controller 로그 없이 5 ms 만에 ABORTED -> 옛 controller_server 가 남아 목표를 거절한 것으로 의심, 확인 중).
- 아직 모름: Gazebo depth 점구름의 토픽 이름(terrain_camera/points 예상)·좌표 규약(광학 예상), 카메라 2대 추가 뒤 RTF.

## 21. terrain1 미설치 상태 실행 결과 (2026-11-08 00:09)
- 0단계: 22:56 에 뜬 /a1_2 lifecycle_manager_controller 가 그대로 남아 있었음(정리 단계 SIGTERM 에 안 죽음). 같은 이름 노드 중복 -> 5 ms ABORTED 의 원인 후보(확정은 못 함, 이번 실행에서는 첫 목표 FollowPath 가 정상 시작). 새 정리 단계는 -9 로 죽이고 중복 노드를 경고.
- terrain1.zip 이 다운로드 폴더에 없어 설치 안 됨 -> 실행은 이전 코드(지형 층 없음).
- nav 27/29: 목표 2 개 중 첫 목표 'Failed to make progress'. 0.6 m 전진 뒤 북쪽을 보던 a1_2 에 남동쪽(약 150° 뒤) 목표 -> 제자리 회전 약 8 s 가 진행 검사(10 s 안 0.3 m)를 거의 다 씀. Humble SimpleProgressChecker 는 회전을 진행으로 안 침 -> movement_time_allowance 20 s 로 (terrain_setup 의 nav2 블록에서 tune_controller).
- mission PASS: a1_1 16.6 m, a1_2 12.5 m, 넘어짐 없음, 연결 구역 복귀 0, RTF 0.47~0.59, 지도 15208/16409 칸, 진행 실패 10건·블랙리스트 11건. a1_1 끝 위치 (-1.19, -2.31) 기울기 20° = 다시 경사로 서쪽 가장자리 옆 -> 지형 층 필요성 재확인.
- 묶음 terrain2.zip = terrain1 + 진행 검사 20 s (모듈 이름 nav2_terrain -> nav2_patch).

## 22. terrain2 기동 실패 원인과 수정 (2026-11-08 01:00)
- terrain_check 가 오래 걸림 -> gzserver 가 기동 직후 죽음: '[a1_1.gazebo_ros2_control]: parser error Couldn't parse parameter override rule: --param robot_description:=...' + terminate (exit -6). 로봇 0 대, Nav2 없음.
- 원인: gazebo_ros2_control 은 robot_description 을 명령행 인자로 넘기고 rcl 이 YAML 로 읽음 -> URDF 주석 속 ':' 가 깨뜨림 (gazebo_ros2_control issue #295). 내가 넣은 블록 주석 '되돌리기: revert' 의 ':'. 원본 a1.urdf.xacro 는 주석 0 개라 문제 없었음. 클라우드에서 xml 파싱만 확인하고 Gazebo 경로(명령행 인자)를 놓친 것이 내 실수.
- 수정: URDF 블록은 주석 없이 ASCII 만, apply 가 1차 블록을 지우고 다시 넣음, 주석 속 ':' 검사. 1차 블록 상태에서 업그레이드 -> 블록 1 개, 원본 대비 31 줄 추가·삭제 0, xacro 처리 OK, revert = 원본 확인.
- 시험 도구: start_sim 이 gzserver 죽음을 보면 150 s 기다리지 않고 원인 출력 후 종료. Nav2 대기 wait_nav2 (전체 90 s, 실패 시 서버별 상태·TF 로그). 이전 루프는 최악 약 21분 무출력 대기였음.

## 23. 첫 Gazebo 지형 확인 결과와 수정 (2026-11-08 01:10)
- terrain3: gzserver 정상 기동, 깊이 점구름 160x120 수신(벽시계 2.4 Hz, RTF 0.46), 프레임 맞음(앞 바닥 높이 중앙값 -0.020 m: state_estimation 몸통 높이 0.196 vs 실제 0.213 차이와 같음). 경사로 표면 위험 0. 그러나 옆 가장자리 위험 0 -> FAIL 3/6.
- 원인 1 (판정): 경사로 가운데에서 보면 옆 가장자리 바로 바깥 바닥이 가장자리에 가려 안 보이고(가장자리 0.1 m, 카메라 0.27 m -> 0.5 m 넘는 그늘) 시야 87° 밖이기도 함. 이웃 칸 비교는 양쪽이 다 보여야 해서 못 잡음. 1차 단위 시험이 모든 점을 다 넣어(가림 없음) 이걸 못 봤음 = 내 시험 설계 실수. 그리고 Gazebo 시험이 가운데 정면에서 옆 가장자리를 기대한 것도 잘못(그 방향으로 걷는 동안은 위험하지 않음).
- 원인 2 (처리량): a1_1 이 22 s 동안 7 프레임만 처리. 노드가 /ns/tf 전부(관절 TF 초당 수천 개)를 파이썬으로 받으며 밀린 것으로 봄.
- 수정: (4) 가림 너머 낙차 — 카메라에서 멀어지는 방향(60°)으로 바로 옆이 안 보이고 0.6 m 안 첫 보이는 칸이 0.04 m 넘게 낮으면 높은 칸 위험. 카메라 쪽은 안 봄(먼 오르막 성긴 표본 오검출 방지). 판정은 계속 한 프레임 안에서만(EKF 가 2D 라 경사로 위 로봇 높이를 모르고 기울기 잔차도 있어 프레임 합치면 가짜 턱), 결과만 기억. 카메라 자세 = EKF odom(x, y, yaw, 보간) + IMU roll/pitch + 고정 TF, /tf 는 몸통 높이 한 번만 읽고 구독 해제.
- 시험: 카메라에서 광선을 쏘는 높이장 레이캐스터로 장면 생성(가림 포함). 평지 무위험, 경사로 정면 표면·먼 꼭대기 무위험, 경사로 위에서 옆 가장자리 보기 위험 18칸(규칙 4 끄면 12), 바닥에서 옆면 보기 위험, 상자 위험·3 cm 턱 통과, 기억 이동·잊기·지우기. 가짜 로봇(2D odom·IMU·고정 TF·가림 점구름)으로 노드 끝까지: 가장자리 19칸, 안쪽 표면 0, 4 Hz, 9 ms.
- terrain_check 새 시점: x -0.45 로 경사로에 올라 (-0.45, -2.45) 에서 서쪽(가장자리 0.5 m)을 봄 = 넘어졌던 상황. 오르는 동안 표면 위험 0, 서쪽 가장자리 위험 3칸 이상, 비용지도 가장자리 치명. 묶음 terrain4.zip.

## 24. terrain5 Gazebo 결과와 수정 (2026-11-08 01:40)
- terrain4 의 'Nav2 0/4' 는 ros2 lifecycle get CLI Traceback 때문이었음(같은 실행의 lifecycle_manager 로그에 'Managed nodes are active' 4 줄) -> terrain5 에서 로그로 판정, 4 s 만에 4/4.
- terrain5 지형 확인 6/7: 처리량 두 로봇 비슷(20 s 에 25/24 프레임, 6~17 ms), 서쪽 옆 가장자리 위험 8칸, 국소 비용지도 치명 68/72, 전역 67/72. FAIL: 오르는 동안 경사로 표면 위험 최대 39칸.
- 원인: 경사로 위에서 몸이 10° 들리면 꼭대기 너머 내리막을 얕은 각으로 보게 되어 행 사이가 몇 칸씩 벌어지고, 그 빈칸을 (4) 가림으로 오판. 시뮬로 재현(x 0.9, pitch -10°: 표면 24칸). 거리 제한(1.2 m)으로는 비스듬히(±15°) 오를 때 남음(108 조합 중 10).
- 수정: (4) 는 카메라가 25° 넘게 내려다보는 칸에서만 (평지 약 0.55 m 안, 옆 가장자리는 몸을 돌리면 0.3~0.5 m 앞 28~40°). 검증 시뮬: 오르기 180 조합(위치 12 x IMU 오차 ±3° x 진행 방향 ±25°) 표면 오검출 0, 경사로 위 옆보기 27 조합 가장자리 최소 6칸. 단위 시험 34 (경사로 위 시험 2 개 추가).
- 1차 Gazebo 경로에서 경사로 위 제자리 회전 중 0.6 m 미끄러짐 (-0.45, -2.45) -> (-0.85, -1.96). 새 경로: 평지 (0, -1.75) 에서 돌고 경사로를 비스듬히 올라 (-0.6, -2.2) 에서 멈춤(서쪽 가장자리 0.35 m, 시야 안). 목표와 0.15 m 안인지도 확인. 묶음 terrain6.zip.

## 25. 지형 확인 통과 (2026-11-08 01:52)
- terrain6 terrain_check 7/7: 점구름 2.6 Hz(RTF 0.51)·프레임 맞음, 경사로 위 (-0.61, -2.12) 에서 멈춤(목표와 0.08 m, GT pitch -9.2°), 오르는 동안 표면 위험 최대 0칸, 서쪽 옆 가장자리 위험 22칸, 국소·전역 비용지도 가장자리 치명 72/72. 처리량 두 로봇 비슷(20 s 에 25~27 프레임, 11~14 ms).
- 메모: 끝 yaw 가 -94°(계획 -143°): 경로 추종이 목표 근처에서 방향을 남쪽으로 바꿈. 가장자리는 다가가는 동안 보고 기억(30 s)한 것. 판정 결과에는 영향 없음.
- 다음: 지형 층을 켠 채 gazebo_test_mission.sh nav mission (경사로 근처 넘어짐 재발 여부, RTF, 블랙리스트).

## 26. 지형 층 켠 탐색 결과와 A/B 준비 (2026-11-08 02:05)
- nav 28/29: 유일한 FAIL '도착 목표 재전송' 은 시험 전제 문제 — 목표2 가 0.39 m 까지 왔지만 Nav2 허용 0.2 m 를 못 채우고 진행 실패로 끝나 '도착' 기록이 없었음. -> nav_executor 사건 토픽(/ns/nav_executor/events '도착 x y'/'실패 이유')을 두고, 성공으로 끝났을 때만 검사(아니면 SKIP).
- mission PASS(넘어짐 없음, 연결 구역 복귀 0) 그러나 탐색 크게 줄어듦: a1_1 4.5 m / a1_2 5.1 m (지형 층 없던 직전 16.6 / 12.5 m), 지도 9421 / 9226 칸 (직전 15208 / 16409), 블랙리스트 15, 진행 실패 8, RTF 평균 0.44 (직전 0.52). a1_1 30~150 s 동안 1.3 m 에서 멈춤, 끝 위치 (-1.01, -1.93) = 경사로 북서 모서리.
- 해석(미확정): 지형 층이 로봇 근처를 막는지, 깊이 카메라 2대로 RTF 가 떨어진 탓인지, 단순히 300 s 한 번의 편차인지 구분 못 함. 탐색 결과는 실행마다 편차가 큼(이전 실행들 4.2~16.6 m).
- 준비: mission_noterrain(지형 층 끔, 카메라 렌더링은 그대로) vs mission 비교. A1_TERRAIN=0 은 이제 지형 층만 끄고 라이다 최소 거리·진행 검사 20 s 는 유지(공정 비교). 모니터에 목표 도착/실패 수, '멈춤'(Nav2 가 모는데 30 s 동안 0.3 m 미만) 시작 때 위치·실행기 상태·0.6 m 안 지형 위험·0.45 m 안 국소 비용지도 치명 칸. 묶음 mission3.zip.

## 27. 지형 층 A/B 결과 (2026-11-08 02:28)
| | 지형 층 끔 | 지형 층 켬 |
|---|---|---|
| 이동 a1_1 / a1_2 | 7.6 / 5.7 m | 8.2 / 2.6 m |
| 끝 지도 | 13389 / 13060 | 12945 / 8210 |
| a1_2 목표 도착 / 계획 실패 | 0 / 20 | 0 / 25 |
| 블랙리스트 | 23 | 28 |
| RTF 평균 | 0.46 | 0.41 |
| 멈춤 때 국소 비용지도 0.45 m 안 치명 | 0칸 (4번 모두) | 126~171칸 (대부분), 지형 위험 가장 가까운 0.24~0.32 m |
- 결론 1: 지형 층이 로봇 둘레를 치명 칸으로 막아 멈추게 함. 원인 (a) 몸 상자 끝(앞 0.30, 옆 0.20)에서 다리가 잡혀 0.24~0.32 m 에 위험, (b) 지형 층 clearing 이 '이번 프레임에 본 빈 칸'까지 광선뿐이라 카메라가 다시 안 보는 옆·뒤 표시가 안 지워짐(0.6 m 안 위험 0칸인데 치명 137칸인 경우).
  수정(terrain7): 로봇 중심 0.35 m 안 위험 버림, 몸 상자 앞 0.42/뒤 0.40/옆 0.25/위 0.30, terrain_free 를 로봇 둘레 3 m 원 위 180 점으로 -> 원 안 지형 층을 매번 지우고 기억(60 s)의 위험만 다시 찍음.
- 결론 2: a1_2 는 지형 층과 무관하게 두 실행 모두 도착 0, 계획 실패 20~25. 북쪽에서 남쪽 프런티어로 가려면 Husky 옆 틈을 지나야 하는데, goal_assigner 는 장애물을 0.15 m 만 부풀려 '갈 수 있다'고 보고 Nav2 는 로봇 반지름 0.23 + 팽창이라 계획 실패 -> 그 실패를 공용 블랙리스트에 올려 a1_1 이 갈 수 있는 프런티어까지 지움(필터 뒤 15 -> 0~2 개). 내 통합의 빈틈(공용 블랙리스트). 팀원 goal_assigner 를 고쳐야 하는 결정 사항.
- RTF 0.41~0.46 은 깊이 카메라 2대 포함 수준, 연결 구역인데 복귀 0.

## 28. terrain7 결과, 로봇별 블랙리스트 (2026-11-08 02:44)
- terrain_check 7/7 유지(서쪽 가장자리 위험 5칸, 국소 치명 56/72, 전역 72/72, 표면 오검출 0).
- mission (지형 층 켬): a1_1 6.4 m / a1_2 8.2 m (A/B 때 지형 켬 8.2 / 2.6), 지도 12365 / 11215, 블랙리스트 8 (28), a1_2 실패 3 (25), 멈춤 때 0.45 m 안 치명 0~66칸 (126~171), 가장 가까운 위험 0.37 m 이상 (0.24~0.32) -> 로봇을 가두던 문제 해결.
- 남은 것: a1_1 이 처음 ~130 s 동안 스폰 근처 (-0.03, -1.34) 에서 거의 못 움직임. 그때 주변 위험 0·치명 0. 목표 (-0.02, -1.57) 는 0.23 m 앞 -> 목표 근처에서 DWB 가 아주 작은 속도만 내 CHAMP 가 안 걷는 것으로 의심(미확인). 모니터가 멈춤 때 Nav2 명령(최근 10 s vx/wz 평균·최대)을 찍게 함.
- 사용자 결정: 팀원 goal_assigner 수정 허용, 둘 다 진행. assigner_setup.sh (*.bak_assigner, revert): (1) /<ns>/exploration_blacklist 로봇별 120 s·0.9 m — nav_executor 는 계획·추종 실패를 그 로봇 토픽으로만, 공용은 통신 구역 밖·도착 반복만. (2) 걸을 수 있는 칸 장애물 여유 0.15 -> 0.25 m (Nav2 robot_radius 0.23). 클라우드 기능 시험: a1_2 실패 프런티어는 a1_1 이 받음, 0.40 m 틈은 0.25 에서 배정 안 함·0.70 m 틈은 배정. 여러 번 적용해도 같음, 팀원이 파일을 바꾸면 그 파일을 새 백업으로. 묶음 mission4.zip.

## 29. mission4 결과와 중간 점검 (2026-11-08 03:03)
- 로봇별 블랙리스트 동작: a1_1 4 / a1_2 1, 공용 0 (이전 공용 8~28). a1_2 계획 실패 0, 추종 실패 2.
- 그러나 두 로봇 모두 목표 '도착' 0 (a1_1 실패 5, a1_2 2). a1_1 이 스폰 근처에서 약 120 s 멈춤: 주변 위험·치명 0 인데 Nav2 명령 vx 평균 0.000~0.013, wz 대부분 0 + 가끔 0.5. -> DWB 가 '서 있기'를 고름. 지형·블랙리스트와 무관한 Nav2 컨트롤러 문제로 봄. 의심(미확인): 목표 근처 감속·마지막 방향 맞추기(yaw 허용 0.25, RotateToGoal, stateful) 때 작은 명령을 CHAMP 가 실행 못 해 진행 검사 실패, 또는 다리 odom 속도 흔들림이 DWB 속도 창(±0.1 m/s)을 막음.
- 모니터 오류: a1_2 명령 출처 {} -> 중재기가 출처를 바뀔 때만 보내 늦게 붙은 모니터가 못 받음, a1_2 멈춤 감지도 안 됨. 출처 토픽을 transient local 로.
- 중간 점검: 회귀 시험(comm 4/4, nav 28/29, terrain 7/7)과 안전 지표(넘어짐 0, 가짜 복귀 0)는 유지. 300 s 한 번의 탐색 거리·지도 칸은 편차가 커서 판정에 쓰지 않고, 사건 단위 진단(멈춤 때 원인, 도착/실패 수)으로 판단. 지금 탐색의 병목은 Nav2 추종(도착 0).
- 다음: nav_debug.sh — 탐색 없이 a1_1 에 목표 3 개(정면, 60°, 135°), 명령 분포·정지 때 odom 속도·DWB 평가(/a1_1/evaluation: 정지를 고른 주기의 전진 후보 불법 비율, 전진이 진 비평가). 결과로 DWB 보정(예: 탐색용 yaw 허용, 최소 전진 속도)을 정함. 묶음 navdbg1.zip (comm_relay 만).

## 30. nav_debug 결과: Nav2 '도착 0' 원인 확정 — RotateToGoal 창 불일치 (2026-11-08 03:35)
- Gazebo nav_debug (a1_1, 목표 3 개): 셋 다 목표 0.23 / 0.23 / 0.24 m 앞에서 멈추고 실패 (A·B ABORTED 'Failed to make progress', C 60 s 초과).
  A: 정지를 고른 321 주기 전부 전진 후보 불법. (도구가 '충돌'로 표시했으나 잘못된 이름 — 불법은 어느 비평가든 될 수 있음. 비평가 이름 표시로 고침.)
- 원인: DWB RotateToGoal 은 FollowPath.xy_goal_tolerance (팀원 설정에 없음 -> 기본 0.25) 안에 들면 전진을 불법으로 하고 회전만 허용.
  목표 판정기는 0.20. 0.20~0.25 에서 멈추면 회전만 하다가 진행 검사 실패. Humble 소스 rotate_to_goal.cpp 확인.
  미끄러져 0.20 안으로 들어가면 성공 -> 예전 nav 시험의 가끔 실패(goal2 진행 실패), mission '도착 0' 과도 맞음.
- 클라우드 재현: 실제 nav2 controller_server(Humble 1.1.18, robostack) + 팀원 launch 가 만드는 a1_1 파라미터 그대로 + 완전 운동학 가짜 로봇.
  원래: ABORTED, 0.238~0.240 m 앞, 불법 82/82 주기 전부 RotateToGoal. 진행 검사 20 s 만 바꿔도 실패 (0.240 m) -> 창이 원인.
  보정(FollowPath.xy_goal_tolerance = 0.20): 목표 방향 0°, 180°, -90° 셋 다 성공 (0.187~0.196 m, 4~7 s).
- 보정 위치: terrain_perception/nav2_patch.tune_controller (팀원 launch 에 이미 들어간 호출 블록이 부름, launch 내용 변경 없음).
  RotateToGoal 쓰는 컨트롤러의 xy_goal_tolerance = min(기존 또는 0.25, 목표 판정기 xy). 단위 시험 2 개 추가 (terrain 11 passed).
- B 의 정지 주기는 전진 불법 0, 이유가 BaseObstacle(장애물 비용) — SLAM 자세 오차로 DWB 는 창 밖이라 여겼을 수 있음. 보정 후 다시 봄.
- 묶음 navfix1.zip (두 패키지). 다음: nav_debug 재실행(3/3 성공 기대) -> mission 재실행(도착 수). 이후: 탐색 목표의 마지막 방향 맞추기
  (CHAMP 회전 약 0.4 rad/s 라 시간 소모) 를 없앨지 결정, a1_2, 팀 공유, 마네킹.

## 31. navfix1 결과 (2026-11-08 17:51)
- nav_debug: B·C 성공 (전엔 0/3). A 실패 — 이번엔 0.15 m (판정 0.20 안), DWB 는 RotateToGoal 창 안이라 전진 불법 375/375, 남은 일은 제자리 90° 회전인데
  정지 71% · 회전 8% 를 고름 -> 진행 검사 실패. 클라우드(완전 운동학, 회전 데드밴드 0.15 흉내까지)에선 같은 90° 목표가 성공 -> Gazebo 만의 이유. 미확인.
  nav_debug 에 '그냥 선 주기에서 회전 후보가 왜 졌나(불법 비평가 / 가중 점수 차)' 와 끝 방향 출력 추가.
- 내 설명 오류 정정: DWB 속도 창은 acc_lim x sim_time(1.0 s) = ±1.0 (Humble standard_traj_generator: startNewIteration(vel, sim_time_)), ±0.1 이 아님.
  'odom 속도 흔들림이 창을 막는다' 가설은 틀림. 도구 문구 삭제.
- mission (300 s): a1_2 도착 2 (처음으로 0 아님), a1_1 도착 0 + 넘어짐: (-0.97, -3.15) 에서 뒤집힘(기울기 180°).
  월드: ramp_south_b (내리막판, 1.9 x 1.0, 10°, x ±0.95) 서쪽 가장자리, 꼭대기 근처 — 판 높이 약 0.14 m. terrain_check 는 오르막판(a) 서쪽 가장자리를
  바닥에서 비스듬히 다가가는 경우만 시험했음 (판 위에서 가장자리 쪽으로 가는 경우·내리막판은 시험 안 함 = 빈틈).
  가능한 원인(미확인): 위험을 못 봄 / 봤는데 계획·DWB 가 지나감 / 판 위에서 돌다 미끄러짐 (125 s 에 a1_1 이 (0.53, -2.12) 판 위에서 vx 0.117 명령에도 30 s 동안 0.3 m 미만 = 판 위 헛걸음).
- a1_2 는 133 s 도착 뒤 끝까지 새 계획 없음 (idle 54%). goal_assigner 가 안 줬는지(짝 배정 실패 시 한 대만 배정 — 넘어진 a1_1 이 뽑혔을 수 있음), 실행기가 무시했는지 모름.
- frontier_detector·nav_goal_bridge 'process has died' (exit 1) + RCLError Traceback 2: 종료 시 팀원 스크립트 오류로 보임 (확인은 목표 수신 시각으로).
- navfix2.zip (comm_relay 만): mission_monitor 가 넘어지면 직전 12 s (1 s 간격 월드 위치·방향·roll/pitch·Nav2 명령·그때 지형 위험 앞 ±60° 0.8 m 개수와 가장 가까운 칸) +
  마지막 계획 경로가 넘어진 곳을 지났는지, 로봇별 goal_assigner 목표 수·마지막 시각. 클라우드: 합성 데이터로 보고 형식·odom->월드 변환 확인, 단위 25, 가상 6/6.
- 다음: nav_debug + mission 다시 -> 넘어짐 원인 갈래에 따라 (못 봄: 판 위 가장자리 검출 시험 추가 / 지나감: 비용·계획 / 미끄러짐: 경사 위 회전 제한).

## 32. navfix2 결과 -> 경로가 뒤쪽이면 DWB 가 못 돎 (2026-11-08 18:44)
- nav_debug: A 성공(18 s). B·C 는 0.00 m 이동, 정지 100% 로 실패. A 가 목표 방향 0°(동쪽)로 끝나 B·C 경로(서쪽)가 몸 뒤쪽이 됨.
  navfix1 때 B·C 가 성공한 건 A 가 실패해 남쪽을 본 채 시작했기 때문 -> 시작 방향에 따라 갈림.
  DWB 평가: 전진은 PathAlign·PathDist·GoalAlign 때문에 정지보다 나쁨.
- 클라우드 재현 (실제 controller_server + DWB, 팀원 설정): 경로가 정확히 뒤(180°)면 정지 100%, 이동 0, 'Failed to make progress'.
  회전이 정지에 진 이유 PathAlign 158 / GoalAlign 119 (가중 점수 차 합). 가짜 로봇이라 Gazebo 와 무관한 DWB 자체 성질.
  참고: DWB 는 짧게 끊어 점수를 매김(short_circuit) -> 진 후보 점수는 일부 비평가만 들어 있어, 도구의 '이유' 는 대략적인 값.
- 보정: Nav2 표준 nav2_rotation_shim_controller 로 DWB 를 감쌈 (nav2_patch._add_rotation_shim, 패키지 없으면 그대로 두고 알림).
  경로 앞 0.5 m 점과 몸 방향 차 > 45° 면 0.5 rad/s 로 제자리 회전 후 DWB. 파라미터는 Humble 공통 5 개만 (primary_controller,
  angular_dist_threshold, forward_sampling_distance, rotate_to_heading_angular_vel, max_angular_accel, simulate_ahead_time).
  클라우드: 뒤 180° 7/8 성공 (첫 1 회 실패: 61° 까지만 돌고 진행 검사 실패, 시작 때 TF extrapolation 오류 1 줄, 원인 미확인), 옆 -90°,
  앞+목표방향 90° 차, 뒤 135° + 회전 데드밴드 0.15 모두 성공. 단위 시험 13.
- mission: 넘어짐 없음 (넘어짐 기록 도구는 아직 못 써 봄). a1_1 도착 4 · 실패 3, 멈춤 206 s (스폰·Husky 남쪽 (-0.24, -0.83) 에서 113~272 s,
  Nav2 명령 vx 0 — 경로가 뒤쪽인 경우였을 가능성, 미확인). a1_2 도착 1 · 계획 실패 7 (0점): goal_assigner 는 공유 지도로 목표를 고르는데
  플래너는 각 로봇 SLAM 지도(allow_unknown false)라 a1_1 쪽 구역 목표는 a1_2 가 계획 못 함 — 설계 차이, 로봇별 블랙리스트가 흡수 (6). 나중 과제.
- 팀원 스크립트 'process has died': goal_assigner 목표가 299 s 까지 옴 -> 종료 때 오류 확정 (탐색 중 죽은 것 아님).
- 묶음 navfix3.zip (두 패키지). 다음: nav_debug(3/3 기대) + mission.

## 33. navfix3 결과 -> 셰임은 TF 지연에 약함, 실행기가 먼저 돌기 (2026-11-08 19:20)
- nav_debug: A 성공 18 s, B 성공 28 s (navfix2 에서 실패하던 같은 시작 방향, 제자리 회전 48%), C 는 계획 실패
  (NavFn 'Failed to create a plan from potential when a legal potential was found' — 컨트롤러와 무관, 목표가 Husky 근처).
- mission: 넘어짐 0, a1_1 도착 2 · 실패 4, a1_2 도착 0 · 계획 실패 5(공유 지도/로봇 지도 차이) · 추종 실패 2.
  a1_1 이 처음 90 s 스폰에서 0 m (FOLLOWING, Nav2 명령 10 s 에 4 개 — 컨트롤러가 무엇을 했는지 모름), a1_2 (0.27, 2.05) 에서 명령 0 으로 멈춤.
  컨트롤러 로그 ERROR 27 중 대부분이 transformPoseInTargetFrame Extrapolation (요약이 시각 숫자 때문에 안 묶여 3 줄만 보였음 -> 고침).
- 원인(소스 확인 + 클라우드 재현): Humble RotationShimController 는 경로 점을 '현재 시각'으로 찍어 base 로 변환(0.1 s 대기),
  실패하면 그 경로에서는 회전을 포기(path_updated_=false)하고 DWB 에 넘김 -> 경로가 뒤면 DWB 는 서 있음.
  가짜 로봇 TF 0.05 s 지연: 60° 돌다 멈춰 실패, 0.25 s: 안 돌고 실패. (navfix3 클라우드 시험 8 회 중 1 실패도 이것.) A1 TF 는 0.2~0.3 s 늦음.
- 보정: nav_executor 가 계획 직후 경로 앞 0.5 m 방향과 /<ns>/odom 방향을 비교, 45° 넘으면 TURNING: 0.5 rad/s 로
  /<ns>/cmd_vel_nav_test 에 회전 명령 (20° 안 또는 30 s) -> FollowPath. TF 안 씀. map 과 odom 방향이 같다고 가정
  (SLAM map 은 odom 과 같은 방향에서 시작, 보정 회전 몇 도). 취소·교체·넘어짐·복귀 때 회전 멈춤(gen).
  클라우드: 실제 controller_server(셰임+DWB) + 실제 nav_executor + 가짜 플래너 + TF 0.25 s 지연 로봇, 경로 뒤 180°:
  끔 -> 추종 실패 20 s / 켬 -> IDLE->TURNING->FOLLOWING->도착 10 s. 단위 41, 가상 6/6 (nav 시나리오에서 TURNING 거침).
  셰임은 그대로 둠 (TF 맞을 땐 동작, nav_debug 에 필요). 먼저 돌기가 45° 안으로 맞추면 셰임은 끼어들지 않음.
- 모니터: /rosout 에서 로봇별 controller/planner 경고·오류 (숫자 묶음) — 멈춤 때 최근 15 s, 끝에 전체. TF 오류는 로봇 미상 '?'.
- 묶음 navfix4.zip (comm_relay 만). 다음: mission (+ nav_debug 는 선택).

## 34. navfix4 결과 (2026-11-08 20:49)
- a1_1: 이동 15.7 m (지금까지 최고, 이전 3.1~11.3), 도착 2 · 실패 1, 멈춤 52 s (이전 140~206 s), nav 96%. 먼저 돌기 동작 확인
  (+165°, -103° 등). 넘어짐 0.
- '멈춤' 중 일부는 먼저 돌기(제자리)였음 — 모니터가 위치만 봐서 셈. 명령 7~8 개 창은 IDLE 뒤 막 TURNING 시작한 경우로 봄
  (클라우드에서 TURNING 명령 약 9.5 Hz 확인). 새 목표가 와서 돌기 중간에 끊기는 경우가 보임 (433.98 +165° 시작 -> 9 s 뒤 새 목표).
- a1_2: 도착 1, 계획 실패 24 / 목표 85, 로봇별 블랙리스트 21, 놀기 40% — 공유 지도 목표를 자기 지도(allow_unknown False)로 못 계획.
  이제 a1_2 의 주 병목.
- Nav2 경고: Control loop missed rate (a1_1 18, a1_2 10), TF Extrapolation 26 (셰임 쪽, 먼저 돌기 뒤라 영향 작음).
- 보정(navfix5.zip, 두 패키지):
  · nav2_patch.tune_planner: NavFn allow_unknown True (A1_PLAN_UNKNOWN=0 이면 그대로). terrain_setup.sh 가 팀원
    nav2_planners.launch.py 호출 블록에 한 줄 추가 (이전 블록 교체 방식 그대로). 클라우드: navfix3 블록 -> 새 블록 교체 시험,
    launch 가 만든 파라미터 allow_unknown True / 끄면 False, 전역 층 static·terrain·inflation, 컨트롤러 셰임 유지.
    대가: 모름 속 벽을 지나는 경로 가능 — 가 보며 국소 비용지도가 막고, 못 가면 진행 실패 -> 그 로봇 블랙리스트.
  · 모니터: 멈춤은 실행기 FOLLOWING 일 때만, 먼저 돌기 횟수·완료/새 목표로 끊김·시간 따로.
- 다음: mission -> a1_2 계획 실패·도착·놀기 비율, 먼저 돌기 끊김 비율. 끊김이 많으면 돌기 중 목표 교체 막기 검토.

## 35. navfix5 결과 (2026-11-08 21:19)
- 플래너 모름 허용 효과: a1_2 계획 실패 24 -> 0, 로봇별 블랙리스트 21 -> 1, 도착 1 -> 2. a1_1 도착 2 · 실패 5 (추종), 이동 12.9 m.
- 먼저 돌기: a1_1 5 번, a1_2 5 번 모두 완료 (새 목표로 끊김 0), 평균 5.8 / 7.4 s, 최장 10.5 s. 돌기 중 목표 교체 막기는 필요 없음.
- 새 문제: goal_assigner 목표가 a1_1 212 s, a1_2 197 s 에서 끊김 -> 둘 다 끝(301 s)까지 놀기 (idle 32~38%, 이동 정체).
  frontier_comm_filter 는 그 뒤에도 후보 11~13 개를 보냄. 팀원 goal_assigner 는 후보가 있어도 '공유 지도의 아는 빈칸(0~20)
  + 장애물 여유(내 보정 0.25 m)' 로 BFS 해서 닿는 목표가 없으면 아무것도 안 보냄 ('No safe reachable goal' 로그, 같은 문장 한 번만).
  가설(미확인): 탐색이 진행돼 남은 프런티어가 좁은 틈·벽 너머라 여유 0.25 m BFS 로 못 닿음, 또는 로봇이 벽에 붙어 시작 칸이 막힘.
- a1_1 최대 기울기 33° — (1.14, -4.76) 벽(Wall_80 근처) 옆에서 'No valid trajectories' x4 와 함께 멈춤(국소 비용지도상 벽 안쪽).
  A1 다리 포함 크기(약 0.5 x 0.3 m)에 비해 robot_radius 0.23 이 작아 벽에 몸이 닿았을 가능성 (미확인).
- '/a1_1/gazebo_ros2_control 노드 중복' 경고 1 회 (이전엔 없음, RTF 정상) — 옛 gzserver 가 남았을 수 있음.
- navfix6.zip (comm_relay 만, 모니터): 기울기 25° 넘으면 직전 12 s 기록, 목표 30 s 끊기면 goal_assigner 최근 기록, 끝에 마지막 기록 3 개.
- 다음: mission -> assigner 가 왜 멈추는지 (문장) 확인 후 보정 (여유·시작 칸 처리 등), 벽 접촉 여부.

## 36. 진행 방식 바꿈: 표·그림 기록 (사용자 결정, 2026-11-08 21:43)
- 회고: 3일간 묶음 약 22개, 그중 진단 도구만 바꾼 묶음 다수. 해석 실수 대부분이 그때그때 덧붙인 콘솔 출력에서 나옴
  (DWB 창 ±0.1 오판, '충돌' 이름, 모니터 출처 누락, 오류 요약이 숫자 때문에 안 묶여 TF 오류 26개 숨김, 먼저 돌기를 멈춤으로 셈).
  설정 변경 효과를 300 s 한 번으로 판단한 곳도 있음. 마네킹 인식은 아직 시작 안 함.
- 결정: 측정할 것을 전부 표로 남기고 계산·그림으로 판단. 결과는 Windows 폴더 C:\Users\fllno\comm_relay_results 에 저장해
  내가 PC 연결로 직접 읽음 (콘솔 붙여넣기 불필요). 변경마다 같은 설정 2 회 실행.
- record1.zip (comm_relay 만): mission_monitor -> run_tables.py 로 events / goals(실행기 목표 시도 1 줄: 계획·먼저 돌기 각도·시간·결과·
  이동·남은 거리) / timeseries(0.5 s: GT 자세, 명령, 출처, 상태, 근처 지형 위험·치명) / summary CSV + map.npz(공유 지도, 지형 위험 누적,
  끝 프런티어), runs.csv 누적. run_report.py: run_map.png(지도 위 상태별 궤적·목표 결과·멈춤·기울기·넘어짐·위험·프런티어),
  timeline.png(상태 띠·실제/명령 속도·wz·치명 칸·roll/pitch·사건 세로선). gazebo_test_mission.sh mission 이 폴더 만들고 로그 gz·콘솔·오류 요약 복사.
  'gazebo_test_mission.sh mission mission' = 2 회 반복.
- 기록하면서 찾은 것: (1) 실행기 state 토픽이 0.5 s 하트비트뿐이라 짧은 PLANNING 을 놓침 -> 바뀔 때 바로도 발행.
  (2) 모니터 '멈춤' 정의가 30 s 처음-끝 거리라 갔다 돌아오는 경로도 멈춤으로 셈 (가상 시험 그림에서 0.15 m/s 로 걷는 중 멈춤 표시) ->
  '30 s 동안 0.3 m 원 밖으로 안 나감' 으로. 이전 실행들의 멈춤 수에 이런 가짜가 섞였을 수 있음.
- 확인(클라우드): 가상 탐색(mock_mission.sh: 가상 로봇·Nav2 + fake_explore) 60 s -> goals 6 (도착 6, 먼저 돌기 각도 -72~-147°), 그림 확인.
  단위 45 (기록 표 3 개 추가: 사건·상태 순서 바뀜, 교체, 누적 표), 가상 6/6.
- 다음: record1 설치 -> 'mission mission' (기준선 2 회) -> 폴더 접근 권한 -> 표·그림으로 목표 끊김·벽 접촉 판정.

## 37. 기준선 2 회 (record1, 2026-11-08 22:20) — 표·그림으로 판정
- 결과 폴더 접근 허용 (C:\Users\fllno\comm_relay_results), 표·그림 직접 읽음.
- 기준선: a1_1 이동 13.1 / 9.7 m, 도착 1 / 2, 넘어짐 2/2 회. a1_2 10.3 / 12.9 m, 도착 0 / 2, 넘어짐 0.
- (1) 목표 교체가 대부분: 시도 중 '교체' a1_1 15/17, 11/14, a1_2 8/9, 13/16 (평균 12~15 s, 0.5~0.8 m 이동 후).
  goal_assigner 가 3 s 마다 다시 고르고 실행기는 8 s 뒤 바꿈 -> 도착 전에 계속 바뀜. 이것이 탐색 효율의 가장 큰 병목.
- (2) a1_1 이 경사로 위에 오래 있음: 기울기 > 5° 시간 227 / 118 s (300 s 중). 스폰 바로 앞이 경사로(ramp_south_a), 남쪽 목표 경로가 경사로를 넘음.
- (3) 넘어짐 1 (run1, (-1.0, -2.84) 경사로 서쪽 가장자리): FOLLOWING 중 vx 0.13~0.15 로 남쪽으로 걸으며 x -0.63 -> -0.79 로 서쪽으로 흐름.
  지형 노드 위험 칸은 0.34 -> 0.03 m 까지 다가옴(앞 ±60° 15~31 칸)인데 국소 비용지도 0.45 m 안 치명 0.
  run1 a1_1 은 120 s 뒤 끝까지 '위험 있음·치명 0' (그 전·run2 는 치명 있음) -> 비용지도가 지형 표시를 못 받았거나 모니터가 받은
  비용지도가 멈춤 (구분 못 함 -> 기록에 비용지도·지형 메시지 나이 추가).
  별도로 내 설정 실수: 지형 층 footprint_clearing_enabled True — 로봇이 가장자리 0.23 m 안으로 미끄러지는 순간 그 칸을 지움 -> False.
- (4) 넘어짐 2 (run2, (1.0, -2.08) 경사로 동쪽 가장자리): 먼저 돌기(TURNING, vx 0, wz 0.5) 6 s 동안 경사로 위에서 0.5 m 옆으로 미끄러짐
  (roll -2 -> -38°). 치명 120~139 칸 있었음 (먼저 돌기는 비용지도를 안 봄, 미끄러짐은 물리). 내가 넣은 먼저 돌기가 원인.
- (5) run1 공유 지도 남쪽 절반에 호(弧) 모양 가짜 벽·방사 줄무늬: 경사로에서 몸이 10° 기울면 2D 라이다(높이 약 0.3 m)가
  약 1.7 m 앞 바닥을 맞힘 -> SLAM 지도 오염. run1 (경사 227 s) 이 run2 (118 s) 보다 심함. 그 지도로 목표·경로가 만들어짐.
- record2.zip (두 패키지):
  · 지형 층 footprint_clearing_enabled False.
  · 실행기: 기울기 > 6° 면 먼저 돌기 안 함, 도는 중 > 8° 면 멈추고 추종.
  · 실행기: 목표 유지 — 지금 목표 1.0 m 안 프런티어가 남아 있으면 새 목표 보류 (도착·실패·90 s 뒤 시작). 가상 시험(min_dwell 2 s,
    목표 4 s 마다 바뀜, 프런티어 유지): 끔 교체 4 / 도착 2 -> 켬 교체 1 / 도착 4.
  · 회전 셰임 뺌 (먼저 돌기로 대체, TF 시각 오류 25 회/실행 원인 제거). 컨트롤러 = DWB.
  · 기록: timeseries 에 cm_age(국소 비용지도 메시지 나이), hz_age(지형 위험 메시지 나이).
  · 단위 44, 가상 6/6. 묶음은 결과 폴더 bundles\ 에도 직접 넣음.
- 남은 결정: (5) 라이다 기울 때 스캔 버리기 (팀원 scan_ready_relay.py 또는 새 노드 + 국소 비용지도 스캔 토픽), 경사면을 경로 비용으로(피해 가기).

## 38. record2 결과 2 회 (2026-11-08 23:01) -> record3
| | record1 (2 회) | record2 (2 회) |
|---|---|---|
| 넘어짐 a1_1 | 2/2 | 0/2 (최대 기울기 21°, 18°) |
| a1_1 경사로 위 시간 | 227 / 118 s | 40 / 70 s |
| 시도 중 '교체' a1_1 / a1_2 | 15/17, 11/14 / 8/9, 13/16 | 7/12, 6/9 / 3/7, 4/8 |
| 도착 a1_1 / a1_2 | 1, 2 / 0, 2 | 0, 2 / 4, 2 |
| 이동 a1_1 / a1_2 | 13.1, 9.7 / 10.3, 12.9 m | 12.6, 16.9 / 15.1, 13.2 m |
- 경사 위 먼저 돌기 생략 2 회, 도는 중 기울어 멈춤 5 회 기록됨 (실행기 로그). 남은 '교체' 는 다가가는 동안 그 목표 근처 프런티어가 지워진 경우
  (설계대로: 목표 1 m 안 프런티어가 없어지면 바꿈). 0.3~3.4 m 남기고 교체.
- 새로 찾은 설정 오류 (내 실수): 지형 층 marking 소스에 sensor_frame 이 없어 obstacle_max_range 3.5 m 를 점구름 프레임(odom/map) 원점 = 스폰
  에서 잼 -> 스폰에서 3.5 m 넘는 곳 위험은 국소·전역 비용지도 둘 다 안 찍힘. 표: 스폰 3.8 m 넘는 곳에서 위험 0.4 m 안 110 표본 중 치명 0 %,
  가까울 때 91~100 %. record1 run1 a1_1 의 '120 s 뒤 치명 0' 도 경사로에서 odom 이 흘러 odom 상 3.5 m 를 넘은 것으로 봄(그땐 odom 기록 없음).
  클라우드 실제 nav2 costmap(Humble) 재현: 로봇 x 5 m, 위험 13 점 -> 옛 설정 치명 0 칸 / 새 설정 12 칸 (x 1 m 에선 둘 다 12).
- record2 run3 지도: 남쪽 절반 벽이 몇 도 기울고 겹침(SLAM 방향 흐름) + 호 모양 벽 일부 -> 사용자 결정: 팀원 scan_ready_relay.py 에 기울기 검사.
- record3.zip: terrain_layer marking sensor_frame=terrain_camera_link, obstacle_max_range 4.0 / scan_tilt_setup.sh (*.bak_scantilt, revert):
  스캔 시각 앞뒤 0.1 s IMU 기울기 최대 > 5° 면 SLAM 에 안 넘김, 로봇 이름은 리맵된 스캔 토픽에서 (a1_2 relay 가 a1_1 IMU 를 안 봄).
  클라우드: 패치한 relay 를 a1_2 리맵으로 띄워 기울기 2° -> 30/30 넘김, 10° -> 0/30, 다시 2° -> 30/30 (a1_1 IMU 는 12° 로 계속 보냄),
  두 번 적용해도 같음, revert = 원본. 평지 걸음 기울기 99 % < 5° (a1_2 표). 기록에 odom_x/odom_y. 단위 45, 가상 6/6.

## 39. record3 결과: 스캔 버리기가 Nav2 를 망가뜨림 -> record4 (2026-11-08 23:32)
- record3 2 회: a1_1 이동 5.2 / 0.7 m, 도착 1 / 0, 실패 13 / 29 (계획 실패 21). a1_2 10.0 / 7.0 m, 도착 1 / 0. 넘어짐 0. 지금까지 가장 나쁨.
- 원인 (내 설계 실수): 기운 스캔을 통째로 버렸음. slam_toolbox(Humble) publishTransformLoop 는 map->odom 을 '마지막 스캔 시각 +
  transform_timeout' 으로 찍음 (소스 확인) -> 스캔이 끊기면 TF 가 멈춤 -> Nav2 'Unable to transform robot pose into global plan's frame'
  (a1_1 11 / 8 번, a1_2 4 번 이상) -> 추종 실패. run6 a1_1 은 경사로 아래끝 (-0.16, -2.03) 에서 pitch -6.7° 로 280 s 서 있었고
  스캔 1401 개 버려짐 -> 지도 안 늘어 계획 실패 21 -> 움직이지 않음 (교착).
- 좋아진 것: run5 공유 지도는 호 모양 가짜 벽 없이 직선 벽 (기울기 처리 효과). 지형 표시: 스폰에서 먼 곳 표본은 없었음(멀리 못 감) — 다음 실행에서 확인.
  odom 흐름 최대 0.09~0.21 m (경사로에 오래 안 있었음).
- record4.zip: scan_tilt_setup.sh 를 '바닥 맞는 빔만 NaN' 으로 — 빔마다 아래각 z = -sin(pitch)cos(a) + sin(roll)cos(pitch)sin(a) - sin(0.5°),
  z<0 이고 거리 > 0.9 x 0.28 / -z 면 지움. 스캔 흐름은 그대로. 1 차 적용 상태에서 업그레이드(백업 원본으로 되돌린 뒤 다시 패치).
  클라우드: 평지 2° -> 지운 빔 0, 10° 앞숙임·전부 1.5 m -> 앞쪽 47/360 만 NaN, 1.2 m(바닥보다 가까운 벽) -> 0, roll -10°(왼쪽 아래) -> 왼쪽만,
  넘긴 스캔 20/20 (끊김 없음), revert = 원본.

## 40. record4 결과 (2026-11-09 00:12) -> record5
- 1 회차(유효): 'Unable to transform' 0 (record3 4~11) -> 빔 지우기로 TF 문제 해결 확인. 넘어짐 0. a1_1 9.4 m 도착 1 실패 3,
  멈춤 8 번 114 s (경사로 북서 모서리 (-0.9, -1.9) 근처, 치명 79~122). a1_2 6.5 m 도착 2 실패 7 — 206~224 s 에 벽(y 1.6) 0.20~0.23 m
  옆 (1.33, 1.8) 에서 'No valid trajectories' 5 번, 목표 6 개 연속 0~5 s 만에 실패. 로봇 중심이 장애물 테두리(내접 0.23 m) 안 ->
  DWB 후보 전부 불법 -> 못 빠져나옴. 지형 층 몸 둘레 지우기를 끈 뒤(record2) 생긴 부작용 (예전엔 몸 밑 지형 위험이 지워졌음).
- 2 회차(무효): 탐색 시작 안 함 — Fast DDS 'RTPS_TRANSPORT_SHM Error open_and_lock_file failed', a1_2 relay 'Dropped scan: TF unavailable'
  1711 번, comm_mode a1_2 WAIT, 목표 0. 앞 실행이 강제 종료되며 남긴 공유 메모리 잠금 파일로 봄 (확정 아님, sim.log 는 그땐 안 남김).
- record5.zip (comm_relay 만 바뀜):
  · 실행기 ESCAPING: 추종 실패 뒤 중심 0.1 m 안에 비용지도 >=99 칸이 있으면, 0.5 m 안 치명(100) 칸들에서 멀어지는 쪽으로
    0.08 m/s 평행 이동(vx, vy, 회전 없음), 최대 4 s, 중심이 테두리 밖이면 끝 -> 보류 목표 시작. 새 목표는 끝날 때까지 보류, 복귀·넘어짐이면 취소.
    클라우드: 실제 controller_server(지형 층 포함 국소 비용지도) + 실행기 + 가짜 로봇, 지형 위험 줄 0.18 m 옆: 'No valid trajectories' ->
    ESCAPING -> 1.7 s 에 0.13 m 벗어남 -> IDLE -> 다시 계획·추종.
  · 정리 단계: 남은 프로세스 없으면 /dev/shm/fastrtps_* 지우고 ros2 daemon stop. 결과 폴더에 sim.log.gz.
  · 모니터: 목표 0 개이거나 comm_mode WAIT 그대로면 '무효 실행' 표시, summary·runs.csv 에 valid 열.
  · 단위 45, 가상 6/6, 가상 탐색 PASS.

## 41. 목표 동작 영상 + 목표값 제안 (2026-11-09)
- target_behavior.mp4 (결과 폴더). 단순 운동학 모의, 실측 아님. disaster.world 벽·경사로·Husky·사람 위치를 그대로 씀.
- 제안 목표 (정할 것) / 지금 실측 (record2 2 회 + record4 유효 1 회):
  넘어짐 0 / 0 · 통신 구역 탐색 ≥85 % / 56–73 % · 도착률 ≥70 % / 0–57 % · 교체 ≤20 % / 43–67 % · 멈춤 실행당 ≤15 s / 5–114 s ·
  쉬는 비율 ≤10 % / 2–24 % · 경사로 위 0 s, 기울기 <10° / 40–70 s, 18–21° · 단절 구역 체류 ≤5 s, 판정 뒤 2 s 안 복귀 / 아직 안 넘어감 ·
  사람 발견 (통신 구역 안) 2/2 / 시작 안 함.
- 영상 속 모의는 경사로 판 위로 올라가지 않음 (경로 비용으로 피함). 실제 Nav2 는 아직 판을 경로에 넣음 -> 경사면 비용화가 남은 일.
- 정할 것: 단절 구역 안 사람 (-3.8, 5.4) — 짧게 들어갔다 나오기 / Husky 이동 / 제외.
