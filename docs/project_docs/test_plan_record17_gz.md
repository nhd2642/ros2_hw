# record17_gz 이후 시험 계획 (2026-11-10 13:xx)

출발점: HANDOFF_NEXT_SESSION.md. 결과 폴더 확인 시 fix1 실행 결과 없음(최신 실행 = Classic record16 20261110_003934_mission). gazebo_test_mission.sh / walk_check.sh 는 아직 sim_headless.launch.py 로 헤드리스 -> 11-10 '시각화 같이 띄우기' 지시 미반영.

## 단계
0. (막힘) 사용자 PC: fix1 apply -> walk_check.sh. 판정: 서버 생존, 직진 >= 90 %, odom/raw ~95 %, /odom 비율과 RTF 기록.
1. 시각화 옵션 (fix2 후보): 시험 스크립트에 VIS=1 기본 -> Gazebo GUI(-g 별도 클라이언트), RViz(공유·로봇별 지도, survivor_markers, 목표·경로), 결과 폴더에 위 카메라 동영상(overhead.sdf+cap.py) 또는 replay.py 재생. 검증: 헤드리스와 RTF 차이 기록, 동영상 파일 생성 확인.
2. 사용자 PC 미션: mission_commoff -> DUR=600 mission. 판정은 HANDOFF §5 (구역 >= 95 %, 넘어짐 0, off 수신 > 0 후 복귀·재개, 64~107 m², 사람 위치 오차, '장애물 팽창 영역 -> 옮김' 로그). Classic 에서 회차 편차(record16 a1_1 구역 81 %)가 있었으므로 판정 전 600 s 미션 2 회 권장.
3. 일반화 (Classic 수준 확인 뒤): 경계 y=0 -> Husky·시작점 계산, comm_zones.yaml -> 월드 설정, 시작·정답 -> 설정, 벽시계 한도 -> 시뮬 시간.

## 시험 피라미드
- 단위 (클라우드 pytest, 현재 comm_relay 66 + terrain 17): 빠진 것만 추가 — refine_goal 경계(255 모름 제외, 0.5 m 안 여유 칸 없음 -> 블랙리스트), 완료 판단(frontier_options -1·30 s), 통신 off 중 발견 큐 -> 복구 때 전송·중복 없음, arbiter 우선순위 teleop>hold>return>nav, 일반화 시 경계 계산(Husky 위치 바꿔 3 경우), 시뮬 시간 타이머(RTF 0.1/1.0).
- 통합 (클라우드 mock_mission.sh + mock_nav2/mock_robot): off->복귀->on->재개 상태 순서, 발견 off 큐 flush, DONE 후 cmd_vel 0.
- 시스템 (사용자 PC Gazebo Sim): walk_check, mission_commoff, 600 s mission + 동영상.
- 넘기는 것: CHAMP 보행 내부, Nav2 자체, 일회성 진단 스크립트.

## 클라우드 환경
이 세션에는 /home/claude/gz 없음 -> 단위·통합 시험을 돌리려면 cloud_tools.zip README 로 재구축(약 20 분).
