# fix7 결과 (2026-11-10 17:05) — 20261110_162136_mission, DUR=600 (시뮬), RTF 0.31  (+ 17:50 0 명령 진단 1 차)

## 숫자
- 이동 a1_1 54.1 m / a1_2 58.4 m, 넘어짐 0, 최대 기울기 17° / 10°.
- 공유 지도 알려진 면적(coverage 이벤트): 시뮬 150 s 64.4, 300 s 85.6 (fix6 commoff 80.1, 통신 복귀 30 s 포함이라 공정 비교 아님), 450 s 97.5, 600 s 106.0 m² (zone_cov: 남 60.2 / 북 25.8 / 북쪽 방 20.0). Classic 최고 record15 106.6 m²(벽 600 s = 시뮬 약 270 s)와 같은 면적을 시뮬 600 s 에 도달.
- 사람: a1_1 이 남동쪽 방 person_standing 발견 (점수 0.97, 거리 0.95 m, 오차 0.20 m), 전달 0.1 s. 북서 마네킹(-3.77,5.41) 미도달.
- 목표: a1_1 도착 9·실패 13(0 명령 감시 7, 추종 실패 4, 상대 구역 2), a1_2 도착 14·실패 7(추종 5, 0 명령 2). 블랙리스트 a1_1 12 / a1_2 6.
- 시간(시뮬): a1_1 FOLLOWING 368 / IDLE 149 / TURNING 53 / ESCAPING 30 s; a1_2 423 / 96 / 71 / 9 s.
- controller.log: No valid trajectories 3 (fix6 7), Failed to make progress 6.

## fix7 효과
- 서·동 홀 진입: 됨. fix6 에서는 둘 다 못 들어감.
- 동쪽 틈은 아직 아슬아슬: a1_2 가 (1.33,0.28)(장애물까지 0.30 m)에서 시뮬 121~205 s 동안 vx 최대 0.022 로 기어가며 추종 실패 3 번 뒤 통과.

## 끝까지 갔나 -> 아직 아님 (600 s)
- 남은 프런티어: 서쪽 홀 서쪽 끝, 동쪽 홀 동쪽 끝, 북쪽 위(y 4.9~6), 남동 방 안쪽, 북서 영역(y>4, x<0.5, 마네킹 있음).

## 남은 병목
1. **0 명령 감시 9 번(a1_1 7) — 열린 곳에서**: 실패 위치 장애물까지 0.40~1.05 m, 목표까지 1.2~4 m. Nav2 vx·wz 둘 다 0 이 8 s, controller.log 오류 없음. -> 실패 -> 로봇 블랙리스트(TTL 120 s) -> a1_1 IDLE 149 s(25 %).
2. 동쪽 틈 통과 느림.
3. 구역 경계: 경로 y +0.16~0.17 로 margin 0.15 넘어 2 번 거부.
4. 회전 9~12 %.

## 0 명령 진단 (fix8 도구, 17:50)
- 도구: `DWB_DIAG=1` -> nav2_controllers FollowPath.publish_evaluation, tools/dwb_diag.py 가 'FOLLOWING 인데 Nav2 명령 0 이 시뮬 3 s' 순간 DWB 평가 기록 -> 결과 폴더 dwb_diag.jsonl.
- **클라우드 1 건 (시뮬 49 s, a1_1)**: 궤적 159 중 유효 89, 무효 70 개 전부 Oscillation critic. DWB 가 고른 것 = 정지(vx 0, wz 0) 합 36.2 (GoalAlign 17.4, GoalDist 18.0, PathAlign 0.8, PathDist 0.0). 유효한 전진 최고 = vx 0.067 wz -0.143 합 38.4 (GoalDist 18.0 같음, PathDist 0.8, PathAlign 1.6) -> 정지가 이김.
- 메커니즘(가설, DWB humble 소스로 확인한 부분): OscillationCritic 은 부호가 바뀐 방향의 궤적을 '무효' 로 막고, 로봇이 0.05 m 또는 0.2 rad 움직여야 풀림. oscillation_reset_time 기본 -1(시간으로는 안 풀림). 정지 궤적은 이 critic 에서 늘 유효(점수 0). 그래서 한 번 정지를 고르면 안 움직이니 플래그가 안 풀리고 계속 정지 -> 8 s 뒤 0 명령 감시. 빠른 전진 궤적이 막힌 상태에서 남은 느린 전진은 1 s 안에 목표 쪽으로 1 칸도 못 가 GoalDist 이득이 없고 경로에서 벗어나는 만큼 PathDist·PathAlign 이 커져 정지에 짐.
- 확정하려면 사용자 PC 진단 실행에서 0 명령 정지 대부분이 같은 모양(무효 원인 Oscillation, 고른 것 정지)인지 볼 것. 그렇다면 후보 조치: Oscillation.oscillation_reset_time 양수(예 1~2 s) — 시간으로도 풀리게.

## 다음 후보
- (진단 확정 뒤) Oscillation reset_time.
- 구역 경계 margin 또는 경로 거리 기반 분할.
- DUR=900.
