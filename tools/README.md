# tools

- `compare_runs.py <결과 폴더> <월드 파일> [실행 폴더 ...]` — 실행별 비교표 + `<결과 폴더>/compare.csv`. 정답 = 월드 벽 상자(높이 0.3 m 이상) + Husky 0.99x0.67 + 마네킹 0.5x0.4, 건물 안(|x|<4.85, |y|<6.5), 스폰에서 4 방향으로 닿는 자기 구역 칸 (남 58.7 / 북 58.8 m²). 탐색률은 progress/map_0300s·0600s.npz(공유 지도, 월드 정렬)에서, 나머지는 events.csv 의 sim(미션 시뮬 초) 600 s 까지.
  필요한 파일: runs.csv, 실행별 events.csv, progress/map_0300s.npz, progress/map_0600s.npz. 경사로 가장자리 구역 멈춤/실패·계획 실패 수는 아직 이 도구에 없음 (세션에서는 따로 짧은 스크립트로 셈: 구역 |x| 0.6~1.5, y -3.2~-1.7, events 의 stuck_start/stuck_end·fail).
- `planner_probe/` — planner_server 하나만 띄워 실행 지도(map.npz)로 경로를 구하는 시험 (fix18 팽창 반경 비교에 씀). `fake.py`(map.npz -> /a1_1/map, 가짜 /clock·TF), `params.yaml.in`(RADIUS 를 바꿔 씀), `client.py`(스폰 -> 5 곳 ComputePathToPose, 길이·벽 여유). 실행 예는 history/CHANGELOG 의 fix18 참고. map.npz 는 결과 폴더의 progress/map_0600s.npz 를 복사.
- `terrain_hazard_count.py` — /a1_1/terrain_hazards 점 수와 가장자리 위치 (terrain_perception/tools/fake_depth.py 와 같이 씀, fix16 확인용).
