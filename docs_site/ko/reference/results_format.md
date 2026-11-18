# 결과 파일 형식

`run_mission.sh` 결과 폴더: `results/<YYYYMMDD_HHMMSS>_set<k>/`

## timeline.csv

`mission_watch.py` 가 시뮬 10 초마다 한 줄씩 씁니다. 위치는 Gazebo 정답입니다.

| 칸 | 단위·값 | 뜻 |
| --- | --- | --- |
| `t` | s | 임무 시작 뒤 시뮬 시간 |
| `phase` | `IDLE` `TRANSIT` `EXPLORE` `REGROUP` `DONE` | 임무 단계 |
| `set` | 0~4 | 세트 번호 (4 = 끝) |
| `comm` | 0 / 1 | 통신 |
| `husky_x`, `husky_y` | m (world) | Husky 정답 위치 |
| `a1_1_x`, `a1_1_y`, `a1_2_x`, `a1_2_y` | m (world) | A1 정답 위치 |
| `husky` | `IDLE` `GO` `AVOID` `BLOCKED` `HOLD` `ARRIVED` | husky_driver 상태 |
| `a1_1`, `a1_2` | `IDLE` `PLANNING` `TURNING` `FOLLOWING` `ESCAPING` `PAUSED` `FALLEN` `DONE` … | nav_executor 상태 |
| `a1_1_comm`, `a1_2_comm` | `WAIT` `NORMAL` `RETURNING` `HOLD` `STOPPING` | return_relay 상태 |
| `people` | 건 | Husky 가 받은 사람 보고 수 |
| `a1_1_slam_err`, `a1_2_slam_err` | m | `/<ns>/pose_shared` 와 정답의 거리 |

## summary.txt

`mission_watch.py` 의 표준 출력 전체. 상태 줄(공백 구분, timeline 과 같은 칸), 단계가 바뀔 때의 줄(`--- t=   371 s  EXPLORE set2 (세트 1), 사람 2`), 끝의 `=== 요약` 블록:

```text
=== 요약
  <단계>  세트 <k>:  <시작> ->  <끝> s ( <걸린> s)      # 단계마다
  사람 보고 <n> 건, 정답 <m> 명 중 1.0 m 안 일치 <j> 명: [[x, y], ...]
  넘어짐: 없음 | <로봇> <시각> ...
  최소 중심 거리: a1_1-husky … m, a1_2-husky … m, a1_1-a1_2 … m, 추종 중 Husky 와 최대 … m
  SLAM 공유 자세 최대 오차: a1_1 … m, a1_2 … m
  복구 장치 작동 (mission.log 줄 수, * 는 throttle 로그라 덜 셀 수 있음):
    a1_1: 0 명령 감시 <n>, 기울기 가드 <n>, 벗어나기 <n>, 옮겨 보기 <n>, … , 도착 목표 다시 옴* <n>   # 로봇마다, 장치 13 개
RESULT PASS|FAIL (마지막 단계 …, 한도 초과 세트 …, 사람 j/m)
```

**PASS** = 끝까지 감 + 넘어짐 0 + REGROUP 0 + 맡은 세트의 사람을 전부 정답 1.0 m 안으로 보고.

## mission_events.txt

`mission.log` 에서 `[MISSION]`, `[COMM]`, `[HUSKY]` 로 시작하는 줄만 뽑은 것 (앞의 ROS 로그 머리 제거, 200 자에서 자름).

## 로그

| 파일 | 내용 |
| --- | --- |
| `sim.log` | `sim.launch.py` (Gazebo, CHAMP, 컨트롤러, bridge) |
| `slam.log` | `dual_slam.launch.py` |
| `planner.log`, `controller.log` | Nav2 |
| `terrain.log` | `terrain.launch.py` |
| `mission.log` | `mission.launch.py` (임무 계층 전체) |

로그 머리말 꼬리표: `[MISSION]` 단계·세트·사람 집계, `[COMM]` 통신 전이, `[HUSKY]` Husky 주행, `[SURVIVOR]` 사람 인식·전달.
