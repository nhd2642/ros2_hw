# Result File Formats

`run_mission.sh` result folder: `results/<YYYYMMDD_HHMMSS>_set<k>/`

## timeline.csv

Written by `mission_watch.py`, one row every 10 s of sim time. Positions are Gazebo ground truth.

| Column | Unit / values | Meaning |
| --- | --- | --- |
| `t` | s | sim time since mission start |
| `phase` | `IDLE` `TRANSIT` `EXPLORE` `REGROUP` `DONE` | mission phase |
| `set` | 0-4 | set number (4 = finished) |
| `comm` | 0 / 1 | comm |
| `husky_x`, `husky_y` | m (world) | Husky true position |
| `a1_1_x`, `a1_1_y`, `a1_2_x`, `a1_2_y` | m (world) | A1 true positions |
| `husky` | `IDLE` `GO` `AVOID` `BLOCKED` `HOLD` `ARRIVED` | husky_driver state |
| `a1_1`, `a1_2` | `IDLE` `PLANNING` `TURNING` `FOLLOWING` `ESCAPING` `PAUSED` `FALLEN` `DONE` … | nav_executor state |
| `a1_1_comm`, `a1_2_comm` | `WAIT` `NORMAL` `RETURNING` `HOLD` `STOPPING` | return_relay state |
| `people` | count | person reports received by the Husky |
| `a1_1_slam_err`, `a1_2_slam_err` | m | distance between `/<ns>/pose_shared` and truth |

## action.log

Output of the `ros2 action send_goal --feedback /husky/run_mission ...` that `run_mission.sh` uses to request the mission: feedback every second (phase, set, found), then the result (`success`, survivor list, `found`, `expected`, `duration`) and `Goal finished with status: SUCCEEDED`.

## summary.txt

The full standard output of `mission_watch.py` (in Korean): status lines (space separated, same columns as the timeline), a line on each phase change (`--- t=   371 s  EXPLORE set2 (세트 1), 사람 2` = set 1, 2 people), and the final `=== 요약` (summary) block:

```text
=== 요약
  <phase>  세트 <k>:  <start> ->  <end> s ( <duration> s)          # per phase
  사람 보고 <n> 건, 정답 <m> 명 중 1.0 m 안 일치 <j> 명: [[x, y], ...]   # n reports, j of m matched within 1.0 m
  넘어짐: 없음 | <robot> <time> ...                                   # falls: none | ...
  홈 복귀 (a1_1 / a1_2): 세트 <k> 도착|실패|-/도착|실패|-, ...       # return home per set: arrived (도착) | failed (실패) | none
  최소 중심 거리: a1_1-husky … m, a1_2-husky … m, a1_1-a1_2 … m, 추종 중 Husky 와 최대 … m   # min center distances; max distance to Husky while following
  SLAM 공유 자세 최대 오차: a1_1 … m, a1_2 … m                        # max SLAM shared-pose error
  복구 장치 작동 (mission.log 줄 수, * 는 throttle 로그라 덜 셀 수 있음):   # recovery-layer activations (mission.log lines; * = throttled log, may undercount)
    a1_1: 0 명령 감시 <n>, 기울기 가드 <n>, 벗어나기 <n>, 옮겨 보기 <n>, … , 도착 목표 다시 옴* <n>   # per robot, 13 layers
RESULT PASS|FAIL (마지막 단계 …, 한도 초과 세트 …, 사람 j/m, 홈 실패 h)   # last phase, sets over limit, people, home failures
```

**PASS** = reached the end + 0 falls + 0 REGROUP + every person of the assigned sets reported within 1.0 m of truth + 0 home failures (added 2026-11-19; before that an A1 that gave up after 8 return attempts and stopped where it was still counted as returned).

## mission_events.txt

Only the lines of `mission.log` starting with `[MISSION]`, `[COMM]` or `[HUSKY]` (ROS log prefix removed, cut at 200 characters).

## Logs

| File | Content |
| --- | --- |
| `sim.log` | `sim.launch.py` (Gazebo, CHAMP, controllers, bridge) |
| `slam.log` | `dual_slam.launch.py` |
| `planner.log`, `controller.log` | Nav2 |
| `terrain.log` | `terrain.launch.py` |
| `mission.log` | `mission.launch.py` (the whole mission layer) |

Log tags: `[MISSION]` phases, sets, people tally; `[COMM]` comm transitions; `[HUSKY]` Husky driving; `[SURVIVOR]` person detection and delivery.
