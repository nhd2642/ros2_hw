# 5. Read a Result Folder

`run_mission.sh` creates a `results/<date_time>_set<k>/` folder for each run. This tutorial uses the committed passing run `results/20261114_062151_set0` to read the results and plot them.

:::{note}
The committed example run is from the **earlier setup (2026-11-14, 3 sets + common building, 9 people)**. Files are read the same way, but set count, people and timing differ from today (4 sets + common building, 11 people). Current runs also write `motion.csv`, `stuck.txt` and `action.log`.
:::

:::{note}
The analysis tools print Korean. A few words you will see: `요약` summary, `세트` set, `사람` person/people, `건` reports, `명` persons, `넘어짐` falls, `없음` none, `최소 중심 거리` minimum center distance, `오차` error.
:::

## Folder contents

| File | Content | Look first |
| --- | --- | --- |
| `summary.txt` | all status lines + the final summary and `RESULT PASS/FAIL` | ★ last 15 lines |
| `timeline.csv` | one row every 10 s of sim time (phase, positions, node states, people, SLAM error) | for plots |
| `mission_events.txt` | only the `[MISSION]` `[COMM]` `[HUSKY]` lines | ★ what happened when |
| `mission.log` | full mission layer log (nav_executor, survivor_detector …) | finding causes |
| `sim.log`, `slam.log`, `planner.log`, `controller.log`, `terrain.log` | per-step launch logs | startup failures, Nav2 errors |

## 1. The verdict

```bash
tail -15 results/20261114_062151_set0/summary.txt
```

```text
=== 요약
  TRANSIT  세트 0:      0 ->     57 s (   57 s)
  EXPLORE  세트 0:     57 ->    322 s (  265 s)
  ...
  사람 보고 9 건, 정답 9 명 중 1.0 m 안 일치 9 명: [[10.31, 6.1], ...]
  넘어짐: 없음
  최소 중심 거리: a1_1-husky 1.49 m, a1_2-husky 1.39 m, a1_1-a1_2 0.46 m, 추종 중 Husky 와 최대 2.71 m
  SLAM 공유 자세 최대 오차: a1_1 1.29 m, a1_2 0.34 m
RESULT PASS (마지막 단계 DONE 세트 4, 한도 초과 세트 0, 사람 9/9)
```

| Line | Reading |
| --- | --- |
| per-phase times | which set took long. In this run set 2 exploration was the longest at 787 s |
| person reports | number of reports and how many people matched the truth (Gazebo placement) within 1 m. Reports > people means duplicates or false positives |
| minimum center distance | the closest any two robots got. An A1 is 0.6 m long, so under 0.5 m they may have touched |
| max SLAM error | how far the A1 map drifted. Close to 1 m, zones and person positions can be off |

## 2. Following the events

```bash
less results/20261114_062151_set0/mission_events.txt
```

Here you see set start and arrival, person reports, comm transitions and Husky waits in time order. For deeper causes (plan failures, follow failures) search `mission.log` by robot name and time.

```bash
grep "a1_1" results/20261114_062151_set0/mission.log | grep -E "실패|홈|도착" | head    # failure | home | arrived
```

## 3. Plotting

The result plots in these docs are drawn by `docs_site/scripts/make_figures.py` from `timeline.csv`.

```bash
cd ~/ros2_hw && python3 docs_site/scripts/make_figures.py
```

```{figure} ../../images/figures/trajectories.png
:width: 100%

Robot paths over the full mission (Gazebo ground truth, 10 s apart). Grey rectangles are each A1's exploration zone, stars are true person positions, white squares are Husky parking spots.
```

```{figure} ../../images/figures/slam_error.png
:width: 100%

SLAM shared-pose error. Grey bands are TRANSIT, the pink band is the comm loss. Just before set 2 the a1_1 error grows to 1.27 m, then drops to 0.05 m with the Husky-referenced correction on arrival.
```

```{figure} ../../images/figures/people.png
:width: 100%

Person reports received by the Husky. Two per set and three in the common building, for 9.
```

To plot another run, change `MAIN` and `RUNS` at the top of the script to that folder name.

## 4. Comparing runs

`tools/compare_runs.py`, used to compare runs of the previous architecture (fix stages), expects a different result format. For the current mission structure compare the summary lines of `summary.txt` and `timeline.csv` directly. The cumulative record is in `history/results/runs.csv`.

Every column of every file is listed in {doc}`../reference/results_format`.
