# Test Results

Test setup: Gazebo Sim headless, WSL2, RTF about 0.3 (2026-11-12 to 14). Positions and person truth come from Gazebo ground truth and are used **for evaluation only**.

Verdict (`mission_watch.py`): **PASS** = reached the end + 0 falls + 0 exploration timeouts (REGROUP) + every person of the assigned sets reported within 1.0 m of truth.

## Summary

::::{grid} 2 2 4 4
:gutter: 2

:::{grid-item-card}
:text-align: center
**9 / 9**
^^^
people found (latest run)
:::

:::{grid-item-card}
:text-align: center
**0**
^^^
falls
:::

:::{grid-item-card}
:text-align: center
**1,907 s**
^^^
sim time (about 2 h real)
:::

:::{grid-item-card}
:text-align: center
**3 / 7**
^^^
full-mission PASS rate (since first PASS)
:::
::::

```{figure} ../../images/figures/phase_timeline.png
:width: 100%

Per-phase times of the two committed passing runs. Red = exploration, grey = transit.
```

## Full mission with the current code — `results/20261114_062151_set0` (PASS)

This run includes a comm 0/1 toggle during set 1 exploration.

| Phase | Sim time | People (within 1 m / configured) | Returned | SLAM shared-pose error mean / max (a1_1, a1_2) |
| --- | --- | --- | --- | --- |
| transit → set 1 | 57 s | | | |
| explore set 1 (incl. 60 s comm loss) | 265 s | 2/2 | both | 0.52 / 0.97, 0.10 / 0.12 m |
| transit → set 2 | 49 s | | | |
| explore set 2 | 787 s | 2/2 | both | 0.27 / 0.38, 0.13 / 0.19 m |
| transit → set 3 | 49 s | | | |
| explore set 3 | 260 s | 2/2 | both | 0.20 / 0.30, 0.13 / 0.19 m |
| transit → common | 19 s | | | |
| explore common | 421 s | 3/3 | both | 0.58 / 0.99, 0.15 / 0.28 m |
| **total** | **1907 s** | **9/9** | | |

```{figure} ../../images/figures/trajectories.png
:width: 100%

Robot paths (ground truth, 10 s apart).
```

```{figure} ../../images/figures/slam_error.png
:width: 100%

SLAM shared-pose error. On arrival at set 2 the 1.37 m correction of a1_1 drops its error from 1.27 to 0.05 m.
```

- 0 falls, 0 timeouts, 0 plan failures (0 NavFn errors). Exactly 9 person reports (no duplicates or false positives).
- **Comm 0 (144-194 s)**: both A1s went RETURNING 1.1 s later and the Husky stopped. `a1_1` reached HOLD 50 s later (in front of the door, 0.85 m from the explore start — retracing uses odom, so it is off by the odom drift); `a1_2` was still retracing. Comm 1: STOPPING (1 s) → NORMAL.
- **Husky-referenced pose correction** (on set arrival): `a1_1` **1.37 m** at set 2, 0.71 m at set 3, 0.25 m at common. `a1_2` 0.14 / 0.21 / 0.17 m. The `a1_1` map drifted sideways up to 1.3 m, but following was anchored to the Husky seen by lidar, so the real position stayed in the lane (|y| 0.72-0.82 m in transit).
- Follow failures: `a1_1` 7, return attempts 5 / `a1_2` 3 and 6 (all four sets eventually reached). The 787 s of set 2 exploration is `a1_1` working through the remaining frontiers in the south hall (3 pillars, 2 boxes).
- **Common building cooperation**: `a1_1` spent 79 % of exploration time in the south, `a1_2` 90 % in the north. 31 / 23 visited 1 m cells, 5 overlapping.
- **Closest approaches** (center distance, 1 Hz): A1 to A1 **0.46 m** (body length 0.6 m — they may have touched; contact was not measured), a1_1-Husky 1.49 m, a1_2-Husky 1.39 m.

```{figure} ../../images/figures/people.png
:width: 100%

Person reports received by the Husky.
```

## Earlier PASS — `results/20261113_035152_set0` (no comm toggle)

| Phase | Sim time | People | SLAM error mean / max (a1_1, a1_2) |
| --- | --- | --- | --- |
| transit → set 1 / explore | 60 / 184 s | 2/2 | 0.17 / 0.55, 0.12 / 0.18 m |
| transit → set 2 / explore | 49 / 240 s | 2/2 | 0.11 / 0.17, 0.08 / 0.20 m |
| transit → set 3 / explore | 49 / 207 s | 2/2 | 0.13 / 0.20, 0.43 / 0.78 m |
| transit → common / explore | 19 / 566 s | 3/3 | 0.13 / 0.24, 0.16 / 0.65 m |
| **total** | **1375 s** (about 80 min real) | **9/9** | |

- 11 person reports: 9 true + 2 with off positions in the common building.
- Following in transit: both A1s about 1.9 m behind the Husky in ±0.65 m lanes (at that time the Husky center estimate was biased 0.15 m inward → later fixed with the box fit).

## Reproducibility

Counting from the first PASS, **7 full missions were run, 4 reached the end, and 3 of those passed with 9/9 people** (`20261113_035152_set0`, `20261114_011715_set0`, `20261114_062151_set0`). Each run fixed the problem found in the previous one, so the code differs slightly between runs.

| Run | Change | Result |
| --- | --- | --- |
| `20261113_091042_set0` | (same code as the first PASS) | stopped in set 1: a1_1 returned by the 90 s rule after an inner-room follow failure (people 1/2), a1_2 failed its return plan 3 times (NavFn A* error) |
| `20261113_100158_set0` | NavFn Dijkstra, retry failed goals after 60 s | sets 1 and 2 passed, 0 plan failures. Stuck in set 2 at a 1.6 m turning entrance |
| `20261113_113239_set0` | turning entrances 2.4 m | revisited a never-clearing frontier in set 2 south; no end after 700 s |
| `20261113_123720_set0` | limit of 3 visits to the same spot | **reached the end** (2007 s). People 7/9 (entrance follow failures, a timer bug) |
| `20261114_011715_set0` | "no goal" timer reset every set | **PASS** 1895 s, 9/9 (2 extra reports, scores 0.52 and 0.56) |
| `20261114_062151_set0` | Husky box fit | **PASS** 1907 s, 9/9, 0 extra reports |

## Causes found in testing and fixes

| Symptom | Cause | Fix |
| --- | --- | --- |
| a1_2 walks 11° askew while following | shared-frame alignment trusted the spawn yaw (body rotates 10° when standing up) | alignment rotation 0 |
| no exploration candidates inside buildings | the top 15 frontiers were all on the road | search inside the zone only |
| smeared map, position off by 2.3 m | the moving Husky and other A1 left trails in the map | erase beams hitting other robots |
| return plan fails (hazard line along the road) | terrain layer marked the back of the Husky ahead as a step | terrain only in exploration phases + mask other robots |
| 2.3 m position jump in a tight room | suspected loop closure | `do_loop_closing: false` |
| every remaining goal rejected as "closed path" | closed-spot radius 0.8 m blocked a whole door | radius 0.4 m + done after 90 s without a goal |
| no goal in the common building | start position 4 m from the door | approach goal while outside the zone |
| return fails on a person model's base | goal cells allowed 0.15 m from obstacles | goal cells at least 0.4 m from obstacles |
| a1_2 enters the next building after 40 m | 1 m SLAM drift | Husky-anchored following + per-set correction + `odom_scaler` |
| a1_1 doesn't leave | took the small wall for the Husky | drop clusters more than 0.7 m from the received pose |
| 14 person reports / 7 matched | located by the nearest lidar point | point matching the box-height range, 7 m range |
| return plan with a zero-point path | NavFn A* error | Dijkstra |
| one follow failure abandons a room | failure exclusion 120 s > "no goal" 90 s | retry after 60 s, up to 3 failures per place |
| a set runs over 700 s | revisiting never-clearing frontiers | stop after 3 arrivals at the same spot |
| returns right away in a new set | timer based on the previous set | reset every set |
| 0.47 m wrong pose correction | Husky center estimated 0.4 m along the line of sight | box fit |
| low RTF | every Python node receiving /clock at 1 kHz | `/clock_slow` 50 Hz, TF 200 → 50 Hz |

## A1 gait

See {doc}`../how_to/measure_gait`. Actual/commanded 95-108 %, tilt 2.3-5.5° (0.1-0.5 m/s).
