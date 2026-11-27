# Test Results

Test setup: Gazebo Sim headless, WSL2, RTF about 0.3 (2026-11-12 to 20). Positions and person truth come from Gazebo ground truth and are used **for evaluation only**.

Verdict (`mission_watch.py`): **PASS** = reached the end + 0 falls + 0 exploration timeouts (REGROUP) + every person of the assigned sets reported within 1.0 m of truth + 0 home failures (added 2026-11-19; before that an A1 that gave up after 8 return attempts and stopped where it was still counted as returned).

## Summary

::::{grid} 2 2 4 4
:gutter: 2

:::{grid-item-card}
:text-align: center
**11 / 11**
^^^
people found (final setup, passing run)
:::

:::{grid-item-card}
:text-align: center
**0**
^^^
falls and home failures (4 MPPI full missions)
:::

:::{grid-item-card}
:text-align: center
**1,910 s**
^^^
sim time (5 sets, about 1.6 h real)
:::

:::{grid-item-card}
:text-align: center
**2 / 4**
^^^
MPPI full-mission PASS (final setup 1 / 2)
:::
::::

## Robustness experiments (2026-11-19 to 20)

Complex obstacles are what shows robustness, so the layout was kept (a rule-breaking set 4 was even added) and navigation, gait and SLAM were changed instead.

- **Method**: one behavior change per bundle, verdict criteria written before running, and the number of times the changed behavior actually fired counted first. The benchmark is a partial run of three hard scenes (set 2 south hall, set 4, common building) × 2.
- **Diagnosis**: `motion.csv` + `stuck_report.py` split stalls (5 s of following with under 0.1 m of true motion) into gait vs. navigation. 99 % of baseline stall time was navigation (DWB zero commands, oscillation, turning in place).

| Bundle | Change | Benchmark result (6 runs) | Verdict |
| --- | --- | --- | --- |
| baseline | DWB + NavFn | PASS 5/6, 1 home failure, stall total 397 s, 24 "Failed to make progress", exploration total 2470 s | |
| N2 | DWB reverse allowed + progress check 0.2 m | PASS 1 of 5, 3 home failures, stalls up in every scene (back-and-forth oscillation, odom drift up to 4.19 m) | rejected |
| **N1** | **local controller DWB → MPPI** (forward 0-0.2 m/s, no reverse) | **PASS 6/6, 0 home failures, stall total 56 s, 3 "Failed to make progress", exploration total 2061 s** | **adopted** |
| N3 | global planner NavFn → Smac 2D | PASS 4 of 5, no exploration-time gain | rejected |
| G2 | MPPI sideways steps (Omni, vy 0.1 m/s) | PASS 6/6, stall 0 s, exploration total 2058 s | adopted, then reverted |
| S1 | slam_toolbox matching window / minimum travel | 8 replays of the same bag with the same settings gave max error 0.97-4.03 m — not reproducible | stopped |
| **E1** (11-26) | **custom interfaces + frontiers within 1 m of reached goals removed, frontiers under 0.5 m ignored + exploration max 0.35 m/s** (tested together) | **PASS 6/6, 0 home failures, exploration total 1281 s (−38 % vs N1), revisit ratio 17.6 → 14.0 %, mean follow speed 0.22-0.29 m/s** | **adopted** |
| **S3** (11-27) | **slam_toolbox `minimum_travel_distance` 0.2 → 0** (scans processed while turning in place on a slope) | **ramp bench ×3: scans processed during the turn 0-1 → 30-33, max error 1.12-1.26 → 0.07-0.16 m. Full missions 2/2 PASS** (below) | **adopted** |
| S4 (11-27) | gate which scans go to SLAM (`scan_ready_relay`: first by odom motion, then by walk/turn command) | bench passed but full missions 0/2 PASS (people 10/11, 8/11). "no reachable goal for 90 s" 0 → 4-6, closed paths 0 → 39-58 lines | rejected (keep S3) |

- **Why G2 was reverted**: while following, the body heading differed from the direction of travel by more than 20° in 45-50 % of samples (N1: 1-2 %). Walking at a slant half the time, the camera did not look where the robot was going, and in a full mission a person was missed at 0.63 m because they were outside the field of view. The stall reduction was also 0 s under N3 (DiffDrive), so it was taken as run-to-run noise.
- **Cost of MPPI**: thousands of "control loop missed its desired rate" warnings per run (10 Hz wall clock; DWB 0). See {doc}`limitations`.
- **E1 revisit numbers**: revisit ratio = distance walked over 0.5 m cells already crossed more than 60 s earlier / distance walked (return path included). It dropped a lot for `a1_1` (set4 31 → 22/25 %, common 30/19 → 1/10 %); `a1_2` was higher than baseline in set4 run 2 (27 %) and common run 1 (21 %). The exploration-time gain comes from both speed and fewer revisits and was not split.

## Full missions with the E1 setup (N1 + custom interfaces + revisit prevention + 0.35 m/s) — 5 sets, 11 people

| Run | Result | People | Home fail / falls | Sim time | Exploration (set 1 / 2 / 3 / 4 / common) | Stall | SLAM max (a1_1, a1_2) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `20261126_162742_set0_e1full_1` | PASS | 11/11 | 0 / 0 | 1059 s | 113 / 128 / 132 / 216 / 263 s | 6 s | 0.22, 0.41 m |
| `20261126_173431_set0_e1full_2` | FAIL | 10/11 | 0 / 0 | 1268 s | 129 / 136 / 137 / 225 / 416 s | 70 s | 0.61, 0.53 m |

- PASS 1/2, the same as the N1 full missions below (1/2). Total time 1910 / 1582 s → 1059 / 1268 s, revisit ratio 25.4 / 18.7 → 15.2 / 12.7 %.
- The person missed in run 2 (corner of the cluttered north room of set 4) was within the field of view of `a1_2` for over 10 s from 1.1 m away but was dropped by detection: the camera saw it at 0.93, but no lidar points were found in that direction (likely hidden behind boxes and pillars). Fewer revisits may have meant fewer second looks (one case).

## D1 (11-26 to 27): person detection MobileNet-SSD → YOLOv4-tiny, locating fixed — full missions
| Run | Result | People | Home fail / falls | Sim time | Exploration (set 1 / 2 / 3 / 4 / common) | Stall | SLAM max (a1_1, a1_2) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `20261127_000932_set0_d1bfull_1` | FAIL | 2/11 | 0 / 0 | (stuck) | stuck in transit after set 1 | - | over 4 m (a1_1) |
| `20261127_011132_set0_d1bfull_2` | PASS | 11/11 | 0 / 0 | 1094 s | 118 / 158 / 125 / 205 / 262 s | 11 s | 0.21, 0.24 m |

- Cause 1 (miss in E1 run 2): SSD could not see people from the side (north person of set 4: 0.00 in 21 of 26 frames at 2-6.6 m). YOLOv4-tiny scored 0.53-0.94 on the same frames with 0 false positives. Set 4 partial runs: 2/2 PASS, that person found at 6.2-6.3 m (2.6-4.9 m with SSD).
- Cause 2 (north person of set 3 missed in the first YOLO-only full run): detected three times but dropped by locating. The depth estimated from the box height was compared with the straight-line lidar range (1/cos off at the edge of the view), and YOLO boxes are taller so the 1.75 m reference height underestimated the distance → compare depths, reference height 2.0 m.
- Two runs after the fix: run 1 stopped in transit because `a1_1`'s SLAM drifted 1.7 m on the south ramp of set 1 (unrelated to detection); run 2 PASS 11/11. Details in `docs/experiments/D1_기준.txt`.

## S3 (11-27): SLAM gets no scans while turning in place on a slope → `minimum_travel_distance` 0.2 → 0
- Trigger: in run 1 after the D1 fix, `a1_1` turned in place for 7.5 s with MPPI at the north end of the ramp in the south room of set 1. The body was pushed 0.67 m while leg odom saw almost nothing; walking across the slope then added 0.6 m more slip, and the SLAM error went 1.7 m → 4.3 m.
- Cause: slam_toolbox (Humble) gates the next scan on odom travel distance only (`minimum_travel_distance` 0.2 m; rotation is not considered). Turning in place barely adds odom distance, so during a 16 s full turn only 0-1 scans were processed and the slip went unseen (map→odom constant through the turn). Walking speed was not the cause (slip per minute on the slope was similar at 0.2 and 0.35 m/s over 10 full missions).
- Fix: `minimum_travel_distance` 0.0 in `a1_exploration/config/slam_params.yaml` → a scan every `minimum_time_interval` (0.5 s).
- Ramp bench `comm_relay/tools/ramp_check.sh` (sim + SLAM only, no Nav2, about 5 min per run): `a1_1` is placed in front of the ramp in the south room of set 1 at (6.4, -2.4) and walks up, turns 360° in place on the slope, crosses the slope with a 180° turn, walks down to the flat, turns 180° and stands — twice. Each phase writes slip (true motion minus odom motion), SLAM error and the number of scans SLAM processed to `ramp.txt`.

| Bench, 3 runs each (6 × 360° turns) | baseline (0.2) | S3 (0.0) | criterion (set before running) |
| --- | --- | --- | --- |
| scans processed during the 360° turn on the slope | 0-1 | 30-33 | mean ≥ 8 |
| mean SLAM error right after the turn | 0.81 m | 0.01 m | ≤ 0.40 m |
| SLAM error standing on the flat, mean / max | 0.53 / 1.13 m | 0.01 / 0.02 m | ≤ 0.37 / 0.6 m |
| max SLAM error in the run | 1.12-1.26 m | 0.07-0.16 m | |
| slip per turn (slope / flat) | 0.47-0.53 / 0.07-0.10 m | 0.44-0.51 / 0.10-0.16 m | |
| falls | 0 | 0 | 0 |

| Run | Result | People | Home fail / falls | Sim time | Exploration (set 1 / 2 / 3 / 4 / common) | Stall | SLAM max (a1_1, a1_2) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `20261127_085716_set0_s3full_2` | PASS | 11/11 | 0 / 0 | 979 s | 114 / 135 / 137 / 175 / 211 s | 0 s | 0.50, 0.44 m |
| `20261127_095400_set0_s3full_1b` | PASS | 11/11 | 0 / 0 | 1046 s | 136 / 110 / 144 / 171 / 256 s | 0 s | 0.37, 0.28 m |

- The slip itself is unchanged. What changed is that SLAM now gets scans during the turn and corrects the slip.
- Full missions: 0 slam_toolbox processing-delay warnings, RTF 0.28 / 0.32 (D1 PASS 0.31). The max SLAM error is higher than the D1 PASS (0.21 / 0.24 m), mostly in set 4 and the common building (sets 1-3: 0.12-0.27 m), and within the range of earlier MPPI full missions (0.21-0.72 m).
- Nav2 failed to come up at startup in 3 of 5 starts (earlier about 5 in 30). `20261127_085156_set0_s3full_1` failed both starts and ended without a mission, so it was rerun as 1b. Whether this is related to the setting (processing at 2 Hz even when standing) was not checked. Details in `docs/experiments/S3_기준.txt`.

## S4 (11-27, rejected): feed fewer scans to SLAM while standing
- Why: after S3 the common-building SLAM max error (0.43 / 0.34 m) was above the earlier MPPI median of 0.22 m. S3 processes at 2 Hz even when standing.
- First version (forward every 0.18 m of odom motion or 20° of turn): while a turn stalled on the slope and the body slid 0.4 m over 4.5 s, odom saw only 0.03 m, so no scans went through. Max error during the turn 0.35-0.48 m. Dropped.
- Second version S4c (forward everything within 1 s of a walk/turn command, one scan every 5 s when the command is zero): bench ×3 passed (31-34 scans in the turn, mean max error 0.17 m, 2 scans while standing).
- Full missions ×2: FAIL / FAIL (people 10/11, 8/11, home failures 0). "no reachable goal for 90 s" appeared 4 / 6 times and closed-path lines 39 / 58, none of which appeared in the 5 S3/E1/D1 full missions.
  This looks related to map and pose updates stopping while standing, but was not checked. → keep S3, code reverted. Record in `docs/experiments/S4_기준.txt`.


## Full missions with the final setup (baseline + N1) — 5 sets, 11 people

| Run | Result | People | Home fail / falls | Sim time | Exploration (set 1 / 2 / 3 / 4 / common) | Stall | SLAM max (a1_1, a1_2) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `20261120_184028_set0_finaln1_1` | **PASS** | 11/11 | 0 / 0 | 1910 s | 228 / 405 / 400 / 348 / 324 s | 18 s | 0.72, 0.48 m |
| `20261120_203446_set0_finaln1_2` | FAIL | 10/11 | 0 / 0 | 1582 s | 169 / 262 / 221 / 304 / 397 s | 6 s | 0.25, 0.23 m |
| (G2) `20261120_134110_set0_final_1` | FAIL | 10/11 | 0 / 0 | 1738 s | 181 / 428 / 232 / 358 / 315 s | 0 s | 0.26, 0.32 m |
| (G2) `20261120_153450_set0_final_2` | PASS | 11/11 | 0 / 0 | 1548 s | 178 / 266 / 203 / 361 / 334 s | 0 s | 0.43, 0.43 m |

- 4 MPPI full missions: 2 PASS, 2 FAIL (each missed one person). All four finished all five sets with 0 home failures, falls, timeouts and "Failed to make progress". The last full mission with the baseline code (`20261118_220301_set0`, before set 4 was added) failed with 3 home failures, SLAM max 0.83 / 0.86 m.
- **Person missed in N1 run 2**: the inner room at the end of the set 4 south L-shaped corridor. a1_1 failed path following twice near the 1.0 m door → "close as dead end and back off" → goals toward the inner room were then dropped as "passes a closed dead end" and exploration ended (2.6 m from the person, behind the corridor wall). The other 7 MPPI runs all found this person.
- One of the 12 reports in N1 run 1 was a duplicate of the set 3 north person (1.5 m from truth).
- 2D time-lapse video for presentations: `results/20261120_184028_set0_finaln1_1/mission_x20.mp4` (`tools/mission_anim.py`, 20×, 96 s). Set 4 3D scene clip: `results/20261120_224610_set3_clip/set4_x8.mp4` (final setup, set 4 partial run PASS, 8×, 39 s, stitched from Gazebo GUI screenshots taken every 0.5 s of sim time).

## Last PASS in the DWB era — `results/20261114_062151_set0` (3 sets + common building)

```{figure} ../../images/figures/phase_timeline.png
:width: 100%

Per-phase times of the two passing runs from the DWB era. Red = exploration, grey = transit.
```

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

## Earlier DWB-era PASS — `results/20261113_035152_set0` (no comm toggle)

| Phase | Sim time | People | SLAM error mean / max (a1_1, a1_2) |
| --- | --- | --- | --- |
| transit → set 1 / explore | 60 / 184 s | 2/2 | 0.17 / 0.55, 0.12 / 0.18 m |
| transit → set 2 / explore | 49 / 240 s | 2/2 | 0.11 / 0.17, 0.08 / 0.20 m |
| transit → set 3 / explore | 49 / 207 s | 2/2 | 0.13 / 0.20, 0.43 / 0.78 m |
| transit → common / explore | 19 / 566 s | 3/3 | 0.13 / 0.24, 0.16 / 0.65 m |
| **total** | **1375 s** (about 80 min real) | **9/9** | |

- 11 person reports: 9 true + 2 with off positions in the common building.
- Following in transit: both A1s about 1.9 m behind the Husky in ±0.65 m lanes (at that time the Husky center estimate was biased 0.15 m inward → later fixed with the box fit).

## Reproducibility (DWB era)

Counting from the first PASS, **7 full missions were run, 4 reached the end, and 2 of those passed (counting home failures)** (`20261113_035152_set0`, `20261114_062151_set0`). Each run fixed the problem found in the previous one, so the code differs slightly between runs.

| Run | Change | Result |
| --- | --- | --- |
| `20261113_091042_set0` | (same code as the first PASS) | stopped in set 1: a1_1 returned by the 90 s rule after an inner-room follow failure (people 1/2), a1_2 failed its return plan 3 times (NavFn A* error) |
| `20261113_100158_set0` | NavFn Dijkstra, retry failed goals after 60 s | sets 1 and 2 passed, 0 plan failures. Stuck in set 2 at a 1.6 m turning entrance |
| `20261113_113239_set0` | turning entrances 2.4 m | revisited a never-clearing frontier in set 2 south; no end after 700 s |
| `20261113_123720_set0` | limit of 3 visits to the same spot | **reached the end** (2007 s). People 7/9 (entrance follow failures, a timer bug), a1_1 home failure in the common building |
| `20261114_011715_set0` | "no goal" timer reset every set | PASS at the time, **FAIL** by the current rule: a1_1 home failure in the common building. 1895 s, 9/9 (2 extra reports, scores 0.52 and 0.56) |
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
| stalls while following, 6-32 "Failed to make progress" per run | 99 % of stall time was navigation (DWB zero commands, oscillation, turning in place) | MPPI local controller (N1) |
| more home failures and stalls after allowing reverse | reverse amplified back-and-forth oscillation and odom drift | keep no reverse (N2 rejected) |
| person missed at close range after allowing sideways steps | 45-50 % of travel at a slant, camera not facing the path | sideways off (G2 reverted) |
| SLAM parameters can't be compared | replays of the same bag are not reproducible (cause unknown) | S1 stopped; fixed the replay tool's orphan-node bug |

## A1 gait

See {doc}`../how_to/measure_gait`. Forward actual/commanded 95-108 %, tilt 2.3-5.5° (0.1-0.5 m/s). Measured 2026-11-19: 0.05 m/s gives 81 %, sideways ±0.1 / ±0.2 m/s 105-108 %, turning in place ±0.3 / ±0.5 rad/s 107-109 %.
