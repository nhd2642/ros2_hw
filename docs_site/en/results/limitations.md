# Known Limitations

The limitations known today, written down plainly. Use this as a starting point when choosing new work.

## Reproducibility

:::{warning}
**The sample is small.** All 7 full missions of the current setup line (since 2026-11-27) passed, but the setup changed slightly within those 7 (about 2 runs per change), so there are not enough runs to say how often it passes.
:::

## Position

- **SLAM drift within a set.** An A1 map can drift even within one set (DWB era `a1_1`: up to 0.97 m in set 1, 0.99 m in the common building; 4 MPPI full missions: max 0.23-0.72 m; 2 after S3: 0.28-0.50 m; 2 with W1: 0.25-0.96 m, the 0.96 m in the common building, a1_2; 2 after R1: 0.16-0.26 m in the common building). R1 only aligns the heading to odom at set start, so drift within a set remains (heading error |median| while not turning 0.56-0.82°).
- **SLAM offline replay is not reproducible.** Replaying the same bag with the same settings 8 times through `slam_replay.py` gave max errors of 0.97-4.03 m, so it could not be used to compare SLAM parameters. The cause was not found. It is re-aligned to the Husky on every set arrival, but drift that builds up inside a set stays, so zone rectangles and reported person positions can be off by that much.
- **HOLD position offset.** The spot where a robot stops after a comm loss is odom based and can be away from the explore start (latest run 0.85 m).
- **Slip when turning in place on a slope.** One turn in place on the slope slides the body about 0.5 m (0.1 m on the flat). Leg odom does not see it; SLAM corrects it with a scan every 0.5 s (S3). The slip itself was not reduced.
- **Drift while walking in place.** When an A1 is blocked and walks in place, leg odom keeps advancing and the SLAM position drifts. The cause is not fixed; it is avoided by removing trap features from the layout and re-aligning to the Husky every set.

## Following & exploration

- **Following failures at narrow doors and corridors.** With MPPI, "no progress" failures dropped to 0-3 per run, but path-following failures at narrow doors and corridors remain. After 3 failures at the same place the set gives it up, so the mission continues but people in that room are missed.
- **A dead-end closure can block the only way in.** After two following failures at the same place, the dead-end recovery closes that spot; if it is the only corridor into an inner room, that room is given up (set 4 south 1.2 m L-shaped corridor — why a person was missed in the 11-20 N1 runs). Nothing reopens a closed spot.
- **MPPI compute load.** Thousands of "control loop missed its desired rate" warnings per run (10 Hz wall clock, with batch size cut to 1000). The results come from an Intel Core Ultra 5 226V (8 cores), WSL2 with 7.5 GB, load average about 24 during the sim, with Gazebo cameras drawn by the software renderer (swrast). Some parts are tied to the wall clock, so behavior may differ somewhat on a faster PC.
- **Return failure.** If an A1 can't get back after 8 attempts, it reports the set done where it is. The Husky then waits for it in the next transit. It happened in 3 full missions, in the common building center room (1.6 m east doorway, a person in the middle) and the set 2 south hall (1.6 m between pillars), both below the layout rules. Since 2026-11-19 the PASS verdict counts it as FAIL.
- **The 90 s tail.** Many sets end not with 0 frontiers but with "no reachable goal for 90 s" (frontiers within 0.4 m of obstacles remain) → up to 90 s more per set.
- **A1-to-A1 contact.** In the 7 full missions of the current setup the minimum center distance between the A1s was 0.28-0.61 m, and a standing A1 was confirmed to be pushed and rotated by the other (mostly in the common building). Cause: the lidar plane is above the body so only the other robot's lidar puck (3 cm radius) is seen, the global map removes the other robot on purpose, and goal assignment only spreads goals. Not fixed yet (`docs/experiments/A1_근접중복_분석.txt`, Korean).

## Person detection

- **Camera field of view.** A person is found only once inside the camera's field of view (about ±52°). If the body faces elsewhere while passing or stopping, a person can be missed at close range (confirmed outside the view at 0.63 m in a G2 run). Exploration ends when the (lidar) map is complete; what the camera has seen is not tracked.
- Extra reports with off positions can occur (11-13 reports for 11 people in the 7 full missions of the current setup). Most have a range error of 1-3 m, mostly at 4.5 m or farther. They also appear in `GetSurvivors` and `RunMission` results.
- Holding reports while disconnected and sending them later was confirmed only once (one report) in the simulator.
- Comparison with the configured person count is by count, so extra reports can mark "all found" early (set completion is decided only by both A1s returning).

## Scope

- Tested in simulation only. What a real deployment needs is listed in {doc}`../service/deployment`.
- Sets 1-3 and the common building are easier than the original map (corridors at least 2 m, ramps 8° humps without side edges); set 4 is the one place that breaks the rules. This mission structure was not run on the original `disaster.world`.
- The map merge assumes known spawn poses (it is not map registration).
- Walking on slopes was not measured separately. The 8° humps were crossed many times in mission runs without falls.
- `odom_scaler` was not re-measured after the change to 1.125.

## Operations

- Sometimes Nav2 doesn't come up (about 5 in 30 starts; 3 of 5 after S3; it also happened before S3, cause not checked). `run_mission.sh` restarts only Nav2 up to twice, then the whole stack once (startup test: 10 of 10 came up on the first try).
- Runs with the GUI on, and typing directly into the `husky_mission` terminal, were not tested (mission requests were tested only via the `RunMission` action, comm 0/1 only via the `SetComm` service — 2026-11-27 C2 PASS; sending held reports was seen once on 2026-11-28).

## Manhattan-world assumptions

A few simplifications assume orthogonal, axis-aligned walls. They fit this map but could cause problems on general maps.

| Where | Assumption | Breaks when | Way to generalize |
| --- | --- | --- | --- |
| goal_assigner BFS | grid step distance | overestimates diagonals, favors axis directions | 8-way BFS or Nav2 path length |
| zones | axis-aligned rectangles | buildings at an angle | polygon zones or path-distance partitioning |
| map merge | known spawn poses | start positions change | map registration (scan matching) |
| turn first | map and odom share orientation | large SLAM rotation corrections | compute from the map → base TF |
