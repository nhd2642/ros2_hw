# Known Limitations

The limitations known today, written down plainly. Use this as a starting point when choosing new work.

## Reproducibility

:::{warning}
**Only one run has passed the full mission with the current code** (three counting earlier code). Results varied between runs, and the same set took 180-810 s to explore. Not enough runs were made to say how often it passes.
:::

## Position

- **SLAM drift within a set.** An A1 map can drift close to 1 m even within one set (latest run `a1_1`: up to 0.97 m in set 1, 0.99 m in the common building). It is re-aligned to the Husky on every set arrival, but drift that builds up inside a set stays, so zone rectangles and reported person positions can be off by that much.
- **HOLD position offset.** The spot where a robot stops after a comm loss is odom based and can be away from the explore start (latest run 0.85 m).
- **Drift while walking in place.** When an A1 is blocked and walks in place, leg odom keeps advancing and the SLAM position drifts. The cause is not fixed; it is avoided by removing trap features from the layout and re-aligning to the Husky every set.

## Following & exploration

- **Turning entrances.** DWB following still sometimes fails with "no progress" on paths around a wall end (a run gave up a room after 3 failures even at a 2.4 m entrance). After 3 failures at the same place the set gives it up, so the mission continues but people in that room are missed.
- **Return failure.** If an A1 can't get back after 8 attempts, it reports the set done where it is. The Husky then waits for it in the next transit. It happened in 3 full missions, in the common building center room (1.6 m east doorway, a person in the middle) and the set 2 south hall (1.6 m between pillars), both below the layout rules. Since 2026-11-19 the PASS verdict counts it as FAIL.
- **The 90 s tail.** Many sets end not with 0 frontiers but with "no reachable goal for 90 s" (frontiers within 0.4 m of obstacles remain) → up to 90 s more per set.
- **Clearance between robots.** A1 centers came as close as 0.46 m (body length 0.6 m). Contact was not measured.

## Person detection

- Extra reports with off positions or low scores can occur (latest run 0; the two earlier passing runs 2 each).
- Comparison with the configured person count is by count, so extra reports can mark "all found" early (set completion is decided only by both A1s returning).

## Scope

- The layout is easier than the original map (corridors at least 2 m, ramps 8° humps without side edges). This mission structure was not run on the original `disaster.world`.
- The map merge assumes known spawn poses (it is not map registration).
- Walking on slopes was not measured separately. The 8° humps were crossed many times in mission runs without falls.
- `odom_scaler` was not re-measured after the change to 1.125.

## Operations

- Sometimes Nav2 doesn't come up because it gets TF late (about 5 in 30 starts). `run_mission.sh` restarts once.
- Runs with the GUI on, and typing directly into the `husky_mission` terminal, were not tested (commands were tested only via the `/husky/cmd` topic).

## Manhattan-world assumptions

A few simplifications assume orthogonal, axis-aligned walls. They fit this map but could cause problems on general maps.

| Where | Assumption | Breaks when | Way to generalize |
| --- | --- | --- | --- |
| goal_assigner BFS | grid step distance | overestimates diagonals, favors axis directions | 8-way BFS or Nav2 path length |
| zones | axis-aligned rectangles | buildings at an angle | polygon zones or path-distance partitioning |
| map merge | known spawn poses | start positions change | map registration (scan matching) |
| turn first | map and odom share orientation | large SLAM rotation corrections | compute from the map → base TF |
