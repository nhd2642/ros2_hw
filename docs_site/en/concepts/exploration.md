# Exploration: SLAM to Nav2

In EXPLORE each A1 visits **frontiers** in its zone (the edge between seen free cells and unseen cells) one by one to fill in the map. This page follows the whole path from scan to map, map to goal, and goal to footsteps.

```{graphviz} ../../diagrams/exploration_pipeline.dot
```

## 1. Scan preprocessing — scan_ready_relay

Before SLAM, scans get two fixes (`/<ns>/scan` → `/<ns>/scan_ready`).

- **Beams that hit the floor while the body is tilted are removed.** On a ramp the tilted lidar (0.28 m high) hits the floor and creates false walls in the map. IMU roll/pitch decides per beam whether it reaches the floor.
- **Beams that hit other robots are removed.** The moving Husky and the other A1 left trails of points in the map that blocked paths in the global costmap and pulled scan matching (in set 1 the `a1_2` map smeared and its position was off by 2.3 m). The bearing range each robot occupies is computed from the positions received over comm, and those beams are erased.

The local costmap still uses the raw scan, so nearby robots are still avoided.

## 2. Per-robot SLAM — slam_toolbox

| Setting | Value | Why |
| --- | --- | --- |
| mode | asynchronous mapping, Ceres (Levenberg-Marquardt) pose graph | default |
| resolution | 0.05 m | |
| map update | 5 s | |
| travel before adding a scan | 0.2 m or 0.5 rad | in Humble, turning in place alone does not grow the map (verified) |
| loop closure | **off** | suspected cause of 2.3 m position jumps in tight rooms |
| input | `/<ns>/scan_ready` | preprocessing above |

## 3. Map merge — nav_goal_bridge

The two robots' maps are shifted **by their known spawn poses** and merged into one shared map (`/shared_map`, world aligned). It is not scan matching between maps.

- Each robot's SLAM pose is converted to world coordinates the same way and published on `/<ns>/pose_shared` (10 Hz, used by following, scan preprocessing and evaluation).
- On set arrival, a `/<ns>/pose_fix` shifts that robot's alignment by that amount ({doc}`transit_following`).
- In the other direction, goals in shared-map coordinates (`/<ns>/exploration_goal`) are converted to that robot's map coordinates (`/<ns>/nav_goal_local`) for the executor. Home (`/<ns>/home_shared` → `home_local`) works the same way.
- The alignment rotation is 0: each map starts in the IMU ENU orientation, already the world orientation. (Trusting the spawn yaw at first made `a1_2` walk 11° askew, because the body rotated 10° while standing up.)

## 4. Frontier extraction — frontier_detector

- A free cell (value 0-20) with an unknown (−1) 4-neighbor is a frontier.
- Frontier cells are grouped with 8-connectivity; groups under 5 cells are dropped. The largest groups go out first on `/frontier_candidates`.
- It searches **only inside the zone**. At first the top 15 were all on the road side and there were no candidates inside the buildings.

## 5. Zone and failure filter — frontier_comm_filter

`/frontier_candidates` → `/frontier_candidates_comm`

- Keeps only frontiers **0.3 m inside** the zone rectangle from `/mission/state` (removing road frontiers outside the door). Outside exploration phases it removes everything.
- Removes anything within 0.9 m of `/exploration_blacklist` (failed goals) for 120 s.

## 6. Goal assignment — goal_assigner (every 3 s)

1. **Walkable cells**: value 0-20, excluding cells within the clearance of obstacles (value 50+).
2. **Path distance**: BFS steps from the robot cell × 0.05 m.
3. **Candidate score**: among safe cells near the frontier, the one with the smallest `score = path distance + 1.2 × distance to frontier`. Goal cells must be at least 0.4 m from obstacles.
4. **Two-robot assignment**: the pair of different frontiers with the smallest total score (exhaustive). Two goals closer than 3 m are penalized (so they split up in the common building).
5. Within 0.9 m of a robot's own blacklist (`/<ns>/exploration_blacklist`), goals are withheld from that robot only, for 60 s.
6. For each robot, the number of "reachable frontiers in its zone" is published on `/<ns>/frontier_options` → done detection.

## 7. Executor — nav_executor

The teammate Nav2 setup has only `planner_server`/`controller_server` (no `bt_navigator`, no recoveries). So this node does **plan → follow → failure handling** itself.

```{graphviz} ../../diagrams/nav_executor_states.dot
```

### Basic flow

`nav_goal_local` → `ComputePathToPose` → (turn in place if needed) → `FollowPath` → arrived

### Goal management

- The same goal within 0.5 m is ignored.
- A different goal replaces the current one only after following it **at least 8 s**.
- If frontiers remain within 1.0 m of the current goal, it doesn't switch. (Previously the goal changed every 3 s: 15 of 17 attempts were "replaced" and only one arrived.)

### Turn first (TURNING)

If the path ahead (0.5 m) differs from the body heading by more than 45°, it turns in place at 0.5 rad/s before `FollowPath` and stops within 20°. In the DWB era the controller picked **stopping** over turning when the path was behind the robot and stood still forever; this was kept after the switch to MPPI. Nav2's RotationShimController gives up turning when TF is late (A1: about 0.2-0.3 s), so it isn't used. Above 6° of tilt (on a ramp) there is no turn-first — an A1 once slid 0.5 m while turning in place on a 10° ramp and fell.

### Failure handling

| Mechanism | When | Does |
| --- | --- | --- |
| blacklist | plan failure, follow failure (no progress), 90 s timeout | withhold that goal area from that robot; retry after 60 s |
| same-spot failure limit | 3 failures at the same place (1 m) | give up that place for the set |
| same-spot arrival limit | 3 arrivals at the same place | a frontier that never clears — stop for the set |
| escape (ESCAPING) | follow failure + lethal cells within 0.1 m of the robot center | move away from lethal cells at 0.08 m/s, up to 10 s (forward/back axis only) |
| nudge | 3 consecutive plan failures at the same spot | move 0.3 m (up to 0.6 m) toward the most open direction → SLAM adds a scan and re-aligns |
| path zone check | path enters the other robot's zone by more than 0.15 m | don't follow; blacklist (not checked in the common building) |
| fall | IMU tilt > 45° | FALLEN: cancel the goal, no blacklisting. Back after standing below 20° for 3 s |

:::{note}
Why nudging is needed: `a1_2`, standing 0.27 m from a wall, once failed 61 plans in 534 s, each within 0.01 s. In the SLAM map the robot was on the wall line, but slam_toolbox only adds scans after moving more than 0.2 m, so the misalignment could never fix itself while standing — a deadlock.
:::

### Done and return home

Any of these means exploration is done; it then plans and follows to this set's home with Nav2 (retry up to 8 times, 300 s limit).

- `frontier_options` is **0 for 30 s** and the executor is IDLE.
- While exploring it stays IDLE **without starting any follow for 90 s** (all remaining frontiers are blacklisted or fail the zone check). This timer resets every set.
- The phase becomes REGROUP.

On arrival it goes `DONE` and publishes `/<ns>/set_done = k` (latched).

## 8. Nav2 settings

| Item | Value |
| --- | --- |
| planner | NavFn, **Dijkstra** (`use_astar: false`), tolerance 0.25 m, unknown cells allowed |
| global costmap | SLAM map (static) + terrain layer + inflation **0.70 m** |
| local costmap | 3 × 3 m rolling window, 5 Hz, lidar obstacle layer + terrain layer + inflation 0.35 m |
| footprint | 0.60 × 0.32 m rectangle (it used to be a 0.23 m radius circle → could not pass the 0.5 m gap beside the Husky) |
| controller | MPPI (`nav2_mppi_controller`, DiffDrive): rolls out 1000 noisy (vx, wz) trajectories 4 s ahead (40 × 0.1 s) and takes their cost-weighted average (footprint collision, path following, goal). No reverse, no sideways steps. Replaced DWB on 2026-11-19 ({doc}`../results/test_results`) |
| speed | max 0.2 m/s, 0.5 rad/s |
| goal | xy 0.20 m, any heading |
| progress check | fail if less than 0.3 m in 10 s |

:::{tip}
Why A* is off: in A* mode NavFn produced zero-point paths with "Failed to create a plan from potential", the return-home plan failed and an A1 was left inside a building. Since switching to Dijkstra that error appears 0-1 times per run.
:::
