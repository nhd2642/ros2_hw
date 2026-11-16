# Tune Parameters

This page collects **where the commonly changed values live** and how to change them. The full parameter list is in {doc}`../reference/nodes`.

## Three ways to change a value

1. **Pass it in the launch file** — add a keyword to the `node(...)` call in `mission.launch.py` (e.g. `node('nav_executor', idle_done=120.0)`). No rebuild needed (`--symlink-install`).
2. **Change it at runtime** — `ros2 param set /nav_executor done_after 45.0`. Most nodes read parameters only at startup, though, so you need a restart for it to apply.
3. **Config files** — SLAM (`a1_exploration/config/slam_params.yaml`), Nav2 (dicts inside `a1_exploration/launch/nav2_*.launch.py`), gait (`a1_champ/config/gait.yaml`).

:::{warning}
`a1_exploration/launch/nav2_*.launch.py` contain **patch blocks** inserted by scripts such as `terrain_setup.sh`. Don't overwrite those files wholesale; change only the lines you need.
:::

## Mission (husky_mission)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `explore_timeout` | 1500 s | exploring longer than this triggers REGROUP |
| `regroup_timeout` | 300 s | after waiting this long in REGROUP, move to the next set |
| `arrive_tol` | 0.6 m | an A1 within this of its slot counts as arrived |
| `arrive_wait` | 60 s | after the Husky arrives, wait at most this long for the A1s |
| `lag_stop` / `lag_go` | 2.5 / 1.5 m | Husky stops when an A1 lags this much / starts again when it is back within this |

## Speeds

| What | Where | Value |
| --- | --- | --- |
| Husky drive max | `husky_driver` `v_max` | 0.25 m/s |
| A1 follow max | `a1_follower` `v_max` | 0.35 m/s |
| A1 exploration (Nav2 DWB) | `nav2_controllers.launch.py` `max_vel_x` | 0.2 m/s |
| Direct return drive | `return_relay` `vx` | 0.15 m/s |
| CHAMP limit | `gait.yaml` `max_linear_velocity_x` | 0.5 m/s |

Before raising a speed, check its response and tilt with {doc}`measure_gait`. DWB speeds above 0.2 m/s were not tested.

## Exploration executor (nav_executor)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `done_after` | 30 s | 0 reachable frontiers for this long → exploration done → home |
| `idle_done` | 90 s | done if no follow could start for this long during exploration |
| `goal_timeout` | 90 s | maximum time following one goal |
| `min_dwell` | 8 s | minimum time on a goal before switching |
| `commit_radius` | 1.0 m | keep the current goal while frontiers remain within this |
| `turn_first` / `turn_done` | 0.785 / 0.35 rad | turn in place first if the path is more than 45° off, stop within 20° |
| `turn_max_tilt` | 6° | no turning in place above this tilt (avoids sliding on ramps) |
| `fall_tilt` | 45° | tilt above this means fallen (FALLEN) |
| `home_retries` / `home_timeout` | 8 / 300 s | return-home retries / time limit |
| `fail_area_n` | 3 | give up a spot after this many failures there per set |
| `nudge_after` / `nudge_dist` | 3 / 0.3 m | plan failures at the same spot / nudge distance |

## Goal assignment (goal_assigner)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `goal_spread` | 3.0 m | penalty when the two robots' goals are closer than this (spreads them in the common building) |
| `goal_clearance` | 0.40 m | goal cells must be at least this far from obstacles |
| `robot_blacklist_radius` / `_ttl` | 0.9 m / 60 s | don't give that robot goals within this of a failed goal, for this long |
| `zone_goal_margin` | 0.3 m | margin from the zone edge |

## Person detection (survivor_detector)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `conf` | 0.5 | person score threshold. Real people scored 0.89-1.00, a false positive (next to a pillar) 0.41 |
| `period` | 1.0 s | inference interval per robot |
| `model_dir` | `<ws>/person_ai_models/MobileNet-SSD` | model file folder |

## Nav2 costmaps

| Value | Where | Current |
| --- | --- | --- |
| Global inflation radius | `nav2_planners.launch.py` `inflation_radius` | 0.70 m (0.35 → 0.70 in fix18) |
| Local inflation radius | `nav2_controllers.launch.py` `inflation_radius` | 0.35 m |
| footprint | both files | 0.60 × 0.32 m rectangle |
| planner | `nav2_planners.launch.py` | NavFn Dijkstra (`use_astar: False`), tolerance 0.25 m |
| goal check | `nav2_controllers.launch.py` | xy 0.20 m, any heading (yaw 3.14) |

:::{tip}
When you change one value, run the same setting **at least twice** and compare. Even with identical code, set exploration time varies from 180 to 810 s.
:::
