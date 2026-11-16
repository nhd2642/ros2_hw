# Nodes

`<ns>` is `a1_1` or `a1_2`. Per-robot nodes are still **one process** that handles both robots inside (the `robots` parameter).

## husky_mission

Mission phases, user commands, zone and home assignment, comm heartbeat, people tally. Also reads terminal input (`start`/`0`/`1`).

| Direction | Topic | Type |
| --- | --- | --- |
| sub | `/husky/cmd` | `std_msgs/String` |
| sub | `/husky/driver_state`, `/husky/pose` | `String`, `PoseStamped` |
| sub | `/husky/survivor_report` | `String` (JSON) |
| sub | `/<ns>/pose_shared`, `/<ns>/pose_fix`, `/<ns>/set_done` | `PoseStamped`, `PointStamped`, `Int32` |
| pub | `/mission/state` | `String` (JSON, 1 Hz) |
| pub | `/husky/goal`, `/husky/go` | `PoseStamped`, `Bool` |
| pub | `/<ns>/comm_status` | `Bool` (heartbeat) |
| pub | `/<ns>/home_shared` | `PoseStamped` |
| pub | `/husky/survivors` | `String` (JSON, latched) |

| Parameter | Default |
| --- | --- |
| `config` | `''` (path to mission.yaml, required) |
| `start_set` | 0 |
| `explore_timeout` | 1500.0 |
| `regroup_timeout` | 300.0 |
| `arrive_tol` | 0.6 |
| `arrive_wait` | 60.0 |
| `lag_stop` / `lag_go` | 2.5 / 1.5 |

## husky_driver

Drives to a goal point with 2D lidar avoidance; pose estimation (wheel speed × IMU yaw).

| Direction | Topic |
| --- | --- |
| sub | `/husky/goal`, `/husky/go`, `/husky/odom`, `/husky/imu`, `/husky/scan` |
| pub | `/husky/cmd_vel`, `/husky/pose` (world), `/husky/driver_state` (`IDLE` `GO` `AVOID` `BLOCKED` `HOLD` `ARRIVED`, latched) |

| Parameter | Default |
| --- | --- |
| `spawn` | [0.0, 0.0, 0.0] |
| `lidar_x` | 0.30 |
| `v_max` | 0.25 |

## a1_follower

Follows the slot behind the Husky during TRANSIT with lidar avoidance; Husky-referenced pose correction at the end of transit.

| Direction | Topic |
| --- | --- |
| sub | `/mission/state`, `/husky/pose`, `/<ns>/pose_shared`, `/<ns>/scan` |
| pub | `/<ns>/cmd_vel_follow`, `/<ns>/slot_error` (`Float32`), `/<ns>/pose_fix` (`PointStamped`, world) |

| Parameter | Default |
| --- | --- |
| `robots` | ['a1_1', 'a1_2'] |
| `v_max` | 0.35 |
| `slot.<ns>` | `slots` from mission.yaml (set by launch) |

## nav_executor

Nav2 planning and following in EXPLORE/REGROUP, failure handling, done detection, return home.

| Direction | Topic |
| --- | --- |
| sub | `/mission/state`, `/frontier_candidates_comm`, `/<ns>/nav_goal_local`, `/<ns>/home_local`, `/<ns>/frontier_options`, `/<ns>/comm_mode`, `/<ns>/return_path`, `/<ns>/imu/data`, `/<ns>/odom`, `/<ns>/local_costmap/costmap` |
| pub | `/<ns>/cmd_vel_nav_test` (turns, escapes), `/<ns>/nav_executor/state`, `/<ns>/nav_executor/events`, `/<ns>/nav_executor/path`, `/<ns>/set_done`, `/<ns>/exploration_blacklist`, `/exploration_blacklist` |
| action | `/<ns>/compute_path_to_pose`, `/<ns>/follow_path` (client) |

| Parameter | Default | Parameter | Default |
| --- | --- | --- | --- |
| `same_goal_tol` | 0.5 | `min_dwell` | 8.0 |
| `goal_timeout` | 90.0 | `goal_margin` | 0.2 |
| `planner_id` | GridBased | `controller_id` | FollowPath |
| `turn_first` | 0.785 | `turn_done` | 0.35 |
| `turn_speed` | 0.5 | `turn_max_tilt` | 6.0 |
| `commit` | True | `commit_radius` | 1.0 |
| `escape_radius` | 0.1 | `escape_speed` | 0.08 |
| `escape_time` | 10.0 | `nudge_after` | 3 |
| `nudge_dist` | 0.3 | `nudge_time` | 20.0 |
| `zero_cmd_time` | 8.0 | `done_after` | 30.0 |
| `home_timeout` | 300.0 | `home_retries` | 8 |
| `path_zone_margin` | 0.15 | `fall_tilt` | 45.0 |
| `tilt_guard` | 25.0 | `closed_radius` | 0.4 |
| `idle_done` | 90.0 | `fail_area_radius` | 1.0 |
| `fail_area_n` | 3 | `fail_area_ttl` | 900.0 |
| `return_retries` | 2 | `goal_refine_radius` | 0.5 |

## return_relay

Comm loss → retrace breadcrumbs home.

| Direction | Topic |
| --- | --- |
| sub | `/mission/state`, `/<ns>/comm_status`, `/<ns>/odom`, `/<ns>/nav_executor/state`, `/<ns>/scan` |
| pub | `/<ns>/cmd_vel_return`, `/<ns>/comm_mode` (transient local), `/<ns>/return_events`, `/<ns>/return_path` |

| Parameter | Default | Parameter | Default |
| --- | --- | --- | --- |
| `comm_timeout` | 1.0 (wall clock) | `odom_timeout` | 0.5 |
| `spacing` | 0.10 | `reach_tol` | 0.25 |
| `max_breadcrumbs` | 5000 | `vx` | 0.15 |
| `wz_turn` / `wz_max` | 0.4 / 0.4 | `turn_threshold` | 0.35 |
| `stop_duration` | 1.0 | `jump_threshold` | 0.5 |
| `return_drive` | `direct` | `nav_timeout` | 2.0 |
| `obstacle_check` | True | `half_width` / `look_ahead` | 0.15 / 0.5 |
| `min_points` | 3 | `output_topic` | `cmd_vel_return` |

## cmd_vel_arbiter

| Direction | Topic |
| --- | --- |
| sub | `/<ns>/cmd_vel_teleop`, `_hold`, `_return`, `_nav_test`, `_follow` |
| pub | `/<ns>/cmd_vel` (20 Hz), `/<ns>/cmd_vel_source` |

| Parameter | Default |
| --- | --- |
| `source_names` | ['teleop', 'hold', 'return', 'nav', 'follow'] |
| `source_priorities` | [4, 3, 2, 1, 0] |
| `timeout` | 0.5 |
| `rate` | 20.0 |
| `output_topic` | `cmd_vel` |

## frontier_comm_filter

| Direction | Topic |
| --- | --- |
| sub | `/mission/state`, `/frontier_candidates`, `/exploration_blacklist` |
| pub | `/frontier_candidates_comm` |

| Parameter | Default |
| --- | --- |
| `margin` | 0.3 |
| `blacklist_radius` | 0.9 |
| `blacklist_ttl` | 120.0 |

## survivor_detector

| Direction | Topic |
| --- | --- |
| sub | `/<ns>/front_camera/image_raw`, `/<ns>/front_camera/camera_info`, `/<ns>/scan`, `/<ns>/tf`, `/<ns>/tf_static`, `/<ns>/comm_status` |
| pub | `/<ns>/survivor_markers` (latched), `/<ns>/survivor`, `/<ns>/survivor_events`, `/<ns>/person_score`, `/<ns>/cmd_vel_hold`, `/husky/survivor_report` |

| Parameter | Default |
| --- | --- |
| `model_dir` | `<ws>/person_ai_models/MobileNet-SSD` |
| `conf` | 0.5 |
| `period` | 1.0 |
| `hfov` | 1.8 |
| `cam_dx` | 0.29 |
| `max_range` | 7.0 |
| `merge_dist` | 1.0 |
| `hold` | 1.0 |
| `comm_timeout` | 1.0 |

## clock_throttle

`/clock` → `/clock_slow`. Parameter `rate` = 50.0.

## a1_exploration scripts

| Node | Main I/O | Parameters |
| --- | --- | --- |
| `nav_goal_bridge` | in `/<ns>/map`, `/<ns>/tf`, `/<ns>/exploration_goal`, `/<ns>/home_shared`, `/<ns>/pose_fix` / out `/shared_map`, `/<ns>/pose_shared` (10 Hz), `/<ns>/nav_goal_local`, `/<ns>/home_local`, `/robot_position_markers` | `spawn.<ns>` (set by launch) |
| `frontier_detector` | in `/shared_map`, `/mission/state` / out `/frontier_candidates`, `/frontier_markers` | |
| `goal_assigner` | in `/shared_map`, `/frontier_candidates` (remapped to `_comm`), `/mission/state`, `/<ns>/pose_shared`, `/<ns>/exploration_blacklist` / out `/<ns>/exploration_goal`, `/<ns>/frontier_options`, `/<ns>/candidate_path`, `/assigned_goal_markers` | `obstacle_clearance` 0.15, `robot_blacklist_radius` 0.9, `robot_blacklist_ttl` 60.0, `zone_goal_margin` 0.3, `goal_spread` 3.0, `goal_clearance` 0.40, `min_reach_area` 3.0 |
| `scan_ready_relay` | in `/<ns>/scan`, `/<ns>/imu/data`, `/husky/pose`, `/<other>/pose_shared` / out `/<ns>/scan_ready` | `lidar_height` 0.28 |

## terrain_node

| Direction | Topic |
| --- | --- |
| sub | `/<ns>/terrain_camera/points`, `/<ns>/odom`, `/<ns>/imu/data`, `/<ns>/tf`, `/<ns>/tf_static`, `/<ns>/scan_ready`, `/mission/state`, `/husky/pose`, `/<other>/pose_shared` |
| pub | `/<ns>/terrain_hazards`, `/<ns>/terrain_free` |

| Parameter | Default | Parameter | Default |
| --- | --- | --- | --- |
| `cell` | 0.05 | `max_step` | 0.04 |
| `max_slope_deg` | 25.0 | `max_range` | 2.0 |
| `radius` | 3.0 | `forget` | 60.0 |
| `rate` | 5.0 | `self_radius` | 0.35 |
| `lidar_clear` | 0.25 | `other_radius` | 0.8 |
