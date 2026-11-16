# 노드

`<ns>` 는 `a1_1` 또는 `a1_2` 입니다. 로봇별 노드도 **프로세스는 하나**이고 안에서 두 로봇을 같이 다룹니다(`robots` 파라미터).

## husky_mission

임무 단계, 사용자 명령, 구역·홈 지정, 통신 하트비트, 사람 집계. 터미널 입력(`start`/`0`/`1`)도 받습니다.

| 방향 | 토픽 | 형식 |
| --- | --- | --- |
| 구독 | `/husky/cmd` | `std_msgs/String` |
| 구독 | `/husky/driver_state`, `/husky/pose` | `String`, `PoseStamped` |
| 구독 | `/husky/survivor_report` | `String` (JSON) |
| 구독 | `/<ns>/pose_shared`, `/<ns>/pose_fix`, `/<ns>/set_done` | `PoseStamped`, `PointStamped`, `Int32` |
| 발행 | `/mission/state` | `String` (JSON, 1 Hz) |
| 발행 | `/husky/goal`, `/husky/go` | `PoseStamped`, `Bool` |
| 발행 | `/<ns>/comm_status` | `Bool` (하트비트) |
| 발행 | `/<ns>/home_shared` | `PoseStamped` |
| 발행 | `/husky/survivors` | `String` (JSON, latched) |

| 파라미터 | 기본 |
| --- | --- |
| `config` | `''` (mission.yaml 경로, 필수) |
| `start_set` | 0 |
| `explore_timeout` | 1500.0 |
| `regroup_timeout` | 300.0 |
| `arrive_tol` | 0.6 |
| `arrive_wait` | 60.0 |
| `lag_stop` / `lag_go` | 2.5 / 1.5 |

## husky_driver

목표점 주행 + 2D 라이다 회피, 위치 추정(바퀴 속도 × IMU yaw).

| 방향 | 토픽 |
| --- | --- |
| 구독 | `/husky/goal`, `/husky/go`, `/husky/odom`, `/husky/imu`, `/husky/scan` |
| 발행 | `/husky/cmd_vel`, `/husky/pose` (world), `/husky/driver_state` (`IDLE` `GO` `AVOID` `BLOCKED` `HOLD` `ARRIVED`, latched) |

| 파라미터 | 기본 |
| --- | --- |
| `spawn` | [0.0, 0.0, 0.0] |
| `lidar_x` | 0.30 |
| `v_max` | 0.25 |

## a1_follower

TRANSIT 때 Husky 뒤 자리 추종 + 라이다 회피, 이동 끝에 Husky 기준 자세 보정.

| 방향 | 토픽 |
| --- | --- |
| 구독 | `/mission/state`, `/husky/pose`, `/<ns>/pose_shared`, `/<ns>/scan` |
| 발행 | `/<ns>/cmd_vel_follow`, `/<ns>/slot_error` (`Float32`), `/<ns>/pose_fix` (`PointStamped`, world) |

| 파라미터 | 기본 |
| --- | --- |
| `robots` | ['a1_1', 'a1_2'] |
| `v_max` | 0.35 |
| `slot.<ns>` | mission.yaml 의 `slots` (launch 가 넣음) |

## nav_executor

EXPLORE·REGROUP 때 Nav2 계획·추종, 실패 처리, 완료 판정, 홈 복귀.

| 방향 | 토픽 |
| --- | --- |
| 구독 | `/mission/state`, `/frontier_candidates_comm`, `/<ns>/nav_goal_local`, `/<ns>/home_local`, `/<ns>/frontier_options`, `/<ns>/comm_mode`, `/<ns>/return_path`, `/<ns>/imu/data`, `/<ns>/odom`, `/<ns>/local_costmap/costmap` |
| 발행 | `/<ns>/cmd_vel_nav_test` (회전·벗어나기), `/<ns>/nav_executor/state`, `/<ns>/nav_executor/events`, `/<ns>/nav_executor/path`, `/<ns>/set_done`, `/<ns>/exploration_blacklist`, `/exploration_blacklist` |
| 액션 | `/<ns>/compute_path_to_pose`, `/<ns>/follow_path` (클라이언트) |

| 파라미터 | 기본 | 파라미터 | 기본 |
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

통신 끊김 → breadcrumb 되짚어 복귀.

| 방향 | 토픽 |
| --- | --- |
| 구독 | `/mission/state`, `/<ns>/comm_status`, `/<ns>/odom`, `/<ns>/nav_executor/state`, `/<ns>/scan` |
| 발행 | `/<ns>/cmd_vel_return`, `/<ns>/comm_mode` (transient local), `/<ns>/return_events`, `/<ns>/return_path` |

| 파라미터 | 기본 | 파라미터 | 기본 |
| --- | --- | --- | --- |
| `comm_timeout` | 1.0 (벽시계) | `odom_timeout` | 0.5 |
| `spacing` | 0.10 | `reach_tol` | 0.25 |
| `max_breadcrumbs` | 5000 | `vx` | 0.15 |
| `wz_turn` / `wz_max` | 0.4 / 0.4 | `turn_threshold` | 0.35 |
| `stop_duration` | 1.0 | `jump_threshold` | 0.5 |
| `return_drive` | `direct` | `nav_timeout` | 2.0 |
| `obstacle_check` | True | `half_width` / `look_ahead` | 0.15 / 0.5 |
| `min_points` | 3 | `output_topic` | `cmd_vel_return` |

## cmd_vel_arbiter

| 방향 | 토픽 |
| --- | --- |
| 구독 | `/<ns>/cmd_vel_teleop`, `_hold`, `_return`, `_nav_test`, `_follow` |
| 발행 | `/<ns>/cmd_vel` (20 Hz), `/<ns>/cmd_vel_source` |

| 파라미터 | 기본 |
| --- | --- |
| `source_names` | ['teleop', 'hold', 'return', 'nav', 'follow'] |
| `source_priorities` | [4, 3, 2, 1, 0] |
| `timeout` | 0.5 |
| `rate` | 20.0 |
| `output_topic` | `cmd_vel` |

## frontier_comm_filter

| 방향 | 토픽 |
| --- | --- |
| 구독 | `/mission/state`, `/frontier_candidates`, `/exploration_blacklist` |
| 발행 | `/frontier_candidates_comm` |

| 파라미터 | 기본 |
| --- | --- |
| `margin` | 0.3 |
| `blacklist_radius` | 0.9 |
| `blacklist_ttl` | 120.0 |

## survivor_detector

| 방향 | 토픽 |
| --- | --- |
| 구독 | `/<ns>/front_camera/image_raw`, `/<ns>/front_camera/camera_info`, `/<ns>/scan`, `/<ns>/tf`, `/<ns>/tf_static`, `/<ns>/comm_status` |
| 발행 | `/<ns>/survivor_markers` (latched), `/<ns>/survivor`, `/<ns>/survivor_events`, `/<ns>/person_score`, `/<ns>/cmd_vel_hold`, `/husky/survivor_report` |

| 파라미터 | 기본 |
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

`/clock` → `/clock_slow`. 파라미터 `rate` = 50.0.

## a1_exploration 스크립트

| 노드 | 주요 입출력 | 파라미터 |
| --- | --- | --- |
| `nav_goal_bridge` | 입 `/<ns>/map`, `/<ns>/tf`, `/<ns>/exploration_goal`, `/<ns>/home_shared`, `/<ns>/pose_fix` / 출 `/shared_map`, `/<ns>/pose_shared` (10 Hz), `/<ns>/nav_goal_local`, `/<ns>/home_local`, `/robot_position_markers` | `spawn.<ns>` (launch 가 넣음) |
| `frontier_detector` | 입 `/shared_map`, `/mission/state` / 출 `/frontier_candidates`, `/frontier_markers` | |
| `goal_assigner` | 입 `/shared_map`, `/frontier_candidates`(→ `_comm` 으로 리맵), `/mission/state`, `/<ns>/pose_shared`, `/<ns>/exploration_blacklist` / 출 `/<ns>/exploration_goal`, `/<ns>/frontier_options`, `/<ns>/candidate_path`, `/assigned_goal_markers` | `obstacle_clearance` 0.15, `robot_blacklist_radius` 0.9, `robot_blacklist_ttl` 60.0, `zone_goal_margin` 0.3, `goal_spread` 3.0, `goal_clearance` 0.40, `min_reach_area` 3.0 |
| `scan_ready_relay` | 입 `/<ns>/scan`, `/<ns>/imu/data`, `/husky/pose`, `/<상대>/pose_shared` / 출 `/<ns>/scan_ready` | `lidar_height` 0.28 |

## terrain_node

| 방향 | 토픽 |
| --- | --- |
| 구독 | `/<ns>/terrain_camera/points`, `/<ns>/odom`, `/<ns>/imu/data`, `/<ns>/tf`, `/<ns>/tf_static`, `/<ns>/scan_ready`, `/mission/state`, `/husky/pose`, `/<상대>/pose_shared` |
| 발행 | `/<ns>/terrain_hazards`, `/<ns>/terrain_free` |

| 파라미터 | 기본 | 파라미터 | 기본 |
| --- | --- | --- | --- |
| `cell` | 0.05 | `max_step` | 0.04 |
| `max_slope_deg` | 25.0 | `max_range` | 2.0 |
| `radius` | 3.0 | `forget` | 60.0 |
| `rate` | 5.0 | `self_radius` | 0.35 |
| `lidar_clear` | 0.25 | `other_radius` | 0.8 |
