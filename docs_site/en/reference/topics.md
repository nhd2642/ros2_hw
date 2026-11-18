# Topics

`<ns>` = `a1_1` / `a1_2`. Frames are as in the message header.

## Mission & Husky

| Topic | Type | Publisher | Subscribers | Notes |
| --- | --- | --- | --- | --- |
| `/husky/cmd` | `String` | user | husky_mission | `start` / `0` / `1` |
| `/mission/state` | `String` (JSON) | husky_mission | a1_follower, nav_executor, return_relay, frontier_comm_filter, frontier_detector, goal_assigner, terrain_node | 1 Hz |
| `/husky/goal` | `PoseStamped` (world) | husky_mission | husky_driver | parking spot |
| `/husky/go` | `Bool` | husky_mission | husky_driver | False = hold |
| `/husky/pose` | `PoseStamped` (world) | husky_driver | husky_mission, a1_follower, scan_ready_relay, terrain_node | the coordinate reference |
| `/husky/driver_state` | `String` | husky_driver | husky_mission | latched |
| `/husky/cmd_vel` | `Twist` | husky_driver | Gazebo (bridge) | |
| `/husky/odom`, `/husky/imu`, `/husky/scan` | | Gazebo (bridge) | husky_driver | |
| `/husky/survivor_report` | `String` (JSON) | survivor_detector | husky_mission | robot, id, x, y, score, delay |
| `/husky/survivors` | `String` (JSON) | husky_mission | (monitors) | people list, latched |

## Per-robot mission topics

| Topic | Type | Publisher | Subscribers | Notes |
| --- | --- | --- | --- | --- |
| `/<ns>/comm_status` | `Bool` | husky_mission | return_relay, survivor_detector | heartbeat |
| `/<ns>/home_shared` | `PoseStamped` (shared_map) | husky_mission | nav_goal_bridge | explore start |
| `/<ns>/home_local` | `PoseStamped` (map) | nav_goal_bridge | nav_executor | |
| `/<ns>/set_done` | `Int32` | nav_executor | husky_mission | latched, finished set number |
| `/<ns>/slot_error` | `Float32` | a1_follower | husky_mission | distance to follow slot |
| `/<ns>/pose_fix` | `PointStamped` (world) | a1_follower | nav_goal_bridge, husky_mission | once on set arrival |
| `/<ns>/comm_mode` | `String` | return_relay | nav_executor | transient local |
| `/<ns>/return_events` | `String` | return_relay | monitors | |
| `/<ns>/nav_executor/state` | `String` | nav_executor | mission_watch | |
| `/<ns>/nav_executor/events`, `/path` | `String`, `Path` | nav_executor | monitors, RViz | |
| `/<ns>/survivor_markers` | `MarkerArray` (map) | survivor_detector | RViz | latched |
| `/<ns>/survivor`, `/survivor_events`, `/person_score` | | survivor_detector | monitors | |

## Velocity commands

| Topic | Publisher | Priority |
| --- | --- | --- |
| `/<ns>/cmd_vel_teleop` | human | 4 |
| `/<ns>/cmd_vel_hold` | survivor_detector | 3 |
| `/<ns>/cmd_vel_return` | return_relay | 2 |
| `/<ns>/cmd_vel_nav_test` | Nav2 controller_server, nav_executor | 1 |
| `/<ns>/cmd_vel_follow` | a1_follower | 0 |
| `/<ns>/cmd_vel` | **cmd_vel_arbiter only** → CHAMP | |
| `/<ns>/cmd_vel_source` | cmd_vel_arbiter | name of the winning input |

## Exploration

| Topic | Type | Publisher | Subscribers |
| --- | --- | --- | --- |
| `/<ns>/scan_ready` | `LaserScan` | scan_ready_relay | slam_toolbox, terrain_node |
| `/<ns>/map` | `OccupancyGrid` | slam_toolbox | nav_goal_bridge, Nav2 |
| `/shared_map` | `OccupancyGrid` (world) | nav_goal_bridge | frontier_detector, goal_assigner |
| `/<ns>/pose_shared` | `PoseStamped` (world) | nav_goal_bridge | husky_mission, a1_follower, goal_assigner, scan_ready_relay, terrain_node |
| `/frontier_candidates` | `PoseArray` | frontier_detector | frontier_comm_filter |
| `/frontier_candidates_comm` | `PoseArray` | frontier_comm_filter | goal_assigner, nav_executor |
| `/<ns>/exploration_goal` | `PoseStamped` (shared_map) | goal_assigner | nav_goal_bridge |
| `/<ns>/nav_goal_local` | `PoseStamped` (map) | nav_goal_bridge | nav_executor |
| `/<ns>/frontier_options` | `Int32` | goal_assigner | nav_executor |
| `/<ns>/exploration_blacklist` | `PoseStamped` | nav_executor | goal_assigner (that robot only) |
| `/exploration_blacklist` | `PoseStamped` | nav_executor | frontier_comm_filter (both) |
| `/<ns>/terrain_hazards`, `/<ns>/terrain_free` | `PointCloud2` (odom) | terrain_node | Nav2 costmap terrain layer |
| `/<ns>/local_costmap/costmap` | `OccupancyGrid` | Nav2 | nav_executor |

## Sensors (Gazebo → ROS)

| Topic | Content |
| --- | --- |
| `/<ns>/scan` | planar lidar |
| `/<ns>/front_camera/image_raw`, `camera_info` | front camera |
| `/<ns>/terrain_camera/points` | depth point cloud |
| `/<ns>/imu/data` | IMU |
| `/<ns>/odom` | EKF odom |
| `/<ns>/tf`, `/<ns>/tf_static` | per-robot TF |
| `/<name>/odom/ground_truth` | ground truth (evaluation only) |
| `/clock` (1 kHz), `/clock_slow` (50 Hz) | sim time |
