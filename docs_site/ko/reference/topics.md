# 토픽

`<ns>` = `a1_1` / `a1_2`. 프레임은 메시지 header 기준입니다.

## 임무·Husky

| 토픽 | 형식 | 발행 | 구독 | 비고 |
| --- | --- | --- | --- | --- |
| `/mission/state` | `MissionState` | husky_mission | a1_follower, nav_executor, return_relay, frontier_comm_filter, frontier_detector, goal_assigner, terrain_node | 1 Hz |
| `/husky/goal` | `PoseStamped` (world) | husky_mission | husky_driver | 주차점 |
| `/husky/go` | `Bool` | husky_mission | husky_driver | False = 정지 유지 |
| `/husky/pose` | `PoseStamped` (world) | husky_driver | husky_mission, a1_follower, scan_ready_relay, terrain_node | 좌표 기준 |
| `/husky/driver_state` | `String` | husky_driver | husky_mission | latched |
| `/husky/cmd_vel` | `Twist` | husky_driver | Gazebo (bridge) | |
| `/husky/odom`, `/husky/imu`, `/husky/scan` | | Gazebo (bridge) | husky_driver | |
| `/husky/survivor_report` | `SurvivorReport` | survivor_detector | husky_mission | 로봇·id·위치·score·delay |
| `/husky/survivors` | `SurvivorList` | husky_mission | mission_watch | 사람 목록, latched |

임무 요청·통신 0/1·생존자 조회는 토픽이 아니라 action `/husky/run_mission`, service `/husky/set_comm`·`/husky/get_survivors` 입니다 ({doc}`interfaces`).

## 로봇별 임무 토픽

| 토픽 | 형식 | 발행 | 구독 | 비고 |
| --- | --- | --- | --- | --- |
| `/<ns>/comm_status` | `Bool` | husky_mission | return_relay, survivor_detector | 하트비트 |
| `/<ns>/home_shared` | `PoseStamped` (shared_map) | husky_mission | nav_goal_bridge | 탐색 시작 위치 |
| `/<ns>/home_local` | `PoseStamped` (map) | nav_goal_bridge | nav_executor | |
| `/<ns>/set_done` | `Int32` | nav_executor | husky_mission | latched, 끝낸 세트 번호 |
| `/<ns>/slot_error` | `Float32` | a1_follower | husky_mission | 추종 자리까지 거리 |
| `/<ns>/pose_fix` | `PointStamped` (world) | a1_follower | nav_goal_bridge, husky_mission | 세트 도착 때 한 번 |
| `/<ns>/comm_mode` | `String` | return_relay | nav_executor | transient local |
| `/<ns>/return_events` | `String` | return_relay | 모니터 | |
| `/<ns>/nav_executor/state` | `String` | nav_executor | mission_watch | |
| `/<ns>/nav_executor/events`, `/path` | `String`, `Path` | nav_executor | 모니터, RViz | |
| `/<ns>/survivor_markers` | `MarkerArray` (map) | survivor_detector | RViz | latched |
| `/<ns>/survivor`, `/survivor_events`, `/person_score` | | survivor_detector | 모니터 | |

## 속도 명령

| 토픽 | 발행 | 우선순위 |
| --- | --- | --- |
| `/<ns>/cmd_vel_teleop` | 사람 | 4 |
| `/<ns>/cmd_vel_hold` | survivor_detector | 3 |
| `/<ns>/cmd_vel_return` | return_relay | 2 |
| `/<ns>/cmd_vel_nav_test` | Nav2 controller_server, nav_executor | 1 |
| `/<ns>/cmd_vel_follow` | a1_follower | 0 |
| `/<ns>/cmd_vel` | **cmd_vel_arbiter 만** → CHAMP | |
| `/<ns>/cmd_vel_source` | cmd_vel_arbiter | 지금 이긴 입력 이름 |

## 탐색

| 토픽 | 형식 | 발행 | 구독 |
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
| `/<ns>/exploration_blacklist` | `PoseStamped` | nav_executor | goal_assigner (그 로봇만) |
| `/exploration_blacklist` | `PoseStamped` | nav_executor | frontier_comm_filter (둘 다) |
| `/<ns>/terrain_hazards`, `/<ns>/terrain_free` | `PointCloud2` (odom) | terrain_node | Nav2 비용지도 지형 층 |
| `/<ns>/local_costmap/costmap` | `OccupancyGrid` | Nav2 | nav_executor |

## 센서 (Gazebo → ROS)

| 토픽 | 내용 |
| --- | --- |
| `/<ns>/scan` | 평면 라이다 |
| `/<ns>/front_camera/image_raw`, `camera_info` | 앞 카메라 |
| `/<ns>/terrain_camera/points` | 깊이 점구름 |
| `/<ns>/imu/data` | IMU |
| `/<ns>/odom` | EKF odom |
| `/<ns>/tf`, `/<ns>/tf_static` | 로봇별 TF |
| `/<이름>/odom/ground_truth` | 정답 위치 (평가 전용) |
| `/clock` (1 kHz), `/clock_slow` (50 Hz) | 시뮬 시간 |
