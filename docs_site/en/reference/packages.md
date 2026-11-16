# Packages

All packages live in `code/ros2_hw_gz_src/` (`ws/src` points here).

## comm_relay (ament_python)

The mission layer. Most of the code written in this project.

| Path | Content |
| --- | --- |
| `comm_relay/husky_mission.py` | mission phases, user commands, heartbeat, people tally |
| `comm_relay/husky_driver.py` | Husky driving and pose estimation |
| `comm_relay/a1_follower.py` | A1 following, Husky-referenced pose correction |
| `comm_relay/nav_executor.py` | Nav2 plan, follow, failure handling, done, return home |
| `comm_relay/return_relay.py` | comm-loss return |
| `comm_relay/cmd_vel_arbiter.py` | velocity command arbitration |
| `comm_relay/frontier_comm_filter.py` | zone and blacklist frontier filter |
| `comm_relay/survivor_detector.py` | person detection, location, reporting |
| `comm_relay/clock_throttle.py` | `/clock` → `/clock_slow` |
| `comm_relay/*_logic.py` | decision logic without ROS (`drive`, `mission`, `return`, `nudge`, `survivor`, `arbiter`) |
| `launch/mission.launch.py` | the whole mission layer |
| `tools/` | `run_mission.sh`, `mission_watch.py`, diagnostic tools |
| `test/` | unit tests |

## a1_exploration (ament_cmake)

SLAM and Nav2 launch files and exploration scripts (based on teammate code).

| Path | Content |
| --- | --- |
| `launch/dual_slam.launch.py` | per-robot slam_toolbox + scan_ready_relay |
| `launch/slam.launch.py` | SLAM for one robot |
| `launch/nav2_planners.launch.py` | per-robot planner_server + global costmap |
| `launch/nav2_controllers.launch.py` | per-robot controller_server (DWB) + local costmap |
| `config/slam_params.yaml` | slam_toolbox settings |
| `scripts/nav_goal_bridge.py` | map merge (inherits `shared_map_with_robots`), coordinate conversion |
| `scripts/frontier_detector.py` | frontier extraction |
| `scripts/goal_assigner.py` | goal assignment for both robots |
| `scripts/scan_ready_relay.py` | scan preprocessing (tilt, other robots) |
| `scripts/shared_map_merger.py`, `shared_map_with_robots.py` | merge base classes |
| `scripts/preview_nav2_paths.py`, `test_follow_paths.py` | development helpers |

## terrain_perception (ament_python)

| Path | Content |
| --- | --- |
| `terrain_perception/terrain_node.py` | depth cloud → hazard / free cells |
| `terrain_perception/grid.py` | hazard rules and memory (no ROS) |
| `terrain_perception/nav2_patch.py` | terrain layer patch for the Nav2 launch |
| `launch/terrain.launch.py` | `max_step`, `cloud_frame` arguments |
| `tools/terrain_setup.sh`, `tools/fake_depth.py` | apply the patch, fake depth input |

## disaster_sim (ament_cmake)

| Path | Content |
| --- | --- |
| `scripts/make_world.py` | **the layout source** → `worlds/mission.world`, `config/mission.yaml` |
| `launch/sim.launch.py` | world + Husky + two A1s (`gui`, `start_set`) |
| `launch/world.launch.py` | world only (`world`, `gui`, `render_engine`, `software_rendering` …) |
| `models/husky/` | Husky A200 (DiffDrive, 2D lidar, IMU) |
| `models/person_standing/` | person model |
| `worlds/disaster.world` | original map of the previous architecture |

## a1_champ (ament_cmake)

| Path | Content |
| --- | --- |
| `urdf/a1.urdf.xacro` | A1 model + sensors + ros2_control |
| `config/gait.yaml`, `joints.yaml`, `links.yaml`, `ros_control.yaml` | CHAMP settings |
| `launch/a1.launch.py` | one robot (`robot_ns`, `x`, `y`, `z`, `yaw`) |
| `launch/gait_test.launch.py`, `worlds/gait_test.world` | flat-ground gait test |
| `scripts/gait_eval.py`, `joint_tracking.py`, `run_exp.sh` | gait evaluation |

## Others

| Package | Content |
| --- | --- |
| `champ/` | CHAMP gait controller (`champ`, `champ_base`, `champ_msgs`, `champ_bringup`, `champ_description`, `champ_gazebo`, `champ_config`, `champ_navigation`) |
| `champ_teleop` | keyboard and joystick teleop |
| `a1_rl` | reinforcement-learning gait experiments (not used by the current mission) |
