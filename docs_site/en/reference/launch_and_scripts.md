# Launch Files & Scripts

## run_mission.sh

```bash
bash comm_relay/tools/run_mission.sh [start_set | clean]
```

| Argument / variable | Default | Meaning |
| --- | --- | --- |
| first argument `start_set` | 0 | 0 = from the beginning, k = start just before set k |
| first argument `clean` | | only clean up leftover processes |
| `GUI` | 0 | 1 = Gazebo window |
| `START` | 1 | 0 = don't send `start` automatically |
| `SETS` | 99 (all) | stop after n sets |
| `DUR` | 6000 | sim-second limit |
| `COMM_OFF_AT` | 0 (off) | comm 0 at this many sim seconds after exploration starts |
| `COMM_OFF_FOR` | 60 | comm 1 this many sim seconds later |
| `OUT` | `<repo>/results/<time>_set<k>` | result folder |
| `ROS_DOMAIN_ID` | random 20-79 | avoids collisions |

What it does: clean up → sim → (controllers active + 8 s) → SLAM → Nav2 + terrain → (4 servers active; retry once on failure) → mission layer → `start` ×3 → `mission_watch.py` → extract `mission_events.txt` → clean up on exit.

## Launch files

### disaster_sim/sim.launch.py

World + Husky + two A1s. Husky spawns at 4 s, `a1_1` at 6 s, `a1_2` at 20 s.

| Argument | Default | Meaning |
| --- | --- | --- |
| `gui` | true | false = server only |
| `start_set` | 0 | 1+ places the robots at the previous set's parking spot and slots |

### disaster_sim/world.launch.py

| Argument | Default | Meaning |
| --- | --- | --- |
| `world` | `mission.world` | file in `worlds/` (original map: `disaster.world`) |
| `gui` | true | |
| `render_engine` | `ogre` | `ogre` (for WSL) / `ogre2` |
| `software_rendering` | true | software GL for the sim server only (avoids the lidar range problem) |
| `headless_rendering` | false | |
| `verbose` | false | |

### a1_exploration

| Launch | Content |
| --- | --- |
| `dual_slam.launch.py` | slam_toolbox + scan_ready_relay for both robots |
| `nav2_planners.launch.py` | planner_server + global costmap + lifecycle_manager for both robots |
| `nav2_controllers.launch.py` | controller_server + local costmap for both robots. With `A1_DWB_DIAG=1` it publishes DWB evaluations |

### terrain_perception/terrain.launch.py

| Argument | Default |
| --- | --- |
| `max_step` | 0.04 |
| `cloud_frame` | `auto` |

### comm_relay/mission.launch.py

| Argument | Default | Meaning |
| --- | --- | --- |
| `start_set` | 0 | same value as sim.launch.py |
| `husky` | true | false = the user starts `husky_mission` separately |

Starts: `husky_driver`, `a1_follower`, `return_relay`, `cmd_vel_arbiter`, `frontier_comm_filter`, `nav_executor`, `survivor_detector`, (`husky_mission`), and from `a1_exploration/scripts` `nav_goal_bridge.py`, `frontier_detector.py`, `goal_assigner.py`. Every Python node gets `/clock:=/clock_slow` and `use_sim_time:=true`.

:::{note}
`clock_throttle` starts with the world in `world.launch.py` (hence in `sim.launch.py`). `/clock_slow` must exist before the mission layer.
:::

## Tool scripts (comm_relay/tools)

| Script | Purpose |
| --- | --- |
| `run_mission.sh` | the whole mission |
| `mission_watch.py` | observation log and verdict (`<folder> <start_set> <sets> <limit> [comm off at] [off for]`) |
| `gait_sweep.sh` | gait speed response |
| `walk_check.sh`/`.py` | walking and odom check |
| `lowspeed_check.sh`/`.py` | slow command following |
| `slam_diag.sh`/`.py` | SLAM map diagnosis |
| `nav_probe.sh`/`.py` | Nav2 planning failure diagnosis |
| `nav_debug.sh`/`.py` | DWB stall diagnosis |
| `terrain_check.sh`/`.py` | terrain hazard check |
| `lidar_check.sh`/`.py` | lidar value check |
| `costmap_check.py`, `costmap_diag.py`, `tf_check.py`, `dwb_diag.py` | costmap, TF and DWB diagnosis |
| `map_snapshot.py` | map + scan PNG |
| `sim_sleep.py` | wait in sim time |
| `common.sh` | shared tool functions (cleanup, visualization `VIS=1`) |
| `fastdds_udp.xml` | Fast DDS UDP-only profile |
| `vis.rviz` | per-robot RViz template (`/NS/`) |
