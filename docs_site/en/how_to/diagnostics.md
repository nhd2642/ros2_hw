# Use the Diagnostic Tools

`comm_relay/tools/` collects tools built during development to isolate problems. Most come in pairs: **the `.sh` starts only what is needed and the `.py` observes and records**. All run headless and take 3-5 minutes.

```bash
cd ~/ros2_hw/ws/src/comm_relay/tools
```

## Tools by symptom

| Symptom | Tool | Starts | Does |
| --- | --- | --- | --- |
| map doesn't grow | `slam_diag.sh` | sim + SLAM | saves the map before and after watching 10 s, a 360° turn, and 1 m forward |
| odd lidar values (all 0.15 m etc.) | `lidar_check.sh` | sim with three rendering modes | reads one scan and one depth cloud each and compares |
| Nav2 planning fails | `nav_probe.sh` | sim + SLAM + Nav2 | records map, costmap and plan result at 5/25/50 s |
| Nav2 (DWB) won't move | `nav_debug.sh` | sim + SLAM + Nav2 + terrain | gives 3 goals and records commands, odom and DWB evaluation |
| odd terrain hazards | `terrain_check.sh` | sim + SLAM + Nav2 + terrain | cloud frame, hazard positions at ramp edges |
| odd walking or odom | `walk_check.sh` | sim | flat 0.2 m/s for 8 s, turn in place 0.5 rad/s for 6 s, compares odom to truth |
| slow commands not followed | `lowspeed_check.sh` | flat sim, one A1 | 3 s ramp + 6 s measurement per speed |

## Tools that attach to a running system

In another terminal while a mission runs (same `ROS_DOMAIN_ID`):

```bash
python3 map_snapshot.py a1_2 /tmp/a1_2_map.png "exploring set 2"   # SLAM map + current scan as PNG
python3 costmap_check.py        # do lidar obstacles reach the costmap?
python3 tf_check.py             # repeats the TF lookup the local costmap lidar layer does
python3 sim_sleep.py 120        # wait 120 s of sim time (for scripts)
```

To record moments when DWB outputs zero commands, start the Nav2 controller with `A1_DWB_DIAG=1 ros2 launch a1_exploration nav2_controllers.launch.py` to enable `/<ns>/evaluation` and record it with `dwb_diag.py` (trajectory scores are heavy, so it is off by default).

## Testing the planner alone (planner_probe)

`tools/planner_probe/` at the repository root starts **only a planner_server** and plans on a saved map. Use it to compare planner settings such as the inflation radius without the sim.

| File | Role |
| --- | --- |
| `fake.py` | `map.npz` → `/a1_1/map`, fake `/clock` and TF |
| `params.yaml.in` | planner config template with `RADIUS` substituted |
| `client.py` | `ComputePathToPose` from spawn to 5 places, prints length and wall clearance |

:::{note}
`tools/compare_runs.py`, `tools/terrain_hazard_count.py` and planner_probe were built for the previous architecture (single building, fix stages). For the current mission compare `summary.txt` and `timeline.csv`.
:::
