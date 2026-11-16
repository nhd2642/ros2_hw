# History & Previous Architecture

## Development history (2026-11-03 to 11-14)

| Stage | Dates | Content |
| --- | --- | --- |
| 1. A1 gait | 11-03 to 04 | investigated CHAMP A1 forward/backward asymmetry in Gazebo Classic, tried an RL gait (rl_sar). Re-tuned the gait and kept CHAMP |
| 2. Comm, return, Nav2, terrain | 11-06 to 07 | rewrote `comm_relay` (return, arbitration, filter, executor). Fixed CHAMP odom. New package `terrain_perception` |
| 3. Nav2 following | 11-08 | goal check window, turn first, planning through unknown cells, result recording system |
| 4. Zones, done, comm, people | 11-09 to 10 | fixed the local costmap lidar layer (`max_obstacle_height`), zone split, done → return, user comm on/off, person detection |
| 5. Move to Gazebo Sim | 11-10 | ported to Fortress, software rendering, sim-time limits, SAHI person detection, rectangular footprint |
| 6. Stalls and traps | 11-10 to 11 | fix9-fix15: excluded DWB stop trajectories, closed paths |
| 7. Experiment rules | 11-11 to 12 | one change, two runs, same-time comparison; fix16-19 |
| **8. Mission v2** | 11-12 to 14 | 3 sets + common building, autonomous Husky, A1 following, Husky-referenced correction → full mission PASS |

Detailed records in the repository: `history/CHANGELOG.md`, `docs/project_docs/` (21 documents), cumulative run summary `history/results/runs.csv` (all in Korean).

## Previous architecture (as of fix15, 2026-11-11)

Before mission v2 there was **one building and a stationary Husky**. The two A1s explored the south and north zones split at the Husky (y = 0) and returned to their spawns when done.

```{figure} ../../images/flowchart.png
:width: 70%

Per-robot flowchart of the previous architecture. Plan failure, arrival and failure handling all loop back to frontier detection; after 30 s with no reachable frontier in its zone the robot returns to spawn.
```

```{figure} ../../images/rqt_graph_1_exploration.png
:width: 80%

Previous architecture rqt_graph 1: exploration and navigation (ellipse = node, box = topic, dashed = action).
```

```{figure} ../../images/rqt_graph_2_command_comm.png
:width: 80%

Previous architecture rqt_graph 2: commands, comm and person detection.
```

### What changed

| Before | Now (mission v2) |
| --- | --- |
| one building (`disaster.world`) | 3 sets + common building (`mission.world`, `make_world.py`) |
| stationary Husky | autonomous Husky (`husky_driver`) |
| fixed zone boundary y = 0 (`husky_zone_planner`) | zone rectangles per set (`/mission/state`) |
| `husky_comm_switch`, `husky_survivor_board` | merged into `husky_mission` |
| return to spawn | return to the explore start of each set |
| map merge = spawn poses | + Husky-referenced correction on set arrival |
| — | A1 following (`a1_follower`) |

### Previous performance (for reference)

| Run (sim time) | area at 300 s m² | area at 600 s m² | total stall s | home reached | mannequins /3 |
| --- | --- | --- | --- | --- | --- |
| fix13 (900 s) | 106.5 | 118.5 | 304 | 1/2 | 2 |
| fix12 (900 s) | 88.7 | 110.3 | 104 | 0/2 | 3 |
| fix12 (600 s) | 104.9 | 115.7 | 60 | 1/2 | 2 |
| fix7 (600 s) | 82.6 | 97.8 | 104 | 0/2 | 1 |

Differences in explored area were within run-to-run noise (±20 m²), which led to the "one change, two runs, same-time comparison" experiment rule.
