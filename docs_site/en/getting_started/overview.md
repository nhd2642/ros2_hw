# Project Overview

This page gives a newcomer **the whole picture in 5 minutes**. The details live in {doc}`../concepts/index`.

## At a glance

```{screenshot} world_overview.png
Top-down Gazebo view of the whole world: the road, the building pairs of sets 1-3, the common building at the east end, and the start formation (Husky + two A1s).
```

| Item | Details |
| --- | --- |
| Simulator | Gazebo Sim (Fortress) + `ros_gz`, run headless on WSL2 |
| Middleware | ROS 2 Humble |
| Robots | 1 Husky A200 (wheeled, 2D lidar + IMU), 2 Unitree A1 (quadruped, CHAMP gait) |
| A1 sensors | 360° planar lidar (10 Hz, 10 m), front RGB camera (2 Hz), terrain depth sensor (5 Hz), IMU (100 Hz) |
| Main stack | slam_toolbox (per-robot SLAM), Nav2 (planner/controller servers), OpenCV DNN YOLOv4-tiny (person detection) |
| Mission | 3 sets (a pair of buildings either side of the road) + 1 common building, 9 people (mannequins) |
| Verdict | reached the end + 0 falls + 0 exploration timeouts + every person reported within 1 m of truth → PASS |

## How one mission flows

```{graphviz} ../../diagrams/mission_states.dot
```

1. **IDLE → TRANSIT**: when the user sends `start`, the Husky leaves for the set 1 parking spot. The two A1s walk in slots 1.6 m behind and 0.8 m to each side.
2. **TRANSIT → EXPLORE**: once the Husky has arrived and both A1s stand in their slots, those positions are recorded as the "explore start (home)" and each A1 receives its exploration zone (a rectangle).
3. **EXPLORE**: inside its building each A1 picks frontiers (the edge between known and unknown space), drives there with Nav2 and fills in the map. When it sees a person it stops for 1 s and reports to the Husky.
4. **Set done**: when there has been no reachable frontier for 30 s, the A1 returns home and publishes `/<ns>/set_done`. When both are back, the team moves to the next set.
5. **DONE**: after the common building (both A1s share one zone) the mission ends.

If exploration takes too long (1,500 s), the Husky calls the A1s back with **REGROUP**. At any time the user can send `0` (comm off) / `1` (comm on) to simulate a link outage.

## Building blocks

```{graphviz} ../../diagrams/architecture.dot
```

The layers stack from the bottom up.

Simulation (`disaster_sim`, `a1_champ`, `champ`)
: World, robot models, gait controller. The layout has a single source: `make_world.py`.

Navigation & perception (`slam_toolbox`, Nav2, `terrain_perception`)
: Per-robot SLAM map, path planning and following, and a terrain layer that puts slopes and steps seen by the depth sensor into the costmap.

Exploration (`a1_exploration`)
: A shared map merged from both maps, frontier extraction, and goal assignment for the two robots.

Mission (`comm_relay`)
: Mission phases, Husky driving, A1 following, the Nav2 executor, comm-loss return, person detection, and velocity command arbitration. Most of the code written in this project lives here.

## Repository layout

```text
disaster_develop/
├── code/ros2_hw_gz_src/      # ROS 2 package sources (ws/src links here)
│   ├── comm_relay/           # mission layer (Husky mission/drive, A1 follow/executor/return/person detection)
│   ├── a1_exploration/       # SLAM & Nav2 launch, frontier / goal assignment / map merge scripts
│   ├── terrain_perception/   # depth sensor → costmap terrain layer
│   ├── disaster_sim/         # world, Husky model, mission.yaml, make_world.py
│   ├── a1_champ/             # A1 URDF, gait config, launch
│   ├── a1_rl/                # (experimental) RL locomotion
│   ├── champ/, champ_teleop/ # CHAMP gait controller, keyboard teleop
├── ws/                       # colcon workspace (build/install are not in git)
│   └── person_ai_models/     # YOLOv4-tiny model files
├── results/                  # run_mission.sh results (only two passing runs are committed)
├── docs/                     # original design notes (mission_design.md etc.)
├── docs_site/                # this documentation site (Sphinx)
├── history/                  # step-by-step history (CHANGELOG.md, runs.csv)
└── tools/                    # run comparison / planner test tools
```

## Next steps

- Want to run it now? → {doc}`installation`
- Want to understand the structure first? → {doc}`../concepts/architecture`
