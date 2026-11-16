# Architecture

This page explains how packages and nodes form layers, which topics connect them, and in what order they start.

## Layers

```{graphviz} ../../diagrams/architecture.dot
```

| Layer | Packages | What this project did |
| --- | --- | --- |
| Mission | `comm_relay` | almost all new: mission phases, Husky drive, A1 follow, Nav2 executor, comm return, person detection, arbitration |
| Exploration | `a1_exploration` | on top of teammate code (frontiers, goal assignment, map merge) added zones, blacklists, pose correction, etc. |
| Navigation & perception | `slam_toolbox`, Nav2, `terrain_perception` | SLAM and Nav2 configuration, a new terrain layer |
| Simulation | `disaster_sim`, `a1_champ`, `champ` | new mission v2 world and Husky model, ported to Gazebo Sim |

A few design principles run through everything.

- **Coordinates come from one config file.** Nodes keep no positions in code; they only read `mission.yaml` ({doc}`world_layout`).
- **Decisions are separate from ROS.** `*_logic.py` is pure logic testable without ROS; node files only convert messages ({doc}`../how_to/run_unit_tests`).
- **Only one thing writes to CHAMP.** All five velocity sources go through `cmd_vel_arbiter` ({doc}`communication_return`).
- **Nav2 is only the planner and controller servers.** With no `bt_navigator` or recovery behaviors, `nav_executor` does plan → follow → failure handling itself ({doc}`exploration`).

## Which node drives in which phase

The same A1 is driven by different nodes depending on the mission phase.

| Phase | Node driving the A1 | Velocity topic | Arbiter priority |
| --- | --- | --- | --- |
| TRANSIT (road) | `a1_follower` | `/<ns>/cmd_vel_follow` | 0 |
| EXPLORE / REGROUP | `nav_executor` → Nav2 controller | `/<ns>/cmd_vel_nav_test` | 1 |
| comm lost | `return_relay` | `/<ns>/cmd_vel_return` | 2 |
| person found (1 s) | `survivor_detector` | `/<ns>/cmd_vel_hold` | 3 |
| human teleop | teleop | `/<ns>/cmd_vel_teleop` | 4 |

## Main topic flows

**Mission state broadcast** — `husky_mission` publishes `/mission/state` (a JSON string) at 1 Hz with the phase, set number, per-robot zone rectangles and comm state. `a1_follower`, `nav_executor`, `return_relay`, `frontier_comm_filter` and `terrain_node` change behavior based on it.

**Exploration goals** — SLAM map → `nav_goal_bridge` (shared map) → `frontier_detector` → `frontier_comm_filter` → `goal_assigner` → `nav_goal_bridge` (to robot map coordinates) → `nav_executor` → Nav2. Details in {doc}`exploration`.

**Person reports** — `survivor_detector` → `/husky/survivor_report` → `husky_mission` merges the same person and tallies on `/husky/survivors`.

**Set completion** — when `nav_executor` reaches home, `/<ns>/set_done` (latched) → `husky_mission`.

Full list in {doc}`../reference/topics`.

## Startup order

```{graphviz} ../../diagrams/launch_order.dot
```

Why the order matters:

1. **Sim first.** Without `/clock`, every node on sim time stalls. The A1s must have active joint controllers and stand stably (about 8 s) before SLAM starts, or SLAM gets tilted scans.
2. **SLAM → Nav2.** The Nav2 global costmap uses the SLAM map (`map` frame) as its static layer.
3. **Mission layer last.** `nav_executor` only publishes its state heartbeat once the Nav2 action servers are ready.

## Time and CPU

- Every decision and limit uses **sim time**. With an RTF of 0.3, 900 s of sim time is about 50 minutes real.
- CHAMP gait needs `/clock` at 1 kHz, but an rclpy node used about 20 % CPU just receiving `/clock` (14 Python nodes, RTF 0.36). So `clock_throttle` produces a 50 Hz `/clock_slow` and all Python nodes remap `/clock:=/clock_slow` ({doc}`coordinates_time`).
- Exception: the heartbeat timeout (`return_relay`) uses wall-clock time.

## Differences from the previous architecture

Before mission v2 (2026-11-12 on), there was one building, a stationary Husky, and a fixed zone boundary at y = 0. The old nodes `husky_comm_switch`, `husky_zone_planner` and `husky_survivor_board` were merged into `husky_mission`, and `los_checker`, `zones.py` and `comm_zones.yaml` were replaced by the zone rectangles in `/mission/state`. Old diagrams and records are in {doc}`../results/history`.

```{screenshot} rqt_graph_v2.png
rqt_graph during a mission (Nodes/Topics (active), only the a1_1 namespace expanded).
```
