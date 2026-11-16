# Comm, Return & Arbitration

## The comm model

Link outages are simulated by **user control** (`0` / `1`), not computed from distance or walls (a project goal, 2026-11-09).

- While comm is on, `husky_mission` publishes a `/<ns>/comm_status` (Bool) heartbeat per robot.
- `0` stops the heartbeat and stops the Husky (`/husky/go = False`).
- The robot side treats **1 s of wall-clock time** without heartbeat as a loss. It uses wall-clock time so the timeout feels the same regardless of the real-time factor.

## Return — return_relay

```{graphviz} ../../diagrams/return_states.dot
```

| State | Meaning |
| --- | --- |
| `WAIT` | no heartbeat or odom received yet → sends no commands |
| `NORMAL` | link fine. Records the path as breadcrumbs every 0.1 m (up to 5,000) |
| `RETURNING` | 1 s without heartbeat → retrace the breadcrumbs in reverse (immediate-return policy) |
| `HOLD` | all breadcrumbs retraced, at home, still no link → keep sending stop |
| `STOPPING` | link restored while returning → stop for 1 s, then `NORMAL` |

### Retracing

- Breadcrumbs and retracing both use only the robot's own **odom frame**. The path doesn't jump with SLAM corrections, but home is off by however much odom drifted (latest run 0.85 m).
- The default (`return_drive: direct`) drives directly: if the next breadcrumb is more than 0.35 rad off, turn in place first (0.4 rad/s), otherwise move at 0.15 m/s while correcting heading.
- **Obstacles on the path**: only while walking, if the strip along the path to retrace (±0.15 m wide, 0.5 m ahead) contains 3 or more lidar points in both of the last two scans, it stops, and goes again once clear. Breadcrumbs are where the body actually passed, so the strip was empty then; only new obstacles (another robot, etc.) trigger it.
- An odom jump over 0.5 m breaks the breadcrumb trail.
- If the link drops during TRANSIT, it doesn't retrace; it just stops where it is.

### Relation to the executor

`return_relay` publishes its state on `/<ns>/comm_mode` (transient local). When it is `RETURNING`/`HOLD`, `nav_executor` holds its exploration goal (`PAUSED`), and restarts when it returns to `NORMAL`.

### Why STOPPING is needed

CHAMP has no `cmd_vel` timeout, so it **keeps walking with the last command**. The teammate original just stopped publishing at the end of a return, so the robot kept walking. Hence an explicit zero command for 1 s.

## Arbitration — cmd_vel_arbiter

```{graphviz} ../../diagrams/cmd_vel_arbiter.dot
```

For each robot this makes **one** writer for CHAMP's `/<ns>/cmd_vel`.

| Priority | Input | Sender |
| --- | --- | --- |
| 4 | `/<ns>/cmd_vel_teleop` | human teleop |
| 3 | `/<ns>/cmd_vel_hold` | `survivor_detector` (1 s stop on a person) |
| 2 | `/<ns>/cmd_vel_return` | `return_relay` |
| 1 | `/<ns>/cmd_vel_nav_test` | Nav2 controller_server, `nav_executor` turns and escapes |
| 0 | `/<ns>/cmd_vel_follow` | `a1_follower` |

- Of the inputs received within 0.5 s, the highest priority one is published at 20 Hz.
- With no inputs at all it publishes a **zero command** (fixing the missing CHAMP timeout here).
- The name of the winning input goes out on `/<ns>/cmd_vel_source`.
- It warns if another publisher appears on `/<ns>/cmd_vel`.

The decision logic is `arbiter_logic.Arbiter`, which is unit tested.
