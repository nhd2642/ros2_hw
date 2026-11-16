# Frames & Time

## Frames

```{graphviz} ../../diagrams/frames.dot
```

| Frame | Set by | Used for |
| --- | --- | --- |
| `world` | Gazebo world | every coordinate in `mission.yaml`, `/husky/pose`, `/husky/goal`, zone rectangles, `pose_fix` |
| `shared_map` | `nav_goal_bridge` (same orientation and origin as world) | `/shared_map`, `/<ns>/pose_shared`, frontier candidates, goal assignment, `/<ns>/home_shared` |
| `<ns>` `map` | slam_toolbox | that robot's SLAM map, Nav2 global planning, `/<ns>/nav_goal_local`, `/<ns>/home_local`, person markers |
| `<ns>` `odom` | EKF (`state_estimation`) | Nav2 local planning, breadcrumb return, terrain layer |
| `<ns>` `base_footprint` | EKF | the robot body |

### Separate TF per robot

Each A1 publishes TF on its own `/<ns>/tf`, `/<ns>/tf_static` (both use the frame names `map`, `odom`, `base_footprint`). So RViz runs once per robot with `/tf:=/<ns>/tf`. The place where both robots share one frame is `shared_map`.

### map → world alignment

`nav_goal_bridge` places each robot map in the world by **translating it by the spawn pose**. Rotation is 0 — the map starts in the IMU ENU orientation, which already matches the world. On set arrival a `pose_fix` shifts it further.

This relies on the assumption that **the spawn poses are known**. If start positions change it goes wrong; it does not align maps by scan matching.

### The Husky is the reference

A1 SLAM drifts, but the Husky pose estimate (wheel speed × IMU yaw) stayed within 0.15 m over 40 m. So every set the A1 maps are re-aligned to the Husky ({doc}`transit_following`).

### Ground truth for evaluation

`mission_watch.py` measures positions, errors and distances from Gazebo ground truth (`/<name>/odom/ground_truth`). **Robot nodes never use these values.** They are for evaluation only.

## Time

### Sim time is the reference

Every node runs with `use_sim_time: true`. All mission limits (exploration 1,500 s, done detection 30 s, goal 90 s …) are in sim seconds. With an RTF of 0.2-0.4, a 1,900 s mission takes about 2 hours real time.

There are two exceptions:

- the heartbeat timeout in `return_relay` (wall-clock 1 s) — to match the outage time a user experiences.
- the maximum nudge time `nudge_time` in `nav_executor` (wall-clock 20 s).

### /clock_slow

Gazebo publishes `/clock` every sim step (1 kHz). CHAMP gait control (C++) needs that rate, but an rclpy node used about 20 % CPU just receiving `/clock` (14 Python nodes, RTF 0.36).

The `clock_throttle` node republishes `/clock` as a **50 Hz `/clock_slow`**. `mission.launch.py` starts every Python node with `/clock:=/clock_slow`. TF publishing was also reduced from 200 to 50 Hz.

:::{tip}
When adding a Python node, the `node()` helper in `mission.launch.py` adds the remap for you. When starting one by hand, don't forget `-r /clock:=/clock_slow`.
:::
