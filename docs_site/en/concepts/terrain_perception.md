# Terrain Perception

A 2D lidar sees a single plane (about 0.30 m high). It misses low steps, ramp side edges and drop-offs. `terrain_node` in the `terrain_perception` package finds these hazards with a forward-looking **depth sensor point cloud** and feeds them into the Nav2 costmap.

## Inputs and outputs

| Direction | Topic | Content |
| --- | --- | --- |
| in | `/<ns>/terrain_camera/points` | depth point cloud (160 × 120, ±43° horizontal, ±35° vertical, 0.1-4 m, 5 Hz) |
| in | `/<ns>/odom` | robot position (odom frame) |
| in | `/<ns>/imu/data` | body roll/pitch |
| in | `/<ns>/scan_ready` | cells hit by the lidar (for suppression) |
| in | `/mission/state`, `/husky/pose`, `/<other>/pose_shared` | phase, other robot positions |
| out | `/<ns>/terrain_hazards` | hazard cell centers → costmap marking |
| out | `/<ns>/terrain_free` | points on a circle around the robot → costmap clearing |

## Hazard rules (grid.py)

The cloud is leveled with IMU roll/pitch and heights are binned into 0.05 m cells. A cell is a hazard if any of these hold (`max_step` = 0.04 m = CHAMP `swing_height`):

1. height range inside the cell > `max_step` — vertical faces, the front of a step
2. height difference to any 8-neighbor > `max_step` — steps, visible drop-offs
3. slope to the cell 2 away > 25° — unclimbable slope
4. drop beyond an occlusion — if the first visible cell is lower by more than `max_step`, the higher cell is a hazard (seen from above, just outside a ramp side edge)

:::{note}
Why the IMU correction: the EKF (`state_estimation`) TF has roll/pitch fixed at 0. Without correction, on a 10° ramp the whole ground looked tilted and rule (3) misfired.
:::

## Memory and clearing

- Results are **remembered** within 3 m of the robot (`radius`; forgotten after 60 s unseen, `forget`). Hazards beside and behind the robot, which the camera no longer sees, persist.
- Only points within 2 m of the robot are used (`max_range`; the sensor reaches 4 m).
- `terrain_free` is a set of points on a circle around the robot. Costmap clearing raytraces from the camera to that circle, wiping the terrain layer inside it, and the hazards are immediately marked again → the terrain layer equals this node's memory.
- Hazards within 0.35 m of the robot center are dropped (its own legs at the body edge were blocking its own spot in the costmap).
- Hazards within 0.25 m of a cell hit by the lidar are not published (wall bases belong to the lidar).

## Phases and other robots

- Processing runs **only in exploration phases (EXPLORE, REGROUP)**. During transit the back of the Husky ahead was marked as a "step hazard", leaving a line of hazards along the road in the global costmap, and the plan home in the next set failed.
- Cells around the Husky and the other A1 are not stored (in the common building they saw each other as hazards).

## Wiring into Nav2

`terrain_perception/terrain_perception/nav2_patch.py` and `tools/terrain_setup.sh` write a patch block into the `a1_exploration` Nav2 launch files that adds the terrain layer (ObstacleLayer, marking = `terrain_hazards`, clearing = `terrain_free`). The block lives inside those launch files, so don't overwrite them wholesale.

## Launch arguments

```bash
ros2 launch terrain_perception terrain.launch.py max_step:=0.04 cloud_frame:=auto
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `max_step` | 0.04 | largest step the robot can cross (m). Keep equal to CHAMP `swing_height` |
| `cloud_frame` | `auto` | point cloud frame. `auto` uses the message header |
