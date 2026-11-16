# A1 Locomotion (CHAMP)

## Components

The A1 walks with the [CHAMP](https://github.com/chvmp/champ) gait controller. CHAMP takes `cmd_vel` (vx, vy, wz), generates foot trajectories and computes joint angles with inverse kinematics. Joint commands reach the Gazebo joints through `ign_ros2_control`.

| Part | Package | Node |
| --- | --- | --- |
| robot model | `a1_champ` (`urdf/a1.urdf.xacro`, `scripts/make_a1_urdf.py`) | — |
| gait control | `champ_base` | `quadruped_controller_node` |
| state estimation | `champ_base` + `robot_localization` | `state_estimation_node`, `ekf_node` |
| joint control | `ign_ros2_control` | `joint_group_effort_controller` |
| launch | `a1_champ/launch/a1.launch.py` | `robot_ns`, `x`, `y`, `z`, `yaw` arguments |

`sim.launch.py` spawns `a1_1` after 6 s and `a1_2` after 20 s. After startup a 3 s soft start plus time to stand stably are needed, so `run_mission.sh` waits 8 s more after the controllers are active.

## Sensors

| Sensor | Type | Settings | Topic |
| --- | --- | --- | --- |
| planar lidar | gpu_lidar | 360 points, 360°, 0.15-10 m, 10 Hz, about 0.28 m high | `/<ns>/scan` |
| front camera | camera | 1.8 rad (about 103°) horizontal FOV, 640 × 480, 2 Hz | `/<ns>/front_camera/image_raw` |
| terrain depth | gpu_lidar (used as a depth sensor) | 160 × 120, ±43° horizontal, ±35° vertical, 0.1-4 m, 5 Hz | `/<ns>/terrain_camera/points` |
| IMU | imu | 100 Hz | `/<ns>/imu/data` |

:::{note}
The sim server runs with **software rendering**. With GPU rendering on WSL every lidar range came out as 0.15 m (range_min).
:::

## Gait settings (a1_champ/config/gait.yaml)

| Value | Setting | Meaning |
| --- | --- | --- |
| `max_linear_velocity_x` | 0.5 m/s | forward/back max |
| `max_linear_velocity_y` | 0.25 m/s | sideways max (in practice sidestepping barely moves) |
| `max_angular_velocity_z` | 1.0 rad/s | turn max |
| `swing_height` | 0.04 m | foot lift → keep equal to terrain `max_step` |
| `stance_duration` | 0.25 s | stance time |
| `nominal_height` | 0.2 m | body height |
| `odom_scaler` | 1.125 | leg odom scale (1.11 → 1.125 on 2026-11-12) |
| `com_x_translation` | −0.015 m | center-of-mass offset |

Measurements: {doc}`../how_to/measure_gait`.

## Properties to know

- **Sidestepping barely moves.** Mostly-sideways motion gave an actual/odom ratio of 0.05-0.24 (only the leg odom counted it as moving). So escapes and steering use only the forward/back axis, turning in place first when needed.
- **Turning in place on a slope slides.** On a 10° ramp an A1 slid 0.5 m during a 6 s turn and fell. No turning in place above 6° of tilt.
- **Leg odom advances even when blocked.** When stuck and walking in place, odom keeps growing and the SLAM position drifts.
- **No cmd_vel timeout.** It keeps following the last command, so `cmd_vel_arbiter` sends zero when there is no input.

## Differences from upstream CHAMP

Moving from Gazebo Classic (ODE) to Gazebo Sim (DART) required re-tuning the gait and odom. The process is in `docs/project_docs/champ_fix_log.md`, `gz_migration.md` and `gait_experiment_log.md`. The reinforcement-learning gait attempt remains in the `a1_rl` package and `rl_sar_integration.md`, but the current mission doesn't use it.
