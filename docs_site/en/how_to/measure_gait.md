# Measure the Gait Speed Response

If you change the CHAMP gait config (`a1_champ/config/gait.yaml`), measure again how well the commanded speed is followed and how much the body tilts. The follow and Nav2 speed limits were set from these numbers.

## Run

```bash
cd ~/ros2_hw/ws && source install/setup.bash
bash src/comm_relay/tools/gait_sweep.sh                    # default speed list
bash src/comm_relay/tools/gait_sweep.sh "0.2 0.0" "0.25 0.3" "0.25 -0.3"   # your own "vx wz" list
```

It starts only the sim, headless, walks `a1_2` along the (flat) road for 8 s per command, and measures distance and tilt from **Gazebo ground truth**.

## Baseline (2026-11-12)

```{figure} ../../images/figures/gait_sweep.png
:width: 100%

Actual/commanded ratio and maximum tilt per commanded speed (flat ground, 8 s each).
```

| Commanded vx m/s | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 |
| --- | --- | --- | --- | --- | --- |
| actual / command | 99 % | 95 % | 107 % | 108 % | 107 % |
| max tilt | 2.3° | 2.3° | 3.7° | 4.6° | 5.5° |

- Turning while walking (0.25 m/s, ±0.3 rad/s): distance matches the expected arc chord (1.56 and 1.53 m, expected 1.55 m), rotation is 107-112 % of command, symmetric left/right, tilt under 4°.
- From these numbers the **follow maximum is 0.35 m/s and the Husky 0.25 m/s**. Nav2 (MPPI) exploration speed stays at 0.2 m/s.

## Calibrating the odom scale

If `/odom` distance is shorter or longer than truth, change `odom_scaler` in `gait.yaml`. On 2026-11-12 it was 1.3 % short, so it went `1.11 → 1.125` (not re-measured since). Left uncorrected, this error shrinks the SLAM map by 0.5 m over the 40 m road.

```yaml
gait:
  odom_scaler: 1.125
  max_linear_velocity_x: 0.5
  swing_height: 0.04
  nominal_height: 0.2
```

If you change `swing_height`, set the terrain perception `max_step` to match (`terrain.launch.py max_step:=...`).
