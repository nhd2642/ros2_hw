# Drive an A1 by Hand

While the mission nodes are running, only `cmd_vel_arbiter` writes A1 velocity commands to CHAMP. So teleop commands must go to **`/<ns>/cmd_vel_teleop`**. That input has the highest priority (4) and beats Nav2, return and follow commands while you send it.

```{graphviz} ../../diagrams/cmd_vel_arbiter.dot
```

## Keyboard (champ_teleop)

```bash
source ~/ros2_hw/ws/install/setup.bash
ros2 run champ_teleop champ_teleop.py --ros-args -r cmd_vel:=/a1_1/cmd_vel_teleop -p use_sim_time:=true
```

To drive `a1_2`, change only the remap. The keys are the same as `teleop_twist_keyboard` (`i` forward, `,` back, `j`/`l` turn, `k` stop).

:::{warning}
Writing **directly** to `/<ns>/cmd_vel` mixes with the zero commands the arbiter sends at 20 Hz and the robot stutters to a halt. The arbiter warns when it sees another publisher.
:::

## Sending a command

```bash
# a1_1 forward at 0.2 m/s (only while publishing; returns to 0 if nothing arrives within 0.5 s)
ros2 topic pub -r 10 /a1_1/cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

After `Ctrl+C`, within 0.5 s control returns to the next-priority input (or a zero command).

## Who is driving right now

```bash
ros2 topic echo /a1_1/cmd_vel_source
```

Prints the name of the winning input: `teleop`, `hold`, `return`, `nav` or `follow`.

## Testing gait without the mission

Without the mission layer (`mission.launch.py`) there is no arbiter, so writing straight to `/<ns>/cmd_vel` is fine. For gait measurement tools see {doc}`measure_gait`.
