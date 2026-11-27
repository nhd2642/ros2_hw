# 3. Comm Loss and Return

At a disaster site the link to a robot inside a building drops often. This project simulates that by having **the user switch the link on and off**. In this tutorial you cut the link mid-exploration and check that the A1s retrace the path they came along.

## What should happen

| Moment | Husky | A1 |
| --- | --- | --- |
| `0` sent | stops (`/husky/go` = False) | after 1 s without heartbeat → `RETURNING`: follow the breadcrumbs backwards to this set's explore start |
| home reached, still no link | stopped | `HOLD`: stands there |
| `1` sent | resumes the current phase | `STOPPING` (1 s stop) → `NORMAL`, exploration resumes. People found while cut off are reported now |

If the link drops during TRANSIT, the A1 does not retrace; it just stops where it is.

```{graphviz} ../../diagrams/return_states.dot
```

## Option 1: automatic (script)

`COMM_OFF_AT` is **how many sim seconds after exploration starts to cut the link**, and `COMM_OFF_FOR` is **how many seconds later to restore it**.

```bash
cd ~/ros2_hw/ws && source install/setup.bash
COMM_OFF_AT=80 COMM_OFF_FOR=60 SETS=1 bash src/comm_relay/tools/run_mission.sh
```

This sends `0` 80 s into set 1 exploration, `1` 60 s later, and stops after set 1. The waits are in sim time (`sim_sleep.py`), not wall-clock time, so the timing holds even when the real-time factor changes.

## Option 2: by hand

Start things as in {doc}`gui_manual_start`, then send these once the A1s are inside their buildings.

```bash
ros2 service call /husky/set_comm mission_interfaces/srv/SetComm "{enabled: false}"
# ... watch for 30-60 s
ros2 service call /husky/set_comm mission_interfaces/srv/SetComm "{enabled: true}"
```

## How to check it

Watch the `comm`, `a1_1_comm` and `a1_2_comm` columns of the status line.

```text
t    phase    set  comm  a1_1       a1_2       a1_1_comm  a1_2_comm
133  EXPLORE  0    1     FOLLOWING  FOLLOWING  NORMAL     NORMAL
144  EXPLORE  0    0     PAUSED     PAUSED     RETURNING  RETURNING
...
184  EXPLORE  0    0     PAUSED     PAUSED     RETURNING  RETURNING
194  EXPLORE  0    0     PAUSED     PAUSED     HOLD       RETURNING
204  EXPLORE  0    1     TURNING    FOLLOWING  NORMAL     NORMAL
```

(These lines are from `timeline.csv` of the passing run `20261114_062151_set0`.) While cut off, the executor (`a1_1`, `a1_2` columns) is `PAUSED` and holds its exploration goal, while the return node drives the robot in `RETURNING`. When the link comes back, the executor restarts exploration right away.

`mission_events.txt` records the transitions as `[COMM]` lines. More detail from the node is on the `/<ns>/return_events` topic.

```bash
ros2 topic echo /a1_1/return_events
```

```{screenshot} comm_loss_return.png
RViz right after comm 0: the executor path disappears and the A1 retraces its path back to the explore start (in front of the door).
```

## A real example

In the passing run `20261114_062151_set0` the link was cut from 144 to 194 s during set 1 exploration.

- Both A1s switched to `RETURNING` **1.1 s later** and the Husky stopped.
- `a1_1` reached home 50 s later and went to `HOLD`. But it stopped in front of the door, **0.85 m** from the explore start — retracing uses odom, so it is off by however much odom drifted.
- After `1` it went `STOPPING` (1 s) → `NORMAL`, resumed exploring, and set 1 found 2/2 people.

## Good to know

- `return_relay` drives the breadcrumbs backwards **directly** (0.15 m/s, turning in place first when needed). If lidar points show up in the strip along the path to retrace (±0.15 m wide, 0.5 m ahead) in both of the last two scans (3 or more), it stops.
- The heartbeat timeout is **wall-clock** 1 s (not sim time).
- The principles are in {doc}`../concepts/communication_return`.
