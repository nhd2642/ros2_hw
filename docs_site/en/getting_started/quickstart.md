# Quickstart: Your First Mission

Once {doc}`installation` is done, one command runs **everything from the simulator to the end of the mission**.

## One command

```bash
cd ~/ros2_hw/ws && source install/setup.bash
bash src/comm_relay/tools/run_mission.sh
```

`run_mission.sh` starts everything in this order and moves on as each part becomes ready.

```{graphviz} ../../diagrams/launch_order.dot
```

| Step | What it does | Waits for |
| --- | --- | --- |
| 1 | World + Husky + two A1s (`sim.launch.py`) | both A1 joint controllers active (up to 180 s) |
| 2 | Per-robot SLAM (`dual_slam.launch.py`) | 5 s |
| 3 | Nav2 planning and following servers | 4 servers `active` (if not within 60 s, only Nav2 is restarted, up to twice) |
| 4 | Terrain perception (`terrain.launch.py`) | together with step 3 |
| 5 | Mission layer (`mission.launch.py`) | 10 s |
| 6 | Requests the mission: action `/husky/run_mission` (`num_sets` = `SETS`, 0 = to the end) | — |

Then `mission_watch.py` prints **one status line every 10 s of sim time**.

```text
t,phase,set,comm,husky_x,husky_y,a1_1_x,a1_1_y,a1_2_x,a1_2_y,husky,a1_1,a1_2,...
21 TRANSIT 0 1 0.79 -0.00 -1.54 -1.37 -1.21 0.82 GO IDLE IDLE NORMAL NORMAL 0 0.05 0.07
31 TRANSIT 0 1 3.29 -0.00  1.46 -0.80  1.47 0.81 GO IDLE IDLE NORMAL NORMAL 0 0.05 0.04
...
```

:::{important}
A full mission takes **about 50 minutes of real time** (about 900-1,050 s of sim time, real-time factor around 0.3, on the development PC). For a first try, run **just the common building** as in {doc}`../tutorials/partial_sets`, which is much shorter.

```bash
SETS=1 bash src/comm_relay/tools/run_mission.sh 4
```
:::

## Watching it

By default it runs headless. Add `GUI=1` to open the Gazebo window.

```bash
GUI=1 bash src/comm_relay/tools/run_mission.sh
```

```{screenshot} start_formation.png
Gazebo with GUI=1: the Husky at the start and the two A1s behind it (including the small wall in front of a1_1).
```

To see map, lidar, camera and path in RViz, open one RViz per robot. Each robot has its own TF on `/<ns>/tf`, so the repository template `tools/vis.rviz` has its `/NS/` replaced by the robot name.

```bash
T=~/ros2_hw/ws/src/comm_relay/tools
sed "s#/NS/#/a1_1/#g; s#XPOS#0#" $T/vis.rviz > /tmp/vis_a1_1.rviz
rviz2 -d /tmp/vis_a1_1.rviz --ros-args -r /tf:=/a1_1/tf -r /tf_static:=/a1_1/tf_static -p use_sim_time:=true
```

That view shows the robot's SLAM map (`/a1_1/map`), lidar, robot model, executor path (`/a1_1/nav_executor/path`), person markers (`/a1_1/survivor_markers`) and the front camera image (Fixed Frame `map`). Start `a1_2` the same way.

:::{note}
`run_mission.sh` picks a random `ROS_DOMAIN_ID` (20-79) for each run. To use `ros2 topic` or RViz from another terminal, match the value the script printed at the start.

```bash
export ROS_DOMAIN_ID=<printed value>
```
:::

## Checking the result

At the end you get a summary like this (`20261128_005121_set0_r1full_1`).

```text
=== 요약
  TRANSIT  세트 0:      0 ->     39 s (   39 s)
  EXPLORE  세트 0:     39 ->    111 s (   72 s)
  ...
  사람 보고 11 건, 정답 11 명 중 1.0 m 안 일치 11 명
  넘어짐: 없음
RESULT PASS (마지막 단계 DONE 세트 5, 한도 초과 세트 0, 사람 11/11, 홈 실패 0)
```

(The tool prints Korean: `요약` = summary, `세트` = set, `사람 보고 11 건 … 일치 11 명` = 11 reports, 11 matched within 1 m, `넘어짐: 없음` = no falls, `홈 실패` = return failures.)

Results go to `~/ros2_hw/results/<date_time>_set<k>/`. The action result (survivor positions) is in `action.log`. See {doc}`../tutorials/reading_results` for what is inside.

## Stopping early

`Ctrl+C` stops it and the script cleans up every process it started. If anything is left over:

```bash
bash src/comm_relay/tools/run_mission.sh clean
```

## Next steps

- {doc}`../reference/interfaces` — request a mission, cut the link and query survivors yourself
- {doc}`../tutorials/gui_manual_start` — start it yourself while watching
- {doc}`../tutorials/comm_loss` — test a comm loss and the return behavior
- {doc}`../concepts/architecture` — how everything you just started connects
