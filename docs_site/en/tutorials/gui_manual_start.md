# 2. Start It by Hand with the GUI

This time you open the Gazebo window and, without the automatic `start`, **start the mission and control the link yourself**. There are two ways.

| Way | When |
| --- | --- |
| A. `run_mission.sh` + commands on a topic | simplest. The script starts things, you send commands |
| B. Run each step yourself in 6 terminals | when you are changing one node and testing it, or want to type into the `husky_mission` terminal |

## Way A: start with the script, command over a topic

```bash
cd ~/ros2_hw/ws && source install/setup.bash
GUI=1 START=0 bash src/comm_relay/tools/run_mission.sh
```

With `START=0` it only starts things up and waits. In another terminal, match the `ROS_DOMAIN_ID` and send the command.

```bash
source ~/ros2_hw/ws/install/setup.bash
export ROS_DOMAIN_ID=<value printed by the script>
ros2 action send_goal --feedback /husky/run_mission mission_interfaces/action/RunMission "{num_sets: 0}"
```

The link is turned off and on with a service (types in {doc}`../reference/interfaces`).

```bash
ros2 service call /husky/set_comm mission_interfaces/srv/SetComm "{enabled: false}"   # comm off
ros2 service call /husky/set_comm mission_interfaces/srv/SetComm "{enabled: true}"    # comm on
```

## Way B: six terminals

In each terminal run `source ~/ros2_hw/ws/install/setup.bash` first and keep `ROS_DOMAIN_ID` the same everywhere.

::::{tab-set}

:::{tab-item} 1. Sim
```bash
ros2 launch disaster_sim sim.launch.py
```
World + Husky + two A1s. Wait about 30 s for `a1_2` to stand up.
:::

:::{tab-item} 2. SLAM
```bash
ros2 launch a1_exploration dual_slam.launch.py
```
:::

:::{tab-item} 3. Nav2
```bash
ros2 launch a1_exploration nav2_planners.launch.py
# another terminal
ros2 launch a1_exploration nav2_controllers.launch.py
```
Wait until the log shows `Managed nodes are active`.
:::

:::{tab-item} 4. Terrain
```bash
ros2 launch terrain_perception terrain.launch.py
```
:::

:::{tab-item} 5. Mission layer
```bash
ros2 launch comm_relay mission.launch.py husky:=false
```
`husky:=false` starts everything except `husky_mission`.
:::

:::{tab-item} 6. Husky mission
```bash
ros2 run comm_relay husky_mission --ros-args \
  -p config:=$(ros2 pkg prefix disaster_sim)/share/disaster_sim/config/mission.yaml \
  -p use_sim_time:=true -r /clock:=/clock_slow
```
Type `start`, `0`, `1` in this terminal and press Enter.
:::
::::

```{screenshot} terminal_start.png
The husky_mission terminal right after typing start ([MISSION] TRANSIT set 1 …).
```

:::{warning}
`husky_mission` needs the path to `mission.yaml` in its `config` parameter. The launch file (`husky:=true`) passes it automatically; with `ros2 run`, pass it yourself as above. This path (typing into the terminal) was not run during development, so if it gives you trouble use Way A.
:::

## A tour of the Gazebo window

```{screenshot} gazebo_gui.png
The Gazebo window with GUI=1: entity tree on the left (husky, a1_1, a1_2, person_*), 3D view in the middle, real-time factor (RTF) at the bottom.
```

- An **RTF** (Real Time Factor) of 0.2-0.4 at the bottom is normal. The GUI makes it slower.
- Right-click `a1_1` in the entity tree → *Follow* to have the camera track it.
- Few runs were done with the GUI on. Use headless runs for real results.

## Looking inside with RViz

To see map, lidar, camera and path in RViz, open one RViz per robot. Each robot has its own TF on `/<ns>/tf`, so the repository template `tools/vis.rviz` has its `/NS/` replaced by the robot name.

```bash
T=~/ros2_hw/ws/src/comm_relay/tools
sed "s#/NS/#/a1_1/#g; s#XPOS#0#" $T/vis.rviz > /tmp/vis_a1_1.rviz
rviz2 -d /tmp/vis_a1_1.rviz --ros-args -r /tf:=/a1_1/tf -r /tf_static:=/a1_1/tf_static -p use_sim_time:=true
```

That view shows the robot's SLAM map (`/a1_1/map`), lidar, robot model, executor path (`/a1_1/nav_executor/path`), person markers (`/a1_1/survivor_markers`) and the front camera image (Fixed Frame `map`). Start `a1_2` the same way.

## Next

In {doc}`comm_loss` you send `0` during exploration and test the return behavior.
