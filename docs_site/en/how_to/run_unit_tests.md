# Run the Unit Tests

Decision logic lives in separate modules so it can be tested without ROS. Always run these before starting the sim (under a minute).

## Run

```bash
source /opt/ros/humble/setup.bash          # test_turn imports nav_executor (rclpy)
cd ~/ros2_hw/ws/src/comm_relay && python3 -m pytest -q test
cd ~/ros2_hw/ws/src/terrain_perception && python3 -m pytest -q test
```

Expected (checked 2026-11-17):

```text
64 passed in 0.79s      # comm_relay
18 passed in 3.42s      # terrain_perception
```

One file or test only:

```bash
python3 -m pytest -q test/test_drive.py
python3 -m pytest -q test/test_logic.py -k return
```

## What is tested

| Test file | Module | Tests | Checks |
| --- | --- | --- | --- |
| `comm_relay/test/test_drive.py` | `drive_logic` | 10 | obstacle-avoiding steering, finding the Husky body, box fit |
| `comm_relay/test/test_logic.py` | `return_logic`, `arbiter_logic` | 20 | return state transitions, breadcrumb retracing, priority arbitration |
| `comm_relay/test/test_mission_logic.py` | `mission_logic` | 7 | done detection, zone and path checks, people tally, start poses, goal refinement |
| `comm_relay/test/test_nudge.py` | `nudge_logic` | 11 | plan-failure streaks, nudge direction, zero-command watch |
| `comm_relay/test/test_survivor.py` | `survivor_logic` | 9 | person position, merging the same person, report queue |
| `comm_relay/test/test_turn.py` | `nav_executor` | 3 | path heading ahead, angle difference (used for turn-first) |
| `comm_relay/test/test_costmap_diag.py` | `tools/costmap_diag` | 4 | costmap diagnostic tool |
| `terrain_perception/test/test_grid.py` | `grid` | 10 | step/slope hazard cells, memory |
| `terrain_perception/test/test_nav2_patch.py` | `nav2_patch` | 6 | patch that adds the terrain layer to the Nav2 launch |
| `terrain_perception/test/test_on_ramp.py` | `grid` | 2 | ground is not a hazard while standing on a ramp |

## Adding new logic

Rather than writing decisions directly into ROS nodes (`*_node.py`, `husky_mission.py` …), follow the current pattern: put pure functions and classes in `*_logic.py` and keep the node to message conversion. Then you can test it without the sim, as above.
