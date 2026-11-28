# Troubleshooting

Symptoms, where to look, and what to do.

## Startup

Nav2 servers don't come up (`실패: Nav2 서버 4 개 active` — "failed: 4 Nav2 servers active")
: An occasional startup fault (apparently a lifecycle response timeout; cause not checked). `run_mission.sh` automatically restarts only Nav2 up to twice, then the whole stack once. If it still fails, run `bash run_mission.sh clean` and try again.

A1 controllers never activate (`실패: 두 A1 컨트롤러 활성`)
: Check `sim.log`. A leftover Gazebo from a previous run conflicts → `bash run_mission.sh clean`. If the error says `ign_ros2_control` is missing, redo step 2 of {doc}`../getting_started/installation`.

Two runs at the same time
: They get different `ROS_DOMAIN_ID`s, but the CPU runs short and both break. Run one at a time.

## Sim performance

RTF below 0.2
: Turn off the GUI (`GUI=1`, RViz). Visualization has dropped the RTF from 0.6 to the 0.2 range. Python nodes must be remapped to `/clock_slow` (50 Hz) — if you added a node, use the `node()` helper in `mission.launch.py`.

Every lidar value is 0.15 m (range_min)
: A GPU lidar range problem. The sim server must run with software rendering (`software_rendering:=true`, the default). `tools/lidar_check.sh` compares the values across rendering modes.

## Mission progress

The Husky doesn't leave
: the mission request may not have arrived. Check the phase with `ros2 topic echo /mission/state` and the result folder's `action.log`. If it is `IDLE`, send the `RunMission` goal again ({doc}`../reference/interfaces`). Make sure `ROS_DOMAIN_ID` matches the script.

The Husky stays in HOLD
: A following A1 is more than 2.5 m behind. Check `/<ns>/slot_error`. The A1 may have fallen (`FALLEN`) or be blocked.

One A1 doesn't leave
: It once mistook the small wall at the start for the Husky (now clusters more than 0.7 m from the received Husky pose are discarded). If you changed the world, make sure nothing of similar size sits near the Husky.

A set never finishes
: Usually a frontier that never clears gets revisited. Three arrivals at the same spot ends it for that set. Count `실패` (failure) and `도착` (arrived) per robot in `mission.log`. After 1,500 s it moves to REGROUP.

An A1 enters a room and can't get out
: Usually a narrow entrance. Entrances around a wall end must be at least 2.4 m ({doc}`../concepts/world_layout`).

## Position and map

An A1 walks into the next building's door
: The SLAM map drifted more than 1 m. Check `mission.log` for the Husky-referenced correction on set arrival (`/<ns>/pose_fix`). The odom scale (`odom_scaler`) may also be off ({doc}`measure_gait`).

Position suddenly jumps 2 m in a tight spot
: slam_toolbox loop closure was the suspected cause, so `do_loop_closing: false`. Don't turn it on.

An A1 walks in place and its SLAM position drifts
: It is stuck and only its legs move. Leg odom keeps advancing and SLAM follows. Beyond removing trap features (rails, narrow gaps) from the layout there is no real fix.

## Person detection

No people found at all
: Look for a model warning in `mission.log`. `ws/person_ai_models/yolov4-tiny/` must contain `yolov4-tiny.cfg` and `yolov4-tiny.weights` and `python3 -c "import cv2"` must work.

More reports than people
: Positions more than 1 m off, or low-score false positives. Raising `conf` reduces them but may miss people ({doc}`tune_parameters`).

## Cleanup

Kill everything left over
: ```bash
  bash ~/ros2_hw/ws/src/comm_relay/tools/run_mission.sh clean
  ```
  Cleans up Gazebo, CHAMP, Nav2, SLAM, mission nodes, shared memory (`/dev/shm/fastrtps_*`) and the ROS 2 daemon.
