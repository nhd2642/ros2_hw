# Service Definition and Real-World Design

This page describes the proposed service **as if it were deployed at a real site**, and for each element lists what the simulation uses in its place and what a real system would need.

:::{important}
The "real deployment" columns are **design assumptions**. Nothing was tested on physical robots. Only the Gazebo Sim runs were tested ({doc}`../results/test_results`).
:::

## 1. What the service does

**Survivor search at a disaster site.** A rescue commander requests a mission; one wheeled robot (Husky) and two quadrupeds (A1) go along the road building by building, explore inside, and report where survivors are. The goal is to know **which building has people, and where,** before rescuers go in.

| Item | Content |
| --- | --- |
| User | rescue commander (requests and monitors from outside, on a laptop) |
| Input | site setup: Husky parking spot per building set, exploration zone per robot, expected number of people (`mission.yaml`) |
| Requests | start and cancel a mission, comm-loss test, survivor query ({doc}`../reference/interfaces`) |
| Output | list of survivor positions (meters from the start point), mission phase, per-robot maps |
| Roles | Husky = transport, command, tally; A1 = explore inside buildings and find people |

## 2. Operating procedure

| Step | Operator | System |
| --- | --- | --- |
| 1. Site setup | writes parking spots, zones and expected counts into `mission.yaml` from floor-plan information | — |
| 2. Place robots | puts the Husky on the reference point and the two A1s 1.6 m behind, 0.8 m to each side, same heading | — |
| 3. Bring up | starts the system | `husky_mission` waits in `IDLE` |
| 4. Request | sends a `RunMission` goal (`num_sets`) | per set: drive → explore → return |
| 5. Monitor | watches feedback (phase, count); checks positions with `GetSurvivors` | stops 1 s and reports when it sees a person |
| 6. Result | receives the survivor list in the action result | `DONE` |
| 7. Judge | **checks positions to sort out duplicates and misses** (section 5) | — |

To stop midway, cancel the action. To drive one A1 by hand, `/<ns>/cmd_vel_teleop` overrides every automatic command ({doc}`../how_to/teleop`).

In the simulator, `run_mission.sh` performs steps 3 and 4 ({doc}`../getting_started/quickstart`).

## 3. Elements

### Hardware

| Element | In simulation | Real deployment assumption |
| --- | --- | --- |
| Transport and command robot | Husky A200 model (Gazebo DiffDrive), 2D lidar, IMU | a physical A200 with the same sensors; `/husky/cmd_vel` wired to its drive |
| Search robots | two Unitree A1 models, CHAMP gait | physical A1s; the gait settings (`gait.yaml`) were tuned in simulation and need re-tuning |
| Range sensor | planar lidar 360°, 10 Hz, 10 m (about 0.30 m high) | a 2D lidar of similar spec; lower or higher obstacles need something else |
| Camera | front RGB 640×480, 103° horizontal, 2 Hz | RGB camera; dark places need lighting or thermal |
| Depth sensor | terrain depth cloud, 5 Hz | depth camera |
| Compute | everything on one PC (including the simulator) | one onboard computer per robot (table below) |

### Where the software runs (proposal)

| Location | Nodes | Note |
| --- | --- | --- |
| each A1 | gait control, EKF, `scan_ready_relay`, slam_toolbox, Nav2, `terrain_node`, `nav_executor`, `a1_follower`, `return_relay`, `cmd_vel_arbiter`, `survivor_detector` | what must keep working to stop and return when the link drops. Today several of these handle both robots in one process; splitting them with the `robots` parameter was not tested |
| Husky | `husky_mission`, `husky_driver`, `nav_goal_bridge`, `frontier_detector`, `frontier_comm_filter`, `goal_assigner` | the side that merges both maps and splits goals |
| commander laptop | calls `RunMission`, `SetComm`, `GetSurvivors`; display | no dedicated UI was built; `ros2` command line and RViz are used |

### Communication

| Element | In simulation | Real deployment assumption |
| --- | --- | --- |
| Link | ROS 2 (DDS) inside one PC | Wi-Fi or mesh radio; relays may be needed inside buildings |
| Loss detection | no `/<ns>/comm_status` heartbeat for 1 s (wall clock) | usable as is. Loss by distance or walls is not modeled; `SetComm` only imitates it |
| On loss | A1s retrace their path to the exploration start, Husky stops, findings are held and sent later | retracing uses only the robot's own odometry. Hold-and-send was confirmed once (one report) in the simulator |
| Data crossing the link | robot maps, poses, goals, phase, person reports | maps (`OccupancyGrid`) are the largest; bandwidth was not measured |
| Security | none | authentication and encryption (e.g. SROS 2) needed |

### Localization and coordinates

| Element | In simulation | Real deployment assumption |
| --- | --- | --- |
| Reference | start poses are known (`spawn` in `mission.yaml`) | survey the start formation and enter it |
| Husky pose | wheel speed × IMU heading | long distances need GNSS or map matching |
| A1 pose | per-robot SLAM, corrected each set against the Husky seen by lidar | same method; the A1 must be able to see the Husky |
| Result coordinates | x, y from the start point (m) | converting to real map coordinates (GNSS) was not built |

### Exploration and mobility

| Element | In simulation | Real deployment assumption |
| --- | --- | --- |
| Road driving | straight road, lidar obstacle avoidance only | bends and junctions need path planning |
| Building search | frontiers + Nav2 (open source) | same. Stairs, doors to open and multiple floors are not supported |
| Zones | axis-aligned rectangles | buildings at an angle need polygon zones |
| Terrain | low steps and 8° humps | rough terrain such as rubble piles was not tested |

### Survivor detection

| Element | In simulation | Real deployment assumption |
| --- | --- | --- |
| Detector | YOLOv4-tiny (COCO), standing mannequins | **people lying down or partly visible were not tested.** Retraining on disaster data, thermal and audio would be needed |
| Range | box height (assumes 2.0 m) + lidar | assumes a standing person, so it does not fit someone sitting or lying; use a depth camera |
| Confirmation | automatic report | a step where the commander confirms on video is needed (video streaming was not built) |

### Safety and power

| Element | What exists | Needed for deployment |
| --- | --- | --- |
| Velocity commands | only `cmd_vel_arbiter` writes to the robot; zero when input stops | hardware emergency stop |
| Manual control | teleop has top priority | wireless emergency stop |
| Falls | goal cancelled above 45° tilt | self-righting, alerts |
| Collisions | lidar and costmaps | **the A1s barely see each other** (section 5); rules for keeping distance from people |
| Power | not modeled | battery monitoring and return on low charge. Check that real batteries last the mission (about 15-18 minutes of sim time) |

## 4. When something goes wrong

| Situation | System behavior | Operator action |
| --- | --- | --- |
| Link lost | A1s return, Husky stops, resumes when restored | move relays |
| A set takes over 1,500 s | `REGROUP`: call the A1s back, go to the next set | record the set as incomplete |
| Path blocked | blacklist, escape, nudge; gives up the room if it keeps failing | check by hand if people are missing |
| A1 cannot return (8 attempts) | set done where it stands; Husky waits at the next drive | recover or drive the robot |
| Fewer people than expected | recorded as "n not found", mission goes on | decide whether to search again |

## 5. Known problems and what must be done before deployment

Known today (details in {doc}`../results/limitations`):

- **The report count can exceed the real number of people** (11-13 for 11 people in full runs). Judge by positions.
- **The two A1s can touch.** The lidar plane is above the other robot's body.
- **People outside the camera view are missed.** Exploration ends when the lidar map is complete.
- A narrow corridor can be closed off, giving up the inner room.

To be built before a real deployment:

1. Operator display (map, survivor positions, video check, emergency stop)
2. Per-robot onboard configuration and radio link tests
3. Conversion to real map coordinates
4. Disaster-condition person detection (lying people, thermal)
5. Battery monitoring and hardware emergency stop
6. Collision avoidance between the A1s
