# Mission interfaces (mission_interfaces)

The **user-facing interfaces** of the survivor search service are defined by the team in the package `mission_interfaces`.
This page alone is meant to be enough to request a mission, watch its progress and get the result.

| Kind | Name | Type | Provided by | What it does |
| --- | --- | --- | --- | --- |
| action | `/husky/run_mission` | `mission_interfaces/action/RunMission` | husky_mission | mission request → progress feedback → survivor list result. Cancellable |
| service | `/husky/set_comm` | `mission_interfaces/srv/SetComm` | husky_mission | turn the Husky-A1 link on/off (comm-loss test) |
| service | `/husky/get_survivors` | `mission_interfaces/srv/GetSurvivors` | husky_mission | query survivors confirmed so far (all or per set) |
| topic | `/mission/state` | `mission_interfaces/msg/MissionState` | husky_mission | mission phase, set, zones, comm (1 Hz, latched) |
| topic | `/husky/survivor_report` | `mission_interfaces/msg/SurvivorReport` | survivor_detector (A1) | one survivor found |
| topic | `/husky/survivors` | `mission_interfaces/msg/SurvivorList` | husky_mission | confirmed survivor list (latched) |

Parts that are hard to build ourselves, such as path planning and following, use open source: `nav_executor` calls the Nav2
actions `ComputePathToPose` and `FollowPath` and the service `GetCostmap` ({doc}`../concepts/exploration`).

## Flow

```text
user ──RunMission goal {num_sets}──▶ husky_mission ──/mission/state (TRANSIT, EXPLORE, ...)──▶ A1 nodes
  ▲                                     │  ▲
  │◀──feedback 1 Hz {state, found, expected}  └── /husky/survivor_report ◀── survivor_detector (A1 camera)
  │◀──result {success, survivors, ...}──┘
  ├──SetComm {enabled}──▶ husky_mission (comm 0/1: A1s return, Husky stops)
  └──GetSurvivors {set_name}──▶ husky_mission (people found so far)
```

1. Start the simulation and robot nodes ({doc}`../tutorials/first_mission`). `husky_mission` waits in `IDLE`.
2. Sending a `RunMission` goal runs, per set, `TRANSIT` (Husky drives, A1s follow) → `EXPLORE` (both A1s explore their zones) → next set once both are back at their exploration start.
3. When an A1 finds a person a `SurvivorReport` arrives; `husky_mission` merges reports within 1.5 m into one person and publishes a `SurvivorList`.
4. After the requested sets the phase becomes `DONE` and the action result carries the survivor list.

## action: RunMission

```text
# goal
uint32 num_sets                 # number of sets to run (0 = from the start set to the end). The start set is chosen when the sim is launched (start_set)
---
# result
bool success                    # all requested sets finished (false if cancelled or stopped)
string message
mission_interfaces/SurvivorList survivors
uint32 found                    # survivors found
uint32 expected                 # configured survivors in the finished sets
float32 duration                # mission duration, sim seconds
---
# feedback (every 1 s)
mission_interfaces/MissionState state
uint32 found
uint32 expected                 # configured survivors in the sets started so far
```

- Accepted only once, in `IDLE`. Requests while running or after `DONE` are rejected; relaunch the sim to run again.
- Cancelling ends the mission on the spot (`DONE`, robots stop exploring); the goal status is `CANCELED` and `success` = false.

```bash
ros2 action send_goal --feedback /husky/run_mission mission_interfaces/action/RunMission "{num_sets: 0}"
```

## service: SetComm

```text
bool enabled                    # true = comm on (default), false = off
---
bool success
string message
```

Turning it off stops the A1 heartbeat (`/<ns>/comm_status`). After 1 s the A1s walk back to this set's exploration start and the
Husky stops. Turning it on resumes the phase; people found while cut off are reported then ({doc}`../tutorials/comm_loss`).

```bash
ros2 service call /husky/set_comm mission_interfaces/srv/SetComm "{enabled: false}"
ros2 service call /husky/set_comm mission_interfaces/srv/SetComm "{enabled: true}"
```

## service: GetSurvivors

```text
string set_name                 # empty = all, a set name (set1 ...) = only inside that set's zones
---
bool success                    # false for an unknown set name
string message
mission_interfaces/SurvivorList survivors
uint32 expected                 # configured survivors in that scope
```

```bash
ros2 service call /husky/get_survivors mission_interfaces/srv/GetSurvivors "{set_name: set2}"
```

## msg: MissionState (`/mission/state`)

```text
string phase                    # IDLE / TRANSIT / EXPLORE / REGROUP / DONE (also defined as PHASE_* constants)
int32 set_index                 # current set number (order of sets in mission.yaml, from 0)
string set_name                 # set1 / set2 / set3 / set4 / common
mission_interfaces/Zone[] zones # filled only in EXPLORE and REGROUP
bool comm                       # Husky-A1 link
```

`Zone` = `string robot`, `float64 x_min, x_max, y_min, y_max` (world = `shared_map` coordinates, m): the rectangle each robot explores.
The A1-side nodes (`a1_follower`, `nav_executor`, `return_relay`, `frontier_detector`, `frontier_comm_filter`, `goal_assigner`, `terrain_node`)
decide what to do from this message alone. Phase meanings: {doc}`../concepts/mission_state_machine`.

## msg: SurvivorReport (`/husky/survivor_report`)

```text
std_msgs/Header header          # frame_id = map (that robot's TF tree map, origin = its spawn, axes = world)
string robot                    # a1_1 / a1_2
uint32 id                       # survivor number within that robot
geometry_msgs/Point position    # survivor position (m)
float32 score                   # detection score 0-1
float32 delay                   # sim seconds from detection to report (grows while the link is down)
```

## msg: SurvivorList (`/husky/survivors`, service and action results)

```text
std_msgs/Header header          # frame_id = world
geometry_msgs/Point[] people    # survivor positions (world, m)
```

## Using it from Python

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mission_interfaces.action import RunMission

rclpy.init()
node = Node('mission_client')
client = ActionClient(node, RunMission, '/husky/run_mission')
client.wait_for_server()
goal = client.send_goal_async(RunMission.Goal(num_sets=0),
                              feedback_callback=lambda f: print(f.feedback.state.phase, f.feedback.found))
rclpy.spin_until_future_complete(node, goal)
result = goal.result().get_result_async()
rclpy.spin_until_future_complete(node, result)
print([(p.x, p.y) for p in result.result().result.survivors.people])
```

`run_mission.sh` starts the mission with the same action and saves feedback and result to `action.log` in the result folder.
The comm test (`COMM_OFF_AT`) has `mission_watch.py` call the `SetComm` service.
