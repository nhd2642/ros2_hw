# Mission State Machine

The mission is driven by a single node, `husky_mission`. This page explains the phases, the transition conditions, what each robot does per phase, and the comm rules.

```{graphviz} ../../diagrams/mission_states.dot
```

## Sets

The set order is the order of `sets` in `mission.yaml`: `set1`, `set2`, `set3`, `common`. The set number `k` in code and logs counts from 0 (0 = set 1, 3 = common building).

Each set has a Husky parking spot (`park`), per-robot exploration zone rectangles (`zones`), and a person count (`people`).

## What happens in each phase

| Phase | Husky | A1 | Ends when |
| --- | --- | --- | --- |
| **IDLE** | stopped | stopped | user `start` |
| **TRANSIT k** | drives to set k parking spot. Waits if an A1 lags more than 2.5 m behind its slot and goes again once it is within 1.5 m | follows its slot behind the Husky (1.6 m back, 0.8 m to the side) | Husky arrived + both A1s within 0.6 m of their slots (or 60 s after Husky arrival) |
| **EXPLORE k** | parked; tallies person reports | explores frontiers in its zone. After 10 s with no reachable frontier returns to the explore start → `set_done` | both A1s report `set_done = k` |
| **REGROUP k** | parked | abandons remaining exploration and returns to the explore start | both back, or 300 s |
| **DONE** | stopped | stopped | — |

If EXPLORE exceeds `explore_timeout` (1,500 s) it becomes REGROUP. A single REGROUP makes the run a FAIL.

## Explore start (home)

2 s after TRANSIT ends and EXPLORE begins, each A1's position at that moment is recorded as **this set's home** and sent down on `/<ns>/home_shared`. When an A1 finishes exploring it returns to this home, not to its spawn. So the next TRANSIT always starts from the road.

## Set completion rules

- Set done = **both A1s** back home (`/<ns>/set_done`).
- If fewer people were found than configured, it records "n not found" and moves on. If the map is complete there is nowhere left to look, so it doesn't wait.
- If an A1 fails to return after 8 attempts, it reports the set done from where it is. The Husky then waits for that A1 during the next TRANSIT.

## Common building

In the common building (`common`) both A1s get **the same rectangle** (x 40-54, y −6-6).

- Frontiers come from the merged map, so anything one robot has seen drops out of the other's candidates too.
- Goal assignment penalizes two goals closer than 3 m, so the robots pick frontiers far apart.
- When the start position is 4 m from the door, the map beyond the door is sparse and no goal appears. Then the robot first approaches the "reachable cell nearest the frontier" (approach goal, only while the robot is outside its zone).

## User requests

Sent as an action or services (types and examples in {doc}`../reference/interfaces`). Typing `start` / `0` / `1` into the `husky_mission` terminal does the same.

| Request | Effect |
| --- | --- |
| action `/husky/run_mission` (`num_sets`) | start the mission (IDLE → TRANSIT). DONE after `num_sets` sets, result = survivor list. Cancelling ends it on the spot (DONE) |
| service `/husky/set_comm` `enabled: false` | comm off: stop the heartbeat. A1s return to this set's home (in TRANSIT they just stop), the Husky stops |
| service `/husky/set_comm` `enabled: true` | comm on (default): resume the current phase. People found while cut off are reported now |
| service `/husky/get_survivors` | survivors confirmed so far (all or per set) |

## Comm heartbeat

While comm is on, `husky_mission` keeps publishing `/<ns>/comm_status` (Bool) for each robot. That stream stopping *is* the link loss. Choosing user control instead of computing the link from distance or walls was a project goal (fixed 2026-11-09). The robot-side reaction is in {doc}`communication_return`.

## /mission/state

A `mission_interfaces/msg/MissionState` (latched) published at 1 Hz. Other nodes learn the phase only from this.

```text
phase: EXPLORE
set_index: 1
set_name: set2
zones:
- {robot: a1_2, x_min: 16.0, x_max: 24.0, y_min: 1.5, y_max: 6.5}
- {robot: a1_1, x_min: 16.0, x_max: 24.0, y_min: -6.5, y_max: -1.5}
comm: true
```

| Field | Meaning |
| --- | --- |
| `phase` | `IDLE` `TRANSIT` `EXPLORE` `REGROUP` `DONE` |
| `set_index`, `set_name` | set number (from 0) and name |
| `zones` | per-robot zone (world m). Filled only in EXPLORE and REGROUP, otherwise an empty array |
| `comm` | comm on true / off false |

In the same cycle it also publishes `/husky/goal` (parking spot, `world`) during TRANSIT, and `/<ns>/home_shared` (home, `shared_map`) during EXPLORE and REGROUP. Home is the A1 position 2 s after EXPLORE begins.
