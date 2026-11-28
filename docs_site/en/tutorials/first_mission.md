# 1. Run and Watch a Full Mission

In this tutorial you run a full mission once and learn **what the robots do in each phase and where to look in the logs**.

:::{admonition} What you will learn
:class: tip
- The startup order of `run_mission.sh` and the result folder
- How to read one `mission_watch` status line
- What normal behavior looks like in TRANSIT, EXPLORE and DONE
:::

## Step 1: Run it

```bash
cd ~/ros2_hw/ws && source install/setup.bash
bash src/comm_relay/tools/run_mission.sh
```

The first lines are the startup.

```text
=== 임무 실행: start_set 0, ROS_DOMAIN_ID 62, 결과 /home/<you>/ros2_hw/results/20261128_005121_set0
두 A1 컨트롤러 활성: 24 s
Nav2 서버 4 개 active: 5 s
start 보냄
```

(`임무 실행` = mission run, `결과` = result folder, `두 A1 컨트롤러 활성` = both A1 controllers active, `start 보냄` = start sent.)

`start 보냄` means the script requested the mission with the action `/husky/run_mission`. To request it yourself, start with `START=0` ({doc}`gui_manual_start`).

:::{note}
Occasionally Nav2 fails to come up. The script then prints `-> Nav2 만 다시 띄움` ("restart only Nav2") up to twice, and after that restarts the whole stack once.
:::

## Step 2: Read the status line

After the mission request you get one line every 10 s of sim time. The columns mean:

| Column | Example | Meaning |
| --- | --- | --- |
| `t` | `371` | sim time since mission start (s) |
| `phase`, `set` | `EXPLORE 1` | mission phase and set number (0 = set 1, 3 = set 4, 4 = common building) |
| `comm` | `1` | comm on (1) / off (0) |
| `husky_x … a1_2_y` | | Gazebo ground-truth positions (for evaluation; the robots don't know these) |
| `husky` | `ARRIVED` | Husky drive state: `GO` `AVOID` `BLOCKED` `HOLD` `ARRIVED` |
| `a1_1`, `a1_2` | `FOLLOWING` | each A1's executor state (`IDLE` `PLANNING` `TURNING` `FOLLOWING` `ESCAPING` `PAUSED` `DONE` …) |
| `a1_1_comm` … | `NORMAL` | return node state: `NORMAL` `RETURNING` `HOLD` `STOPPING` |
| `people` | `2` | number of person reports the Husky received |
| `a1_1_slam_err` … | `0.27` | distance between the SLAM shared pose and ground truth (m) |

## Step 3: TRANSIT — driving down the road

```{screenshot} transit_follow.png
On the road: the Husky ahead and the two A1s walking in their slots behind and to each side (top-down Gazebo view).
```

Normal looks like:

- `husky` is `GO` and `husky_x` grows at about 0.25 m/s.
- The A1s walk about 1.6-1.9 m behind the Husky, near ±0.8 m sideways (`a1_1_y` ≈ −0.8, `a1_2_y` ≈ +0.8).
- If an A1 falls more than 2.5 m behind, the Husky waits in `HOLD`. Short appearances are normal.

It takes about 39 s to reach set 1 and about 49 s between sets (about 19 s before the common building).

## Step 4: EXPLORE — exploring the buildings

When the Husky is `ARRIVED` and both A1s are in their slots, `phase` changes to `EXPLORE`.

```{screenshot} explore_rviz.png
RViz (tools/vis.rviz set up for a1_1): SLAM map, lidar, executor path, person markers, front camera image.
```

Normal looks like:

- The A1 states cycle `PLANNING → TURNING → FOLLOWING → IDLE` (once per frontier reached).
- When a person is found, `people` goes up and a `[MISSION]` line with the report appears in the terminal (or `mission.log`).
- When there are no frontiers left, the A1 goes home and becomes `DONE`. When both are `DONE`, the team moves to the next set.

Exploring one set takes about 70-330 s depending on the run (7 runs of the current setup). Blocked goals go through failure handling (blacklist, escape, nudge) and add time — see {doc}`../concepts/exploration`.

## Step 5: DONE — the summary

After the common building, the summary and verdict are printed.

```text
RESULT PASS (마지막 단계 DONE 세트 5, 한도 초과 세트 0, 사람 11/11, 홈 실패 0)
결과 폴더: /home/<you>/ros2_hw/results/20261128_005121_set0
```

PASS needs five things: **reached the end**, **0 falls**, **0 exploration timeouts (REGROUP)**, **every person reported within 1.0 m of truth**, **0 return failures**. Duplicate reports do not count against it, so a passing run can report more than 11.

## Recap

- `run_mission.sh` handles the startup order and the waits.
- One status line shows phase, positions, each node's state and the SLAM error together.
- In the next tutorial, {doc}`gui_manual_start`, you open the GUI and send commands yourself.
