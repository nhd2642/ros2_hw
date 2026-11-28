# 4. Run a Single Set

A full mission takes about 50 minutes. When checking a node you changed, running **only the set you need** is much faster.

## Two knobs

`start_set` (first argument of the script)
: which set to start from. `0` = the beginning (start area), `1` = just before set 2, `2` = just before set 3, `3` = just before set 4, `4` = just before the common building. With 1 or more, it starts as if the earlier sets were done: **the Husky at the previous set's parking spot and the A1s in their slots behind it**.

`SETS` (environment variable)
: how many sets to do before stopping. Default: all.

| Want | Command |
| --- | --- |
| set 1 only | `SETS=1 bash run_mission.sh` |
| set 2 only | `SETS=1 bash run_mission.sh 1` |
| set 3 to the end | `bash run_mission.sh 2` |
| set 4 only (the hard scene) | `SETS=1 bash run_mission.sh 3` |
| common building only | `SETS=1 bash run_mission.sh 4` |

(Run from `~/ros2_hw/ws/src/comm_relay/tools/`, or add the path.)

## Example: common building only

```bash
cd ~/ros2_hw/ws && source install/setup.bash
SETS=1 bash src/comm_relay/tools/run_mission.sh 4
```

The result folder name ends in `..._set4`. In the common building both A1s split **one shared zone** (14 × 12 m), which makes it a good place to see them cooperate.

```{screenshot} common_building.png
Exploring the common building: the two A1s split north and south and head for different frontiers.
```

## Time limit

`DUR` is a limit in sim seconds (default 6000). Use it to cut a run that drags on.

```bash
DUR=900 SETS=1 bash src/comm_relay/tools/run_mission.sh 1
```

## Useful combinations

```bash
# comm test in set 2
COMM_OFF_AT=60 COMM_OFF_FOR=45 SETS=1 bash src/comm_relay/tools/run_mission.sh 1
# common building with the GUI
GUI=1 SETS=1 bash src/comm_relay/tools/run_mission.sh 4
```

:::{note}
`start_set` must be passed with the same value to both `sim.launch.py` and `mission.launch.py`. The script does this for you; when starting by hand, give `start_set:=k` to both launch files.
:::
