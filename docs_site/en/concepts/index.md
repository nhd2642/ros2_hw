# Concepts

Concept pages explain **why things are built this way and how they work inside**. They focus on principles and design reasons, including the evidence behind decisions changed during testing, rather than commands.

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} Architecture
:link: architecture
:link-type: doc
How packages, layers, nodes and topics connect; startup order.
:::

:::{grid-item-card} Mission state machine
:link: mission_state_machine
:link-type: doc
IDLE → TRANSIT → EXPLORE → REGROUP → DONE, set completion and comm rules.
:::

:::{grid-item-card} World & layout rules
:link: world_layout
:link-type: doc
Road, sets, common building; layout rules from testing and why.
:::

:::{grid-item-card} Transit: Husky drive & A1 follow
:link: transit_following
:link-type: doc
Lidar steering, Husky pose estimation, following the Husky in view, pose correction.
:::

:::{grid-item-card} Exploration: SLAM to Nav2
:link: exploration
:link-type: doc
Map merge, frontiers, goal assignment, executor failure handling, Nav2 settings.
:::

:::{grid-item-card} Terrain perception
:link: terrain_perception
:link-type: doc
Finding steps and slopes with a depth sensor and feeding them to the costmap.
:::

:::{grid-item-card} Person detection
:link: survivor_detection
:link-type: doc
YOLOv4-tiny detection, locating with box height and lidar.
:::

:::{grid-item-card} Comm, return & arbitration
:link: communication_return
:link-type: doc
Heartbeat, breadcrumb return, velocity command priorities.
:::

:::{grid-item-card} Frames & time
:link: coordinates_time
:link-type: doc
world / map / odom, the shared map, sim time and /clock_slow.
:::

:::{grid-item-card} A1 locomotion (CHAMP)
:link: locomotion
:link-type: doc
CHAMP gait controller, sensors, gait settings.
:::
::::

```{toctree}
:hidden:
:maxdepth: 1

architecture
mission_state_machine
world_layout
transit_following
exploration
terrain_perception
survivor_detection
communication_return
coordinates_time
locomotion
```
