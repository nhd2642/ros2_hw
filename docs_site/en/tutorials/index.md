# Tutorials

Tutorials are **learn-by-doing** pages, start to finish. Go through them in order and you will have run a mission, controlled it, interpreted the results and changed the world.

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} 1. Run and watch a full mission
:link: first_mission
:link-type: doc
Run the mission with `run_mission.sh` and learn what to look for in each phase.
:::

:::{grid-item-card} 2. Start it by hand with the GUI
:link: gui_manual_start
:link-type: doc
Open Gazebo and RViz and type `start`, `0`, `1` into the `husky_mission` terminal.
:::

:::{grid-item-card} 3. Comm loss and return
:link: comm_loss
:link-type: doc
Cut the link during exploration and watch the A1s retrace their path.
:::

:::{grid-item-card} 4. Run a single set
:link: partial_sets
:link-type: doc
Use `start_set` and `SETS` to test only the set you care about.
:::

:::{grid-item-card} 5. Read a result folder
:link: reading_results
:link-type: doc
Read the summary, timeline, events and node logs, and plot them.
:::

:::{grid-item-card} 6. Change the world layout
:link: editing_world
:link-type: doc
Change building interiors with `make_world.py` and regenerate `mission.yaml`.
:::
::::

:::{tip}
Every tutorial assumes {doc}`../getting_started/installation` is done and you are in a terminal that ran `source ~/ros2_hw/ws/install/setup.bash`.
:::

```{toctree}
:hidden:
:maxdepth: 1

first_mission
gui_manual_start
comm_loss
partial_sets
reading_results
editing_world
```
