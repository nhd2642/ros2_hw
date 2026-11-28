# 6. Change the World Layout

To change walls, boxes, ramps or people inside the buildings, edit **only `make_world.py`**. It regenerates both the world file and the mission config, so there are no coordinates in node code.

```text
disaster_sim/scripts/make_world.py   ──►  disaster_sim/worlds/mission.world   (Gazebo world)
      (the single layout source)     └──►  disaster_sim/config/mission.yaml   (spawn, follow slots, per-set parking/zones/people, person truth)
```

:::{warning}
**Don't edit** `mission.world` or `mission.yaml` by hand. The next run of `make_world.py` overwrites them.
:::

## Coordinate conventions

- World coordinates (m), **road = x axis** (3 m wide). The Husky starts at (0, 0).
- Set 1 = x 4-12, set 2 = x 16-24, set 3 = x 28-36, set 4 = x 40-48, common building = x 52-66, y −6-6.
- Each set has an 8 × 5 m building pair either side of the road (south = `a1_1`, north = `a1_2`), with a 1.6 m door centered on the road side.
- Interiors use **building coordinates (u, v)**: u = along the road (door center 0, −4-4), v = inward from the door (0-5). South buildings are mirrored in y.

## Writing a layout

Add entries per building name (`n1` = set 1 north, `s1` = set 1 south, …) to the `INTERIORS` dict.

```python
INTERIORS = {
    'n1': dict(walls=[(-4, 2.6, -1.4, 2.6), (0.2, 2.6, 4, 2.6)],   # (u1, v1, u2, v2) wall segments
               boxes=[(-3.6, 1.3, 0.6, 0.6)],                       # (u, v, width, depth) boxes
               people=[(2.6, 4.6)]),                                # (u, v) people
    's1': dict(walls=[(0.9, 1.8, 0.9, 5)],
               boxes=[(-3.6, 4.6, 0.6, 0.6)],
               ramps=[(-1.55, 2.9, 'v', 2.37)],                     # (crest u, v, direction 'u'|'v', half width)
               people=[(2.5, 4.6)]),
    ...
}
```

`pillars=[(u, v)]` adds pillars.

## Layout rules you must keep

These rules came out of testing. Break them and A1s get stuck or fail to follow paths (reasons in {doc}`../concepts/world_layout`).

:::{important}
1. Corridors and rooms people walk through: **at least 2.0 m wide**. 1.6 m only for doors and openings passed straight through.
2. Openings that need a turn of more than 90° right after them (a room entrance around a wall end): **at least 2.4 m**.
3. Obstacles and people: **against a wall** (0.4 m from the wall center) **or with 1.3 m clear all around**.
4. Ramps: an up-and-down hump (8°) covering **the full room width wall to wall**, no side edges or rails.
5. Obstacles: **at least 0.5 m tall** (the lidar plane is at 0.30 m; anything lower is invisible to the 2D lidar).
:::

## Regenerating

```bash
cd ~/ros2_hw/ws/src/disaster_sim/scripts
python3 make_world.py                 # rewrites mission.world and mission.yaml
python3 make_world.py layout.png      # + a top-down layout image
```

`layout.png` has the same format as the layout figure in these docs (`docs/img/mission_layout.png`): red star = person, blue square = Husky parking spot, grey = ramp, dashed = zone.

![Layout](../../images/mission_layout.png)

If you built with `--symlink-install`, you can start the sim right away without rebuilding.

```bash
SETS=1 bash ~/ros2_hw/ws/src/comm_relay/tools/run_mission.sh 0     # check only the changed set 1
```

## The original map

The single-building map of the previous architecture can be started as a world only (it was not run with the current mission structure).

```bash
ros2 launch disaster_sim world.launch.py world:=disaster.world
```
