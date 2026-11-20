# World & Layout Rules

## Layout

![Mission layout](../../images/mission_layout.png)

| Element | Position / size |
| --- | --- |
| Road | along the x axis, 3 m wide (y −1.5-1.5) |
| Start | Husky (0, 0), `a1_1` (−2.4, −0.8), `a1_2` (−2.4, +0.8). A small wall (1 m long, 0.6 m tall) 1.2 m in front of `a1_1` |
| Sets 1·2·3·4 | x 4-12, 16-24, 28-36, 40-48. An 8 × 5 m building pair either side of the road (south = `a1_1`, north = `a1_2`), 1.6 m door centered on the road side |
| Set 4 (rule-breaking) | a test scene that deliberately breaks the layout rules below (2026-11-19). South: 1.2 m wide L-shaped corridor (two 90° turns) + 1.0 m door → person in the inner room. North: a room of pillars and boxes with 1.0-1.2 m gaps + 2 low debris pieces (0.12 m) below the lidar plane |
| Common building | x 52-66, y −6-6 (14 × 12 m), two doors on the west side (moved 12 m east with its interior unchanged when set 4 was added) |
| Parking spots | set 1 (9.6, 0), set 2 (21.6, 0), set 3 (33.6, 0), set 4 (45.6, 0), common (50.0, 0) |
| People | one per building in each set, 3 in the common building, 11 in total |
| Ramps | set 1 south, set 3 north, common building: 8° up-and-down hump, 0.14 m at the crest |

Inside the buildings are rooms and partitions, boxes and pillars (at least 0.7 m tall).

```{screenshot} ramp.png
An A1 crossing the ramp (8° hump, full room width) in the set 1 south building.
```

## make_world.py is the only source

Running `disaster_sim/scripts/make_world.py` writes two files together.

- `worlds/mission.world` — the Gazebo world (walls, boxes, pillars, ramps, person models)
- `config/mission.yaml` — spawn, follow slots, per-set parking spots, zone rectangles and person counts, and the person truth used for evaluation

Nodes keep no coordinates in code and only read this yaml. So changing the layout never requires changing node code. How to change it: {doc}`../tutorials/editing_world`.

## Layout rules and why

Every rule was set **after an actual failure in testing**.

### 1. Corridors and rooms at least 2.0 m wide

The Nav2 costmap thickens walls by 0.3 m on each side (inflation). A 1.7 m corridor effectively shrinks to 1.0 m, and at that width (the then) DWB spun in place and failed with "no progress". 1.6 m is used only for doors and openings passed **straight through**.

### 2. Turning entrances at least 2.4 m

Openings that require a turn of more than 90° right after them (a room entrance around a wall end) repeatedly caused follow failures and trapped robots at three places that were 1.6 m (set 1 north back-room door, set 1 south inner-room entrance, set 2 north left-room entrance). Rare failures remain even at 2.4 m ({doc}`../results/limitations`).

### 3. Obstacles and people against a wall or with 1.3 m clear

An A1 walked into the 0.9 m gap between a person and the back wall and got stuck. Against a wall (0.4 m from the wall center) there is no gap to enter; at 1.3 m or more it can pass.

### 4. Ramps wall to wall

All 4 falls on the original map happened at **the side edges of ramps**. A 1.9 m wide ramp with rails (0.5 m) then trapped A1s at the rail ends: walking in place → only leg odom advanced and the SLAM position drifted more than 2 m. So ramps are humps that cover the full room width, with no side edges.

### 5. Obstacles at least 0.5 m tall

The A1 lidar plane is about 0.30 m above the ground. Anything lower is invisible to the 2D lidar and only the terrain perception (depth sensor, 4 m ahead) can see it.

:::{note}
Because of these rules the current layout is **easier** than the original map (`disaster.world`): corridors are at least 2 m and ramps have no side edges. The current mission structure was not run on the original map.
:::

## Models

| Model | Source | Notes |
| --- | --- | --- |
| `husky` | official Clearpath A200 meshes, dimensions and mass (`clearpath_platform_description`) | Gazebo DiffDrive (4 wheels), 2D lidar (270° front, 12 m), IMU |
| `person_standing` | Gazebo person model | mannequin, about 1.75 m tall |
| A1 | `a1_champ/urdf/a1.urdf.xacro` | {doc}`locomotion` |
| `disaster_building` | building for the original map | used in the previous architecture |
