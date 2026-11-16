# Transit: Husky Drive & A1 Follow

In TRANSIT the Husky drives down the road to the next parking spot and the two A1s follow. This phase **does not use Nav2**. Replanning to a moving target makes a robot stop and go, so it uses a lightweight steering that avoids obstacles directly with the lidar.

```{graphviz} ../../diagrams/transit_follow.dot
```

## Shared steering: drive_logic.Steer

Husky driving and A1 following use the same steering class.

1. Scan candidate directions **nearest the goal bearing first** (0.175 rad steps either side, up to ±1.75 rad).
2. Pick the first direction whose **body-width strip** (half width `half`) has no lidar point within `min(remaining distance, look) + front`.
3. If the chosen direction differs from the body heading by more than `turn_first`, **turn in place** first (A1s barely sidestep; the Husky is skid-steered).
4. Scale speed down with the clear distance in that direction and the remaining distance. If no direction is clear, stop (`BLOCKED`).
5. When left and right candidates are similar, check last time's side first to avoid dithering.

| | Husky (`husky_driver`) | A1 (`a1_follower`) |
| --- | --- | --- |
| max speed | 0.25 m/s | 0.35 m/s |
| max turn rate | 0.6 rad/s | 0.5 rad/s |
| body half width `half` | 0.55 m | 0.37 m |
| front margin `front` | 0.8 m | 0.45 m |

## Husky pose estimation

The Husky doesn't use SLAM. It **integrates the wheel-odom forward speed along the IMU yaw**, because skid steering makes wheel-odom rotation unreliable. The starting pose is the `spawn` parameter. In testing the error stayed within 0.15 m over 40 m.

Its output `/husky/pose` (world coordinates) is **the coordinate reference** for the whole mission.

## A1 follow: anchored to the Husky in view

An A1 heads for its slot behind the Husky (`slots`: 1.6 m back, 0.8 m to the side). The problem is that **the A1 SLAM pose drifts**. Over the 40 m road an A1 map drifted up to 1 m (1.3 % odom scale error + scan-matching drift). Computing the slot only from the received Husky pose and its own SLAM pose actually led an A1 to walk 1 m off into the next building's door.

So `a1_follower` does this:

1. Convert the Husky pose received over comm into its own body frame to get an **estimate**.
2. Cluster lidar points near it and find a cluster that looks like the Husky (`find_body`). Clusters more than 0.7 m from the received pose are discarded — it once picked the small wall at the start as the Husky.
3. The cluster center comes from **a box fit aligned to the Husky heading** (`box_center`, 0.99 × 0.67 m): if one face is visible, half a box size beyond that face; if both ends are visible, the middle.
4. It tracks the difference "Husky seen by lidar − estimated Husky" slowly (0.7/0.3 exponential smoothing) and adds it to the slot calculation.
5. If the Husky is not visible (occluded, too far) it uses the received pose as is.

As a result, even when SLAM drifts, **the real position stays in the lane** (latest run: `a1_1` |y| 0.72-0.82 m while moving).

:::{note}
At first the Husky center was estimated as "0.4 m along the line of sight from the mean of visible points", which produced a wrong 0.47 m correction on arrival at set 1. With the box fit, the lane matches the configured ±0.8 m (it used to be ±0.65 m).
:::

## Waiting and arrival

- `a1_follower` publishes its distance to its slot on `/<ns>/slot_error`.
- When the worst A1 error exceeds 2.5 m, `husky_mission` stops the Husky with `/husky/go = False`, and lets it go again once the error is within 1.5 m.
- When the Husky reaches the parking spot and both A1 errors are within 0.6 m, it moves to EXPLORE (after 60 s it moves on even if an A1 hasn't arrived).

## Pose correction on set arrival (pose_fix)

When transit ends and exploration begins, `a1_follower` sends once, on `/<ns>/pose_fix` (world dx, dy): **"seen from the Husky, my shared pose is off by this much"**. `nav_goal_bridge` (the map merge) shifts that robot's map alignment by that amount, so the zone rectangles (world) and the A1 map line up again.

- It is not sent if smaller than 0.1 m (not needed), larger than 1.5 m (a misdetection), or if the Husky has not been seen for more than 3 s.
- In the latest run `a1_1` corrected **1.37 m** on arrival at set 2, 0.71 m at set 3, 0.25 m at the common building.
- In a run without this correction, `a1_2` traveled 1 m off, walked into the set 3 building door, and the common building exploration failed.

Drift that builds up **within** a set stays — see {doc}`../results/limitations`.

## When it stops

- If the Husky pose is older than 1 s, or its own pose or scan stops, the A1 stops.
- Outside TRANSIT `a1_follower` sends no commands.
- Follow commands have the lowest priority (0), so a person-found stop, comm return or teleop always wins.
