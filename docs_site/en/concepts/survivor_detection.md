# Person Detection

`survivor_detector` looks for people (mannequins) with each robot's front camera, gets the range from the lidar, converts the position into the robot's map coordinates, stops for 1 s and reports to the Husky.

```{graphviz} ../../diagrams/survivor_pipeline.dot
```

```{screenshot} survivor_detect.png
Detection box and score over an A1 front camera image, plus the survivor_markers in RViz.
```

## 1. The model

| Item | Value |
| --- | --- |
| model | YOLOv4-tiny (Darknet, COCO, person = class 0). MobileNet-SSD before 2026-11-26 |
| runtime | OpenCV DNN, CPU, one 416 × 416 input, about 30 ms |
| rate | one frame per robot per 1 s (`period`) |
| threshold | 0.5 (`conf`) |
| model files | `ws/person_ai_models/yolov4-tiny/` (`yolov4-tiny.cfg`, `yolov4-tiny.weights`; source in that folder's `README.md`) |

Without the model or `cv2` it only logs a warning and does nothing (exploration continues).

### Why MobileNet-SSD was replaced (2026-11-26)

SSD could not see **people from the side**. The north person of set 4 (standing sideways) scored 0.00 in 21 of 26 frames even with the whole body in view at 2-6.6 m, so it was missed in a full mission. Extra slices (the SAHI-style full frame + 3 horizontal slices used before), horizontal flips and OpenCV HOG also scored 0 on the side view.
On the same raw frames YOLOv4-tiny scored 0.53-0.94 on the side view, 0.85-1.00 frontal, with 0 false positives in 212 frames without people, so it replaced SSD. It runs on one image, so slice inference was dropped; overlapping boxes are merged with horizontal-interval IoU 0.5.

### Threshold 0.5

With SSD the 9 people scored 0.89-1.00 and the one false positive (next to a pillar) scored 0.41, so the threshold went from 0.4 to 0.5. It stays at 0.5 with YOLO (lowest side-view score above 0.53, highest no-person frame 0.00).

## 2. Locating

A camera alone gives no range. At first the position was "the nearest lidar point in that direction": 7 of 14 reports matched (it picked pillars and walls in front). Now:

1. Box left/right extent → **bearing range**.
2. Box height and reference height (2.0 m: YOLO boxes are taller than the body, measured against ground-truth poses; 1.75 m with SSD) → **depth estimate** along the camera axis.
3. Among lidar points in that bearing range, pick **the point whose depth is nearest the estimate** (40 % tolerance) and cluster 0.25 m around it. Range limit 7 m.
   (Before 2026-11-27 the depth estimate was compared with the straight-line lidar range, so people near the edge of the view looked 1/cos farther and were dropped — why the north person of set 3 was missed after being detected three times at 0.87-0.99.)
4. Transform the cluster mean into the robot map frame with TF.

## 3. Merging and reporting

- Within 1 m of an entry in its own list it is the same person (average updated), otherwise a new person.
- For a new person:
  - **1 s zero command** on `/<ns>/cmd_vel_hold` (arbiter priority 3).
  - Marker on its own map via `/<ns>/survivor_markers`.
  - `SurvivorReport` on `/husky/survivor_report`: `robot: a1_1, id: 1, position: {x: 10.4, y: -5.45}, score: 0.97, delay: 0.0` (`delay` = sim seconds from detection to delivery; grows while the link is down). Type: {doc}`../reference/interfaces`.
- **While the link is down** reports are queued and sent when `comm_status` returns.
- `husky_mission` merges received reports again with a 1.5 m radius and publishes the list on `/husky/survivors` (`SurvivorList`, world).

## Results

All 7 full runs of the current setup line (2026-11-27 to 28) reported all 11 people within 1 m of truth, but with 11-13 reports. Most extra reports have a **range estimate off by 1-3 m**, mostly for people seen from 4.5 m or farther (full runs since 11-27: 1 of 50 below 4.5 m, 13 of 75 at 4.5 m or more). The cause is not confirmed (box-height range estimate, lidar missing the person, or image/scan time skew). Record: `docs/experiments/A1_근접중복_분석.txt` (Korean).

:::{warning}
Comparison with the configured person count is by **number of reports**, so extra reports can mark "all found" early. Set completion itself is decided only by both A1s returning, so exploration does not end early.
:::
