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
| model | MobileNet-SSD (Caffe, PASCAL VOC, person = class 15) |
| runtime | OpenCV DNN, CPU, 300 × 300 input, scale 0.007843, mean 127.5 |
| rate | one frame per robot per 1 s (`period`) |
| threshold | 0.5 (`conf`) |
| model files | `ws/person_ai_models/MobileNet-SSD/` (`deploy.prototxt`, `mobilenet_iter_73000.caffemodel`) |

Without the model or `cv2` it only logs a warning and does nothing (exploration continues).

### Slice inference (SAHI style)

Shrinking a 640 × 480 image to 300 × 300 misses people who are small and far, or only partly visible (SSD is weak on small objects). So it runs **the full frame + 3 horizontally overlapping slices** (half width, shifted by a quarter) separately, maps results back to the original coordinates and removes duplicates with horizontal-interval IoU 0.5.

Check: of 11 frames with a person visible, frames scoring above 0.4 went from 1 to 5, with 0 false positives in 421 frames without people (same as before).

### Threshold 0.5

In mission v2 runs the 9 people scored 0.89-1.00 and the one false positive (next to a pillar) scored 0.41. So the threshold went from 0.4 to 0.5.

## 2. Locating

A camera alone gives no range. At first the position was "the nearest lidar point in that direction": 7 of 14 reports matched (it picked pillars and walls in front). Now:

1. Box left/right extent → **bearing range**.
2. Box height and person height (1.75 m) → **range estimate**.
3. Cluster lidar points within that bearing range at 0.25 m spacing and pick **the cluster nearest the estimated range** (40 % tolerance). Range limit 7 m.
4. Transform the cluster mean into the robot map frame with TF.

## 3. Merging and reporting

- Within 1 m of an entry in its own list it is the same person (average updated), otherwise a new person.
- For a new person:
  - **1 s zero command** on `/<ns>/cmd_vel_hold` (arbiter priority 3).
  - Marker on its own map via `/<ns>/survivor_markers`.
  - JSON report on `/husky/survivor_report`: `{"robot": "a1_1", "id": 1, "x": 10.4, "y": -5.45, "score": 0.97, "delay": 0.0}` (`delay` = sim seconds from detection to delivery; grows while the link is down).
- **While the link is down** reports are queued and sent when `comm_status` returns.
- `husky_mission` merges received reports again with a 1 m radius and publishes the list on `/husky/survivors` (world).

## Results

Latest full run: **exactly 9 reports**, all 9 people within 1 m of truth (no duplicates or false positives). The two earlier passing runs each had 2 extra reports with off positions or low scores.

:::{warning}
Comparison with the configured person count is by **number of reports**, so extra reports can mark "all found" early. Set completion itself is decided only by both A1s returning, so exploration does not end early.
:::
