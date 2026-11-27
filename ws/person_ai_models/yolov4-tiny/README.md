# YOLOv4-tiny (COCO) — survivor_detector 사람 인식 모델

- `yolov4-tiny.cfg`: https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg
- `yolov4-tiny.weights` (24,251,276 B, sha256 `cf9fbfd0f6d4869b35762f56100f50ed05268084078805f0e7989efe5bb8ca87`):
  https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
- OpenCV DNN (`cv2.dnn.readNetFromDarknet`, OpenCV 4.5.4 에서 확인)으로 읽는다. 사람 = COCO 클래스 0.
- 2026-11-26 MobileNet-SSD 에서 바꿈: SSD 는 옆모습 사람을 못 봤음 (`docs/experiments/D1_기준.txt`).
