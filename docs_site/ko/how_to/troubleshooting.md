# 문제 해결

증상별로 확인할 곳과 해결 방법을 모았습니다.

## 기동

Nav2 서버가 안 켜짐 (`실패: Nav2 서버 4 개 active`)
: Nav2 가 TF 를 늦게 받아 생기는 기동 불량입니다(약 30 번 중 5 번). `run_mission.sh` 는 한 번 자동으로 다시 띄웁니다. 두 번 다 실패하면 `bash run_mission.sh clean` 뒤 다시 실행하세요.

A1 컨트롤러가 활성화되지 않음 (`실패: 두 A1 컨트롤러 활성`)
: `sim.log` 를 보세요. 이전 실행의 Gazebo 가 남아 있으면 충돌합니다 → `bash run_mission.sh clean`. `ign_ros2_control` 이 없다는 오류면 {doc}`../getting_started/installation` 2 단계를 다시.

같은 시각에 두 실행을 돌림
: `ROS_DOMAIN_ID` 는 다르게 잡히지만 CPU 가 모자라 둘 다 망가집니다. 한 번에 하나만 돌리세요.

## 시뮬 성능

RTF 가 0.2 보다 낮음
: 화면(`GUI=1`, RViz)을 끄세요. 시각화를 켜면 RTF 가 0.6 → 0.2 대로 떨어진 기록이 있습니다. 파이썬 노드는 `/clock_slow`(50 Hz)로 리맵되어 있어야 합니다 — 새 노드를 추가했다면 `mission.launch.py` 의 `node()` 도우미를 쓰세요.

라이다 값이 전부 0.15 m (range_min)
: GPU 라이다의 거리값 문제입니다. 시뮬 서버는 소프트웨어 렌더링(`software_rendering:=true`, 기본)으로 돌려야 합니다. `tools/lidar_check.sh` 로 렌더링 방식별 값을 비교할 수 있습니다.

## 임무 진행

Husky 가 출발하지 않음
: `start` 가 안 갔을 수 있습니다. `ros2 topic echo /mission/state` 로 단계를 보세요. `IDLE` 이면 다시 보냅니다. `ROS_DOMAIN_ID` 가 스크립트와 같은지 확인하세요.

Husky 가 계속 HOLD
: 뒤따르는 A1 이 2.5 m 넘게 처졌습니다. `/<ns>/slot_error` 를 보세요. A1 이 넘어졌거나(`FALLEN`), 장애물에 막혔을 수 있습니다.

A1 하나가 출발 안 함
: 출발점 앞 작은 벽을 Husky 로 착각한 적이 있었습니다(지금은 받은 Husky 자세에서 0.7 m 넘게 벗어난 점 묶음을 버림). 월드를 바꿨다면 Husky 근처에 비슷한 크기의 물체가 없는지 보세요.

세트가 끝나지 않음
: 가도 안 지워지는 프런티어를 되풀이 방문하는 경우입니다. 같은 자리 도착 3 번이면 그 세트에서 포기하도록 되어 있습니다. `mission.log` 에서 `실패`·`도착` 을 로봇별로 세어 보세요. 1,500 s 가 넘으면 REGROUP 으로 넘어갑니다.

A1 이 방에 들어갔다 못 나옴
: 대개 좁은 입구 때문입니다. 벽 끝을 돌아 들어가는 입구는 2.4 m 이상이어야 합니다 ({doc}`../concepts/world_layout`).

## 위치·지도

A1 이 옆 건물 문으로 들어감
: SLAM 지도가 1 m 넘게 흘렀습니다. 세트 도착 때 Husky 기준 보정(`/<ns>/pose_fix`)이 나왔는지 `mission.log` 에서 확인하세요. odom 눈금(`odom_scaler`)이 틀렸을 수도 있습니다 ({doc}`measure_gait`).

좁은 곳에서 위치가 갑자기 2 m 튐
: slam_toolbox 루프 클로징이 원인으로 추정되어 `do_loop_closing: false` 로 꺼 두었습니다. 켜지 마세요.

A1 이 제자리걸음하며 SLAM 위치가 흐름
: 무언가에 끼어 다리만 움직이는 경우입니다. 다리 odom 이 계속 전진해 SLAM 이 따라갑니다. 끼일 지형(난간, 좁은 틈)을 배치에서 빼는 것 말고 근본 해결은 없습니다.

## 사람 인식

사람을 하나도 못 찾음
: `mission.log` 에 모델 경고가 있는지 보세요. `ws/person_ai_models/MobileNet-SSD/` 에 두 파일이 있어야 하고 `python3 -c "import cv2"` 가 되어야 합니다.

보고 건수가 사람 수보다 많음
: 위치가 1 m 넘게 어긋났거나 점수가 낮은 오탐입니다. `conf` 를 올리면 줄지만 놓칠 수 있습니다 ({doc}`tune_parameters`).

## 정리

남은 프로세스 모두 죽이기
: ```bash
  bash ~/ros2_hw/ws/src/comm_relay/tools/run_mission.sh clean
  ```
  Gazebo, CHAMP, Nav2, SLAM, 임무 노드, 공유 메모리(`/dev/shm/fastrtps_*`), ROS 2 데몬까지 정리합니다.
