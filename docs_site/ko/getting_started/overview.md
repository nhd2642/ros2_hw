# 프로젝트 개요

이 페이지는 프로젝트를 처음 보는 사람이 **5 분 안에 전체 그림**을 잡도록 돕습니다. 세부 원리는 {doc}`../concepts/index` 에 있습니다.

## 한눈에 보기

```{screenshot} world_overview.png
Gazebo 에서 위에서 내려다본 전체 월드: 도로, 세트 1·2·3 건물 쌍, 동쪽 끝 공동 건물, 출발 대형(Husky + A1 두 대).
```

| 항목 | 내용 |
| --- | --- |
| 시뮬레이터 | Gazebo Sim (Fortress) + `ros_gz`, WSL2 에서 헤드리스로 실행 |
| 미들웨어 | ROS 2 Humble |
| 로봇 | Husky A200 1 대 (바퀴, 2D 라이다·IMU), Unitree A1 2 대 (사족, CHAMP 보행) |
| A1 센서 | 360° 평면 라이다 (10 Hz, 10 m), 전방 RGB 카메라 (2 Hz), 지형용 깊이 센서 (5 Hz), IMU (100 Hz) |
| 주요 스택 | slam_toolbox (로봇별 SLAM), Nav2 (planner/controller server), OpenCV DNN MobileNet-SSD (사람 인식) |
| 임무 | 세트 3개 (도로 양옆 건물 한 쌍씩) + 공동 건물 1채, 사람(마네킹) 합 9명 |
| 판정 | 끝까지 감 + 넘어짐 0 + 탐색 시간 한도 초과 0 + 사람을 전부 정답 1 m 안으로 보고 → PASS |

## 임무 한 번은 이렇게 흘러갑니다

```{graphviz} ../../diagrams/mission_states.dot
```

1. **IDLE → TRANSIT**: 사용자가 `start` 를 보내면 Husky 가 세트 1 주차점으로 출발합니다. A1 두 대는 Husky 뒤 1.6 m, 좌우 0.8 m 자리를 따라 걷습니다.
2. **TRANSIT → EXPLORE**: Husky 가 도착하고 두 A1 이 자기 자리에 서면, 그 위치를 '탐색 시작 위치(홈)' 로 기록하고 각 A1 에게 탐색 구역(사각형)을 내려 줍니다.
3. **EXPLORE**: A1 은 자기 건물 안에서 프런티어(아는 곳과 모르는 곳의 경계)를 골라 Nav2 로 가며 지도를 채웁니다. 사람을 보면 1 초 서고 Husky 에 보고합니다.
4. **세트 완료**: 갈 프런티어가 30 초 동안 없으면 홈으로 돌아와 `/<ns>/set_done` 을 냅니다. 두 대가 모두 돌아오면 다음 세트로 이동합니다.
5. **DONE**: 공동 건물(두 A1 이 같은 구역을 나눠 탐색)까지 끝나면 임무 종료입니다.

탐색이 너무 길어지면(1,500 초) Husky 가 **REGROUP** 으로 A1 을 불러들입니다. 사용자는 언제든 `0`(통신 끔) / `1`(통신 켬)을 보내 통신 장애를 흉내 낼 수 있습니다.

## 구성 요소

```{graphviz} ../../diagrams/architecture.dot
```

계층은 아래에서 위로 쌓입니다.

시뮬레이션 (`disaster_sim`, `a1_champ`, `champ`)
: 월드·로봇 모델·보행 제어기. 월드 배치는 `make_world.py` 하나가 원본입니다.

항법·인식 (`slam_toolbox`, Nav2, `terrain_perception`)
: 로봇마다 SLAM 지도, 경로 계획·추종, 깊이 센서로 본 경사·단차를 비용지도에 넣는 지형 층.

탐색 (`a1_exploration`)
: 두 지도를 합친 공유 지도, 프런티어 추출, 두 로봇에게 목표 배정.

임무 (`comm_relay`)
: 임무 단계, Husky 주행, A1 추종, Nav2 실행기, 통신 복귀, 사람 인식, 속도 명령 중재. 이 프로젝트에서 새로 만든 코드 대부분이 여기에 있습니다.

## 저장소 구성

```text
disaster_develop/
├── code/ros2_hw_gz_src/      # ROS 2 패키지 소스 (ws/src 가 이곳을 가리키는 링크)
│   ├── comm_relay/           # 임무 계층 (Husky 임무·주행, A1 추종·실행기·복귀·사람 인식)
│   ├── a1_exploration/       # SLAM·Nav2 launch, 프런티어·목표 배정·지도 병합 스크립트
│   ├── terrain_perception/   # 깊이 센서 → 비용지도 지형 층
│   ├── disaster_sim/         # 월드, Husky 모델, mission.yaml, make_world.py
│   ├── a1_champ/             # A1 URDF·보행 설정·launch
│   ├── a1_rl/                # (실험) 강화학습 보행
│   ├── champ/, champ_teleop/ # CHAMP 보행 제어기, 키보드 조종
├── ws/                       # colcon 작업 공간 (build/install 은 git 에 없음)
│   └── person_ai_models/     # MobileNet-SSD 모델 파일
├── results/                  # run_mission.sh 결과 (통과 실행 2 개만 저장소에)
├── docs/                     # 설계 문서 원문 (mission_design.md 등)
├── docs_site/                # 이 문서 사이트 (Sphinx)
├── history/                  # 단계별 이력 (CHANGELOG.md, runs.csv)
└── tools/                    # 실행 비교·경로 계획 시험 도구
```

## 다음 단계

- 바로 돌려 보고 싶다면 → {doc}`installation`
- 구조부터 이해하고 싶다면 → {doc}`../concepts/architecture`
