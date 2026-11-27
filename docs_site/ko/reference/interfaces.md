# 임무 인터페이스 (mission_interfaces)

생존자 탐색 서비스를 쓰는 데 필요한 **사용자용 인터페이스**는 패키지 `mission_interfaces` 에 직접 정의했습니다.
이 페이지만 보고 임무를 요청하고, 진행을 지켜보고, 결과를 받을 수 있게 정리했습니다.

| 종류 | 이름 | 형식 | 제공 | 하는 일 |
| --- | --- | --- | --- | --- |
| action | `/husky/run_mission` | `mission_interfaces/action/RunMission` | husky_mission | 임무 요청 → 진행 피드백 → 생존자 목록 결과. 취소 가능 |
| service | `/husky/set_comm` | `mission_interfaces/srv/SetComm` | husky_mission | Husky-A1 통신 켜기/끄기 (통신 끊김 시험) |
| service | `/husky/get_survivors` | `mission_interfaces/srv/GetSurvivors` | husky_mission | 지금까지 확인한 생존자 조회 (전체 또는 세트별) |
| topic | `/mission/state` | `mission_interfaces/msg/MissionState` | husky_mission | 임무 단계·세트·구역·통신 (1 Hz, latched) |
| topic | `/husky/survivor_report` | `mission_interfaces/msg/SurvivorReport` | survivor_detector (A1) | 생존자 한 명 발견 보고 |
| topic | `/husky/survivors` | `mission_interfaces/msg/SurvivorList` | husky_mission | 확인한 생존자 목록 (latched) |

경로 계획·추종처럼 직접 만들기 어려운 부분은 오픈소스를 씁니다. Nav2 의 action `ComputePathToPose`·`FollowPath` 와
service `GetCostmap` 을 `nav_executor` 가 부릅니다 ({doc}`../concepts/exploration`).

## 어떻게 흘러가나

```text
사용자 ──RunMission 목표 {num_sets}──▶ husky_mission ──/mission/state (TRANSIT, EXPLORE, ...)──▶ A1 노드들
   ▲                                     │  ▲
   │◀──피드백 1 Hz {state, found, expected}│  └── /husky/survivor_report ◀── survivor_detector (A1 카메라)
   │◀──결과 {success, survivors, ...}──────┘
   ├──SetComm {enabled}──▶ husky_mission (통신 0/1: A1 은 복귀, Husky 는 정지)
   └──GetSurvivors {set_name}──▶ husky_mission (지금까지 찾은 사람)
```

1. 시뮬과 로봇 노드를 띄웁니다 ({doc}`../tutorials/first_mission`). `husky_mission` 은 `IDLE` 에서 기다립니다.
2. `RunMission` 목표를 보내면 세트마다 `TRANSIT`(Husky 이동, A1 추종) → `EXPLORE`(A1 두 대가 자기 구역 탐색) → 둘 다 탐색 시작 위치로 돌아오면 다음 세트.
3. A1 이 사람을 찾으면 `SurvivorReport` 가 오고, `husky_mission` 이 1.5 m 안 보고를 한 명으로 합쳐 `SurvivorList` 로 냅니다.
4. 요청한 세트를 다 마치면 `DONE` 이 되고 액션 결과로 생존자 목록이 돌아옵니다.

## action: RunMission

```text
# 목표
uint32 num_sets                 # 할 세트 수 (0 = 시작 세트부터 끝까지). 시작 세트는 시뮬을 띄울 때 정함 (start_set)
---
# 결과
bool success                    # 요청한 세트를 모두 마침 (취소·중단이면 false)
string message
mission_interfaces/SurvivorList survivors
uint32 found                    # 찾은 생존자 수
uint32 expected                 # 마친 세트들의 설정상 생존자 수
float32 duration                # 임무에 걸린 시뮬 초
---
# 피드백 (1 s 마다)
mission_interfaces/MissionState state
uint32 found
uint32 expected                 # 지금까지 시작한 세트들의 설정상 생존자 수
```

- `IDLE` 일 때 한 번만 받습니다. 진행 중이거나 끝난(`DONE`) 뒤의 요청은 거절(REJECT)합니다. 다시 하려면 시뮬을 다시 띄웁니다.
- 취소하면 그 자리에서 `DONE` 이 되고(로봇은 탐색을 멈춤) 결과 상태는 `CANCELED`, `success` = false 입니다.

```bash
ros2 action send_goal --feedback /husky/run_mission mission_interfaces/action/RunMission "{num_sets: 0}"
```

## service: SetComm

```text
bool enabled                    # true = 통신 켬 (기본), false = 끔
---
bool success
string message
```

끄면 `husky_mission` 이 A1 하트비트(`/<ns>/comm_status`)를 멈춥니다. A1 은 1 초 뒤 이번 세트 탐색 시작 위치로 되돌아가고,
Husky 는 멈춥니다. 켜면 하던 단계를 이어 가고, 끊긴 동안 찾은 사람은 이때 보고됩니다 ({doc}`../tutorials/comm_loss`).

```bash
ros2 service call /husky/set_comm mission_interfaces/srv/SetComm "{enabled: false}"
ros2 service call /husky/set_comm mission_interfaces/srv/SetComm "{enabled: true}"
```

## service: GetSurvivors

```text
string set_name                 # 빈 문자열 = 전체, 세트 이름(set1 ...) = 그 세트 구역 안만
---
bool success                    # 세트 이름을 모르면 false
string message
mission_interfaces/SurvivorList survivors
uint32 expected                 # 그 범위의 설정상 생존자 수
```

```bash
ros2 service call /husky/get_survivors mission_interfaces/srv/GetSurvivors "{set_name: set2}"
```

## msg: MissionState (`/mission/state`)

```text
string phase                    # IDLE / TRANSIT / EXPLORE / REGROUP / DONE (상수 PHASE_* 로도 정의)
int32 set_index                 # 지금 세트 번호 (mission.yaml 의 sets 순서, 0 부터)
string set_name                 # set1 / set2 / set3 / set4 / common
mission_interfaces/Zone[] zones # EXPLORE·REGROUP 때만 채움
bool comm                       # Husky-A1 통신
```

`Zone` = `string robot`, `float64 x_min, x_max, y_min, y_max` (월드 = `shared_map` 좌표, m). 로봇마다 탐색할 사각형입니다.
A1 쪽 노드(`a1_follower`, `nav_executor`, `return_relay`, `frontier_detector`, `frontier_comm_filter`, `goal_assigner`, `terrain_node`)는
이 메시지만 보고 무엇을 할지 정합니다. 단계 뜻은 {doc}`../concepts/mission_state_machine`.

## msg: SurvivorReport (`/husky/survivor_report`)

```text
std_msgs/Header header          # frame_id = map (그 로봇 TF 트리의 map, 원점 = 그 로봇 스폰, 방향 = 월드)
string robot                    # a1_1 / a1_2
uint32 id                       # 그 로봇 안에서의 생존자 번호
geometry_msgs/Point position    # 생존자 위치 (m)
float32 score                   # 인식 점수 0~1
float32 delay                   # 발견 -> 보고까지 시뮬 초 (통신 끊김 동안 쌓이면 커짐)
```

## msg: SurvivorList (`/husky/survivors`, 서비스·액션 결과)

```text
std_msgs/Header header          # frame_id = world
geometry_msgs/Point[] people    # 생존자 위치 (월드, m)
```

## Python 에서 쓰기

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mission_interfaces.action import RunMission

rclpy.init()
node = Node('mission_client')
client = ActionClient(node, RunMission, '/husky/run_mission')
client.wait_for_server()
goal = client.send_goal_async(RunMission.Goal(num_sets=0),
                              feedback_callback=lambda f: print(f.feedback.state.phase, f.feedback.found))
rclpy.spin_until_future_complete(node, goal)
result = goal.result().get_result_async()
rclpy.spin_until_future_complete(node, result)
print([(p.x, p.y) for p in result.result().result.survivors.people])
```

`run_mission.sh` 도 같은 액션으로 임무를 시작하고, 결과·피드백을 결과 폴더의 `action.log` 에 남깁니다.
통신 시험(`COMM_OFF_AT`)은 `mission_watch.py` 가 `SetComm` 서비스를 부릅니다.
