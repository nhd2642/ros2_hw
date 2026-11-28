# mission.yaml

위치: `disaster_sim/config/mission.yaml`. **`make_world.py` 가 만드는 파일이므로 직접 고치지 마세요.** 좌표는 Gazebo 월드 m 입니다.

## 전체

지금 파일 내용 (주석만 덧붙임):

```yaml
spawn:                       # 출발 자세 [x, y, yaw]
  husky: [0.0, 0.0, 0.0]
  a1_1: [-2.4, -0.8, 0.0]
  a1_2: [-2.4, 0.8, 0.0]
slots:                       # Husky 기준 추종 자리 [뒤(-)/앞(+), 오른쪽(-)/왼쪽(+)]
  a1_1: [-1.6, -0.8]
  a1_2: [-1.6, 0.8]
sets:                        # 임무 순서
- name: set1
  park: [9.6, 0.0]           # Husky 주차점
  zones:                     # 로봇별 탐색 구역 [x_min, x_max, y_min, y_max]
    a1_2: [4.0, 12.0, 1.5, 6.5]
    a1_1: [4.0, 12.0, -6.5, -1.5]
  people: 2                  # 이 세트의 사람 수 (미발견 표시용)
- name: set2
  park: [21.6, 0.0]
  zones: {a1_2: [16.0, 24.0, 1.5, 6.5], a1_1: [16.0, 24.0, -6.5, -1.5]}
  people: 2
- name: set3
  park: [33.6, 0.0]
  zones: {a1_2: [28.0, 36.0, 1.5, 6.5], a1_1: [28.0, 36.0, -6.5, -1.5]}
  people: 2
- name: set4
  park: [45.6, 0.0]
  zones: {a1_2: [40.0, 48.0, 1.5, 6.5], a1_1: [40.0, 48.0, -6.5, -1.5]}
  people: 2
- name: common
  park: [50.0, 0.0]
  zones: {a1_1: [52.0, 66.0, -6.0, 6.0], a1_2: [52.0, 66.0, -6.0, 6.0]}   # 같은 구역
  people: 3
people_truth:                # 평가용 사람 정답 위치 (mission_watch 만 씀)
- [10.6, 6.1]
- [10.5, -6.1]
- [16.4, 2.9]
- [17.0, -6.1]
- [29.2, 6.1]
- [29.4, -6.1]
- [47.6, 6.1]
- [46.6, -1.9]
- [59.6, 0.0]
- [63.5, 4.5]
- [64.6, -4.6]
```

## 키

| 키 | 읽는 곳 | 뜻 |
| --- | --- | --- |
| `spawn` | sim.launch.py, mission.launch.py (`start_poses`) | 세트 0 에서의 출발 자세 |
| `slots` | a1_follower, husky_mission, `start_poses` | 추종 자리. `start_set` ≥ 1 이면 A1 을 앞 세트 주차점 기준 이 자리에 스폰 |
| `sets[].name` | husky_mission, `/mission/state` | 세트 이름 |
| `sets[].park` | husky_mission → `/husky/goal` | Husky 주차점 |
| `sets[].zones` | husky_mission → `/mission/state` | 로봇별 탐색 구역 |
| `sets[].people` | husky_mission | 설정 사람 수. 적게 찾으면 '미발견 n' |
| `people_truth` | mission_watch.py | 평가 정답 (로봇 노드는 안 읽음) |

`start_poses(cfg, k)` (`comm_relay/mission_logic.py`) 가 `start_set` 에 맞는 스폰 자세를 계산합니다: k = 0 이면 `spawn`, k ≥ 1 이면 Husky 는 `sets[k-1].park`, A1 은 그 기준 `slots`.
