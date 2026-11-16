# 4. 세트 하나만 돌리기

전체 임무는 2 시간이 걸립니다. 노드를 고친 뒤 확인할 때는 **원하는 세트만** 돌리는 편이 훨씬 빠릅니다.

## 두 가지 손잡이

`start_set` (스크립트의 첫 인자)
: 몇 번째 세트부터 시작할지. `0` = 처음(출발점), `1` = 세트 2 직전, `2` = 세트 3 직전, `3` = 공동 건물 직전. 1 이상이면 그 앞 세트가 끝난 것처럼 **Husky 를 앞 세트 주차점에, A1 을 그 뒤 추종 자리에** 놓고 시작합니다.

`SETS` (환경 변수)
: 몇 세트를 하고 끝낼지. 기본은 전부.

| 하고 싶은 것 | 명령 |
| --- | --- |
| 세트 1 만 | `SETS=1 bash run_mission.sh` |
| 세트 2 만 | `SETS=1 bash run_mission.sh 1` |
| 세트 3 부터 끝까지 | `bash run_mission.sh 2` |
| 공동 건물만 | `SETS=1 bash run_mission.sh 3` |

(모두 `~/ros2_hw/ws/src/comm_relay/tools/` 에서, 또는 경로를 붙여 실행)

## 예: 공동 건물만

```bash
cd ~/ros2_hw/ws && source install/setup.bash
SETS=1 bash src/comm_relay/tools/run_mission.sh 3
```

결과 폴더 이름이 `..._set3` 으로 끝납니다. 공동 건물에서는 두 A1 이 **같은 구역**(14 × 12 m)을 나눠 탐색하므로 협업 동작을 보기에 좋습니다.

```{screenshot} common_building.png
공동 건물 탐색: 두 A1 이 남·북으로 갈라져 서로 다른 프런티어를 향하는 모습.
```

## 시간 한도

`DUR` 은 시뮬 초 한도입니다(기본 6000). 오래 끌면 끊고 싶을 때 씁니다.

```bash
DUR=900 SETS=1 bash src/comm_relay/tools/run_mission.sh 1
```

## 같이 쓰면 좋은 조합

```bash
# 세트 2 에서 통신 시험
COMM_OFF_AT=60 COMM_OFF_FOR=45 SETS=1 bash src/comm_relay/tools/run_mission.sh 1
# 화면을 보며 공동 건물
GUI=1 SETS=1 bash src/comm_relay/tools/run_mission.sh 3
```

:::{note}
`start_set` 은 `sim.launch.py` 와 `mission.launch.py` 에 같은 값으로 넘어가야 합니다. 스크립트가 알아서 하지만, 손으로 띄울 때는 두 launch 모두 `start_set:=k` 를 주세요.
:::
