# A1 직접 조종하기

임무 노드가 떠 있을 때 A1 의 속도 명령은 `cmd_vel_arbiter` 하나만 CHAMP 에 씁니다. 그래서 조종 명령은 **`/<ns>/cmd_vel_teleop`** 으로 보내야 합니다. 이 입력은 우선순위가 가장 높아(4), 보내는 동안 Nav2·복귀·추종 명령을 모두 이깁니다.

```{graphviz} ../../diagrams/cmd_vel_arbiter.dot
```

## 키보드 조종 (champ_teleop)

```bash
source ~/ros2_hw/ws/install/setup.bash
ros2 run champ_teleop champ_teleop.py --ros-args -r cmd_vel:=/a1_1/cmd_vel_teleop -p use_sim_time:=true
```

`a1_2` 를 조종하려면 리맵만 바꿉니다. 키 배치는 `teleop_twist_keyboard` 와 같습니다 (`i` 앞, `,` 뒤, `j`/`l` 회전, `k` 정지).

:::{warning}
`/<ns>/cmd_vel` 에 **직접** 쓰면 중재 노드가 20 Hz 로 내는 0 명령과 섞여 로봇이 떨듯이 멈춥니다. 중재 노드는 다른 발행자를 보면 경고를 냅니다.
:::

## 한 번만 명령 보내기

```bash
# a1_1 을 0.2 m/s 로 앞으로 (보내는 동안만; 0.5 s 안에 다음 명령이 없으면 0 으로 돌아감)
ros2 topic pub -r 10 /a1_1/cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

`Ctrl+C` 로 멈추면 0.5 초 뒤 다음 우선순위 입력(또는 0 명령)으로 돌아갑니다.

## 지금 누가 몰고 있는지 보기

```bash
ros2 topic echo /a1_1/cmd_vel_source
```

`teleop`, `hold`, `return`, `nav`, `follow` 중 지금 이긴 입력의 이름이 나옵니다.

## 임무 없이 보행만 시험할 때

임무 계층(`mission.launch.py`)을 띄우지 않았다면 중재 노드가 없으므로 `/<ns>/cmd_vel` 에 바로 써도 됩니다. 보행 측정 도구는 {doc}`measure_gait` 를 보세요.
