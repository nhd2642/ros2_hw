# 보행 속도 응답 재기

CHAMP 보행 설정(`a1_champ/config/gait.yaml`)을 바꿨다면, 명령한 속도를 실제로 얼마나 따르는지와 몸이 얼마나 기우는지 다시 재야 합니다. 추종·Nav2 속도 한도가 이 값으로 정해졌기 때문입니다.

## 실행

```bash
cd ~/ros2_hw/ws && source install/setup.bash
bash src/comm_relay/tools/gait_sweep.sh                    # 기본 속도 목록
bash src/comm_relay/tools/gait_sweep.sh "0.2 0.0" "0.25 0.3" "0.25 -0.3"   # "vx wz" 목록 직접
```

시뮬만 화면 없이 띄우고 `a1_2` 를 도로(평지)를 따라 명령마다 8 초씩 걷게 한 뒤, **Gazebo 정답 위치**로 이동 거리·기울기를 잽니다.

## 기준 결과 (2026-11-12)

```{figure} ../../images/figures/gait_sweep.png
:width: 100%

명령 속도별 실제/명령 비율과 최대 기울기 (평지, 8 s 씩).
```

| 명령 vx m/s | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 |
| --- | --- | --- | --- | --- | --- |
| 실제 / 명령 | 99 % | 95 % | 107 % | 108 % | 107 % |
| 최대 기울기 | 2.3° | 2.3° | 3.7° | 4.6° | 5.5° |

- 걸으며 돌기(0.25 m/s, ±0.3 rad/s): 이동 거리가 예상 호의 현 길이와 같고(1.56·1.53 m, 예상 1.55 m), 회전은 명령의 107~112 %, 좌우 대칭, 기울기 4° 이하.
- 이 값으로 **추종 최대 0.35 m/s, Husky 0.25 m/s** 로 정했습니다. Nav2(MPPI) 탐색 속도는 2026-11-26 에 0.2 → 0.35 m/s 로 올렸습니다 (같은 측정 범위 안).

## odom 눈금 맞추기

`/odom` 이동 거리가 정답보다 짧거나 길면 `gait.yaml` 의 `odom_scaler` 를 고칩니다. 2026-11-12 에 1.3 % 짧아 `1.11 → 1.125` 로 바꿨습니다 (그 뒤 다시 재지는 않았습니다). 이 오차가 쌓이면 도로 40 m 에서 SLAM 지도가 0.5 m 줄어듭니다.

```yaml
gait:
  odom_scaler: 1.125
  max_linear_velocity_x: 0.5
  swing_height: 0.04
  nominal_height: 0.2
```

`swing_height` 를 바꾸면 지형 인식의 `max_step` 도 같게 맞추세요 (`terrain.launch.py max_step:=...`).
