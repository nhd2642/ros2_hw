# 튜토리얼

튜토리얼은 **처음부터 끝까지 따라 하며 배우는** 문서입니다. 위에서부터 차례로 하면 임무를 돌리고, 조작하고, 결과를 해석하고, 월드를 바꾸는 데까지 갑니다.

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} 1. 전체 임무 실행하고 지켜보기
:link: first_mission
:link-type: doc
`run_mission.sh` 하나로 임무를 돌리고, 단계마다 무엇을 봐야 하는지 익힙니다.
:::

:::{grid-item-card} 2. 화면 보며 직접 시작하기
:link: gui_manual_start
:link-type: doc
Gazebo·RViz 를 켜고 `husky_mission` 터미널에 `start`·`0`·`1` 을 직접 입력합니다.
:::

:::{grid-item-card} 3. 통신 끊김과 복귀 시험
:link: comm_loss
:link-type: doc
탐색 중 통신을 끊어 A1 이 지나온 길로 돌아오는 것을 확인합니다.
:::

:::{grid-item-card} 4. 세트 하나만 돌리기
:link: partial_sets
:link-type: doc
`start_set`·`SETS` 로 원하는 세트만 짧게 시험합니다.
:::

:::{grid-item-card} 5. 결과 폴더 읽기
:link: reading_results
:link-type: doc
summary, timeline, 이벤트, 노드 로그를 읽고 그래프로 그립니다.
:::

:::{grid-item-card} 6. 월드 배치 바꾸기
:link: editing_world
:link-type: doc
`make_world.py` 로 건물 안 배치를 바꾸고 `mission.yaml` 을 다시 만듭니다.
:::
::::

:::{tip}
모든 튜토리얼은 {doc}`../getting_started/installation` 을 마치고 `source ~/ros2_hw/ws/install/setup.bash` 한 터미널을 기준으로 합니다.
:::

```{toctree}
:hidden:
:maxdepth: 1

first_mission
gui_manual_start
comm_loss
partial_sets
reading_results
editing_world
```
