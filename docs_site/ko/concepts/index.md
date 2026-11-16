# 개념

개념 문서는 **왜 이렇게 만들었고 안에서 어떻게 동작하는지**를 설명합니다. 명령어보다 원리와 설계 이유, 그리고 시험하며 바꾼 결정의 근거를 적었습니다.

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} 시스템 구조
:link: architecture
:link-type: doc
패키지·계층·노드·토픽이 어떻게 이어지는지, 기동 순서.
:::

:::{grid-item-card} 임무 상태기계
:link: mission_state_machine
:link-type: doc
IDLE → TRANSIT → EXPLORE → REGROUP → DONE, 세트 완료와 통신 규칙.
:::

:::{grid-item-card} 월드와 배치 규칙
:link: world_layout
:link-type: doc
도로·세트·공동 건물, 시험에서 나온 배치 규칙과 그 이유.
:::

:::{grid-item-card} 이동: Husky 주행과 A1 추종
:link: transit_following
:link-type: doc
라이다 조향, Husky 위치 추정, 눈앞의 Husky 기준 추종, 자세 보정.
:::

:::{grid-item-card} 탐색: SLAM부터 Nav2까지
:link: exploration
:link-type: doc
지도 병합, 프런티어, 목표 배정, 실행기의 실패 처리, Nav2 설정.
:::

:::{grid-item-card} 지형 인식
:link: terrain_perception
:link-type: doc
깊이 센서로 단차·경사를 찾아 비용지도에 넣는 방법.
:::

:::{grid-item-card} 사람 인식
:link: survivor_detection
:link-type: doc
MobileNet-SSD + 조각 추론, 상자 높이와 라이다로 위치 잡기.
:::

:::{grid-item-card} 통신·복귀·명령 중재
:link: communication_return
:link-type: doc
하트비트, breadcrumb 복귀, 속도 명령 우선순위.
:::

:::{grid-item-card} 좌표계와 시간
:link: coordinates_time
:link-type: doc
world / map / odom, 공유 지도, 시뮬 시간과 /clock_slow.
:::

:::{grid-item-card} A1 보행 (CHAMP)
:link: locomotion
:link-type: doc
CHAMP 보행 제어기, 센서 구성, 보행 설정값.
:::
::::

```{toctree}
:hidden:
:maxdepth: 1

architecture
mission_state_machine
world_layout
transit_following
exploration
terrain_perception
survivor_detection
communication_return
coordinates_time
locomotion
```
