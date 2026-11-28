# 재난 탐색 시뮬레이션 문서

```{raw} html
<div class="hero">
  <span class="hero-tag">MISSION V2 · ROS 2 HUMBLE · GAZEBO SIM</span>
  <h1>A1 두 대 + Husky 재난 현장 탐색</h1>
  <p>Husky 가 도로를 따라 건물 세트를 차례로 돌면, 사족 보행 로봇 Unitree A1 두 대가 뒤따르다가
  각자 맡은 건물에 들어가 SLAM 지도를 만들고 사람을 찾아 Husky 에 보고합니다.</p>
  <p>이 문서는 설치부터 임무 실행, 내부 동작 원리, 노드·토픽 레퍼런스, 시험 결과까지 한 곳에 정리합니다.</p>
</div>
```

![임무 배치도](../images/mission_layout.png)

## 무엇을 하는 프로젝트인가요?

[Gazebo Sim](https://gazebosim.org/) 위에 만든 재난 현장에서 **로봇 세 대가 한 팀으로** 움직입니다.

- **Husky (Clearpath A200)** 는 바퀴 로봇입니다. 도로를 따라 각 건물 세트 앞 주차점까지 자율주행하고, 임무 단계를 정하며, A1 이 찾은 사람을 모읍니다.
- **A1 두 대 (`a1_1`, `a1_2`)** 는 [CHAMP](https://github.com/chvmp/champ) 보행 제어기로 걷는 사족 로봇입니다. 이동할 때는 Husky 뒤 자기 자리를 따라가고, 도착하면 남쪽(`a1_1`)·북쪽(`a1_2`) 건물에 들어가 프런티어 탐색으로 지도를 만듭니다.
- 사람(마네킹)을 카메라로 찾으면 1 초 멈추고 위치를 Husky 에 보냅니다. Husky 와 통신이 끊기면 지나온 길을 되짚어 돌아옵니다.

전체 임무(세트 4개 + 공동 건물, 사람 11명)는 시뮬 시간 약 900~1,050 초, 실제로는 약 50 분 걸립니다. 지금 설정 계열(2026-11-27 이후)의 전체 실행 7 회는 모두 **사람 11/11 발견, 넘어짐 0** 으로 통과(PASS)했습니다 ({doc}`results/test_results`).

## 문서 둘러보기

::::{grid} 1 2 2 3
:gutter: 3

:::{grid-item-card} 🚀 시작하기
:link: getting_started/overview
:link-type: doc

프로젝트 개요, 설치, 빌드, 첫 임무 실행까지 30 분 안에.
:::

:::{grid-item-card} 🧭 서비스 설계
:link: service/deployment
:link-type: doc

제안하는 생존자 탐색 서비스: 사용자·운용 절차, 요소별로 시뮬레이션과 실제 구현.
:::

:::{grid-item-card} 📘 튜토리얼
:link: tutorials/index
:link-type: doc

임무 실행, 화면 보며 직접 시작하기, 통신 끊김 시험, 부분 실행, 결과 읽기, 월드 바꾸기.
:::

:::{grid-item-card} 🛠️ How-To 가이드
:link: how_to/index
:link-type: doc

단위 시험, 직접 조종, 진단 도구, 보행 측정, 파라미터 조정, 문제 해결.
:::

:::{grid-item-card} 🧠 개념
:link: concepts/index
:link-type: doc

구조, 임무 상태기계, 월드 배치 규칙, 추종·탐색·지형·사람 인식·통신 복귀의 원리.
:::

:::{grid-item-card} 📚 레퍼런스
:link: reference/index
:link-type: doc

패키지, 노드별 토픽·파라미터, 토픽 목록, 실행 스크립트 옵션, mission.yaml, 결과 파일 형식.
:::

:::{grid-item-card} 📊 결과와 한계
:link: results/test_results
:link-type: doc

통과한 실행의 수치·그래프, 찾아 고친 원인, 남은 한계, 이전 구조의 기록.
:::
::::

## 처음이라면 이 순서로

1. {doc}`getting_started/overview` — 무엇이 어떻게 움직이는지 5 분 개요
2. {doc}`service/deployment` → {doc}`reference/interfaces` — 어떤 서비스이고 어떻게 요청하는지
3. {doc}`getting_started/installation` → {doc}`getting_started/quickstart` — 설치하고 임무 한 번 돌리기
4. {doc}`tutorials/reading_results` — 결과 폴더 읽는 법
5. {doc}`concepts/architecture` — 노드와 토픽이 어떻게 이어지는지

```{toctree}
:hidden:
:caption: 서비스
:maxdepth: 2

service/deployment
```

```{toctree}
:hidden:
:caption: 시작하기
:maxdepth: 2

getting_started/overview
getting_started/installation
getting_started/quickstart
```

```{toctree}
:hidden:
:caption: 튜토리얼
:maxdepth: 2

tutorials/index
```

```{toctree}
:hidden:
:caption: How-To 가이드
:maxdepth: 2

how_to/index
```

```{toctree}
:hidden:
:caption: 개념
:maxdepth: 2

concepts/index
```

```{toctree}
:hidden:
:caption: 레퍼런스
:maxdepth: 2

reference/index
```

```{toctree}
:hidden:
:caption: 결과와 한계
:maxdepth: 2

results/test_results
results/limitations
results/history
```

```{toctree}
:hidden:
:caption: 기여
:maxdepth: 1

contributing/docs_guide
```
