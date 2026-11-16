# 문서 작성·빌드 가이드

이 사이트는 `docs_site/` 에 있는 Sphinx 프로젝트입니다. 테마는 Read the Docs, 본문은 Markdown(MyST)입니다.

:::{important}
**지금은 배포하지 않습니다.** 실험 결과가 더 나와 내용이 확정될 때까지 이 사이트는 각자 PC 에서만 띄워 보고 고칩니다. 아래 명령은 모두 내 PC(`127.0.0.1`)에서만 열리며 외부로 내보내는 것이 없습니다.
:::

## 고치면서 보기 (권장)

```bash
cd ~/ros2_hw/docs_site
python3 -m pip install -r requirements.txt      # 처음 한 번
make live                                         # 한국어, http://127.0.0.1:8000
make live DOCLANG=en                              # 영어
```

`make live` 는 파일을 저장할 때마다 다시 빌드하고 브라우저를 새로 고칩니다. 원고(`ko/`, `en/`)뿐 아니라 테마(`shared/`), 다이어그램(`diagrams/`), 그림(`images/`)도 지켜봅니다.

## 전체 빌드

```bash
make html                                         # ko, en 둘 다 → _build/html/
make serve                                        # http://127.0.0.1:8000 (언어 선택 첫 화면 포함)
```

## 새 실험 결과 반영하기

결과가 새로 나오면 이 순서로 고칩니다.

1. **그래프 다시 그리기** — 새 결과 폴더 이름(`results/` 아래)을 넘깁니다. 첫 번째 실행으로 경로·SLAM 오차·사람 그래프를, 넘긴 실행 전부로 단계 시간 그래프를 그립니다.

   ```bash
   make figures RUNS="20261120_101500_set0 20261114_062151_set0"
   ```

   인자 없이 `make figures` 하면 지금 문서의 두 PASS 실행으로 돌아갑니다.
2. **숫자 고치기** — 결과 숫자가 들어 있는 페이지 (두 언어 모두):

   | 페이지 | 고칠 것 |
   | --- | --- |
   | `results/test_results.md` | 요약 카드, 단계별 표, 재현성 표, 원인·조치 표 |
   | `results/limitations.md` | 해결된 한계 빼기, 새 한계 넣기 |
   | `index.md` | 첫 화면의 "사람 9/9, 넘어짐 0" 문장 |
   | `tutorials/reading_results.md`, `tutorials/comm_loss.md` | 예로 든 실행 이름과 출력 |
   | `how_to/measure_gait.md` | 보행을 다시 쟀다면 표와 `make_figures.py` 의 `gait()` 값 |
3. **설정값이 바뀌었다면** — `reference/nodes.md`, `how_to/tune_parameters.md` 의 기본값 표.
4. `make html` 이 경고 없이 끝나는지 확인합니다 (`python3 -m sphinx -W -b html ko /tmp/check`).

그림 다이어그램은 빌드 때 Graphviz 로 그리므로 `dot` 이 있어야 합니다.

```bash
sudo apt install -y graphviz
```

| make 대상 | 하는 일 |
| --- | --- |
| `make html` | 한국어·영어 모두 빌드, `_build/html/index.html` 은 브라우저 언어로 이동 |
| `make ko` / `make en` | 한 언어만 |
| `make figures [RUNS="…"]` | `results/` 의 timeline 으로 결과 그래프 다시 그림 |
| `make serve` / `make live` | 빌드 후 로컬 서버 / 저장할 때마다 다시 빌드 |
| `make clean` | `_build/` 삭제 |

## 폴더 구조

```text
docs_site/
├── ko/, en/            # 언어별 원고 (같은 파일 이름·구조), 각자 conf.py
├── shared/             # 공통 설정 conf_common.py, 테마 CSS·JS·템플릿, screenshot 확장
├── diagrams/           # Graphviz .dot (두 언어가 같이 씀, 라벨은 영어)
├── images/             # 기존 그림, figures/ (생성 그래프), screenshots/ (스크린샷)
├── scripts/            # make_figures.py, shot.sh
└── Makefile, requirements.txt, index.html
```

**페이지를 고치면 두 언어 모두 고치세요.** 파일 이름과 위치가 같아야 사이드바의 언어 전환 버튼이 같은 페이지로 갑니다.

## 스크린샷 넣기

문서 곳곳에 빗금 상자로 된 **'스크린샷 자리'** 가 있습니다. 상자에 적힌 파일 이름으로 PNG 를 저장하고 다시 빌드하면 그 자리에 그림이 들어갑니다.

```bash
cp ~/Pictures/world.png ~/ros2_hw/docs_site/images/screenshots/world_overview.png
make clean html
```

지금 필요한 스크린샷:

| 파일 | 넣을 화면 | 쓰는 페이지 |
| --- | --- | --- |
| `world_overview.png` | Gazebo 위에서 본 전체 월드 | 프로젝트 개요 |
| `start_formation.png` | 출발점 Husky + A1 두 대 | 빠른 시작 |
| `transit_follow.png` | 도로 이동 중 추종 | 튜토리얼 1 |
| `explore_rviz.png` | RViz 지도·경로·사람 표시 | 튜토리얼 1 |
| `terminal_start.png` | `husky_mission` 터미널에 `start` | 튜토리얼 2 |
| `gazebo_gui.png` | Gazebo 창 전체 (엔티티 트리, RTF) | 튜토리얼 2 |
| `comm_loss_return.png` | 통신 0 직후 복귀하는 RViz | 튜토리얼 3 |
| `common_building.png` | 공동 건물에서 두 A1 이 갈라지는 모습 | 튜토리얼 4 |
| `ramp.png` | 경사로를 넘는 A1 | 월드와 배치 규칙 |
| `survivor_detect.png` | 앞 카메라 인식 상자 + 사람 표시 | 사람 인식 |
| `rqt_graph_v2.png` | 지금 구조의 rqt_graph | 시스템 구조 |

Markdown 에서 새 자리를 만들려면:

````markdown
```{screenshot} my_shot.png
여기에 넣을 화면 설명 (그림이 생기면 캡션이 됨).
```
````

## 쓰는 요소

````markdown
```{graphviz} ../../diagrams/mission_states.dot
```

```{figure} ../../images/figures/trajectories.png
:width: 100%

캡션
```

:::{note}
참고 상자 (note, tip, warning, important)
:::
````

## 색과 스타일

| 이름 | 색 | 쓰는 곳 |
| --- | --- | --- |
| Denim 950 | `#0E1014` | 사이드바 머리, 히어로 |
| Denim 800 | `#1F242B` | 다이어그램 노드, 표 머리, 코드 블록 |
| Denim 600 | `#3A4250` | 보조 노드, 테두리 |
| Knit red | `#C8102E` | **유일한 강조색**: 링크, 제목 줄, 활성 메뉴, 주요 흐름 |
| Knit deep | `#8F0B20` | 링크 hover, 강조 노드 |
| Paper | `#FAFAF8` | 본문 배경 |

그래프 색은 개체마다 고정입니다: `a1_1` = 빨강 `#C8102E`, `a1_2` = 데님 블루 `#4C6A92`, Husky = 진회색 `#2A3038` 점선. 색맹 시뮬레이션에서 모든 쌍의 구분이 ΔE 12 이상입니다.

## 화면 확인

WSL 에서 Windows Chrome 으로 빌드 결과를 캡처할 수 있습니다.

```bash
bash scripts/shot.sh ko/index.html /tmp/ko_index.png 1400 2400
```
