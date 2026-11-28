# Writing & Building These Docs

This site is a Sphinx project in `docs_site/`. The theme is Read the Docs and pages are written in Markdown (MyST).

:::{important}
**The site is not published yet.** Until more test results are in and the content settles, everyone runs and edits it on their own PC. Every command below serves only on your machine (`127.0.0.1`); nothing is uploaded anywhere.
:::

## Preview while editing (recommended)

```bash
cd ~/ros2_hw/docs_site
python3 -m pip install -r requirements.txt      # once
make live                                         # Korean, http://127.0.0.1:8000
make live DOCLANG=en                              # English
```

`make live` rebuilds and refreshes the browser on every save. It watches the pages (`ko/`, `en/`) as well as the theme (`shared/`), diagrams (`diagrams/`) and images (`images/`).

## Full build

```bash
make html                                         # ko and en → _build/html/
make serve                                        # http://127.0.0.1:8000 (with the language chooser page)
```

## Updating for new test results

When new results come in, update in this order.

1. **Redraw the plots** — pass the new result folder names (under `results/`). The first run is used for the path, SLAM error and people plots; all runs go into the phase timeline.

   ```bash
   make figures RUNS="20261120_101500_set0 20261114_062151_set0"
   ```

   `make figures` without arguments goes back to the two PASS runs used now.
2. **Fix the numbers** — pages that contain result numbers (in both languages):

   | Page | What to update |
   | --- | --- |
   | `results/test_results.md` | summary cards, per-phase table, reproducibility table, causes-and-fixes table |
   | `results/limitations.md` | remove solved limits, add new ones |
   | `index.md` | the full-run result sentence on the home page (also times and counts in `getting_started/overview.md`, `quickstart.md` and numbers in `service/deployment.md`) |
   | `tutorials/reading_results.md`, `tutorials/comm_loss.md` | example run names and output |
   | `how_to/measure_gait.md` | if the gait was re-measured, the table and the `gait()` values in `make_figures.py` |
3. **If settings changed** — the default tables in `reference/nodes.md` and `how_to/tune_parameters.md`.
4. Check that the build finishes without warnings (`python3 -m sphinx -W -b html en /tmp/check`).

Diagrams are drawn by Graphviz at build time, so `dot` must be installed.

```bash
sudo apt install -y graphviz
```

| make target | Does |
| --- | --- |
| `make html` | builds Korean and English; `_build/html/index.html` redirects by browser language |
| `make ko` / `make en` | one language only |
| `make figures [RUNS="…"]` | redraws the result plots from the timelines in `results/` |
| `make serve` / `make live` | build then serve locally / rebuild on every save |
| `make clean` | deletes `_build/` |

## Folder layout

```text
docs_site/
├── ko/, en/            # pages per language (same file names and structure), each with its conf.py
├── shared/             # shared conf_common.py, theme CSS/JS/templates, the screenshot extension
├── diagrams/           # Graphviz .dot (shared by both languages, English labels)
├── images/             # existing figures, figures/ (generated plots), screenshots/
├── scripts/            # make_figures.py, shot.sh
└── Makefile, requirements.txt, index.html
```

**When you change a page, change it in both languages.** File names and locations must match so the sidebar language switch lands on the same page.

## Adding screenshots

Hatched **"Screenshot needed"** boxes appear throughout the docs. Save a PNG under the file name shown in the box and rebuild; the picture replaces the box.

```bash
cp ~/Pictures/world.png ~/ros2_hw/docs_site/images/screenshots/world_overview.png
make clean html
```

Screenshots currently needed:

| File | What to capture | Used on |
| --- | --- | --- |
| `world_overview.png` | top-down Gazebo view of the whole world | Project Overview |
| `start_formation.png` | Husky + two A1s at the start | Quickstart |
| `transit_follow.png` | following on the road | Tutorial 1 |
| `explore_rviz.png` | RViz map, path, person markers | Tutorial 1 |
| `terminal_start.png` | `start` typed in the `husky_mission` terminal | Tutorial 2 |
| `gazebo_gui.png` | the whole Gazebo window (entity tree, RTF) | Tutorial 2 |
| `comm_loss_return.png` | RViz while returning right after comm 0 | Tutorial 3 |
| `common_building.png` | the two A1s splitting up in the common building | Tutorial 4 |
| `ramp.png` | an A1 crossing a ramp | World & Layout Rules |
| `survivor_detect.png` | front camera detection box + person marker | Person Detection |
| `rqt_graph_v2.png` | rqt_graph of the current architecture | Architecture |

To add a new placeholder in Markdown:

````markdown
```{screenshot} my_shot.png
Description of what to capture (becomes the caption once the image exists).
```
````

## Building blocks

````markdown
```{graphviz} ../../diagrams/mission_states.dot
```

```{figure} ../../images/figures/trajectories.png
:width: 100%

Caption
```

:::{note}
A callout (note, tip, warning, important)
:::
````

## Colors and style

| Name | Color | Used for |
| --- | --- | --- |
| Denim 950 | `#0E1014` | sidebar header, hero |
| Denim 800 | `#1F242B` | diagram nodes, table headers, code blocks |
| Denim 600 | `#3A4250` | secondary nodes, borders |
| Knit red | `#C8102E` | **the only accent**: links, heading rules, active menu, main flows |
| Knit deep | `#8F0B20` | link hover, emphasized nodes |
| Paper | `#FAFAF8` | page background |

Plot colors are fixed per entity: `a1_1` = red `#C8102E`, `a1_2` = denim blue `#4C6A92`, Husky = dark grey `#2A3038`, dashed. In color-vision-deficiency simulation every pair is separated by at least ΔE 12.

## Checking the look

On WSL you can capture a built page with Windows Chrome.

```bash
bash scripts/shot.sh en/index.html /tmp/en_index.png 1400 2400
```
