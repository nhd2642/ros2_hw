# Disaster Exploration Sim Docs

```{raw} html
<div class="hero">
  <span class="hero-tag">MISSION V2 · ROS 2 HUMBLE · GAZEBO SIM</span>
  <h1>Two A1s + a Husky exploring a disaster site</h1>
  <p>A Husky drives down a road past a series of building sets. Two Unitree A1 quadrupeds follow it, then each walks into
  its own building, builds a SLAM map, finds people and reports them back to the Husky.</p>
  <p>These docs cover everything from installation and running a mission to how it works inside, node and topic reference, and test results.</p>
</div>
```

![Mission layout](../images/mission_layout.png)

## What is this project?

Three robots work as **one team** in a disaster site built on [Gazebo Sim](https://gazebosim.org/).

- **Husky (Clearpath A200)** is a wheeled robot. It drives itself to a parking spot in front of each building set, runs the mission phases, and collects the people the A1s find.
- **Two A1s (`a1_1`, `a1_2`)** are quadrupeds walking with the [CHAMP](https://github.com/chvmp/champ) gait controller. While moving they follow their own slot behind the Husky; on arrival they enter the south (`a1_1`) and north (`a1_2`) buildings and map them with frontier exploration.
- When a camera spots a person (a mannequin), the A1 stops for 1 s and sends the position to the Husky. If the link to the Husky drops, it retraces the path it came along.

A full mission (4 sets + a common building, 11 people) takes about 900-1,050 s of sim time, roughly 50 minutes of real time. All 7 full runs of the current setup line (since 2026-11-27) **passed: 11/11 people found, 0 falls** ({doc}`results/test_results`).

## Find your way

::::{grid} 1 2 2 3
:gutter: 3

:::{grid-item-card} 🚀 Getting Started
:link: getting_started/overview
:link-type: doc

Overview, install, build and your first mission in about 30 minutes.
:::

:::{grid-item-card} 🧭 Service design
:link: service/deployment
:link-type: doc

The proposed survivor search service: users, operating procedure, and for each element what the simulation uses and what a real deployment needs.
:::

:::{grid-item-card} 📘 Tutorials
:link: tutorials/index
:link-type: doc

Run a mission, start it by hand with the GUI, test a comm loss, run one set, read results, change the world.
:::

:::{grid-item-card} 🛠️ How-To Guides
:link: how_to/index
:link-type: doc

Unit tests, teleop, diagnostic tools, gait measurement, parameter tuning, troubleshooting.
:::

:::{grid-item-card} 🧠 Concepts
:link: concepts/index
:link-type: doc

Architecture, the mission state machine, layout rules, and how following, exploration, terrain, person detection and comm return work.
:::

:::{grid-item-card} 📚 Reference
:link: reference/index
:link-type: doc

Packages, per-node topics and parameters, the topic list, script options, mission.yaml, result file formats.
:::

:::{grid-item-card} 📊 Results & Limits
:link: results/test_results
:link-type: doc

Numbers and plots from passing runs, causes found and fixed, known limits, the previous architecture.
:::
::::

## New here? Read in this order

1. {doc}`getting_started/overview` — a 5-minute picture of what moves and how
2. {doc}`service/deployment` → {doc}`reference/interfaces` — what the service is and how to request it
3. {doc}`getting_started/installation` → {doc}`getting_started/quickstart` — install and run one mission
4. {doc}`tutorials/reading_results` — how to read a result folder
5. {doc}`concepts/architecture` — how the nodes and topics connect

```{toctree}
:hidden:
:caption: Service
:maxdepth: 2

service/deployment
```

```{toctree}
:hidden:
:caption: Getting Started
:maxdepth: 2

getting_started/overview
getting_started/installation
getting_started/quickstart
```

```{toctree}
:hidden:
:caption: Tutorials
:maxdepth: 2

tutorials/index
```

```{toctree}
:hidden:
:caption: How-To Guides
:maxdepth: 2

how_to/index
```

```{toctree}
:hidden:
:caption: Concepts
:maxdepth: 2

concepts/index
```

```{toctree}
:hidden:
:caption: Reference
:maxdepth: 2

reference/index
```

```{toctree}
:hidden:
:caption: Results & Limits
:maxdepth: 2

results/test_results
results/limitations
results/history
```

```{toctree}
:hidden:
:caption: Contributing
:maxdepth: 1

contributing/docs_guide
```
