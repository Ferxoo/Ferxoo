# Open-Source MATLAB Simulation Environment for Collision Avoidance on the ABB IRB 1200

## Overview

This repository provides a complete, self-contained MATLAB simulation environment for studying collision avoidance and human-robot collaboration safety on the ABB IRB 1200 5/0.9 industrial robot arm. It implements four motion-planning algorithms (RRT, RRT-Connect, RRT*, and APF) together with an ISO/TS 15066 Speed and Separation Monitoring (SSM) safety layer that dynamically scales robot velocity and triggers replanning when an operator approaches. All code is written in pure MATLAB using only the Robotics System Toolbox — no ROS, no Simscape, and no proprietary hardware required.

The project was developed as a Final Degree Project (PFG) at Universidad Francisco de Vitoria and is intended as both a research tool and an open educational resource for the robotics community.

---

## Author and Institution

| | |
|---|---|
| **Author** | Fernando Aquilino Gatell Valor |
| **Institution** | Universidad Francisco de Vitoria — Escuela Politécnica Superior |
| **Degree** | Grado en Ingeniería en Sistemas Industriales |
| **Tutor** | Roque Antonio Peña Pidal |
| **Academic year** | 2025/26 |

---

## Requirements

| Requirement | Notes |
|---|---|
| MATLAB R2025b | Tested on **R2025b**. All planners are custom implementations — no toolbox version constraint beyond R2025b. |
| **Robotics System Toolbox** | **Mandatory.** Used only for `importrobot`, `rigidBodyTree`, `getTransform`, and `checkCollision`. |
| Statistics and Machine Learning Toolbox | Recommended for benchmark statistics; built-in `mean`/`std` functions are used as fallback |
| **abb-noetic-devel** folder | Must be present in the parent directory (see Quick Start) |

### Obtaining the ABB robot model

Clone the ROS-Industrial ABB driver (only the `abb_irb1200_support` package is needed):

```bash
git clone https://github.com/ros-industrial/abb \
    entorno_OS_robotsimulation/abb-noetic-devel
```

---

## Quick Start

```matlab
% 1. Place this folder at: entorno_OS_robotsimulation/irb1200_simulation/
% 2. Ensure abb-noetic-devel/ is in entorno_OS_robotsimulation/
% 3. Open MATLAB R2025b
cd irb1200_simulation
main_simulation
```

The script runs in approximately 2–5 minutes (excluding the optional full benchmark).

---

## Repository Structure

| File / Folder | Description |
|---|---|
| `main_simulation.m` | Master script — runs the full demo end-to-end |
| `setup_environment.m` | Path setup, toolbox check, URDF verification, directory creation |
| `README.md` | This file |
| `LICENSE` | MIT Licence |
| **robot/** | |
| `robot/load_irb1200.m` | Loads `rigidBodyTree` from URDF; adds analytical collision primitives |
| `robot/validate_kinematics.m` | FK sanity check against known configurations |
| `robot/get_joint_limits.m` | Returns 6×2 limit matrix in radians |
| **environment/** | |
| `environment/build_scenario.m` | Factory function — returns obstacle array by scenario ID |
| `environment/scenario_free_space.m` | Scenario 1: no obstacles |
| `environment/scenario_central.m` | Scenario 2: single box at workspace centre |
| `environment/scenario_corridor.m` | Scenario 3: two walls forming a corridor |
| `environment/scenario_dense.m` | Scenario 4: five mixed-geometry obstacles |
| **operator/** | |
| `operator/get_operator_position.m` | Returns operator [x,y,z] at time t for four motion modes |
| **ssm/** | |
| `ssm/compute_min_distance.m` | Conservative surface-to-surface distance: robot ↔ operator sphere |
| `ssm/ssm_state_machine.m` | NORMAL/SLOW/STOP_REPLAN state machine with hysteresis |
| **planners/** | |
| `planners/plan_rrt.m` | Basic RRT (LaValle 1998) |
| `planners/plan_rrt_connect.m` | Bidirectional RRT / RRT-Connect (Kuffner & LaValle 2000) |
| `planners/plan_rrt_star.m` | RRT* — **main algorithm** (Karaman & Frazzoli 2011) |
| `planners/plan_apf.m` | Artificial Potential Fields in joint space (Khatib 1986) |
| **trajectory/** | |
| `trajectory/execute_with_ssm.m` | Animates trajectory with live SSM monitoring and replanning |
| `trajectory/compute_metrics.m` | Computes time, path length, jerk per planned path |
| **evaluation/** | |
| `evaluation/run_benchmark.m` | 30 iter × 4 scenarios × 4 algorithms |
| `evaluation/aggregate_stats.m` | mean, std, success rate per combination |
| `evaluation/save_results.m` | Exports `.mat` and `.csv`; prints ASCII summary table |
| **visualization/** | |
| `visualization/draw_scene.m` | 3D robot + obstacles + operator + SSM zone wireframes |
| `visualization/draw_path.m` | 6-panel joint-space trajectory plot with limit lines |
| `visualization/draw_benchmark.m` | 4-panel grouped bar chart with error bars |
| `visualization/draw_ssm_log.m` | Distance vs time with coloured safety zones and speed factor |
| **results/** | Auto-created at runtime |

---

## Algorithms

| Algorithm | Class | Reference | Role |
|---|---|---|---|
| **RRT** | Sampling-based | LaValle, S.M. (1998). *Rapidly-exploring random trees: A new tool for path planning.* TR 98-11, Iowa State. | Baseline probabilistic planner |
| **RRT-Connect** | Sampling-based (bidirectional) | Kuffner, J.J. & LaValle, S.M. (2000). *RRT-Connect: An efficient approach to single-query path planning.* ICRA. | Faster single-query variant |
| **RRT\*** | Sampling-based (optimal) | Karaman, S. & Frazzoli, E. (2011). *Sampling-based algorithms for optimal motion planning.* IJRR 30(7). | **Main planner** — asymptotically optimal |
| **APF** | Reactive gradient | Khatib, O. (1986). *Real-time obstacle avoidance for manipulators and mobile robots.* IJRR 5(1). | Baseline reactive planner |

---

## Safety Implementation — ISO/TS 15066 SSM Mode

Speed and Separation Monitoring regulates robot speed based on the measured distance to a detected operator:

```
Distance d          SSM State      Speed factor
──────────────────────────────────────────────────────
d > 1.05 m          NORMAL         1.0  (full speed)
0.55 < d ≤ 1.05 m   SLOW           0.05 … 1.0  (linear)
d ≤ 0.55 m          STOP_REPLAN    0.0  (full stop + replan)

Hysteresis offsets (+5 cm) prevent chattering at zone boundaries.
```

ASCII state diagram:

```
           d ≤ 1.0 m                      d ≤ 0.5 m
  NORMAL ──────────────► SLOW ──────────────────► STOP_REPLAN
    ▲                      │                           │
    │      d ≥ 1.05 m      │           d ≥ 0.55 m      │
    └──────────────────────┘ ◄─────────────────────────┘
```

---

## Benchmark Scenarios

| ID | Name | Description | Obstacles |
|---|---|---|---|
| 1 | Free space | Unobstructed workspace; baseline | 0 |
| 2 | Central box | Single 180×350×450 mm box at [0.32, 0, 0.30] m | 1 box |
| 3 | Corridor | Two 60×600×700 mm walls creating a passage | 2 boxes |
| 4 | Dense | Mixed boxes and cylinders across full workspace | 5 (3 boxes + 2 cylinders) |

---

## Evaluation Metrics

| Metric | Unit | Description |
|---|---|---|
| Computation time | s | Wall-clock time from `plan()` call to return |
| Path length | rad | Sum of L2 norms between consecutive 6-DOF configurations |
| Mean jerk | rad/s³ | Mean magnitude of the third finite difference of joint positions |
| Success rate | % | Fraction of planning calls that returned a valid collision-free path |

---

## License

MIT License — see [LICENSE](LICENSE).

---

## Citation

```bibtex
@mastersthesis{GatellValor2025IRB1200,
  author      = {Gatell Valor, Fernando Aquilino},
  title       = {Open-Source MATLAB Simulation Environment for Collision
                 Avoidance on the ABB IRB 1200},
  school      = {Universidad Francisco de Vitoria},
  year        = {2026},
  type        = {Trabajo de Fin de Grado (PFG)},
  program     = {Grado en Ingeniería en Sistemas Industriales},
  address     = {Pozuelo de Alarcón, Madrid, Spain},
  note        = {Tutor: Roque Antonio Peña Pidal.}
}
```

---

## Acknowledgements

- **Universidad Francisco de Vitoria** — academic support and laboratory resources.
- **Seoul National University of Science and Technology (Seoultech)** — exchange programme 2025.
- **ROS-Industrial Consortium** — ABB robot URDF models and support packages (`abb-noetic-devel`).
- **MathWorks** — Robotics System Toolbox documentation and examples.
