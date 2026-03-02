# Project Plan: Smeagol (Evolutionary Robotics) Replication in C++

---

## 1. Implementation Phases & TODOs

### Phase 1: Data Structures, YAML & Core Setup

- [x] **1.1** Setup `CMakeLists.txt` integrating Eigen3, Raylib, and yaml-cpp. Created `.gitignore` (standard C++ & Python rules).
- [x] **1.5** Basic unit tests: `tests/test_dependencies.cpp` verifies Eigen3, yaml-cpp, and raylib compile, link, and run correctly (registered with CTest).
- [x] **1.2** Implement the `RobotPart` base class and its subclasses (`Vertex`, `Bar`, `Neuron`, `Actuator`).
- [x] **1.3** Implement the `Robot` class storing standard `std::vector`s of the parts.
- [x] **1.4** Write the YAML serialization logic. Map the paper's `<vertices><bars>...` format strictly into YAML arrays/objects.
  - Requirement: Ensure specific material properties are globally accessible ($E=0.896$ GPa, $\rho=1000$ kg/m$^3$, $S_{yield}=19$ MPa).

### Phase 2: Shared Rendering Engine (Raylib)

- [x] **2.1** Implement `SceneRenderer`. Set up a fixed 3D orbital camera, a ground plane (`DrawGrid`), and the `drawRobot()` routine.
- [x] **2.2** Implement `SnapshotRenderer`. Add logic to render a single frame and use Raylib's `TakeScreenshot()` to output a still image. Link this to `Robot::saveDebugImage()`.
  - *Refined*: `render()` now calls `drawNeuralOverlay(robot, {})` (empty activations → all neurons shown as silent) so snapshots display neural wiring. HUD text updated to show `v / b / n / a` counts.

### Phase 3: The Physics & Neural Simulator

- [x] **3.1** Energy Function. Implement $H = \sum k_i \delta_i^2 + \sum m_j g h_j$.
  - $k_i = \frac{E A_i}{l_i}$ and $\delta_i = \sqrt{(\Delta x)^2 + (\Delta y)^2 + (\Delta z)^2} - l_i$
- [x] **3.2** Quasi-Static Relaxation. Implement the iterative solver.
  - Calculate partial derivatives $\frac{\partial H}{\partial v}$.
  - Update vertices: $v = v - Ds \frac{\partial H}{\partial v}$.
  - Add uniform noise to prevent settling in unstable equilibria.
- [x] **3.3** Environment Physics.
  - Collision: Add high penalty energy if a vertex $z < 0$.
  - Friction: Lock $x,y$ movement if the lateral force derivative is lower than the static friction coefficient times normal force.
- [x] **3.4** Neural Network Tick. Implement the discrete loop updating neuron activations based on thresholds and synapse coefficients.
- [x] **3.5** Actuator Coupling. Update the target resting length ($l_i$) of bars attached to actuators based on neuron output. Actuators are extension-only: when the linked neuron fires the bar extends by `bar_range` (up to 0.01 m); when it quiesces the bar returns to its base rest length. `bar_range ∈ [0, 0.01]` m only — no negative (contraction) values.
- [x] **3.6** Fitness Wrapper. Implement `evaluate(Robot)`, running the physics/neural loop for N cycles and returning the net CoM Euclidean distance. Implemented `FitnessEvaluator` + `FitnessParams` (cycles, steps_per_cycle, step_size). `evaluate_fitness` CLI tool reads robot + params YAML, prints per-cycle CoM trajectory, optional video output. `examples/robot_fitness/` has `robot.yaml`, `params.yaml`, `run.sh`.
- [x] **3.8** `fall_animation` tool. Inverted tetrahedron (apex down, base at top) placed 0.35 m above the floor with the apex slightly off-centre (+2 cm X, +1 cm Y). Bar rest-lengths set to actual initial distances so no elastic pre-strain. Loop `num_frames` times: call `sim.relax(steps_per_frame, step_size, noise=0, tol=0)` to run exactly N gradient-descent steps (tol=0 prevents early exit), `copyPositionsBack`, `vid.addFrame`. Uses quasi-static relaxation (no velocity / bounce — consistent with Lipson & Pollack (2000) which also uses gradient descent, not Newtonian dynamics). `k_floor = k_bar ≈ 1.4e6` — soft floor is fine for visual validation.

- [x] **3.7** Implement `VideoRenderer`. `addFrame(robot, sim_time)` renders into an off-screen Raylib window and writes numbered PNGs to a unique `/tmp/golem_video_<ts>/` directory. `finish(output_path)` calls `ffmpeg -y -framerate <fps> -i frame_%04d.png -c:v libx264 -pix_fmt yuv420p <output_path>`, cleans up temp frames on success, and is idempotent. Destructor calls `finish("")` automatically. `frameCount()` accessor for tests. CoM trail (dark→gold gradient spheres+lines) and gold displacement HUD auto-rendered every frame. 11/11 CTests passing.

- [x] **3.9** Wind force. Added optional `wind` acceleration [m/s²] to `Simulator` (public field, default 0.0). Applies a constant +X body force to all vertices via `computeGradient()` (same form as gravity). Exposed through `FitnessParams::wind` and parsed from `params.yaml`. Used to sanity-check the fitness evaluator without a working locomotion controller.

- [x] **3.10 — Actuator Ramping** — rest length changes are spread linearly over `steps_per_cycle` relaxation steps instead of a step-change, eliminating elastic shock.

- [x] **3.11 — Stability Fix: Constant `stiffness` field on `Bar`** — removed `radius`; stiffness stored directly on each bar (default `k = 50,000 N/m`). Prevents short bars from becoming numerically unstable. Visual radius is a fixed constant for rendering.

### Phase 4: The Evolver (Genetic Algorithm)

---

#### 4.0 New Files

| File | Purpose |
|---|---|
| `include/Mutator.h` | Stateless mutation functions operating on `Robot&` + `std::mt19937&` |
| `src/Mutator.cpp` | Mutation implementations |
| `include/Evolver.h` | `EvolverParams` struct + `Evolver` class declaration |
| `src/Evolver.cpp` | Evolutionary loop, selection, replacement, logging |
| `tools/evolve.cpp` | CLI entry point (`evolve <config.yaml>`) |
| `tests/test_mutator.cpp` | Unit tests — 1000-call validity checks per operator |
| `tests/test_evolver.cpp` | Integration test — short run, fitness improves |

All new `.cpp` files added to `CMakeLists.txt` under `smeagol_core` and the relevant binary targets.

---

#### 4.1 Population Initialisation

- [x] **4.1** Implement `EvolverParams` struct (`population_size`, `max_evaluations`, `seed`, `FitnessParams fitness`, `MutatorParams mutation`, `video_interval`, `run_dir`, `resume`). Parsed from YAML config by the `evolve` tool.

- [x] Initialise `population_` as `std::vector<Robot>` of size `population_size`, each robot created via `Robot()` default constructor. Each gets a unique auto-assigned ID from `Robot::s_next_id`.

- [x] Seed one `std::mt19937 rng_` from `EvolverParams::seed`. If seed = 0, seed from `std::random_device`. Store seed in `run_config.yaml` for reproducibility.

- [x] Initialise `fitnesses_` as `std::vector<double>` of size `population_size`, all 0.0. Do **not** evaluate the null population.

---

#### 4.2 Mutation Operators

The `Mutator` class is a pure-static utility (or a free-function namespace). It takes `Robot& child` and `std::mt19937& rng` by reference. All four operators are applied independently each call; at minimum one is guaranteed to fire (see 4.2.5).

**Important**: `Robot` already implements `removeBar(idx)`, `removeNeuron(idx)`, `removeVertex(idx)` with full index patching of dependent parts. `addBar`, `addNeuron`, `addActuator` also handle `synapse_weights` resizing. Use these — do not manipulate the vectors directly.

- [x] **4.2.1 — Perturb (p=0.10)** — ±10% bar rest_length, ±0.5 threshold or synapse weight.
- [x] **4.2.2 — Add/Remove element (p=0.01)** — 50/50 add or remove; adds bar (between 2 random vertices) or neuron; removes bar or neuron (with full index patching via `Robot::remove*()`).
- [x] **4.2.3 — Structural split (p=0.03)** — 50/50 split vertex (adds offset copy + connecting bar) or split bar (inserts midpoint vertex, re-routes actuators to the first half-bar).
- [x] **4.2.4 — Attach/Detach toggle (p=0.03)** — flips a random bar's actuated state. If structural: add `Actuator(bar, random_neuron, uniform[0,0.01])`; if actuated: remove all actuators on that bar.
- [x] **4.2.4b — Rewire neuron (p=0.03)** — reassign a random actuator's `bar_idx` or `neuron_idx` (50/50).
- [x] **4.2.5 — "At least one" guarantee** — re-roll if no operator fired.
- [x] **4.2.6 — Post-mutation validation** — reroll up to 100× on `isValid()` failure; fall back to unmutated parent clone.

#### 4.3 Evolutionary Loop

- [x] **4.3.1 — Evolver class**

- [x] **4.3.2 — Fitness-Proportionate Selection (`selectParent`)**

- [x] **4.3.3 — Steady-State Replacement**

- [x] **4.3.4 — Termination Condition**

  Flushes `fitness_log_`, saves `best_robot_final.yaml`, prints final stats.

---

#### 4.4 CLI Tool (`tools/evolve.cpp`) + `MutatorParams` — `[x]` all done

- [x] `evolve [config.yaml]` CLI: creates `runs/run_<timestamp>/`, writes `run_config.yaml`, opens `fitness_log.csv` + `lineage.csv`, prints progress every 200 evals, saves `best_robot_final.yaml` on exit. Returns exit code 130 on SIGINT via `ev.wasInterrupted()`.
- [x] `MutatorParams` struct (16 fields, YAML-round-trippable) in `include/Mutator.h`; threaded through `Mutator::mutate()` as optional last arg (default = `MutatorParams{}`).
- [x] `EvolverParams` carries `MutatorParams mutation` + `bool resume` fields.
- [x] `examples/evolve/config.yaml` has full mutation block.

---

#### 4.5 Tests

- [x] **`tests/test_mutator.cpp`** — 7 sub-tests, each seeded with `mt19937(42)`:
  1. `test_perturb_bar`: seed robot with 2 vertices + 1 bar, call `mutate` 1000×, assert `isValid()` after each call.
  2. `test_perturb_neuron`: seed robot with 2 neurons, call 1000×, assert `isValid()`.
  3. `test_add_bar`: start from empty, call `mutate` until bar exists, verify `isValid()`.
  4. `test_remove_bar`: robot with 3 bars + 2 actuators, call `mutate` (remove) 100×, assert `isValid()` and no out-of-range actuator indices.
  5. `test_split_vertex`: 4-vertex robot, call 100×, assert vertex count increases, `isValid()`.
  6. `test_split_bar`: 6-bar tetrahedron, call 100×, assert bar count increases, `isValid()`.
  7. `test_attach_detach`: robot with 3 bars + 2 neurons, call 500×, assert `isValid()`, actuator `bar_idx` and `neuron_idx` always in range.

- [x] **`tests/test_evolver.cpp`** — integration test: 10 robots, 50 evals, wind fitness; also `test_mutator_params_yaml` YAML round-trip.

### Phase 5: Configuration, Logging & Media Autostore

- [x] **5.1 — `MutatorParams` struct** — expose all mutation knobs as YAML-configurable fields
- [x] **5.2 — Full `config.yaml`** — canonical config file including mutation block; update `examples/evolve/config.yaml`
- [x] **5.3 — Run directory layout** — `runs/run_TIMESTAMP/` with `robots/` (all evaluated children) and `checkpoints/` subdirs
- [x] **5.4 — `fitness_log.csv` schema** — 9 columns: eval, generation, best_fitness, mean_fitness, best_robot_id, best_v/b/n/a
- [x] **5.5 — `lineage.csv` schema** — 6 columns: eval, child_id, parent_id, child_fitness, replaced_id, replaced_fitness
- [x] **5.6 — Periodic best-robot checkpoints** — YAML + PNG every `video_interval` evals
- [x] **5.8 — Automated video generation** — MP4 of best robot every `video_interval` evals via VideoRenderer

---

**Actual run directory layout (as implemented):**
```
runs/run_<timestamp>/
├── run_config.yaml
├── fitness_log.csv              (eval, generation, best_fitness, mean_fitness, best_robot_id, best_v, best_b, best_n, best_a)
├── lineage.csv                  (eval, child_id, parent_id, child_fitness, replaced_id, replaced_fitness)
├── checkpoint_latest.yaml       (current best robot — atomically updated via rename)
├── checkpoint_population.yaml   (full pop + fitnesses — atomically updated; used by resume)
├── best_robot_final.yaml
├── robots/                      (every placed child: robot_<id>.yaml)
└── checkpoints/                 (every video_interval evals: best_eval_N.yaml + .png + .mp4)
```

> **Note:** MP4s are in `checkpoints/` alongside YAML/PNG — no separate `videos/` dir.
> All three formats (YAML, PNG, MP4) fire together every `video_interval` evals; no multiplier.

---

### Phase 6: Multicore Parallelism

`FitnessEvaluator::evaluate()` is the only bottleneck. A full run is ~100,000 calls; each call runs up to 60,000 gradient-descent steps per simulation. Mutation, selection, and I/O are collectively < 1% of wall-clock time. The design philosophy is therefore: **keep evaluation cores saturated at all times**, while dedicating the remaining 1–2 cores to the management work (selection, cloning, mutation, result application, logging).

**Checklist:**
- [x] 6.1–6.5 — Manager + worker pool: `EvalJob`/`EvalResult` structs, worker lambda, flight-window loop
- [x] 6.6 — Thread-safety audit (`Robot::s_next_id` → `std::atomic<Robot::ID>`)
- [x] 6.7–6.9 — Core allocation, CMake `-pthread`, expected speedup validation
- [x] 6.10 — Signal handling and graceful shutdown (Ctrl+C)
- [x] 6.11 — Logging thread-safety: verify all `fitness_log_` / `lineage_log_` writes remain on manager thread
- [x] 6.12 — Crash recovery / incremental resume: `checkpoint_latest.yaml` symlink via atomic `rename()`;  resume by loading final checkpoint + partial `fitness_log.csv`

---

**Implementation notes (as built):**
- `W = max(1, hardware_concurrency() - 1)` workers; `EvalJob{child, replace_idx, parent_id}` / `EvalResult{child, replace_idx, fitness, parent_id}` structs at file scope in `src/Evolver.cpp`
- Worker lambda captures `fitness_params` by value — zero shared state; `FitnessEvaluator` + `Simulator` are stack-local inside each worker
- `submit_one()` / `collect_one()` lambdas manage `std::deque<std::future<EvalResult>>`; flight window stays constant at `W` after fill phase
- `Robot::s_next_id` made `std::atomic<Robot::ID>` for thread-safe ID generation
- `std::signal` (not `sigaction`) used for SIGINT/SIGTERM; `wasInterrupted()` accessor on `Evolver` drives exit code 130 in `evolve.cpp`
- `checkpoint_latest.yaml` + `checkpoint_population.yaml` written atomically via `fs::rename()` every `video_interval` evals
- Resume: `EvolverParams::resume = true` restores population from `checkpoint_population.yaml`; appends to existing CSVs

---

## 2. Project Overview & Dependencies

This project aims to faithfully replicate the 2000 Lipson & Pollack evolutionary robotics experiment ("Golem", this project: Smeagol). The system will evolve 3D mechanical structures (trusses) and their neural controllers simultaneously to maximize locomotion across an infinite plane.

### Core Dependencies

- **C++ Standard:** C++17 or C++20 (for standard filesystem operations, smart pointers, and `<random>`).
- **CMake:** For cross-platform build system management.
- **Eigen3:** A header-only C++ library for fast, vectorized linear algebra. Absolutely critical for the 3D physics gradient descent calculations.
- **Raylib:** A lightweight, hardware-accelerated C-library for rendering the 3D environment, handling camera controls, and drawing basic primitives (spheres, cylinders).
- **yaml-cpp:** A YAML parser and emitter for C++. Used to serialize/deserialize the genotype data cleanly.
- **FFmpeg (System-level):** Used via system calls (`std::system`) to stitch dumped Raylib frames into compressed video files (`.mp4`) automatically.

---

## 3. Software Architecture Overview

The codebase is organized into four major conceptual pillars: Data (`Robot` & `RobotPart`), Physics (`Simulation`), Orchestration (`Evolver`), and Presentation (`RaylibInterface`).

### 3.1 Morphology & Genotype Classes

All physical and neural components inherit from a common base to streamline memory management and standard interfaces.

- **`RobotPart` (Base Class):** Abstract base class.
- **`Vertex : public RobotPart`:** Represents a ball joint. Contains `<x, y, z>` (`Eigen::Vector3d`).
- **`Bar : public RobotPart`:** Represents a rod. Contains `<vertex 1 index, vertex 2 index, relaxed length, stiffness>`.
- **`Neuron : public RobotPart`:** Represents a discrete computational node. Contains `<threshold, std::vector<double> synapse_weights>`.
- **`Actuator : public RobotPart`:** Maps a neuron to a bar. Contains `<bar index, neuron index, bar range>`.
- **`Robot`:** The container class. It aggregates lists of the parts above.
  - I/O Methods: `toYAML()` and `fromYAML(const std::string& path)`.
  - Debug Visualization: Can interface with the rendering system to produce a standalone still image (PNG) of its current static topology.

### 3.2 Visualization Hierarchy

To ensure visual consistency (lighting, floor, materials) across different debugging tools, rendering logic is abstracted into an inheritance tree.

- **`class SceneRenderer` (Base):** Initializes Raylib context, sets up the 3D camera, draws the infinite grid, applies lighting, and implements a `drawRobot(const Robot&)` function mapping vertices to `DrawSphere` and bars to `DrawCylinderEx`.
- **`class SnapshotRenderer : public SceneRenderer`:** Takes a `Robot`, sets the camera angle, renders one frame, saves it as a PNG, and closes. Used by the `Robot` class for static debugging.
- **`class VideoRenderer : public SceneRenderer`:** Maintains an open window. Takes sequential physics states from the `Simulation`, renders them, dumps each frame to a local `/tmp_frames` folder, and calls FFmpeg to compile an MP4 upon completion.

### 3.3 The Simulator Class

A sandboxed environment responsible for simulating exactly one robot.

- **State:** Holds the `Robot` instance, an infinite plane (Z=0), and physical constants.
- **Execution:** Contains the `run(int neural_cycles)` method.
- **Physics (`solveFrame`):** Implements the quasi-static relaxation (energy minimization).
- **Brain (`tickNeural`):** Implements the discrete neural network cycles.
- **Output:** Returns the net Euclidean distance traveled by the Center of Mass. Provides hooks for `VideoRenderer` to capture frames.

### 3.4 The Evolver Class

The top-level orchestrator.

- **State:** Holds a population of 200 `Robot` instances and tracking data for the phylogenetic tree.
- **Execution:** Runs the steady-state genetic algorithm for 300 to 600 generations.
- **Logging:** Dumps generation stats, saves best-performing YAMLs, logs lineage (`parent_id`, `child_id`), and automatically requests `VideoRenderer` to record the best robot every $N$ generations.