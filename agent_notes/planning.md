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

- [x] **3.10 — Actuator Ramping (smooth actuation)**

  **Problem:** The current `applyActuators()` instantly sets each bar's rest length to `base + activation * bar_range` at the start of a neural cycle. This creates a step-change in the energy minimum: on the very next relaxation step the gradient is huge, causing an elastic shock that propagates violently through the structure before the solver gradually damps it out. This is physically unrealistic and numerically harsh.

  **Fix:** Model each actuator as a physical device that linearly extends/retracts over the duration of the neural cycle. The rest length should arrive at its new target only after `steps_per_cycle` relaxation steps, not immediately.
  - [x] **3.11 — Stability Fix: Decouple Stiffness from Length (Recommendation)**

  **Problem:** Currently, $k = \frac{EA}{L}$. The evolutionary operators (especially mutating length, or splitting bars) can create extremely short, thick bars. A short, thick bar approaches infinite stiffness ($k \to \infty$). Because the simulator uses a fixed `step_size` for gradient descent, encountering a highly stiff bar violates the stability threshold $\left( \text{step\_size} < \frac{1}{2 k_{\max} d_{\max}} \right)$, causing the simulation to instantly diverge/explode (numerical instability).

  **Recommendation:** Match the original paper's likely approach and treat the structure as a network of idealized springs rather than continuous volumetric rods.
  - **Remove `radius`** from the `Bar` genotype.
  - **Store `stiffness` ($k$) directly** on the `Bar`, either as a fixed global constant (e.g., $50,000 \text{ N/m}$) or as an evolvable parameter bounded within a safe range (e.g., $[10000, 100000] \text{ N/m}$).
  - For rendering, draw all bars with a fixed visual radius (e.g., 1.5mm), or determine visual representation from $k$.
  - For mass, back-calculate assuming a fixed material density, or distribute a fixed mass per vertex.
  - Doing this guarantees that no mutation can ever violate the $k_{safe}$ threshold, allowing us to keep `step_size` constant and fast during the entire evolutionary run.

  **Implementation:**

  - Add two new private vectors to `Simulator`:
    ```cpp
    std::vector<double> target_rest_lengths_;    // where each bar should be at end of cycle
    std::vector<double> rest_length_step_delta_; // per-relaxation-step increment
    ```
    Both initialised to zero / matching `base_rest_lengths_` at construction.

  - `applyActuators()` (called once per neural cycle, before `relax()`):
    - Computes `target = base_rest_lengths_[bar] + clamp(activation * bar_range, 0.0, 0.01)`.
      - When neuron fires (`activation = 1.0`): bar extends to `base + bar_range`.
      - When neuron quiesces (`activation = 0.0`): bar returns to `base` (fully retracted).
      - `bar_range` is always non-negative, so bars can only extend, never contract below their genotype rest length.
    - Stores `target_rest_lengths_[bar] = target`.
    - Computes `rest_length_step_delta_[bar] = (target - rest_lengths_[bar]) / steps_per_cycle`.
    - Does **not** change `rest_lengths_` yet.

  - `relax()` (called each step of the cycle):
    - At the start of each gradient-descent iteration, applies:
      ```cpp
      for each bar i: rest_lengths_[i] += rest_length_step_delta_[i];
      ```
    - After exactly `steps_per_cycle` steps, `rest_lengths_[i] ≈ target_rest_lengths_[i]`.
    - Non-actuated bars have `rest_length_step_delta_[i] = 0.0` so they are unaffected.

  - `applyActuators()` must be passed `steps_per_cycle` so it can compute the per-step delta:
    ```cpp
    void applyActuators(int steps_per_cycle);
    ```
    Update all call sites (`evaluate_fitness.cpp`, `FitnessEvaluator.cpp`, tests).

  **Result:** The elastic energy change is spread evenly across the full `steps_per_cycle` relaxation budget, eliminating the shock. The structure has time to smoothly deform toward the new equilibrium without the solver chasing a sudden large gradient spike.

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

- [x] **4.1** Implement `EvolverParams` struct:
  ```cpp
  struct EvolverParams {
      int    population_size  = 200;
      int    max_evaluations  = 100'000;
      int    seed             = 42;        // 0 = use random_device
      FitnessParams fitness;               // cycles, steps_per_cycle, step_size
      int    video_interval   = 1000;      // save video every N evals
      std::string run_dir     = "";        // auto-set to runs/run_<timestamp>/
  };
  ```
  Parsed from a YAML config file by the `evolve` tool.

- [x] Initialise `population_` as `std::vector<Robot>` of size `population_size`, each robot created via `Robot()` default constructor (empty: 0 vertices, 0 bars, 0 neurons, 0 actuators). Each gets a unique auto-assigned ID from `Robot::s_next_id`.

- [x] Seed one `std::mt19937 rng_` from `EvolverParams::seed`. If seed = 0, seed from `std::random_device`. Store seed in `run_config.yaml` in the run directory for reproducibility.

- [x] Initialise `fitnesses_` as `std::vector<double>` of size `population_size`, all set to 0.0. Do **not** evaluate the null population — they all score 0.0, which is correct and saves $200 \times$ eval cost.

---

#### 4.2 Mutation Operators

The `Mutator` class is a pure-static utility (or a free-function namespace). It takes `Robot& child` and `std::mt19937& rng` by reference. All four operators are applied independently each call; at minimum one is guaranteed to fire (see 4.2.5).

**Important**: `Robot` already implements `removeBar(idx)`, `removeNeuron(idx)`, `removeVertex(idx)` with full index patching of dependent parts. `addBar`, `addNeuron`, `addActuator` also handle `synapse_weights` resizing. Use these — do not manipulate the vectors directly.

- [x] **4.2.1 — Perturb (p = 0.10)**

  Applied independently to bars and neurons; either or both can fire in one call.

  *Bars* (skip if `child.bars` is empty):
  - Pick a random bar index `b`.
  - Perturb `bars[b].rest_length` by a ±10% uniform draw: `L *= uniform(0.9, 1.1)`. Clamp to `[0.01, 1.0]` m to prevent degenerate bars.

  *Neurons* (skip if `child.neurons` is empty):
  - Pick a random neuron index `n`.
  - With equal probability, either:
    - Perturb `neurons[n].threshold` by additive uniform draw in `[-0.5, +0.5]`. Clamp to `[0.0, 2.0]`.
    - Pick a random synapse index `s` (skip if `synapse_weights` is empty) and add uniform `[-0.5, +0.5]`. No clamping — weights can be negative to allow inhibition.

- [x] **4.2.2 — Add / Remove Element (p = 0.01)**

  One coin flip per call: 50% add, 50% remove.

  *Add bar* (requires ≥ 2 vertices):
  - Pick two distinct random vertex indices `v1, v2`.
  - Compute `rest_length = (vertices[v1].position - vertices[v2].position).norm()`. If < 1e-6 (coincident), skip.
  - Call `child.addBar(Bar(v1, v2, rest_length))`.  // uses k_default = 50,000 N/m

  *Add neuron* (always possible):
  - `threshold` = uniform `[0.0, 1.0]`.
  - `synapse_weights` = zero vector of length `N` (current neuron count before add, will be resized to `N+1` by `addNeuron`).
  - Call `child.addNeuron(...)`.

  *Remove bar* (requires ≥ 1 bar):
  - Pick a random bar index and call `child.removeBar(idx)`. This automatically removes dependent actuators and patches remaining actuator `bar_idx` values.

  *Remove neuron* (requires ≥ 1 neuron):
  - Pick a random neuron index and call `child.removeNeuron(idx)`. This automatically patches actuator `neuron_idx` values, removes dependent actuators, and erases the column from all other neurons' `synapse_weights`.

- [x] **4.2.3 — Structural Split (p = 0.03)**

  One coin flip per call: 50% split vertex, 50% split bar.

  *Split vertex* (requires ≥ 1 vertex):
  - Pick a random vertex `v`.
  - Compute a small random offset: `offset = Vector3d(uniform(-0.01,0.01), uniform(-0.01,0.01), uniform(-0.01,0.01))`. The new vertex lands at `vertices[v].position + offset`.
  - Add the new vertex at index `v_new = child.addVertex(...)`.
  - Add a bar connecting `v` and `v_new` with `rest_length = offset.norm()` (or 0.01 if offset is near-zero), `stiffness = Materials::k_default`.

  *Split bar* (requires ≥ 1 bar):
  - Pick a random bar `B` with indices `(v1, v2)` and `rest_length L0`, at index `b`.
  - Compute midpoint `mid = (vertices[v1].position + vertices[v2].position) * 0.5`.
  - Add vertex at `mid`: `v_mid = child.addVertex(Vertex(mid))`.
  - Record all actuator indices pointing to `b` (to re-route them).
  - Call `child.removeBar(b)`. This patches all actuator `bar_idx` values and removes actuators on `b`.
  - Add `Bar(v1, v_mid, L0/2, B.stiffness)` → index `b1`.
  - Add `Bar(v_mid, v2, L0/2, B.stiffness)` → index `b2`.
  - For each recorded actuator that was removed: re-add it pointing to `b1` (the first half), preserving its `neuron_idx` and `bar_range`. This ensures neural control is not lost.

- [x] **4.2.4 — Neural Attach / Detach (p = 0.03)** *(refined: flip-toggle)*

  Picks a random bar and FLIPS its actuated state:
  - If the bar is structural (no actuator): add `Actuator(bar, random_neuron, uniform[0,0.01])`. If no neuron exists yet, one is created via `addConnectedNeuron` first.
  - If the bar is already actuated: remove ALL actuators targeting that bar (full detach).

  This matches the paper's spirit more faithfully than a separate add/remove coin-flip.

- [x] **4.2.4b — Topological Rewire Neuron (p = 0.03)** *(new operator)*

  `Mutator::rewireNeuron`: picks a random actuator and reassigns either its `bar_idx` (physical target) or its `neuron_idx` (neural source) to a different valid index (50/50). Requires ≥ 1 actuator and at least 2 bars or 2 neurons to have an effect. Returns false if unable.

- [x] **4.2.5 — "At Least One" Guarantee** (updated for 5 operators)

- [ ] **4.2.6 — Post-Mutation Validation**

  After all mutations, call `child.isValid()`. If it returns `false`, log a warning and discard the child (use the parent clone unchanged with a small perturbation). This should be extremely rare given that the `Robot` mutators already maintain index consistency.

---

#### 4.3 Evolutionary Loop

- [ ] **4.3.1 — Evolver class**

  ```cpp
  class Evolver {
  public:
      explicit Evolver(EvolverParams params);
      void run();                          // main loop
      const Robot& bestRobot() const;
      double       bestFitness() const;
  private:
      int selectParent() const;            // fitness-proportionate roulette
      int selectReplacement() const;       // uniform random
      void evaluateOne(int idx);           // fitness eval + update fitnesses_[idx]
      void maybeSaveSnapshot(int eval_num);

      EvolverParams            params_;
      std::vector<Robot>       population_;
      std::vector<double>      fitnesses_;
      std::mt19937             rng_;
      int                      eval_count_   = 0;
      int                      best_idx_     = 0;
      // logging (Phase 5)
      std::ofstream            fitness_log_;
      std::ofstream            lineage_log_;
  };
  ```

- [ ] **4.3.2 — Fitness-Proportionate Selection (`selectParent`)**

  - Sum all fitnesses: `total = Σ fitnesses_[i]`.
  - If `total == 0.0` (early run, all robots empty): return `uniform_int(0, N-1)` — uniform random selection.
  - Otherwise: draw `r = uniform(0.0, total)`, iterate with running sum until `sum > r`, return that index. This is standard roulette-wheel selection.
  - Selection pressure scales naturally: a robot with 2× the fitness gets 2× the chance of being selected as parent.

- [ ] **4.3.3 — Steady-State Replacement**

  Each iteration of the main loop:
  1. `parent_idx = selectParent()` — fitness-proportionate.
  2. `Robot child = population_[parent_idx].clone()` — deep copy.
  3. `child.id = Robot::nextId()` — assign fresh ID.
  4. `Mutator::mutate(child, rng_)` — apply 4 operators.
  5. `double child_fitness = FitnessEvaluator(params_.fitness).evaluate(child)` — full evaluation.
  6. `replace_idx = selectReplacement()` — uniform random over entire population (including possibly the parent slot — this is intentional per L&P).
  7. `population_[replace_idx] = std::move(child)` and `fitnesses_[replace_idx] = child_fitness`.
  8. Update `best_idx_` if `child_fitness > fitnesses_[best_idx_]`.
  9. Increment `eval_count_`. Log. Call `maybeSaveSnapshot`.

- [ ] **4.3.4 — Termination Condition**

  Loop until `eval_count_ >= params_.max_evaluations` (default 100,000). This matches the L&P paper's $\approx 10^5$ evaluations over 300–600 generations. With `population_size = 200`, one "generation" = 200 evaluations, so 100,000 evals = 500 generations.

  Optionally handle `SIGINT` cleanly: catch the signal, set a `stop_requested_` flag, and let the loop exit at the next iteration rather than crashing mid-eval.

---

#### 4.4 CLI Tool (`tools/evolve.cpp`)

- [ ] Accepts one argument: path to a YAML config file.
  ```yaml
  # evolve_config.yaml
  population_size:  200
  max_evaluations:  100000
  seed:             42
  video_interval:   1000
  fitness:
    cycles:          12
    steps_per_cycle: 50000
    step_size:       1.0e-7
  ```
- [ ] On startup: creates `runs/run_<ISO-timestamp>/`, writes `run_config.yaml` copy there, opens `fitness.csv` and `lineage.csv`.
- [ ] Prints progress to stdout every 200 evaluations: `eval=N  best=X.XXXm  mean=X.XXXm`.
- [ ] On exit (normal or SIGINT): saves best robot as `best_robot.yaml`.

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

- [ ] **`tests/test_evolver.cpp`** — integration test:
  - `EvolverParams` with `population_size=10`, `max_evaluations=50`, `seed=1`.
  - `fitness.wind = 9.8` (use wind mode so even empty robots get nonzero fitness and selection pressure is immediate).
  - Run `evolver.run()`.
  - Assert `evolver.bestFitness() > 0.0`.
  - Assert `evolver.bestRobot().isValid()`.

### Phase 5: Logging, Analytics & Media Autostore

- [ ] **5.1** File Hierarchy. Ensure the Evolver creates a structured output folder (e.g., `runs/run_TIMESTAMP/`).
- [ ] **5.2** Fitness Logging. Output a `fitness.csv` tracking `[Generation, Eval_ID, Fitness]`. This data will be used to generate the Generation vs. Fitness scatter plots (0–0.38 fitness over 167+ generations).
- [ ] **5.3** Phylogenetic Logging. Output a `lineage.csv` tracking `[Generation, Child_ID, Parent_ID, Fitness]`. This will be used externally (e.g., via Python/matplotlib) to plot the ancestral proximity trees (showing divergence, convergence, speciation, and mass extinction).
- [ ] **5.4** Automated Video Generation. Every $N$ generations, identify the most fit Robot. Pass it to a `Simulation` linked to a `VideoRenderer` and auto-store an MP4 showing its 12-cycle movement in a `videos/` subfolder.

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