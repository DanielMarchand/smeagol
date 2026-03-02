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

- [x] **4.2.6 — Post-Mutation Validation**

  After each mutation attempt, call `child.isValid()`. If invalid, reroll (re-clone + re-mutate) up to 100 times. If still invalid after all rerolls, warn to stderr and fall back to the unmutated parent clone.

---

#### 4.3 Evolutionary Loop

- [x] **4.3.1 — Evolver class**

- [x] **4.3.2 — Fitness-Proportionate Selection (`selectParent`)**

- [x] **4.3.3 — Steady-State Replacement**

- [x] **4.3.4 — Termination Condition**

  Flushes `fitness_log_`, saves `best_robot_final.yaml`, prints final stats.

---

#### 4.4 CLI Tool (`tools/evolve.cpp`) + `MutatorParams`

- [ ] Accepts one argument: path to a YAML config file.
- [ ] On startup: creates `runs/run_<ISO-timestamp>/`, writes `run_config.yaml` copy there, opens `fitness.csv` and `lineage.csv`.
- [ ] Prints progress to stdout every 200 evaluations: `eval=N  best=X.XXXm  mean=X.XXXm`.
- [ ] On exit (normal or SIGINT): saves best robot as `best_robot.yaml`.

---

##### `MutatorParams` struct

All mutation knobs are currently hardcoded in `Mutator.cpp`. They will be extracted into a `MutatorParams` struct (in `include/Mutator.h`) and threaded through `Mutator::mutate()` as an optional argument with sensible defaults:

```cpp
struct MutatorParams {
    // ── Operator fire probabilities (applied independently each call) ────
    double p_perturb    = 0.10;  ///< probability perturbElement fires
    double p_add_remove = 0.01;  ///< probability addRemoveElement fires
    double p_split      = 0.03;  ///< probability splitElement fires
    double p_attach     = 0.03;  ///< probability attachDetach fires
    double p_rewire     = 0.03;  ///< probability rewireNeuron fires

    // ── Perturb magnitudes ───────────────────────────────────────────────
    double perturb_bar_frac      = 0.10; ///< bar rest_length scaled by uniform(1-f, 1+f)
    double perturb_threshold_mag = 0.50; ///< threshold += uniform(-m, +m)
    double perturb_weight_mag    = 0.50; ///< synapse weight += uniform(-m, +m)

    // ── Structural clamp limits ──────────────────────────────────────────
    double bar_length_min   = 0.01; ///< minimum bar rest_length after any perturb/split [m]
    double bar_length_max   = 1.00; ///< maximum bar rest_length after any perturb [m]
    double threshold_min    = 0.00; ///< minimum neuron threshold after perturb
    double threshold_max    = 2.00; ///< maximum neuron threshold after perturb

    // ── Split vertex ─────────────────────────────────────────────────────
    double split_vertex_offset = 0.01; ///< max per-axis offset [m] when splitting a vertex

    // ── Attach / new actuator ────────────────────────────────────────────
    double actuator_range_max = 0.01; ///< bar_range sampled from uniform(0, max) on attach [m]

    // ── New neuron initialisation ────────────────────────────────────────
    double new_synapse_weight_min = 0.50; ///< min |initial synapse weight|
    double new_synapse_weight_max = 1.50; ///< max |initial synapse weight|

    static MutatorParams fromYAML(const YAML::Node& node);
    void toYAML(YAML::Emitter& out) const;
};
```

`Mutator::mutate()` signature becomes:

```cpp
static void mutate(Robot& robot, std::mt19937& rng,
                   const MutatorParams& params = MutatorParams{});
```

All internal helpers (`perturbElement`, `addBarMutation`, etc.) are updated to accept `const MutatorParams&` and use its fields instead of their local `constexpr` values. No change to any call site that doesn't pass params — the default argument preserves backwards compatibility.

`MutatorParams` is added as a field on `EvolverParams`:

```cpp
struct EvolverParams {
    // ... existing fields ...
    MutatorParams mutation;   // all defaults if omitted from YAML
};
```

---

##### Full `config.yaml` with mutation settings

```yaml
population_size:  200
max_evaluations:  100000
seed:             42
video_interval:   1000

fitness:
  cycles:          12
  steps_per_cycle: 5000
  step_size:       1.0e-7
  wind:            0.0

mutation:
  # ── Operator probabilities (each fires independently per mutate() call) ──
  p_perturb:    0.10
  p_add_remove: 0.01
  p_split:      0.03
  p_attach:     0.03
  p_rewire:     0.03

  # ── Perturb magnitudes ────────────────────────────────────────────────────
  # Bar rest_length is multiplied by uniform(1 - frac, 1 + frac)
  perturb_bar_frac:      0.10
  # Neuron threshold gets += uniform(-mag, +mag), clamped to [min, max]
  perturb_threshold_mag: 0.50
  # Synapse weight gets += uniform(-mag, +mag), unclamped
  perturb_weight_mag:    0.50

  # ── Structural clamps ─────────────────────────────────────────────────────
  bar_length_min:  0.01   # metres
  bar_length_max:  1.00   # metres
  threshold_min:   0.00
  threshold_max:   2.00

  # ── Split vertex offset ───────────────────────────────────────────────────
  # New vertex placed at parent ± uniform(-offset, +offset) on each axis
  split_vertex_offset: 0.01   # metres

  # ── Actuator bar_range on attach ──────────────────────────────────────────
  # bar_range sampled from uniform(0, max) when attaching a new actuator
  actuator_range_max: 0.01    # metres

  # ── New neuron initial synapse weight ─────────────────────────────────────
  # |weight| sampled from uniform(min, max), sign randomised 50/50
  new_synapse_weight_min: 0.50
  new_synapse_weight_max: 1.50
```

---

##### Implementation checklist for `MutatorParams`

- [x] Add `MutatorParams` struct to `include/Mutator.h` (with `fromYAML` / `toYAML`)
- [x] Add `#include "Mutator.h"` YAML parsing in `src/Mutator.cpp`
- [x] Thread `const MutatorParams&` through all five operator functions + helpers
- [x] Replace all local `constexpr` literals with `params.field` references
- [x] Add `MutatorParams mutation` field to `EvolverParams` in `include/Evolver.h`
- [x] Extend `EvolverParams::fromYAML` / `toYAML` to round-trip the `mutation:` block
- [x] Update `Evolver::run()` / `submit_one()` to pass `params_.mutation` to `Mutator::mutate()`
- [x] Update `tests/test_mutator.cpp` to construct explicit `MutatorParams` (defaults) — no behaviour change expected
- [x] Update `examples/evolve/config.yaml` with the full mutation block above

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

#### 5.1 — `MutatorParams` Struct

All mutation knobs are currently hardcoded in `Mutator.cpp`. Extract them into a `MutatorParams` struct in `include/Mutator.h` and thread it through `Mutator::mutate()` as an optional argument:

```cpp
struct MutatorParams {
    // ── Operator fire probabilities (applied independently each call) ────
    double p_perturb    = 0.10;  ///< probability perturbElement fires
    double p_add_remove = 0.01;  ///< probability addRemoveElement fires
    double p_split      = 0.03;  ///< probability splitElement fires
    double p_attach     = 0.03;  ///< probability attachDetach fires
    double p_rewire     = 0.03;  ///< probability rewireNeuron fires

    // ── Perturb magnitudes ───────────────────────────────────────────────
    double perturb_bar_frac      = 0.10; ///< bar rest_length scaled by uniform(1-f, 1+f)
    double perturb_threshold_mag = 0.50; ///< threshold += uniform(-m, +m)
    double perturb_weight_mag    = 0.50; ///< synapse weight += uniform(-m, +m)

    // ── Structural clamp limits ──────────────────────────────────────────
    double bar_length_min   = 0.01; ///< minimum bar rest_length [m]
    double bar_length_max   = 1.00; ///< maximum bar rest_length [m]
    double threshold_min    = 0.00; ///< minimum neuron threshold
    double threshold_max    = 2.00; ///< maximum neuron threshold

    // ── Split vertex ─────────────────────────────────────────────────────
    double split_vertex_offset = 0.01; ///< max per-axis offset [m] on vertex split

    // ── Attach / new actuator ────────────────────────────────────────────
    double actuator_range_max = 0.01; ///< bar_range sampled from uniform(0, max) [m]

    // ── New neuron initialisation ────────────────────────────────────────
    double new_synapse_weight_min = 0.50; ///< min |initial synapse weight|
    double new_synapse_weight_max = 1.50; ///< max |initial synapse weight|

    static MutatorParams fromYAML(const YAML::Node& node);
    void toYAML(YAML::Emitter& out) const;
};
```

`Mutator::mutate()` becomes:

```cpp
static void mutate(Robot& robot, std::mt19937& rng,
                   const MutatorParams& params = MutatorParams{});
```

All internal helpers accept `const MutatorParams&` and use its fields instead of `constexpr` literals. The default argument means every existing call site compiles unchanged. `MutatorParams` is added as a nested field on `EvolverParams`:

```cpp
struct EvolverParams {
    // ... existing fields ...
    MutatorParams mutation;   // all defaults if omitted from YAML
};
```

**Implementation checklist:** *(all complete)*
- [x] Add `MutatorParams` to `include/Mutator.h` with `fromYAML` / `toYAML`
- [x] Thread `const MutatorParams&` through all five operator functions + helpers in `src/Mutator.cpp`
- [x] Replace all local `constexpr` literals with `params.field` references
- [x] Add `MutatorParams mutation` field to `EvolverParams` in `include/Evolver.h`
- [x] Extend `EvolverParams::fromYAML` / `toYAML` to round-trip the `mutation:` block
- [x] Update `Evolver::run()` / `submit_one()` to pass `params_.mutation` to `Mutator::mutate()`
- [x] Update `tests/test_mutator.cpp` to construct explicit `MutatorParams` (defaults) — no behaviour change expected

---

#### 5.2 — Full `config.yaml`

The canonical config file covering all tunable parameters. Used by the `evolve` CLI tool and stored verbatim as `run_config.yaml` in the run directory for reproducibility.

```yaml
population_size:  200
max_evaluations:  100000
seed:             42          # 0 = seed from std::random_device
video_interval:   1000        # save checkpoint every N evals

fitness:
  cycles:          12
  steps_per_cycle: 5000
  step_size:       1.0e-7
  wind:            0.0

mutation:
  # ── Operator probabilities (each fires independently per mutate() call) ──
  p_perturb:    0.10
  p_add_remove: 0.01
  p_split:      0.03
  p_attach:     0.03
  p_rewire:     0.03

  # ── Perturb magnitudes ────────────────────────────────────────────────────
  # Bar rest_length is multiplied by uniform(1 - frac, 1 + frac)
  perturb_bar_frac:      0.10
  # Neuron threshold gets += uniform(-mag, +mag), clamped to [min, max]
  perturb_threshold_mag: 0.50
  # Synapse weight gets += uniform(-mag, +mag), unclamped
  perturb_weight_mag:    0.50

  # ── Structural clamps ─────────────────────────────────────────────────────
  bar_length_min:  0.01   # metres
  bar_length_max:  1.00   # metres
  threshold_min:   0.00
  threshold_max:   2.00

  # ── Split vertex offset ───────────────────────────────────────────────────
  # New vertex placed at parent ± uniform(-offset, +offset) on each axis
  split_vertex_offset: 0.01   # metres

  # ── Actuator bar_range on attach ──────────────────────────────────────────
  # bar_range sampled from uniform(0, max) when attaching a new actuator
  actuator_range_max: 0.01    # metres

  # ── New neuron initial synapse weight ─────────────────────────────────────
  # |weight| sampled from uniform(min, max), sign randomised 50/50
  new_synapse_weight_min: 0.50
  new_synapse_weight_max: 1.50
```

Update `examples/evolve/config.yaml` to match this full schema when 5.1 is implemented.

---

#### 5.3 — Run Directory Layout

Every run produces a self-contained directory. Nothing is written outside it.

```
runs/
└── run_20260302_143501/          ← timestamped, created at Evolver construction
    ├── run_config.yaml           ← full resolved EvolverParams (already implemented)
    ├── fitness_log.csv           ← one row per eval (already implemented, needs widening)
    ├── lineage.csv               ← one row per eval: child→parent linkage
    ├── population_final.yaml     ← all 200 robots at end of run (for resume/analysis)
    ├── best_robot_final.yaml     ← best individual at termination (already implemented)
    ├── checkpoints/              ← periodic snapshots of the best robot
    │   ├── best_eval_1000.yaml
    │   ├── best_eval_1000.png    ← static snapshot image (headless-safe)
    │   ├── best_eval_2000.yaml
    │   └── ...
    └── videos/                   ← periodic video renders of the best robot
        ├── best_eval_1000.mp4
        ├── best_eval_2000.mp4
        └── ...
```

The `checkpoints/` and `videos/` subdirectories are created lazily on first write.

---

#### 5.4 — `fitness_log.csv` Schema (expanded from current stub)

Current schema: `eval,best_fitness`. Expand to:

```csv
eval,generation,best_fitness,mean_fitness,best_robot_id,best_v,best_b,best_n,best_a
1,0,0.0,0.0,7,0,0,0,0
200,1,0.0012,0.00031,142,3,2,1,1
400,2,0.0019,0.00044,142,3,2,1,1
...
```

| Column | Type | Description |
|---|---|---|
| `eval` | int | Cumulative evaluation count (1-indexed) |
| `generation` | int | `eval / population_size` (integer division) |
| `best_fitness` | double | `fitnesses_[best_idx_]` |
| `mean_fitness` | double | Mean over all `fitnesses_` |
| `best_robot_id` | uint64 | `population_[best_idx_].id` — links into lineage |
| `best_v/b/n/a` | int | Vertex/bar/neuron/actuator count of the best robot |

Write one row every `log_interval` evals (default: every 1 eval, configurable via `EvolverParams`). The `generation` column makes it trivial to aggregate in pandas/matplotlib without recomputing.

Flush is called at end of `run()` (already done) — also call `fitness_log_.flush()` every 1000 evals to avoid data loss if the process is killed mid-run.

---

#### 5.5 — `lineage.csv` Schema

```csv
eval,child_id,parent_id,child_fitness,replaced_id,replaced_fitness
1,201,7,0.0,53,0.0
2,202,142,0.0012,18,0.0
...
```

| Column | Description |
|---|---|
| `eval` | Eval number at which this child was created |
| `child_id` | `Robot::ID` of the new individual |
| `parent_id` | `Robot::ID` of the parent it was cloned from |
| `child_fitness` | Fitness of the new child |
| `replaced_id` | `Robot::ID` of the individual it replaced in the population |
| `replaced_fitness` | Fitness of the replaced individual |

This table, combined with `fitness_log.csv`, is sufficient to reconstruct the full phylogenetic tree offline. `replaced_id` is needed to detect extinction events (high-fitness lineages being accidentally overwritten by replacement).

The `lineage_log_` ofstream is already declared in `Evolver.h` (placeholder from Phase 4.1 design). Open it in the constructor alongside `fitness_log_`.

In the multicore build (Phase 6), the `EvalResult` struct must carry `parent_id` and `replaced_id` back to the manager so the lineage row can be written correctly:

```cpp
struct EvalResult {
    Robot    child;
    int      replace_idx;
    double   fitness;
    Robot::ID parent_id;    // add: known at submit time
};
// Manager writes: eval, child.id, parent_id, fitness,
//                 population_[replace_idx].id (before overwrite),
//                 fitnesses_[replace_idx]     (before overwrite)
```

---

#### 5.6 — Periodic Best-Robot Checkpoints

`maybeSaveSnapshot(eval_num)` already fires every `video_interval` evals. Expand it:

```cpp
void Evolver::maybeSaveSnapshot(int eval_num)
{
    if (eval_num % params_.video_interval != 0) return;

    const std::string dir  = params_.run_dir + "checkpoints/";
    fs::create_directories(dir);
    const std::string stem = dir + "best_eval_" + std::to_string(eval_num);

    // 1. YAML — always. Lightweight, fast, resumable.
    population_[best_idx_].toYAML(stem + ".yaml");

    // 2. PNG snapshot — requires display. Skipped gracefully if headless.
    try {
        SnapshotRenderer snap;
        snap.render(population_[best_idx_], stem + ".png");
    } catch (const std::exception& e) {
        std::cerr << "[Evolver] snapshot PNG skipped: " << e.what() << "\n";
    }

    // 3. Video (Phase 5.4) — heavier; gated behind a separate video_interval
    //    or a dedicated video_every_n_intervals multiplier.
    //    Deferred to Phase 5.4 implementation.
}
```

The YAML checkpoint is critical: it lets you load `best_eval_N.yaml` directly into `snapshot_robot`, `evaluate_fitness`, or `visualize_robot` tools without a full resume.

---

#### 5.7 — `population_final.yaml` (End-of-Run Dump)

At termination, serialize the entire population to a single multi-document YAML file (one `---` block per robot). This enables:

- **Resume**: reload population + fitnesses and continue a run.
- **Post-hoc analysis**: re-evaluate every surviving individual with different fitness params (e.g., higher `steps_per_cycle` for a more accurate fitness estimate).
- **Seeding future runs**: take the best N individuals as the starting population for a follow-up run with a different fitness function.

```cpp
// End of Evolver::run():
{
    std::ofstream pop_file(params_.run_dir + "population_final.yaml");
    for (int i = 0; i < static_cast<int>(population_.size()); ++i) {
        pop_file << "# fitness: " << fitnesses_[i] << "\n";
        // toYAML writes to a path; refactor to support ostream, or write
        // individual files to  checkpoints/pop_final_<id>.yaml instead.
    }
}
```

Note: `Robot::toYAML()` currently takes a file path. A complementary `toYAMLString()` or `toYAML(std::ostream&)` overload would make multi-document serialization cleaner — add this when implementing 5.9.

---

#### 5.8 — Automated Video Generation

Every $N$ evals (gated behind a `video_every_n_checkpoints` multiplier on `video_interval`), identify the most fit robot. Pass it to a `Simulator` linked to a `VideoRenderer` and auto-store an MP4 in `videos/`. This is the heaviest I/O operation — run it on the manager thread while workers are idle (i.e., inside `maybeSaveSnapshot` which already runs on the manager).

The `video_every_n_checkpoints` multiplier avoids generating a video on every checkpoint: e.g., `video_interval = 1000` + `video_every_n_checkpoints = 5` → video every 5000 evals.

---

#### 5.9 — Logging Thread-Safety in the Parallel Build

All log writes happen on the manager thread, between `collect_one()` calls — **never** inside a worker lambda. The manager guarantees serial access to all `std::ofstream` objects. No locking is needed.

The only ordering concern: in the Phase 6 rolling-window design, `eval_count_` increments inside `collect_one()` which is called sequentially on the manager. Log rows are therefore always written in strictly ascending `eval` order. If `collect_any()` (non-FIFO collection) is used instead, rows may arrive out of order — add an `eval` column sort step in the offline analysis script, or buffer rows in a `priority_queue` keyed on `eval_count_` and flush in order.

---

#### 5.10 — Crash Recovery / Incremental Resume (future)

Not required for Phase 5 but worth noting as a future extension:

- Store a `checkpoint_latest.yaml` symlink (or copy) updated atomically via `rename()` on every `maybeSaveSnapshot` call. This gives a crash-safe latest checkpoint with no extra I/O.
- On startup, if `run_dir` already exists and contains `checkpoint_latest.yaml` + a partial `fitness_log.csv`, reload population and `eval_count_` from disk and continue.
- This requires making `fitnesses_` persistable (e.g., embed fitness as a YAML comment in each robot's checkpoint file, or write a companion `fitnesses.csv`).

---

### Phase 6: Multicore Parallelism

`FitnessEvaluator::evaluate()` is the only bottleneck. A full run is ~100,000 calls; each call runs up to 60,000 gradient-descent steps per simulation. Mutation, selection, and I/O are collectively < 1% of wall-clock time. The design philosophy is therefore: **keep evaluation cores saturated at all times**, while dedicating the remaining 1–2 cores to the management work (selection, cloning, mutation, result application, logging).

**Checklist:**
- [ ] 6.1–6.5 — Manager + worker pool: `EvalJob`/`EvalResult` structs, worker lambda, flight-window loop
- [ ] 6.6 — Thread-safety audit (`Robot::s_next_id` → `std::atomic<Robot::ID>`)
- [ ] 6.7–6.9 — Core allocation, CMake `-pthread`, expected speedup validation
- [ ] 6.10 — Signal handling and graceful shutdown (Ctrl+C)
- [ ] 6.11 — Logging thread-safety: verify all `fitness_log_` / `lineage_log_` writes remain on manager thread
- [ ] 6.12 — Crash recovery / incremental resume: `checkpoint_latest.yaml` symlink via atomic `rename()`;  resume by loading final checkpoint + partial `fitness_log.csv`

---

#### 6.1 — Architecture: Manager + Worker Pool

```
┌─────────────────────────────────────────────────────────────┐
│  Main thread  ("manager")                                   │
│  - Owns: population_, fitnesses_, rng_, fitness_log_        │
│  - Owns: in_flight deque of std::future<EvalResult>         │
│                                                             │
│  Loop:                                                      │
│    while window not full:                                   │
│      selectParent() → clone → mutate/reroll                 │
│      std::async(eval_worker, std::move(job))  ──────────┐  │
│                                                          │  │
│    oldest_future.get()  ◄────────────────────────────────┘  │
│      apply replacement to population_                       │
│      update best_idx_, log                                  │
│      submit one new job                                     │
└─────────────────────────────────────────────────────────────┘

       Worker threads  (W = hardware_concurrency() - 1 or - 2)
       Each owns:  local FitnessEvaluator, local Robot copy
       No shared state — purely functional
```

The key property: every time the manager collects one result it immediately submits one new job, so the number of in-flight evaluations stays constant at `W`. Worker cores are never idle except at startup and wind-down.

---

#### 6.2 — Data Structures

```cpp
// A job packet submitted to a worker.
// Everything is owned — no pointers into shared state.
struct EvalJob {
    Robot        child;        // deep copy — worker owns it
    int          replace_idx;  // decided at submission time by manager
    // (no per-job RNG needed: eval is deterministic given the robot geometry)
};

// What the worker returns to the manager.
struct EvalResult {
    Robot        child;        // moved back for lineage logging (Phase 5)
    int          replace_idx;
    double       fitness;
};
```

`replace_idx` is chosen by the manager at submission time (before the worker starts). This is valid because in steady-state GA any slot can be replaced — we don't need the fitness value to decide the replacement target (unlike tournament selection). The only edge case is that two in-flight jobs may choose the same `replace_idx`; the second write simply overwrites the first. This is acceptable and matches the original serial semantics asymptotically.

---

#### 6.3 — Worker Lambda

The worker is a pure function with zero shared state:

```cpp
// Defined inside Evolver::run(), captures only params_ by value.
auto eval_worker = [fitness_params = params_.fitness](EvalJob job) -> EvalResult {
    double f = FitnessEvaluator(fitness_params).evaluate(job.child);
    return EvalResult{ std::move(job.child), job.replace_idx, f };
};
```

`FitnessEvaluator` and `Simulator` are both stack-local inside the lambda — no heap sharing, no locking.

---

#### 6.4 — Flight-Window Loop

```cpp
void Evolver::run()
{
    const int W = std::max(1,
        static_cast<int>(std::thread::hardware_concurrency()) - 1);
    // Reserve 1 core for the manager. On a 16-core machine: W = 15 workers.
    // To reserve 2 manager cores (useful if mutation is heavy): use - 2.

    std::deque<std::future<EvalResult>> in_flight;

    // Helper: build and submit one job.
    auto submit_one = [&]() {
        const int parent_idx = selectParent();

        // Mutate with reroll (Phase 4.2.6) — all on manager thread.
        Robot child;
        constexpr int kMaxRerolls = 100;
        for (int attempt = 0; attempt <= kMaxRerolls; ++attempt) {
            child = population_[parent_idx].clone();
            child.id = Robot::nextId();   // atomic — see 6.6
            Mutator::mutate(child, rng_);
            if (child.isValid()) break;
            if (attempt == kMaxRerolls) {
                std::cerr << "[Evolver] warning: reroll limit hit at eval "
                          << eval_count_ << " — using unmutated clone\n";
                child = population_[parent_idx].clone();
                child.id = Robot::nextId();
            }
        }

        EvalJob job{ std::move(child), selectReplacement() };
        in_flight.push_back(
            std::async(std::launch::async, eval_worker, std::move(job)));
    };

    // Helper: collect the oldest in-flight result and apply it.
    auto collect_one = [&]() {
        EvalResult res = in_flight.front().get();
        in_flight.pop_front();

        population_[res.replace_idx] = std::move(res.child);
        fitnesses_[res.replace_idx]  = res.fitness;

        // Rescan if we clobbered best_idx_.
        if (res.replace_idx == best_idx_) {
            best_idx_ = static_cast<int>(
                std::max_element(fitnesses_.begin(), fitnesses_.end())
                - fitnesses_.begin());
        } else if (res.fitness > fitnesses_[best_idx_]) {
            best_idx_ = res.replace_idx;
        }

        ++eval_count_;
        fitness_log_ << eval_count_ << "," << fitnesses_[best_idx_] << "\n";

        if (eval_count_ % 200 == 0) {
            double mean = 0.0;
            for (double f : fitnesses_) mean += f;
            mean /= static_cast<double>(fitnesses_.size());
            std::cout << std::fixed << std::setprecision(4)
                      << "eval=" << eval_count_
                      << "  best=" << fitnesses_[best_idx_] << "m"
                      << "  mean=" << mean << "m\n";
        }

        maybeSaveSnapshot(eval_count_);
    };

    // ── Phase 1: fill the window ──────────────────────────────────────────
    while (static_cast<int>(in_flight.size()) < W
           && eval_count_ + static_cast<int>(in_flight.size())
              < params_.max_evaluations)
        submit_one();

    // ── Phase 2: rolling window ───────────────────────────────────────────
    while (eval_count_ < params_.max_evaluations) {
        collect_one();
        if (eval_count_ + static_cast<int>(in_flight.size())
            < params_.max_evaluations)
            submit_one();
    }

    // ── Phase 3: drain any remaining in-flight jobs ───────────────────────
    while (!in_flight.empty())
        collect_one();

    // ── Final bookkeeping ─────────────────────────────────────────────────
    fitness_log_.flush();
    const std::string best_path = params_.run_dir + "best_robot_final.yaml";
    population_[best_idx_].toYAML(best_path);
    std::cout << "[Evolver] run complete.  eval=" << eval_count_
              << "  best_fitness=" << fitnesses_[best_idx_] << "m\n"
              << "[Evolver] best robot saved → " << best_path << "\n";
}
```

---

#### 6.5 — Head-of-Line Blocking

`in_flight.front().get()` always waits for the **oldest** submitted job. If one eval is anomalously slow (a large mature robot), it delays collection of faster results that completed behind it. In practice this is unlikely to matter — eval time variance within a generation should be small — but if it becomes a problem, a non-blocking scan can be added:

```cpp
// Non-blocking poll: find the first ready future in the window.
auto collect_any = [&]() {
    while (true) {
        for (auto it = in_flight.begin(); it != in_flight.end(); ++it) {
            if (it->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                EvalResult res = it->get();
                in_flight.erase(it);
                apply_result(res);   // same logic as collect_one above
                return;
            }
        }
        // Nothing ready yet — yield briefly to avoid busy-spin.
        std::this_thread::yield();
    }
};
```

Swap `collect_one()` for `collect_any()` only if profiling shows significant stall time on the manager thread.

---

#### 6.6 — Thread-Safety Audit

| Component | Safe? | Fix |
|---|---|---|
| `FitnessEvaluator::evaluate()` | ✅ | Nothing — fully stateless |
| `Simulator` (construction + run) | ✅ | Nothing — stack-local inside lambda |
| `Robot::s_next_id` (static counter) | ❌ | Change to `std::atomic<Robot::ID>` |
| `population_` / `fitnesses_` | ✅ after fix | Manager is the only writer; workers hold their own `Robot` copy |
| `fitness_log_` (ofstream) | ✅ after fix | Written only by manager between `collect_one()` calls |
| `rng_` (manager's mt19937) | ✅ | Used only on manager thread inside `submit_one()` |
| `Raylib` (SnapshotRenderer) | ✅ | Called only from manager via `maybeSaveSnapshot()` |

The single required code change:

```cpp
// include/Robot.h
static std::atomic<ID> s_next_id;   // was: static ID s_next_id

// src/Robot.cpp
std::atomic<Robot::ID> Robot::s_next_id{0};  // was: Robot::ID Robot::s_next_id = 0
```

`fetch_add(1, std::memory_order_relaxed)` is implicitly used by `operator++` on `std::atomic` — no other changes needed in `Robot.cpp`.

---

#### 6.7 — How Many Manager Cores?

| Machine | `hardware_concurrency()` | Recommended W | Notes |
|---|---|---|---|
| Laptop (8 logical cores) | 8 | 7 | 1 manager, 7 workers |
| Desktop (16 cores) | 16 | 14 | 2 managers (use `- 2`) if mutation rerolls become expensive |
| Server (32+ cores) | 32+ | 30 | Mutation is fast enough that 1 manager suffices; use `- 1` |

The manager only stalls when calling `collect_one()` waiting for the oldest future. On a healthy run the oldest future is nearly always already complete by the time the manager loops back (workers finish evals in ~1–5 ms; mutation takes ~1 µs). So 1 manager core is wasted < 5% of the time. Using `- 2` is only warranted if you later add expensive mutation logic (e.g., gradient-based perturbation).

---

#### 6.8 — CMake Changes

```cmake
find_package(Threads REQUIRED)
target_link_libraries(smeagol_core Threads::Threads)
```

`std::async` with `std::launch::async` uses the system thread pool via `<future>` — no additional library required on Linux (pthreads are pulled in by `Threads::Threads`). No OpenMP, no third-party scheduler.

---

#### 6.9 — Expected Speedup

| Cores | W | Ideal speedup | Estimated real (Amdahl + overhead) |
|---|---|---|---|
| 4 | 3 | 3× | ~2.8× |
| 8 | 7 | 7× | ~6.5× |
| 16 | 15 | 15× | ~13–14× |
| 32 | 31 | 31× | ~25–28× |

Overhead sources: `std::async` launch (~5–15 µs per future), `Robot::clone()` heap allocation, `collect_one()` manager stall. Together these are < 1% of a typical eval duration and do not limit scaling up to at least 32 cores.

---

#### 6.10 — Signal Handling and Graceful Shutdown (Ctrl+C)

**Does the parallelism make Ctrl+C worse?**

Without a signal handler the default action for SIGINT is immediate process termination — `std::async` worker threads are killed mid-simulation, file buffers are not flushed, the best robot is never saved, and any partial CSV rows are truncated. This is exactly the same problem as the serial build, just with more threads in flight. The fix is the same in both cases: catch the signal, set a flag, let the code finish cleanly.

---

##### Signal handler (async-signal-safe)

Signal handlers may only call async-signal-safe functions — no `malloc`, no `cout`, no mutexes. Writing to `STDERR_FILENO` via `write()` is safe. The complete handler (supporting both single and double Ctrl+C) lives at file scope in `src/Evolver.cpp`:

```cpp
// src/Evolver.cpp — file-scope, before any function definitions
#include <csignal>
#include <unistd.h>   // write(), STDERR_FILENO

static std::atomic<bool> g_stop_requested{false};
static std::atomic<int>  g_signal_count{0};

static void signal_handler(int /*sig*/) noexcept
{
    if (g_signal_count.fetch_add(1, std::memory_order_relaxed) == 0) {
        // First signal: request graceful shutdown and tell the user.
        g_stop_requested.store(true, std::memory_order_relaxed);
        constexpr char msg[] =
            "\n[Evolver] interrupt received — finishing in-flight evals, "
            "please wait...\n"
            "           (press Ctrl+C again to force quit immediately)\n";
        (void)write(STDERR_FILENO, msg, sizeof(msg) - 1);
    } else {
        // Second signal: hard exit.  Data already saved by graceful path.
        constexpr char msg[] = "\n[Evolver] force quit.\n";
        (void)write(STDERR_FILENO, msg, sizeof(msg) - 1);
        _exit(130);   // 130 = 128 + SIGINT, conventional exit code
    }
}
```

Install in `Evolver::run()` before the loop starts:

```cpp
void Evolver::run()
{
    // Install signal handlers.  Restore originals on exit.
    auto prev_sigint  = std::signal(SIGINT,  signal_handler);
    auto prev_sigterm = std::signal(SIGTERM, signal_handler);

    // ... rest of run() ...

    // Restore on clean exit so subsequent code isn't affected.
    std::signal(SIGINT,  prev_sigint);
    std::signal(SIGTERM, prev_sigterm);
}
```

`SIGTERM` is included because `kill <pid>` and job schedulers (SLURM, Docker stop) send SIGTERM, not SIGINT.

---

##### Where to check the flag

Check `g_stop_requested` **only on the manager thread**, at the top of the main loop — before submitting a new job. Never check it inside a worker lambda; workers always run to completion naturally:

```cpp
// Phase 2: rolling window
while (eval_count_ < params_.max_evaluations) {
    if (g_stop_requested.load(std::memory_order_relaxed)) {
        std::cerr << "\n[Evolver] interrupt received — draining "
                  << in_flight.size() << " in-flight eval(s)...\n";
        break;   // falls through to Phase 3 drain below
    }
    collect_one();
    if (eval_count_ + static_cast<int>(in_flight.size())
        < params_.max_evaluations)
        submit_one();
}

// Phase 3: drain (runs whether interrupted or not)
while (!in_flight.empty())
    collect_one();
```

Because the drain loop runs unconditionally, every already-submitted eval completes and its result is written to the log and applied to the population. The window size `W` bounds the worst-case wait after Ctrl+C — on a 16-core machine with 1–5 ms evals the process fully exits in under 100 ms.

---

##### Double Ctrl+C (force quit)

If the user hits Ctrl+C a second time mid-drain, the second SIGINT calls `_exit(130)` immediately. `_exit()` is async-signal-safe (unlike `exit()`) and bypasses C++ destructors — that is fine here because the first Ctrl+C already triggered the graceful save path, so no data is lost.

`(void)write(...)` is used throughout because `write()`'s return value cannot be meaningfully acted on inside a signal handler, and the cast silences the unused-result compiler warning.

---

##### Full shutdown sequence (single Ctrl+C)

```
Ctrl+C pressed
  → signal_handler sets g_stop_requested = true
  → manager sees flag at top of next loop iteration
  → no new jobs are submitted
  → Phase 3 drain: all W in-flight futures are collected
  → fitness_log_.flush() called
  → lineage_log_.flush() called
  → best_robot_final.yaml written
  → signal handlers restored
  → Evolver::run() returns
  → evolve tool prints "interrupted" message and exits 130
```

Nothing is lost beyond the evals that were never submitted after the signal.

---

##### `evolve.cpp` exit code

```cpp
int main(int argc, char** argv)
{
    // ... parse args, construct evolver ...
    ev.run();

    if (g_stop_requested.load())  {
        std::cerr << "[evolve] run interrupted after "
                  << ev.evalCount() << " evals.\n";
        return 130;   // conventional: interrupted by signal
    }
    return 0;
}
```

`g_stop_requested` needs to be accessible from `main` — either expose it via a header (`extern std::atomic<bool> g_stop_requested`) or add `Evolver::wasInterrupted() const` accessor that returns the flag value.

---

##### `sigaction` vs `std::signal`

`std::signal` is portable (C++17 standard) but has unspecified behaviour for signals raised from other threads on POSIX systems. `sigaction` with `SA_RESTART` is more robust on Linux:

```cpp
struct sigaction sa{};
sa.sa_handler = signal_handler;
sa.sa_flags   = SA_RESTART;   // restart interrupted syscalls automatically
sigemptyset(&sa.sa_mask);
sigaction(SIGINT,  &sa, nullptr);
sigaction(SIGTERM, &sa, nullptr);
```

`SA_RESTART` means that if the signal fires while the manager is blocked in `future.get()`, the wait call is automatically restarted rather than returning `EINTR`. Without it, `future.get()` could throw or behave unpredictably on some platforms. Use `sigaction` on Linux; fall back to `std::signal` with `#ifdef _WIN32` for Windows portability.

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