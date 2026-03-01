# Smeagol

A C++17 replication of the evolutionary robotics experiment described in:

> Lipson, H. & Pollack, J.B. (2000). *Automatic design and manufacture of robotic lifeforms.* Nature, 406, 974–978.

The system evolves 3D truss structures and their neural controllers simultaneously, selecting for locomotion distance across an infinite flat plane.

---

## Architecture

The codebase has four layers: **data**, **physics**, **rendering**, and (planned) **evolution**.

### Data — `Robot` and `RobotPart`

All physical and neural components inherit from `RobotPart`. The four concrete types are:

| Class | Role | Key fields |
|---|---|---|
| `Vertex` | Ball joint in 3D space | `Eigen::Vector3d pos` |
| `Bar` | Elastic rod connecting two vertices | `v1`, `v2`, `rest_length`, `radius` |
| `Neuron` | Discrete threshold node | `threshold`, `synapse_weights` |
| `Actuator` | Maps neuron output to a bar's rest length | `bar_idx`, `neuron_idx`, `bar_range` |

`Robot` is the container: it holds `std::vector`s of each type and provides `toYAML()` / `fromYAML()` for serialisation. Bar rest lengths are computed automatically from the initial vertex positions at load time, so a robot is always stress-free at $t = 0$. Bar stiffness is derived from material constants on demand:

$$k_i = \frac{E \cdot A_i}{L_{0,i}}$$

where $A_i = \pi r_i^2$, $E = 0.896\ \text{GPa}$, and $L_{0,i}$ is the rest length.

### Physics — `Simulator`

`Simulator` takes ownership of a robot's geometry (copied into an $N \times 3$ Eigen matrix) and implements quasi-static energy minimisation. This is the same approach used by Lipson & Pollack — there is no velocity or momentum; the system always seeks the nearest energy minimum from its current configuration.

#### Energy function

The total potential energy is the sum of three terms:

$$H = \underbrace{\sum_i k_i \delta_i^2}_{H_{\text{elastic}}} + \underbrace{\sum_j m_j g z_j}_{H_{\text{gravity}}} + \underbrace{\sum_j k_{\text{floor}} \min(z_j, 0)^2}_{H_{\text{floor}}}$$

- $\delta_i = \|p_{v_2} - p_{v_1}\| - L_{0,i}$ is the extension of bar $i$.
- Vertex masses $m_j$ are computed by lumping each bar's mass ($\rho A L_0$) equally onto its two endpoints.
- $k_{\text{floor}} = 1.4 \times 10^6\ \text{N/m}$ (matched to a typical bar stiffness). The floor is intentionally soft — the robot penetrates slightly, but the gradient always pushes it back up.

#### Gradient descent (`relax`)

At each step the gradient $\nabla H$ is computed analytically:

$$\frac{\partial H_{\text{elastic}}}{\partial \mathbf{p}_{v}} = \sum_{i \ni v} \pm \frac{2 k_i \delta_i}{\|\Delta \mathbf{p}_i\|} \Delta \mathbf{p}_i$$

$$\frac{\partial H_{\text{gravity}}}{\partial z_j} = m_j g \qquad \frac{\partial H_{\text{floor}}}{\partial z_j} = 2 k_{\text{floor}} z_j \quad (z_j < 0)$$

Positions are updated by:

$$\mathbf{p} \leftarrow \mathbf{p} - D_s \nabla H$$

where $D_s$ is the step size. A small uniform noise term can be added each step to escape unstable saddle points. The loop terminates when $\|\nabla H\|_F < \epsilon$ or a maximum iteration count is reached.

#### Friction (`applyFriction`)

Before each gradient step, lateral ($x$,$y$) gradient components are zeroed for any vertex in contact with the floor whose static friction threshold has not been exceeded:

$$|\nabla_{xy} H_j| \leq \mu_s \cdot N_j, \quad N_j = 2 k_{\text{floor}} |z_j|$$

$\mu_s = 0.5$ (Coulomb static friction coefficient).

#### Neural tick and actuation

At each frame, before `relax` is called, the neural network is stepped once and its outputs are applied to bar rest lengths.

Each `Neuron` is a discrete threshold unit. All neurons update **in parallel** (synchronous update — each neuron reads the *previous* activations of all its inputs):

$$a_i^{(t+1)} = \begin{cases} 1 & \text{if } \sum_j w_{ij}\, a_j^{(t)} \geq \theta_i \\ 0 & \text{otherwise} \end{cases}$$

Each `Actuator` then maps its neuron's binary output to a signed change in a bar's rest length:

$$L_{0,i}^{(t+1)} = L_{0,i}^{(t)} + a_k^{(t+1)} \cdot r_i$$

where $r_i$ is `bar_range` (metres, signed). When the neuron fires the bar stretches or compresses by $|r_i|$; when silent the rest length is unchanged. Actuation is clamped so no single step moves a rest length by more than 1 cm.

### Rendering — `SceneRenderer` hierarchy

All rendering shares a single base class to guarantee consistent camera, lighting, and floor across tools.

```
SceneRenderer          ← orbital camera, drawRobot(), drawFloor(), drawNeuralOverlay()
├── SnapshotRenderer   ← renders one frame → PNG, closes
└── VideoRenderer      ← keeps window open; addFrame() → numbered PNGs
                         finish() → ffmpeg → MP4, cleans up temp dir
```

`VideoRenderer` uses an off-screen Raylib context (no visible window). Each call to `addFrame(robot, sim_time, activations)` runs one full render pass and writes `frame_XXXX.png` to a unique temporary directory. `finish(output_path)` invokes:

```
ffmpeg -y -framerate <fps> -i <tmp>/frame_%04d.png -c:v libx264 -pix_fmt yuv420p <output>
```

The neural overlay colours neurons **red** when firing and grey when silent, synapse lines **blue** (excitatory) or **red** (inhibitory), and actuator connections **green**.

Coordinate system: the physics simulation uses **Z-up**; `SceneRenderer::toRaylib()` remaps to Raylib's **Y-up** convention before any draw calls.

### Fitness Wrapper — `FitnessEvaluator`

Wraps the simulation loop into a single callable that maps a `Robot` to a scalar locomotion score.

```cpp
FitnessEvaluator eval;          // default: 12 cycles × 5 000 steps at Ds = 1e-7
double score = eval.evaluate(robot);
```

**Algorithm** — for a robot with $N_c$ cycles and $S$ steps per cycle:

1. Construct a fresh `Simulator` from the robot (no inherited state).
2. Record the initial mass-weighted XY centre-of-mass $\mathbf{p}_0$.
3. For each cycle $c = 1 \ldots N_c$:
   - `tickNeural()` — advance the recurrent network one discrete step.
   - `applyActuators()` — update bar rest lengths from neuron outputs.
   - `relax(S, D_s, 0, 0)` — $S$ gradient steps (no noise, zero tolerance so all steps run).
4. Record final CoM $\mathbf{p}_f$ and return $\|\mathbf{p}_f - \mathbf{p}_0\|_{xy}$ (metres).

An optional `trajectory` pointer collects the CoM at every cycle boundary for plotting.

`FitnessParams` exposes the three knobs:

| Field | Default | Meaning |
|---|---|---|
| `cycles` | 12 | evaluation length (neural cycles) |
| `steps_per_cycle` | 5 000 | gradient-descent steps per cycle |
| `step_size` | 1e-7 | $D_s$ — must satisfy $D_s < 1/(2k)$ for stability |

---

## Material constants

All values follow Lipson & Pollack (2000), defined in `include/Materials.h`:

| Constant | Value | Description |
|---|---|---|
| $E$ | $0.896\ \text{GPa}$ | Young's modulus |
| $\rho$ | $1000\ \text{kg/m}^3$ | Mass density |
| $S_{\text{yield}}$ | $19\ \text{MPa}$ | Yield strength (used for validation) |
| $g$ | $9.81\ \text{m/s}^2$ | Gravitational acceleration |
| $k_{\text{floor}}$ | $1.4 \times 10^6\ \text{N/m}$ | Floor penalty stiffness |
| $\mu_s$ | $0.5$ | Static friction coefficient |

---

## Requirements

- **CMake** ≥ 3.16
- **C++17** compiler (GCC ≥ 9, Clang ≥ 10)
- **Eigen3** ≥ 3.3 — header-only linear algebra (`find_package(Eigen3)`)
- **yaml-cpp** — YAML serialisation (`find_package(yaml-cpp)`)
- **Raylib** ≥ 4.0 — 3D rendering (`find_package(raylib)`)
- **FFmpeg** — MP4 compilation (must be on `PATH` at runtime; only needed for `VideoRenderer::finish()`)
- A display server (X11 or Wayland) for any tool that opens a window. On headless machines use `Xvfb`:
  ```
  Xvfb :99 -screen 0 1280x720x24 &
  export DISPLAY=:99
  ```

---

## Build

```bash
cmake -B build
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

---

## Tools

| Binary | Purpose |
|---|---|
| `validate_robot` | Loads a YAML robot and prints validation errors |
| `visualize_robot` | Interactive 3D viewer (orbital camera) |
| `snapshot_robot` | Renders a single PNG from a YAML robot |
| `deformation_series` | Displaces one vertex incrementally; prints energy table + PNG per step |
| `run_simulation` | Runs quasi-static simulation from YAML and produces an MP4 |
| `evaluate_fitness` | Evaluates a robot's locomotion fitness and optionally records a video |

### `evaluate_fitness`

```bash
evaluate_fitness <robot.yaml> [params.yaml] [--video]
```

Runs the [Fitness Wrapper](#fitness-wrapper--fitnessevaluator) over `N` neural cycles, prints a per-cycle CoM trajectory table, and reports the final XY displacement fitness score. Passing `--video` (or setting `output` in `params.yaml`) records an MP4 of the evaluation replay.

`params.yaml` controls the evaluation:

```yaml
cycles:          12       # number of neural cycles
steps_per_cycle: 5000    # gradient-descent steps per cycle
step_size:       1.0e-7  # D_s
fps:             10
width:           640
height:          480
output:          /tmp/fitness_eval.mp4
```

---

### `run_simulation`

```bash
run_simulation <robot.yaml> <simulation.yaml>
```

`simulation.yaml` controls all physics and output parameters:

```yaml
fps:             30
num_frames:      200
steps_per_frame: 5000    # gradient steps between captured frames
step_size:       1.0e-7  # D_s; must be < 1/(2k) for stability
width:           640
height:          480
output:          /tmp/out.mp4
```

---

## Examples

| Directory | What it shows |
|---|---|
| `examples/robot_visualize/` | Interactive view + single snapshot of a robot |
| `examples/deformation_series/` | Energy sanity-check: elastic energy rises from ~0 as a vertex is displaced |
| `examples/robot_fall/` | Physics validation: inverted tetrahedron falls and tips under gravity |
| `examples/robot_sineactuator/` | Actuator validation: bar length driven by a `sin(ωt)` debug waveform |
| `examples/robot_neuralactuator/` | Neural actuation: 2-neuron anti-phase oscillator drives a strut bar ±1 cm per tick |
| `examples/robot_fitness/` | Fitness evaluation: always-firing neuron elongates a strut, producing measurable XY locomotion |

---

### `robot_fall` — physics validation

![Tetrahedron falling under quasi-static gradient descent](docs/media/tetrahedron_fall.gif)

An inverted tetrahedron placed above the floor with its apex slightly off-centre. At $t=0$ the structure is stress-free ($H_\text{elastic} = 0$) because rest lengths are auto-computed from the initial vertex positions (see [Data](#data--robot-and-robotpart)). The only driving force is gravity.

The animation exercises the three-term energy function and gradient descent loop described in [Physics — Simulator](#physics--simulator):

1. **Free fall** — all vertices descend together; $H_\text{gravity}$ falls monotonically, $H_\text{elastic} \approx 0$ because no bar is stretched.
2. **Floor contact** — the apex enters the penalty region ($z < 0$); $H_\text{floor}$ pushes it back. The floor is soft ($k_\text{floor} \approx k_\text{bar}$) so slight penetration occurs.
3. **Tip-over** — the apex is off-centre from the base centroid, so the floor reaction creates a net torque. Gradient descent finds the lower-energy lying-flat configuration and the structure rotates toward it.
4. **Settling** — all vertices reach near-floor altitude; elastic energy stays negligible throughout.

> There is no velocity or momentum. Each frame is exactly `steps_per_frame` gradient steps of size $D_s$, tracing the steepest-descent path through configuration space.

---

### `robot_neuralactuator` — neural actuation

![Tetrahedron driven by a 2-neuron anti-phase oscillator](docs/media/tetrahedron_neuralactuator.gif)

The same tetrahedron geometry, resting on the floor and controlled by a two-neuron recurrent network. Each frame the network is ticked once and its outputs are applied to bar rest lengths before `relax` runs (see [Neural tick and actuation](#neural-tick-and-actuation)).

**Network topology — cross-coupled anti-phase oscillator:**

```
Neuron 0 ──w=1──► Neuron 1
Neuron 1 ──w=1──► Neuron 0
         threshold = 0.5 (both)
```

No self-weights. Neuron 0 is primed ($a_0 = 1$) at $t = 0$; neuron 1 starts silent. Because each neuron fires only when its partner was active last tick, they alternate with period 2:

| Tick | $a_0$ | $a_1$ |
|------|--------|--------|
| 0    | 1      | 0      |
| 1    | 0      | 1      |
| 2    | 1      | 0      |
| …    | …      | …      |

**Actuators** — both wired to the same bar (left strut, bar 3):

| Actuator | Neuron | `bar_range` | Effect when neuron fires |
|---|---|---|---|
| 0 | 0 | +0.010 m | strut **lengthens** by 1 cm |
| 1 | 1 | −0.010 m | strut **shortens** by 1 cm |

The opposing signs mean the strut oscillates ±1 cm around its natural length with no cumulative drift. The quasi-static physics engine responds by rocking the apex back and forth every cycle, visible in the animation above.

The neural overlay shows neurons **red** (firing) or grey (silent), synapse lines blue/red by sign, and green lines connecting each neuron to its actuated bar.

---

### `robot_fitness` — fitness evaluation

A minimal robot with a single always-firing neuron (self-excitatory, $w_{00} = 1$, threshold $= 0.5$) wired to one actuator that elongates bar 3 by 5 mm every cycle. This creates cumulative asymmetric deformation that shifts the CoM across the floor over 12 cycles.

Running `examples/robot_fitness/run.sh` prints a CoM trajectory and calls `evaluate_fitness` with `--video` to produce `/tmp/fitness_eval.mp4`:

```
Cycle   0:  x= 0.100000  y= 0.074162
Cycle   1:  x= 0.100235  y= 0.074412
...
Cycle  12:  x= 0.102825  y= 0.077449

Fitness (XY displacement): 0.004334 m
```

This confirms the fitness wrapper returns a positive, monotonically-growing signal for a locomoting robot and near-zero for a passive one (see [Fitness Wrapper](#fitness-wrapper--fitnessevaluator)).

---

## Project status

| Phase | Description | Status |
|---|---|---|
| 1 | Data structures, YAML serialisation | ✅ |
| 2 | Rendering (SceneRenderer, SnapshotRenderer, VideoRenderer) | ✅ |
| 3.1–3.3 | Energy function, quasi-static relaxation, floor + friction | ✅ |
| 3.4–3.5 | Neural network tick, actuator coupling | ✅ |
| 3.6 | Fitness wrapper (`FitnessEvaluator`, `evaluate_fitness` tool) | ✅ |
| 4 | Genetic algorithm / evolver | planned |
| 5 | Logging, fitness CSV, phylogenetic tree, automated video | planned |

