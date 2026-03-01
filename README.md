# Smeagol

A C++17 replication of the evolutionary robotics experiment described in:

> Lipson, H. & Pollack, J.B. (2000). *Automatic design and manufacture of robotic lifeforms.* Nature, 406, 974–978.

The system evolves 3D truss structures and their neural controllers simultaneously, selecting for locomotion distance across an infinite flat plane.

---

## Fall animation — physics validation

![Tetrahedron falling under quasi-static gradient descent](docs/media/tetrahedron_fall.gif)

An inverted tetrahedron placed above the floor with its apex slightly off-centre.
At $t=0$ the structure is stress-free ($H_\text{elastic} \approx 0$); the only driving force is gravity.

**What the animation shows, step by step:**

1. **Free fall** — gravity pulls all four vertices downward at equal rates.
   $H_\text{gravity}$ decreases monotonically as the CoM descends.
   $H_\text{elastic}$ stays near zero: the bars are not being stretched because all vertices accelerate together.

2. **Floor contact** — the apex (lowest point) enters the floor penalty region ($z < 0$).
   $H_\text{floor} = k_\text{floor} z^2$ pushes it back up, but because $k_\text{floor}$ equals a typical bar stiffness the floor is soft.
   The apex penetrates slightly; the three base vertices are still in free fall.

3. **Asymmetric loading and tip-over** — because the apex is offset from the base centroid, the floor reaction force does not pass through the CoM.
   This creates a net torque. The gradient descent step that minimises total energy corresponds to the structure rotating: the side closer to the floor gains contact first, and the tetrahedron tips toward a lower-energy lying-flat configuration.

4. **Settling** — all four vertices eventually reach near-floor altitude.
   The elastic energy remains negligible throughout because the bars are not forced out of their rest lengths; the structure behaves as a rigid body for this initial validation.

> **Physics note:** Smeagol uses quasi-static gradient descent — the same approach as Lipson & Pollack (2000).
> There is no velocity or momentum.
> Each frame advances the system by a fixed number of gradient steps ($D_s \nabla H$), tracing the path of steepest energy descent rather than a Newtonian trajectory.
> The robot will not bounce, but gravity, floor contact, and friction all behave correctly and are sufficient to validate the physics before adding neural actuation.

---

## Neural actuation — locomotion preview

![Tetrahedron driven by a 2-neuron anti-phase oscillator](docs/media/tetrahedron_neuralactuator.gif)

A tetrahedron on the floor, controlled by a two-neuron network that drives one strut bar to oscillate ±1 cm every cycle.

### How neurons work in Smeagol

Each `Neuron` is a discrete threshold unit. At every neural tick, all neurons update **in parallel**:

$$a_i^{(t+1)} = \begin{cases} 1 & \text{if } \sum_j w_{ij}\, a_j^{(t)} \geq \theta_i \\ 0 & \text{otherwise} \end{cases}$$

where $w_{ij}$ is the synapse weight from neuron $j$ to neuron $i$ and $\theta_i$ is neuron $i$'s threshold.  
The parallel (synchronous) update means each neuron reads the **previous** activations of all its inputs — there are no circular dependency issues even with recurrent connections.

An `Actuator` maps a neuron's binary output to a bar's rest length:

$$L_{0,i}^{(t+1)} = L_{0,i}^{(t)} + a_k^{(t+1)} \cdot r_i$$

where $r_i$ is the `bar_range` (signed, in metres). When the neuron fires the bar is stretched or compressed by exactly $|r_i|$; when it is silent the bar keeps its previous rest length. Actuation is clamped so no single step can change a rest length by more than 1 cm.

The neural overlay rendered above each robot colours neurons **red** when firing and grey when silent. Synapse lines are **blue** for excitatory weights and **red** for inhibitory weights; actuator connections are drawn in **green**.

### This example — anti-phase oscillator

The network contains exactly two neurons with cross-coupling only (no self-weights):

```
Neuron 0 ──w=1──► Neuron 1
Neuron 1 ──w=1──► Neuron 0
         threshold = 0.5 (both)
```

Neuron 0 is primed active at $t=0$; neuron 1 starts silent. Because each neuron fires when it receives the other's previous activation and only then, they alternate every tick:

| Tick | $a_0$ | $a_1$ |
|------|--------|--------|
| 0    | 1      | 0      |
| 1    | 0      | 1      |
| 2    | 1      | 0      |
| …    | …      | …      |

Two actuators are wired to the same bar (left strut, bar 3):

- **Actuator 0** — neuron 0 → bar 3, `bar_range = +0.010 m` (strut lengthens when neuron 0 fires)
- **Actuator 1** — neuron 1 → bar 3, `bar_range = −0.010 m` (strut shortens when neuron 1 fires)

The opposing signs mean the strut oscillates ±1 cm around its natural length with no cumulative drift. The quasi-static physics engine responds by rocking the apex back and forth, producing a visible periodic deformation.

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

`Robot` is the container: it holds `std::vector`s of each type and provides `toYAML()` / `fromYAML()` for serialisation. Bar stiffness is not stored directly; it is derived from material constants on demand:

$$k_i = \frac{E \cdot A_i}{L_{0,i}}$$

where $A_i = \pi r_i^2$, $E = 0.896\ \text{GPa}$, and $L_{0,i}$ is the rest length.

### Physics — `Simulator`

`Simulator` takes ownership of a robot's geometry (copied into an $N \times 3$ Eigen matrix) and implements quasi-static energy minimisation. This is the same approach used by Lipson & Pollack — there is no velocity or momentum; the system always seeks the nearest energy minimum from its current configuration.

#### Energy function

The total potential energy is the sum of three terms:

$$H = \underbrace{\sum_i k_i \delta_i^2}_{H_{\text{elastic}}} + \underbrace{\sum_j m_j g z_j}_{H_{\text{gravity}}} + \underbrace{\sum_j k_{\text{floor}} \min(z_j, 0)^2}_{H_{\text{floor}}}$$

- $\delta_i = \|p_{v_2} - p_{v_1}\| - L_{0,i}$ is the extension of bar $i$.
- Vertex masses $m_j$ are computed by lumping each bar's mass ($\rho A L_0$) equally onto its two endpoints.
- $k_{\text{floor}} = 1.4 \times 10^6\ \text{N/m}$ (matched to a typical bar stiffness). The floor is intentionally soft — the robot penetrates slightly, but the gradient always pushes it back up. Full floor accuracy is not required for quasi-static locomotion evaluation.

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

### Rendering — `SceneRenderer` hierarchy

All rendering shares a single base class to guarantee consistent camera, lighting, and floor across tools.

```
SceneRenderer          ← orbital camera, drawRobot(), drawFloor()
├── SnapshotRenderer   ← renders one frame → PNG, closes
└── VideoRenderer      ← keeps window open; addFrame() → numbered PNGs
                         finish() → ffmpeg → MP4, cleans up temp dir
```

`VideoRenderer` uses an off-screen Raylib context (no visible window). Each call to `addFrame(robot, sim_time)` runs one full render pass and writes `frame_XXXX.png` to a unique temporary directory. `finish(output_path)` invokes:

```
ffmpeg -y -framerate <fps> -i <tmp>/frame_%04d.png -c:v libx264 -pix_fmt yuv420p <output>
```

Coordinate system: the physics simulation uses **Z-up**; `SceneRenderer::toRaylib()` remaps to Raylib's **Y-up** convention before any draw calls.

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
| `fall_animation` | Runs quasi-static simulation from YAML and produces an MP4 |

### `fall_animation`

```bash
fall_animation <robot.yaml> <simulation.yaml>
```

`simulation.yaml` controls all physics and output parameters:

```yaml
fps:             30
num_frames:      200
steps_per_frame: 5000    # gradient steps between captured frames
step_size:       1.0e-7  # D_s; must be < 1/(2k) for stability
width:           640
height:          480
output:          /tmp/tetrahedron_fall.mp4
```

See `examples/robot_fall/` for a worked example with an upside-down tetrahedron.

---

## Examples

| Directory | What it shows |
|---|---|
| `examples/robot_visualize/` | Interactive view + single snapshot of a robot |
| `examples/deformation_series/` | Energy sanity-check: elastic energy rises from ~0 as a vertex is displaced |
| `examples/robot_fall/` | Fall animation: inverted tetrahedron descends under gravity and tips over |
| `examples/robot_sineactuator/` | Sinusoidal actuator: bar length driven by a `sin(ωt)` debug waveform |
| `examples/robot_neuralactuator/` | Neural actuation: 2-neuron anti-phase oscillator drives a strut bar ±1 cm per tick |

---

## Project status

| Phase | Description | Status |
|---|---|---|
| 1 | Data structures, YAML serialisation | ✅ |
| 2 | Rendering (SceneRenderer, SnapshotRenderer, VideoRenderer) | ✅ |
| 3.1–3.3 | Energy function, quasi-static relaxation, floor + friction | ✅ |
| 3.4–3.6 | Neural network tick, actuator coupling, fitness wrapper | planned |
| 3.7 | VideoRenderer | ✅ |
| 4 | Genetic algorithm / evolver | planned |
| 5 | Logging, fitness CSV, phylogenetic tree, automated video | planned |
