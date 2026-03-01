# Project Plan: Golem 2000 (Evolutionary Robotics) Replication in C++

---

## 1. Implementation Phases & TODOs

### Phase 1: Data Structures, YAML & Core Setup

- [x] **1.1** Setup `CMakeLists.txt` integrating Eigen3, Raylib, and yaml-cpp. Created `.gitignore` (standard C++ & Python rules).
- [x] **1.5** Basic unit tests: `tests/test_dependencies.cpp` verifies Eigen3, yaml-cpp, and raylib compile, link, and run correctly (registered with CTest).
- [x] **1.2** Implement the `RobotPart` base class and its subclasses (`Vertex`, `Bar`, `Neuron`, `Actuator`).
- [x] **1.3** Implement the `Robot` class storing standard `std::vector`s of the parts.
- [ ] **1.4** Write the YAML serialization logic. Map the paper's `<vertices><bars>...` format strictly into YAML arrays/objects.
  - Requirement: Ensure specific material properties are globally accessible ($E=0.896$ GPa, $\rho=1000$ kg/m$^3$, $S_{yield}=19$ MPa).

### Phase 2: Shared Rendering Engine (Raylib)

- [ ] **2.1** Implement `SceneRenderer`. Set up a fixed 3D orbital camera, a ground plane (`DrawGrid`), and the `drawRobot()` routine.
- [ ] **2.2** Implement `SnapshotRenderer`. Add logic to render a single frame and use Raylib's `TakeScreenshot()` to output a still image. Link this to `Robot::saveDebugImage()`.
- [ ] **2.3** Implement `VideoRenderer`. Add logic to capture a sequence of frames during a simulation loop. Add a destructor/finish method that invokes `std::system("ffmpeg -framerate 30 -i tmp/%04d.png out.mp4")` to compile the video.

### Phase 3: The Physics & Neural Simulator

- [ ] **3.1** Energy Function. Implement $H = \sum k_i \delta_i^2 + \sum m_j g h_j$.
  - $k_i = \frac{E A_i}{l_i}$ and $\delta_i = \sqrt{(\Delta x)^2 + (\Delta y)^2 + (\Delta z)^2} - l_i$
- [ ] **3.2** Quasi-Static Relaxation. Implement the iterative solver.
  - Calculate partial derivatives $\frac{\partial H}{\partial v}$.
  - Update vertices: $v = v - Ds \frac{\partial H}{\partial v}$.
  - Add uniform noise to prevent settling in unstable equilibria.
- [ ] **3.3** Environment Physics.
  - Collision: Add high penalty energy if a vertex $z < 0$.
  - Friction: Lock $x,y$ movement if the lateral force derivative is lower than the static friction coefficient times normal force.
- [ ] **3.4** Neural Network Tick. Implement the discrete loop updating neuron activations based on thresholds and synapse coefficients.
- [ ] **3.5** Actuator Coupling. Update the target resting length ($l_i$) of bars attached to actuators based on neuron output. Clamp the change to $\le 1$ cm per cycle.
- [ ] **3.6** Fitness Wrapper. Implement `evaluate(Robot)`, running the physics/neural loop for exactly 12 cycles and returning the net CoM Euclidean distance.

### Phase 4: The Evolver (Genetic Algorithm)

- [ ] **4.1** Population Setup. Initialize 200 "null" (empty) individual Robots. Ensure unique, reproducible random seeds for the Evolver.
- [ ] **4.2** Mutation Operators. Implement the 4 independent mutations (at least one applied per offspring):
  - **p=0.10**: Small change in bar length or neuron synaptic weight.
  - **p=0.01**: Add/remove a dangling bar or unconnected neuron.
  - **p=0.03**: Split a vertex and add a bar, OR split a bar and add a vertex.
  - **p=0.03**: Attach or detach a neuron to an actuator/bar.
- [ ] **4.3** Evolutionary Loop. Implement steady-state replacement.
  - Select parent (fitness-proportionate).
  - Mutate to create offspring.
  - Evaluate offspring via `Simulator`.
  - Replace individual (random replacement).
- [ ] **4.4** Run Condition. Loop this process for 300 to 600 generations ($\approx 10^5$ evaluations).

### Phase 5: Logging, Analytics & Media Autostore

- [ ] **5.1** File Hierarchy. Ensure the Evolver creates a structured output folder (e.g., `runs/run_TIMESTAMP/`).
- [ ] **5.2** Fitness Logging. Output a `fitness.csv` tracking `[Generation, Eval_ID, Fitness]`. This data will be used to generate the Generation vs. Fitness scatter plots (0–0.38 fitness over 167+ generations).
- [ ] **5.3** Phylogenetic Logging. Output a `lineage.csv` tracking `[Generation, Child_ID, Parent_ID, Fitness]`. This will be used externally (e.g., via Python/matplotlib) to plot the ancestral proximity trees (showing divergence, convergence, speciation, and mass extinction).
- [ ] **5.4** Automated Video Generation. Every $N$ generations, identify the most fit Robot. Pass it to a `Simulation` linked to a `VideoRenderer` and auto-store an MP4 showing its 12-cycle movement in a `videos/` subfolder.

---

## 2. Project Overview & Dependencies

This project aims to faithfully replicate the 2000 Lipson & Pollack evolutionary robotics experiment ("Golem"). The system will evolve 3D mechanical structures (trusses) and their neural controllers simultaneously to maximize locomotion across an infinite plane.

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