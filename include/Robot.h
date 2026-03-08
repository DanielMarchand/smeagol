#pragma once

#include "Vertex.h"
#include "Bar.h"
#include "Neuron.h"
#include "Actuator.h"
#include "DebugActuator.h"
#include "Materials.h"

#include <Eigen/Core>
#include <atomic>
#include <cstddef>
#include <limits>
#include <queue>
#include <string>
#include <vector>

/**
 * @brief Container for one individual robot in the population.
 *
 * A Robot is a complete genotype: it holds typed lists of all parts that
 * together define both the morphology (vertices + bars) and the neural
 * controller (neurons + actuators).  The Simulator operates directly on
 * these lists during physics and neural evaluation.
 *
 * Index conventions
 * -----------------
 * Bar::v1 / Bar::v2      → index into Robot::vertices
 * Actuator::bar_idx      → index into Robot::bars
 * Actuator::neuron_idx   → index into Robot::neurons
 * Neuron::synapse_weights has size() == neurons.size() (maintained by callers)
 *
 * Mutators return the index of the newly added part.
 */
class Robot
{
public:
    using ID = std::size_t;

    // ── Construction ──────────────────────────────────────────────────────

    Robot();
    explicit Robot(ID id);

    /// Deep-copy: all parts cloned, id preserved, new runtime state.
    [[nodiscard]] Robot clone() const;

    /// Returns the next unique ID without constructing a Robot.
    static ID nextId() { return s_next_id++; }

    // ── Part mutators ─────────────────────────────────────────────────────

    /// Append a vertex; returns its new index.
    int addVertex(const Vertex& v);

    /// Append a bar; returns its new index.
    int addBar(const Bar& b);

    /**
     * Append a neuron; returns its new index.
     * The neuron's synapse_weights is automatically resized to match the
     * current neuron count (new entry initialised to 0).
     */
    int addNeuron(const Neuron& n);

    /// Append an actuator; returns its new index.
    int addActuator(const Actuator& a);

    /**
     * Remove vertex at idx.
     * Any Bar referencing this vertex is also removed; all remaining
     * indices in Bar::v1/v2 are updated to stay consistent.
     */
    void removeVertex(int idx);

    /**
     * Remove bar at idx.
     * All Actuator::bar_idx values are patched; actuators referencing
     * this bar are removed.
     */
    void removeBar(int idx);

    /**
     * Remove neuron at idx.
     * All Actuator::neuron_idx values are patched; actuators referencing
     * this neuron are removed.  All remaining Neuron::synapse_weights
     * have the corresponding column erased.
     */
    void removeNeuron(int idx);

    /// Remove actuator at idx.
    void removeActuator(int idx);

    /// Remove all parts.
    void clear();

    // ── Physics helpers ───────────────────────────────────────────────────

    /**
     * Mass of vertex i [kg].
     * Each bar's mass (ρ·A·L) is split equally between its two endpoints.
     */
    [[nodiscard]] double vertexMass(int i) const;

    /**
     * Centre of mass in world space [m].
     * Returns the origin if the robot has no vertices.
     */
    [[nodiscard]] Eigen::Vector3d centerOfMass() const;

    // ── Validation ────────────────────────────────────────────────────────

    /**
     * Returns true iff all cross-references (Bar vertex indices,
     * Actuator bar/neuron indices) are in-bounds, and every vertex is
     * the endpoint of at least one bar (exception: a single-vertex,
     * zero-bar robot is valid).
     *
     * Also checks graph connectivity: the bar network must form a single
     * connected component.  A robot with two disjoint subgraphs (e.g. two
     * separate clusters of bars with no bar joining them) fails this check.
     */
    /// Returns false if the robot is topologically inconsistent (bad indices,
    /// disconnected graph, isolated vertices, duplicate actuators, etc.).
    /// Also returns false if any bar's rest_length is below @p bar_length_min.
    /// Default bar_length_min=0 preserves the original behaviour for callers
    /// that don't need the length check.
    [[nodiscard]] bool isValid(double bar_length_min = 0.0) const;

    /**
     * Returns true iff the bar graph forms a single connected component.
     *
     * Uses a BFS flood-fill from vertex 0.  Robots with 0 or 1 vertex are
     * trivially connected.  Robots with vertices but no bars are disconnected
     * unless there is exactly one vertex.
     *
     * Complexity: O(V + E) where V = vertices.size(), E = bars.size().
     */
    [[nodiscard]] bool isConnected() const;

    /**
     * Runs the neural network in isolation (no physics) to determine whether
     * any actuator-driving neuron ever changes its output within the probe
     * window.
     *
     * Algorithm
     * ---------
     * 1. Initialise activations from each Neuron::activation field.
     * 2. Tick the network @p delay_cycles times (mirrors the physics warm-up
     *    period; output is ignored).
     * 3. Record the current output of every actuator-driving neuron.
     * 4. Tick for @p probe_cycles more cycles; return true as soon as any
     *    actuator-driving neuron's output differs from the snapshot.
     * 5. If no delta is seen, return false (the network is "dead" — stuck in
     *    a fixed point and incapable of driving movement).
     *
     * A delta check rather than a "fires at all" check catches the
     * permanently-ON case (constant output → no oscillation → no locomotion).
     *
     * @param delay_cycles  Neural ticks to skip before watching (default 5).
     * @param probe_cycles  Neural ticks to watch for output changes (default 10).
     * @return true if at least one actuator-driving neuron changes state.
     */
    [[nodiscard]] bool probeNeuralActivity(int delay_cycles = 5,
                                           int probe_cycles = 10) const;

    /// Returns true if this robot provably cannot produce locomotion.
    ///
    /// Fast structural check: no actuators → bars never change.
    /// Dynamic check: probeNeuralActivity() → all actuator-driving neurons
    /// are stuck in a fixed point (permanently on or off), so no oscillation
    /// can drive movement.  Both cases give fitness 0.
    [[nodiscard]] bool isNonMover() const
    {
        return actuators.empty() || !probeNeuralActivity();
    }

    /**
     * Remove any vertex that is not referenced by any bar.
     * Exception: if the robot has only one vertex and no bars, it is
     * left untouched.
     * Bar and actuator indices are re-patched automatically.
     */
    void pruneIsolatedVertices();

    // ── YAML I/O ───────────────────────────────────────────────────

    void toYAML(const std::string& path) const;
    static Robot fromYAML(const std::string& path);

    /**
     * Render a still PNG of the robot's current topology for debugging.
     * Requires a display server (X11/Wayland).  Uses SnapshotRenderer
     * internally so the window opens and closes automatically.
     *
     * @param path  Output file path (PNG).
     */
    void saveDebugImage(const std::string& path) const;

    // ── Data ──────────────────────────────────────────────────────────────

    ID id;                         ///< Unique individual identifier
    ID parent_id = std::numeric_limits<ID>::max(); ///< Parent's ID (max = no parent)
    double fitness = 0.0;          ///< Most recent evaluated fitness [m]

    std::vector<Vertex>   vertices;
    std::vector<Bar>      bars;
    std::vector<Neuron>   neurons;
    std::vector<Actuator> actuators;

    /**
     * Debug-only actuators driven by a sine wave instead of a neuron.
     * Not serialised to YAML.  Used to manually exercise the physics
     * simulator before the neural controller is implemented.
     */
    std::vector<DebugActuator> debug_actuators;

private:
    static std::atomic<ID> s_next_id;  ///< Auto-incrementing ID counter (atomic for thread-safety)
};
