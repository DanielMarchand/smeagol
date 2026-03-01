#pragma once

#include "Vertex.h"
#include "Bar.h"
#include "Neuron.h"
#include "Actuator.h"
#include "DebugActuator.h"
#include "Materials.h"

#include <Eigen/Core>
#include <cstddef>
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
     * Actuator bar/neuron indices) are in-bounds.
     */
    [[nodiscard]] bool isValid() const;

    // ── YAML I/O (implemented in phase 1.4) ──────────────────────────────

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
    static ID s_next_id;           ///< Auto-incrementing ID counter
};
