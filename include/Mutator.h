#pragma once

#include "Robot.h"
#include <random>
#include <yaml-cpp/yaml.h>

// ── MutatorParams ─────────────────────────────────────────────────────────────

/**
 * @brief All tuneable constants for the five mutation operators.
 *
 * Defaults match the values hardcoded in Lipson & Pollack (2000).
 * Load from a YAML sub-node with MutatorParams::fromYAML(node["mutation"]).
 * Serialise with toYAML(emitter) which writes a YAML mapping block.
 */
struct MutatorParams
{
    // ── Per-operator firing probabilities ─────────────────────────────────────
    double p_perturb    = 0.10;  ///< probability perturbElement fires each step
    double p_add_remove = 0.01;  ///< probability addRemoveElement fires
    double p_split      = 0.03;  ///< probability splitElement fires
    double p_attach     = 0.03;  ///< probability attachDetach fires
    double p_rewire     = 0.03;  ///< probability rewireNeuron fires

    // ── perturbElement knobs ──────────────────────────────────────────────────
    double perturb_bar_frac      = 0.10; ///< bar rest_length multiplied by U[1-f, 1+f]
    double perturb_threshold_mag = 0.50; ///< threshold perturbed by U[-m, +m]
    double perturb_weight_mag    = 0.50; ///< synapse weight perturbed by U[-m, +m]

    // ── Shared bar-length clamp (perturbElement + addBarMutation) ─────────────
    double bar_length_min = 0.01; ///< minimum bar rest_length after clamping
    double bar_length_max = 1.00; ///< maximum bar rest_length after clamping

    // ── Neuron threshold clamp ────────────────────────────────────────────────
    double threshold_min = 0.00; ///< minimum neuron threshold after clamping
    double threshold_max = 2.00; ///< maximum neuron threshold after clamping

    // ── splitElement offset ───────────────────────────────────────────────────
    double split_vertex_offset = 0.01; ///< vertex-split offset drawn from U[-v,+v]

    // ── attachDetach actuator range ───────────────────────────────────────────
    double actuator_range_max = 0.01; ///< new actuator bar_range drawn from U[0, r]

    // ── addConnectedNeuron synapse weight ─────────────────────────────────────
    double new_synapse_weight_min = 0.50; ///< lower bound of |new synapse weight|
    double new_synapse_weight_max = 1.50; ///< upper bound of |new synapse weight|

    /** Load all fields from a YAML mapping node (used as a sub-node of the
     *  top-level config).  Any field absent in the node keeps its default. */
    static MutatorParams fromYAML(const YAML::Node& node);

    /** Emit all fields as a YAML mapping block into @p out (no BeginMap/EndMap
     *  wrapper — caller is responsible for the enclosing map). */
    void toYAML(YAML::Emitter& out) const;
};

// ── Mutator ───────────────────────────────────────────────────────────────────

/**
 * @brief Stateless mutation utility – five operators from Lipson & Pollack (2000)
 *        plus a topological rewiring operator.
 *
 * All methods take a Robot& and std::mt19937& by reference and modify the
 * robot in-place.  Individual operator methods return true when the robot
 * was actually modified (used for the "at least one" guarantee in mutate()).
 *
 * Canonical probabilities (used by mutate()):
 *   perturbElement    p = 0.10  –  nudge a bar length or neuron threshold/weight
 *   addRemoveElement  p = 0.01  –  grow or shrink the graph by one element
 *   splitElement      p = 0.03  –  subdivide a vertex or bar
 *   attachDetach      p = 0.03  –  flip a bar between structural and actuated
 *   rewireNeuron      p = 0.03  –  reassign an actuator's bar or neural source
 */
class Mutator
{
public:
    /**
     * Apply all five operators independently at their canonical probabilities.
     * Guarantees at least one modification (forces an operator if none fired).
     *
     * @param robot   Individual to mutate (modified in-place).
     * @param rng     Seeded Mersenne-Twister (caller owns the state).
     * @param params  Mutation knobs (defaults to MutatorParams{}).
     */
    static void mutate(Robot& robot, std::mt19937& rng,
                       const MutatorParams& params = MutatorParams{});

    // ── Individual operators (public for targeted use in tests / demos) ───

    /**
     * Perturb the rest-length of a random bar (±10%) or the threshold /
     * a synapse weight of a random neuron (additive uniform ±0.5).
     * Both sub-operations run independently in one call.
     * Returns true if at least one value was changed.
     */
    static bool perturbElement(Robot& robot, std::mt19937& rng,
                               const MutatorParams& params = MutatorParams{});

    /**
     * Add or remove a single bar or neuron element.
     * Add-bar uses a new vertex + Gaussian-length bar (Strategy A) or
     * picks two existing distinct vertices (Strategy B); uniqueness is enforced.
     * Add-neuron always produces a connected neuron (≥1 non-zero synapse or
     * an immediate actuator).
     * Returns true if the graph was changed.
     */
    static bool addRemoveElement(Robot& robot, std::mt19937& rng,
                                  const MutatorParams& params = MutatorParams{});

    /**
     * Split a random vertex into two (connected by a tiny new bar) or
     * split a random bar at its midpoint (inserting a new vertex).
     * Actuators on a split bar are re-routed to the first half-bar.
     * Returns true if the graph was changed.
     */
    static bool splitElement(Robot& robot, std::mt19937& rng,
                             const MutatorParams& params = MutatorParams{});

    /**
     * Toggle a random bar between structural and actuated:
     *   structural → actuated:  add Actuator(bar, random_neuron, random_range)
     *   actuated   → structural: remove ALL actuators targeting that bar
     * Returns true if the actuator list was changed.
     */
    static bool attachDetach(Robot& robot, std::mt19937& rng,
                             const MutatorParams& params = MutatorParams{});

    /**
     * Topological rewire: pick a random actuator and reassign either its
     * bar_idx (physical target) or its neuron_idx (neural source) to a
     * different index (50/50 which end gets rewired).
     * Returns true if an actuator was modified.
     */
    static bool rewireNeuron(Robot& robot, std::mt19937& rng,
                             const MutatorParams& params = MutatorParams{});

private:
    /**
     * Add one neuron that is guaranteed to be connected to the network:
     * at least one synapse weight is non-zero OR an actuator is created
     * immediately linking it to a random bar.
     */
    static void addConnectedNeuron(Robot& robot, std::mt19937& rng,
                                    const MutatorParams& params);

    /**
     * Attempt to add one bar via Strategy A (new vertex placed at median-length
     * from an existing vertex, in a random direction) or Strategy B (pick two
     * existing distinct vertices and use their Euclidean distance as rest_length).
     * Returns true if a bar was added.
     */
    static bool addBarMutation(Robot& robot, std::mt19937& rng,
                               const MutatorParams& params);
};
