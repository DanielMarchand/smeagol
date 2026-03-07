#pragma once

#include "Robot.h"
#include <random>
#include <string>
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
    double p_perturb        = 0.10;  ///< probability perturbElement fires each step
    double p_add_bar_new    = 0.005; ///< Strategy A: new vertex + bar dangled off existing
    double p_add_bar_bridge = 0.010; ///< Strategy B: connect two existing unconnected vertices
    double p_add_neuron        = 0.005; ///< add a fully-connected neuron
    double p_remove_bar        = 0.005; ///< remove a random bar (connectivity-guarded)
    double p_remove_neuron     = 0.005; ///< remove a random neuron
    double p_remove_bar_edge   = 0.005; ///< reverse of Strategy A: remove a leaf bar + its leaf vertex (-1v -1b)
    double p_remove_bar_bridge = 0.005; ///< reverse of Strategy B: remove a non-bridge bar between two degree-2+ vertices (-1b)
    double p_join_vertex       = 0.005; ///< reverse of splitElement bar-bisect: merge a degree-2 internal vertex into one bar
    double p_split_vertex      = 0.03;  ///< probability splitElement fires
    double p_attach_neuron     = 0.03;  ///< attach a neuron to an unactuated bar
    double p_detach_neuron     = 0.03;  ///< detach all actuators from an actuated bar
    double p_rewire            = 0.03;  ///< probability rewireNeuron fires

    // ── Retry limit ───────────────────────────────────────────────────────────
    /// When a stochastically-fired operator returns false (e.g. all bars are
    /// bridges so removeBar cannot disconnect-safely), it is retried up to this
    /// many times before the mutation is aborted and the robot returned as a
    /// clone.  This is the per-method retry limit, not the stochastic round count.
    int num_method_retries = 10;

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

    // ── perturbElement growth bias ─────────────────────────────────────────────
    /// Bars whose rest_length is below this threshold will only ever grow when
    /// perturbed (multiplier drawn from U[1.0, 1+frac] instead of U[1-frac, 1+frac]).
    double perturb_bar_grow_only_below = 0.02;

    // ── splitElement minimum bar length ──────────────────────────────────────
    /// Bar-bisect splits are skipped for bars shorter than this; the operator
    /// falls back to vertex-split (or returns false if no vertices exist).
    double split_bar_min_length = 0.03;

    // ── splitElement offset ───────────────────────────────────────────────────
    double split_vertex_offset = 0.01; ///< vertex-split offset drawn from U[-v,+v]

    // ── attachDetach actuator range ───────────────────────────────────────────
    double actuator_range_max = 0.01; ///< new actuator bar_range drawn from U[0, r]

    // ── addConnectedNeuron synapse weight ─────────────────────────────────────
    double new_synapse_weight_min = 0.50; ///< lower bound of |new synapse weight|
    double new_synapse_weight_max = 1.50; ///< upper bound of |new synapse weight|
    // ── stiffness range for new bars added by mutations ──────────────────────────
    double new_bar_stiffness_min = 10000.0;  ///< min stiffness k [N/m] for new bars
    double new_bar_stiffness_max = 100000.0; ///< max stiffness k [N/m] for new bars
    /** Load all fields from a YAML mapping node (used as a sub-node of the
     *  top-level config).  Any field absent in the node keeps its default. */
    static MutatorParams fromYAML(const YAML::Node& node);

    /** Emit all fields as a YAML mapping block into @p out (no BeginMap/EndMap
     *  wrapper — caller is responsible for the enclosing map). */
    void toYAML(YAML::Emitter& out) const;
};

// ── MutationRecord ────────────────────────────────────────────────────────────

/**
 * @brief Records which mutation operators fired during a single mutate() call.
 *
 * Returned by Mutator::mutateRecord() so callers can log/audit what happened.
 * The five bool flags correspond to the five operators; was_forced is true when
 * none fired stochastically and one was auto-selected to guarantee progress.
 */
struct MutationRecord
{
    bool perturb        = false;  ///< perturbElement fired
    bool add_bar_new    = false;  ///< addBarNew fired (Strategy A)
    bool add_bar_bridge = false;  ///< addBarBridge fired (Strategy B)
    bool add_neuron     = false;  ///< addNeuron fired
    bool remove_bar        = false;  ///< removeBar fired
    bool remove_neuron     = false;  ///< removeNeuron fired
    bool remove_bar_edge   = false;  ///< removeBarEdge fired (reverse of addBarNew)
    bool remove_bar_bridge = false;  ///< removeBarBridge fired (reverse of addBarBridge)
    bool join_vertex     = false;  ///< joinElement fired (reverse of splitElement bar-bisect)
    bool split_vertex    = false;  ///< splitElement fired
    bool attach_neuron   = false;  ///< attachNeuron fired
    bool detach_neuron   = false;  ///< detachNeuron fired
    bool rewire          = false;  ///< rewireNeuron fired
    bool was_cloned      = false;  ///< true if retries exhausted — robot returned unchanged

    /// Per-op structural detail (empty if op didn't fire).
    /// For perturb: what was touched, e.g. "bar-len neuron".
    /// For all others: topology delta, e.g. "+1b", "-1n", "+1v +1b".
    std::string perturb_detail;
    std::string add_bar_new_detail;
    std::string add_bar_bridge_detail;
    std::string add_neuron_detail;
    std::string remove_bar_detail;
    std::string remove_neuron_detail;
    std::string remove_bar_edge_detail;
    std::string remove_bar_bridge_detail;
    std::string join_vertex_detail;
    std::string split_vertex_detail;
    std::string attach_neuron_detail;
    std::string detach_neuron_detail;
    std::string rewire_detail;

    /// How many full passes through all stochastic ops before one fired.
    /// 1 means something fired on the first pass (normal); >1 indicates low-probability regime.
    int stoch_rounds = 0;

    /// Number of invalid-robot rerolls before this child was accepted.
    int rerolls = 0;

    /// Final topology counts after all mutation ops.
    int v_after = 0, b_after = 0, n_after = 0, a_after = 0;

    /// Human-readable description with per-op detail, reroll count and final state.
    std::string describe() const;
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

    /**
     * Same as mutate() but also returns a MutationRecord describing which
     * operators fired.  Used by the Evolver to write per-robot log files.
     */
    static MutationRecord mutateRecord(Robot& robot, std::mt19937& rng,
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
     * Add a bar using Strategy A: pick an existing vertex, place a new vertex at
     * median-length offset in a random direction, and connect them.
     * Bootstraps from scratch if no vertices exist.
     * Always returns true.
     */
    static bool addBarNew(Robot& robot, std::mt19937& rng,
                          const MutatorParams& params = MutatorParams{});

    /**
     * Add a bar using Strategy B: find two existing vertices with no bar between
     * them and connect them with a new bar at their Euclidean distance.
     * Returns false if all vertex pairs are already connected.
     */
    static bool addBarBridge(Robot& robot, std::mt19937& rng,
                              const MutatorParams& params = MutatorParams{});

    /**
     * Add one fully-connected neuron (at least one non-zero synapse or an
     * immediate actuator if no other neurons exist).
     * Always succeeds.
     */
    static void addNeuron(Robot& robot, std::mt19937& rng,
                          const MutatorParams& params = MutatorParams{});

    /**
     * Remove a randomly chosen bar.
     * Guarded: if removal disconnects the graph the robot is restored and
     * false is returned.  Returns false if no bars exist.
     */
    static bool removeBar(Robot& robot, std::mt19937& rng,
                          const MutatorParams& params = MutatorParams{});

    /**
     * Remove a randomly chosen neuron.
     * Returns false if no neurons exist.
     */
    static bool removeNeuron(Robot& robot, std::mt19937& rng,
                              const MutatorParams& params = MutatorParams{});

    /**
     * Reverse of addBarNew (Strategy A): remove a leaf bar and its degree-1 vertex.
     * Safe: leaf removal can never disconnect the graph.
     * Returns false when no leaf bars exist (all vertices have degree >= 2).
     */
    static bool removeBarEdge(Robot& robot, std::mt19937& rng,
                              const MutatorParams& params = MutatorParams{});

    /**
     * Reverse of addBarBridge (Strategy B): remove a non-bridge bar whose both
     * endpoints retain degree >= 1 after removal (neither becomes isolated).
     * Returns false when no such bar exists.
     */
    static bool removeBarBridge(Robot& robot, std::mt19937& rng,
                                const MutatorParams& params = MutatorParams{});

    /**
     * Reverse of splitElement (bar-bisect mode).
     * Finds an internal degree-2 vertex w whose both neighbours u, v have
     * degree >= 2 and are not yet directly connected.  Removes w and its two
     * incident bars, inserts bar(u, v) with rest_length = |p_u - p_v| and
     * stiffness = (k1+k2)/2.  Actuators on the two half-bars are merged onto
     * the new bar; orphaned neurons are left intact.
     * Returns false when no eligible vertex exists or the merged bar would be
     * shorter than params.bar_length_min.
     *
     * NOT YET IMPLEMENTED — stub always returns false.
     */
    static bool joinElement(Robot& robot, std::mt19937& rng,
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
     * Attach: pick a bar that has no actuators and wire it to a random neuron
     * (creating one if none exist).  Returns false if all bars are already
     * actuated.
     */
    static bool attachNeuron(Robot& robot, std::mt19937& rng,
                             const MutatorParams& params = MutatorParams{});

    /**
     * Detach: pick a bar that has at least one actuator and remove all of its
     * actuators.  Returns false if no bars are currently actuated.
     */
    static bool detachNeuron(Robot& robot, std::mt19937& rng,
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
};
