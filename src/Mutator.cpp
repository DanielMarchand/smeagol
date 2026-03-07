/**
 * Mutator.cpp
 *
 * Implementation of the mutation operators from Lipson & Pollack (2000),
 * updated to match the paper's description more closely.
 *
 * Key design decisions:
 *  - "Dangling bar" is represented as: new vertex + new bar attached to one
 *    existing vertex (gaussian-sampled length).  A secondary path links two
 *    existing vertices, but enforces uniqueness (no duplicate vertex pairs).
 *  - "Unconnected neuron" is impossible here; every added neuron gets at
 *    least one nonzero synapse weight or an immediate actuator attachment.
 *  - attachDetach toggles a bar between structural and actuated (flips it).
 *  - rewireNeuron reassigns an actuator's bar or neuron target (topological
 *    rewiring of the neural → morphology coupling).
 */

#include "Mutator.h"
#include "Bar.h"
#include "Vertex.h"
#include "Neuron.h"
#include "Actuator.h"
#include "Materials.h"

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <utility>
#include <vector>

// ── anonymous-namespace helpers ───────────────────────────────────────────────

namespace {

double uniform01(std::mt19937& rng)
{
    return std::uniform_real_distribution<double>(0.0, 1.0)(rng);
}

double uniformReal(std::mt19937& rng, double lo, double hi)
{
    return std::uniform_real_distribution<double>(lo, hi)(rng);
}

int uniformInt(std::mt19937& rng, int lo, int hi)
{
    return std::uniform_int_distribution<int>(lo, hi)(rng);
}

/// Compute the median bar rest_length, or fallback if no bars.
double medianBarLength(const Robot& robot, double fallback = 0.1)
{
    if (robot.bars.empty()) return fallback;
    std::vector<double> lens;
    lens.reserve(robot.bars.size());
    for (const auto& b : robot.bars)
        lens.push_back(b.rest_length);
    const std::size_t mid = lens.size() / 2;
    std::nth_element(lens.begin(), lens.begin() + mid, lens.end());
    return lens[mid];
}

/**
 * Returns true if a bar (or its mirror) already exists between v1 and v2.
 * Enforces the uniqueness invariant: at most one bar between any pair.
 */
bool barExists(const Robot& robot, int v1, int v2)
{
    for (const auto& b : robot.bars)
        if ((b.v1 == v1 && b.v2 == v2) || (b.v1 == v2 && b.v2 == v1))
            return true;
    return false;
}

/// Minimum Z coordinate for any new vertex – keeps robots above the floor.
constexpr double kFloorZ = 0.01;

/// Uniform random unit direction in 3D (Marsaglia method).
Eigen::Vector3d randomDirection(std::mt19937& rng)
{
    Eigen::Vector3d d;
    double r;
    do {
        d = { uniformReal(rng, -1.0, 1.0),
              uniformReal(rng, -1.0, 1.0),
              uniformReal(rng, -1.0, 1.0) };
        r = d.norm();
    } while (r < 1e-9 || r > 1.0);
    return d / r;
}

} // namespace

// ── MutatorParams YAML I/O ────────────────────────────────────────────────────

MutatorParams MutatorParams::fromYAML(const YAML::Node& n)
{
    MutatorParams p;
    if (n["p_perturb"])              p.p_perturb              = n["p_perturb"].as<double>();
    if (n["p_add_bar_new"])          p.p_add_bar_new          = n["p_add_bar_new"].as<double>();
    if (n["p_add_bar_bridge"])       p.p_add_bar_bridge       = n["p_add_bar_bridge"].as<double>();
    if (n["p_add_neuron"])           p.p_add_neuron           = n["p_add_neuron"].as<double>();
    if (n["p_remove_bar"])           p.p_remove_bar           = n["p_remove_bar"].as<double>();
    if (n["p_remove_neuron"])        p.p_remove_neuron        = n["p_remove_neuron"].as<double>();
    if (n["p_remove_bar_edge"])      p.p_remove_bar_edge      = n["p_remove_bar_edge"].as<double>();
    if (n["p_remove_bar_bridge"])    p.p_remove_bar_bridge    = n["p_remove_bar_bridge"].as<double>();
    if (n["p_join_vertex"])          p.p_join_vertex          = n["p_join_vertex"].as<double>();
    if (n["p_split_vertex"])         p.p_split_vertex         = n["p_split_vertex"].as<double>();
    if (n["p_attach_neuron"])        p.p_attach_neuron        = n["p_attach_neuron"].as<double>();
    if (n["p_detach_neuron"])        p.p_detach_neuron        = n["p_detach_neuron"].as<double>();
    if (n["p_rewire"])               p.p_rewire               = n["p_rewire"].as<double>();
    if (n["num_method_retries"])     p.num_method_retries     = n["num_method_retries"].as<int>();
    if (n["perturb_bar_frac"])       p.perturb_bar_frac       = n["perturb_bar_frac"].as<double>();
    if (n["perturb_threshold_mag"])  p.perturb_threshold_mag  = n["perturb_threshold_mag"].as<double>();
    if (n["perturb_weight_mag"])     p.perturb_weight_mag     = n["perturb_weight_mag"].as<double>();
    if (n["bar_length_min"])         p.bar_length_min         = n["bar_length_min"].as<double>();
    if (n["bar_length_max"])         p.bar_length_max         = n["bar_length_max"].as<double>();
    if (n["threshold_min"])          p.threshold_min          = n["threshold_min"].as<double>();
    if (n["threshold_max"])          p.threshold_max          = n["threshold_max"].as<double>();
    if (n["perturb_bar_grow_only_below"]) p.perturb_bar_grow_only_below = n["perturb_bar_grow_only_below"].as<double>();
    if (n["split_bar_min_length"])   p.split_bar_min_length   = n["split_bar_min_length"].as<double>();
    if (n["split_vertex_offset"])    p.split_vertex_offset    = n["split_vertex_offset"].as<double>();
    if (n["actuator_range_max"])     p.actuator_range_max     = n["actuator_range_max"].as<double>();
    if (n["new_synapse_weight_min"]) p.new_synapse_weight_min = n["new_synapse_weight_min"].as<double>();
    if (n["new_synapse_weight_max"]) p.new_synapse_weight_max = n["new_synapse_weight_max"].as<double>();
    if (n["new_bar_stiffness_min"])  p.new_bar_stiffness_min  = n["new_bar_stiffness_min"].as<double>();
    if (n["new_bar_stiffness_max"])  p.new_bar_stiffness_max  = n["new_bar_stiffness_max"].as<double>();
    return p;
}

void MutatorParams::toYAML(YAML::Emitter& out) const
{
    out << YAML::Key << "p_perturb"             << YAML::Value << p_perturb;
    out << YAML::Key << "p_add_bar_new"         << YAML::Value << p_add_bar_new;
    out << YAML::Key << "p_add_bar_bridge"      << YAML::Value << p_add_bar_bridge;
    out << YAML::Key << "p_add_neuron"          << YAML::Value << p_add_neuron;
    out << YAML::Key << "p_remove_bar"          << YAML::Value << p_remove_bar;
    out << YAML::Key << "p_remove_neuron"       << YAML::Value << p_remove_neuron;
    out << YAML::Key << "p_remove_bar_edge"     << YAML::Value << p_remove_bar_edge;
    out << YAML::Key << "p_remove_bar_bridge"   << YAML::Value << p_remove_bar_bridge;
    out << YAML::Key << "p_join_vertex"         << YAML::Value << p_join_vertex;
    out << YAML::Key << "p_split_vertex"        << YAML::Value << p_split_vertex;
    out << YAML::Key << "p_attach_neuron"       << YAML::Value << p_attach_neuron;
    out << YAML::Key << "p_detach_neuron"       << YAML::Value << p_detach_neuron;
    out << YAML::Key << "p_rewire"              << YAML::Value << p_rewire;
    out << YAML::Key << "num_method_retries"    << YAML::Value << num_method_retries;
    out << YAML::Key << "perturb_bar_frac"      << YAML::Value << perturb_bar_frac;
    out << YAML::Key << "perturb_threshold_mag" << YAML::Value << perturb_threshold_mag;
    out << YAML::Key << "perturb_weight_mag"    << YAML::Value << perturb_weight_mag;
    out << YAML::Key << "bar_length_min"        << YAML::Value << bar_length_min;
    out << YAML::Key << "bar_length_max"        << YAML::Value << bar_length_max;
    out << YAML::Key << "threshold_min"         << YAML::Value << threshold_min;
    out << YAML::Key << "threshold_max"         << YAML::Value << threshold_max;
    out << YAML::Key << "perturb_bar_grow_only_below" << YAML::Value << perturb_bar_grow_only_below;
    out << YAML::Key << "split_bar_min_length"  << YAML::Value << split_bar_min_length;
    out << YAML::Key << "split_vertex_offset"   << YAML::Value << split_vertex_offset;
    out << YAML::Key << "actuator_range_max"    << YAML::Value << actuator_range_max;
    out << YAML::Key << "new_synapse_weight_min"<< YAML::Value << new_synapse_weight_min;
    out << YAML::Key << "new_synapse_weight_max"<< YAML::Value << new_synapse_weight_max;
    out << YAML::Key << "new_bar_stiffness_min" << YAML::Value << new_bar_stiffness_min;
    out << YAML::Key << "new_bar_stiffness_max" << YAML::Value << new_bar_stiffness_max;
}

// ── MutationRecord::describe() ───────────────────────────────────────────────

std::string MutationRecord::describe() const
{
    // Short-circuit: entire mutation was aborted (retries exhausted).
    if (was_cloned) {
        std::string topo;
        topo += std::to_string(v_after) + "v ";
        topo += std::to_string(b_after) + "b ";
        topo += std::to_string(n_after) + "n ";
        topo += std::to_string(a_after) + "a";
        return "clone [retries exhausted]  \u2192  " + topo;
    }

    std::string s;
    auto add_op = [&](const char* name, bool fired, const std::string& detail) {
        if (!fired) return;
        if (!s.empty()) s += "  ";
        s += name;
        if (!detail.empty()) { s += '('; s += detail; s += ')'; }
    };
    add_op("perturb",        perturb,        perturb_detail);
    add_op("add_bar_new",    add_bar_new,    add_bar_new_detail);
    add_op("add_bar_bridge", add_bar_bridge, add_bar_bridge_detail);
    add_op("add_neuron",     add_neuron,     add_neuron_detail);
    add_op("remove_bar",     remove_bar,     remove_bar_detail);
    add_op("remove_neuron",  remove_neuron,  remove_neuron_detail);
    add_op("remove_bar_edge",   remove_bar_edge,   remove_bar_edge_detail);
    add_op("remove_bar_bridge", remove_bar_bridge, remove_bar_bridge_detail);
    add_op("join_vertex",    join_vertex,    join_vertex_detail);
    add_op("split_vertex",   split_vertex,   split_vertex_detail);
    add_op("attach_neuron",  attach_neuron,  attach_neuron_detail);
    add_op("detach_neuron",  detach_neuron,  detach_neuron_detail);
    add_op("rewire",         rewire,         rewire_detail);
    if (s.empty()) s = "none";
    if (stoch_rounds > 1) {
        s += "  [";
        s += std::to_string(stoch_rounds);
        s += " rounds]";
    }
    if (rerolls > 0) {
        s += "  (";
        s += std::to_string(rerolls);
        s += rerolls == 1 ? " reroll)" : " rerolls)";
    }
    s += "  \u2192  ";
    s += std::to_string(v_after) + "v ";
    s += std::to_string(b_after) + "b ";
    s += std::to_string(n_after) + "n ";
    s += std::to_string(a_after) + "a";
    return s;
}

// ── mutate() ─────────────────────────────────────────────────────────────────
//
// Each stochastically-fired operator is retried up to params.num_method_retries (method retry limit) times.
// If all attempts fail the entire mutation is aborted: robot is restored to the
// pre-call snapshot (returned as an unchanged clone from the parent).
// The force path (no op fired naturally) uses a single-attempt approach and
// falls back to infallible bootstrap ops — it cannot abort.

void Mutator::mutate(Robot& robot, std::mt19937& rng, const MutatorParams& params)
{
    const Robot snapshot = robot;
    bool any_fired = false;

    // Run fn up to num_retries times if the probability fires.
    // Returns false only when retries are exhausted (robot restored, caller returns).
    auto try_op = [&](double p, auto fn) -> bool {
        if (uniform01(rng) >= p) return true;           // didn't fire — fine
        for (int i = 0; i < params.num_method_retries; ++i)
            if (fn()) { any_fired = true; return true; }
        robot = snapshot;
        return false;                                    // signal: abort
    };

    while (!any_fired) {
        if (!try_op(params.p_perturb,        [&]{ return perturbElement(robot, rng, params); })) return;
        if (!try_op(params.p_add_bar_new,    [&]{ return addBarNew(robot, rng, params); }))      return;
        if (!try_op(params.p_add_bar_bridge, [&]{ return addBarBridge(robot, rng, params); }))   return;
        if (!try_op(params.p_add_neuron,     [&]{ addNeuron(robot, rng, params); return true; })) return;
        if (!try_op(params.p_remove_bar,       [&]{ return removeBar(robot, rng, params); }))      return;
        if (!try_op(params.p_remove_neuron,    [&]{ return removeNeuron(robot, rng, params); }))   return;
        if (!try_op(params.p_remove_bar_edge,  [&]{ return removeBarEdge(robot, rng, params); }))  return;
        if (!try_op(params.p_remove_bar_bridge,[&]{ return removeBarBridge(robot, rng, params); })) return;
        if (!try_op(params.p_join_vertex,      [&]{ return joinElement(robot, rng, params); }))     return;
        if (!try_op(params.p_split_vertex,     [&]{ return splitElement(robot, rng, params); }))   return;
        if (!try_op(params.p_attach_neuron,    [&]{ return attachNeuron(robot, rng, params); }))   return;
        if (!try_op(params.p_detach_neuron,    [&]{ return detachNeuron(robot, rng, params); }))   return;
        if (!try_op(params.p_rewire,         [&]{ return rewireNeuron(robot, rng, params); }))   return;
    }
}

// ── mutateRecord() ────────────────────────────────────────────────────────────
//
// Mirrors mutate() with full per-operator recording.  Retry-to-clone semantics
// match mutate(): exhausted retries set was_cloned = true and restore robot.

MutationRecord Mutator::mutateRecord(Robot& robot, std::mt19937& rng,
                                     const MutatorParams& params)
{
    MutationRecord rec;
    const Robot snapshot = robot;

    // Snapshot topology counts: {vertices, bars, neurons, actuators}.
    auto snap = [&]() -> std::array<int, 4> {
        return { (int)robot.vertices.size(), (int)robot.bars.size(),
                 (int)robot.neurons.size(),  (int)robot.actuators.size() };
    };

    // Build a delta string from two snapshots, e.g. "+1b -1n".
    auto delta_str = [](const std::array<int,4>& before,
                        const std::array<int,4>& after) -> std::string {
        const char* names[] = {"v","b","n","a"};
        std::string s;
        for (int i = 0; i < 4; ++i) {
            int d = after[i] - before[i];
            if (d != 0) {
                if (!s.empty()) s += ' ';
                if (d > 0) s += '+';
                s += std::to_string(d);
                s += names[i];
            }
        }
        return s.empty() ? "no-change" : s;
    };

    // Abort helper: restore robot, stamp was_cloned, fill final topology, return true.
    auto abort_clone = [&](const char* /*op_name*/) -> bool {
        robot = snapshot;
        rec = MutationRecord{};
        rec.was_cloned = true;
        rec.v_after = (int)robot.vertices.size();
        rec.b_after = (int)robot.bars.size();
        rec.n_after = (int)robot.neurons.size();
        rec.a_after = (int)robot.actuators.size();
        return true;   // "should abort"
    };

    // For perturb: describe what structural elements exist to be touched.
    auto perturb_what = [&]() -> std::string {
        std::string s;
        if (!robot.bars.empty())    s += "bar-len";
        if (!robot.neurons.empty()) { if (!s.empty()) s += ' '; s += "neuron"; }
        return s.empty() ? "nothing" : s;
    };

    bool any_fired = false;
    int  stoch_rounds = 0;

    // ── stochastic phase (loop until something fires) ─────────────────────────

    while (!any_fired) {
    ++stoch_rounds;

    if (uniform01(rng) < params.p_perturb) {
        const std::string what = perturb_what();
        bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = perturbElement(robot, rng, params))) break;
        if (!ok) { if (abort_clone("perturbElement")) return rec; }
        else { rec.perturb = true; rec.perturb_detail = what; any_fired = true; }
    }

    if (uniform01(rng) < params.p_add_bar_new) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = addBarNew(robot, rng, params))) break;
        if (!ok) { if (abort_clone("addBarNew")) return rec; }
        else { rec.add_bar_new = true; rec.add_bar_new_detail = delta_str(before, snap()); any_fired = true; }
    }

    if (uniform01(rng) < params.p_add_bar_bridge) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = addBarBridge(robot, rng, params))) break;
        if (!ok) { if (abort_clone("addBarBridge")) return rec; }
        else { rec.add_bar_bridge = true; rec.add_bar_bridge_detail = delta_str(before, snap()); any_fired = true; }
    }

    if (uniform01(rng) < params.p_add_neuron) {
        auto before = snap();
        addNeuron(robot, rng, params);   // always succeeds
        rec.add_neuron = true; rec.add_neuron_detail = delta_str(before, snap()); any_fired = true;
    }

    if (uniform01(rng) < params.p_remove_bar) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = removeBar(robot, rng, params))) break;
        if (!ok) { if (abort_clone("removeBar")) return rec; }
        else { rec.remove_bar = true; rec.remove_bar_detail = delta_str(before, snap()); any_fired = true; }
    }

    if (uniform01(rng) < params.p_remove_neuron) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = removeNeuron(robot, rng, params))) break;
        if (!ok) { if (abort_clone("removeNeuron")) return rec; }
        else { rec.remove_neuron = true; rec.remove_neuron_detail = delta_str(before, snap()); any_fired = true; }
    }

    if (uniform01(rng) < params.p_remove_bar_edge) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = removeBarEdge(robot, rng, params))) break;
        if (!ok) { if (abort_clone("removeBarEdge")) return rec; }
        else { rec.remove_bar_edge = true; rec.remove_bar_edge_detail = delta_str(before, snap()); any_fired = true; }
    }

    if (uniform01(rng) < params.p_remove_bar_bridge) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = removeBarBridge(robot, rng, params))) break;
        if (!ok) { if (abort_clone("removeBarBridge")) return rec; }
        else { rec.remove_bar_bridge = true; rec.remove_bar_bridge_detail = delta_str(before, snap()); any_fired = true; }
    }

    if (uniform01(rng) < params.p_join_vertex) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = joinElement(robot, rng, params))) break;
        if (!ok) { if (abort_clone("joinElement")) return rec; }
        else { rec.join_vertex = true; rec.join_vertex_detail = delta_str(before, snap()); any_fired = true; }
    }

    if (uniform01(rng) < params.p_split_vertex) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = splitElement(robot, rng, params))) break;
        if (!ok) { if (abort_clone("splitElement")) return rec; }
        else { rec.split_vertex = true; rec.split_vertex_detail = delta_str(before, snap()); any_fired = true; }
    }

    if (uniform01(rng) < params.p_attach_neuron) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = attachNeuron(robot, rng, params))) break;
        if (!ok) { if (abort_clone("attachNeuron")) return rec; }
        else { rec.attach_neuron = true; rec.attach_neuron_detail = delta_str(before, snap()); any_fired = true; }
    }

    if (uniform01(rng) < params.p_detach_neuron) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = detachNeuron(robot, rng, params))) break;
        if (!ok) { if (abort_clone("detachNeuron")) return rec; }
        else { rec.detach_neuron = true; rec.detach_neuron_detail = delta_str(before, snap()); any_fired = true; }
    }

    if (uniform01(rng) < params.p_rewire) {
        auto before = snap(); bool ok = false;
        for (int i = 0; i < params.num_method_retries; ++i)
            if ((ok = rewireNeuron(robot, rng, params))) break;
        if (!ok) { if (abort_clone("rewireNeuron")) return rec; }
        else { rec.rewire = true; rec.rewire_detail = delta_str(before, snap()); any_fired = true; }
    }

    } // end while (!any_fired)

    rec.stoch_rounds = stoch_rounds;

    // Record final topology.
    rec.v_after = (int)robot.vertices.size();
    rec.b_after = (int)robot.bars.size();
    rec.n_after = (int)robot.neurons.size();
    rec.a_after = (int)robot.actuators.size();

    return rec;
}

// ── perturbElement() ─────────────────────────────────────────────────────────

bool Mutator::perturbElement(Robot& robot, std::mt19937& rng, const MutatorParams& params)
{
    bool fired = false;

    // ── Bars: nudge rest_length by ±perturb_bar_frac ─────────────────────────
    // Short bars (< perturb_bar_grow_only_below) always grow: multiplier is
    // drawn from U[1.0, 1+frac] instead of U[1-frac, 1+frac].
    if (!robot.bars.empty()) {
        const int b = uniformInt(rng, 0, static_cast<int>(robot.bars.size()) - 1);
        const double lo = (robot.bars[b].rest_length < params.perturb_bar_grow_only_below)
                          ? 1.0
                          : 1.0 - params.perturb_bar_frac;
        robot.bars[b].rest_length *= uniformReal(rng, lo, 1.0 + params.perturb_bar_frac);
        robot.bars[b].rest_length  = std::clamp(robot.bars[b].rest_length,
            params.bar_length_min, params.bar_length_max);
        fired = true;
    }

    // ── Neurons: perturb threshold OR a synapse weight ───────────────────────
    if (!robot.neurons.empty()) {
        const int n = uniformInt(rng, 0, static_cast<int>(robot.neurons.size()) - 1);

        const bool no_weights = (robot.neurons[n].synapse_weights.size() == 0);
        if (uniform01(rng) < 0.5 || no_weights) {
            robot.neurons[n].threshold += uniformReal(rng,
                -params.perturb_threshold_mag, params.perturb_threshold_mag);
            robot.neurons[n].threshold  = std::clamp(robot.neurons[n].threshold,
                params.threshold_min, params.threshold_max);
            fired = true;
        } else {
            const int s = uniformInt(rng, 0,
                static_cast<int>(robot.neurons[n].synapse_weights.size()) - 1);
            robot.neurons[n].synapse_weights[s] += uniformReal(rng,
                -params.perturb_weight_mag, params.perturb_weight_mag);
            fired = true;
        }
    }

    return fired;
}

// ── addConnectedNeuron() ──────────────────────────────────────────────────────
//
// Internal helper: adds a neuron guaranteed to be connected to the network.
// The new neuron receives a random nonzero synapse weight from an existing
// neuron.  If there are no other neurons but bars exist, an actuator is also
// added to give it a physical role.  Always succeeds.

void Mutator::addConnectedNeuron(Robot& robot, std::mt19937& rng, const MutatorParams& params)
{
    const int N = static_cast<int>(robot.neurons.size());

    // Build the new neuron's input weight vector: all inputs randomised,
    // matching RobotFactory::randomRobot which sets every weight randomly.
    // Previously only ONE weight was non-zero here, plus addNeuron() always
    // zeros the new-neuron column in all existing neurons — leaving most of
    // the connectivity dead and unrecoverable except by slow perturbation.
    Eigen::VectorXd w = Eigen::VectorXd::Zero(N);
    for (int j = 0; j < N; ++j)
        w[j] = uniformReal(rng, params.new_synapse_weight_min, params.new_synapse_weight_max);

    Neuron new_neuron(uniformReal(rng, 0.0, 1.0), w);
    // Seed activation randomly (0 or 1) so the network can fire immediately,
    // matching RobotFactory::randomRobot behaviour.  All-zero activations mean
    // weighted_sum == 0 on every tick, so a network born with threshold > 0
    // would never fire at all without this kickstart.
    new_neuron.activation = (uniform01(rng) < 0.5) ? 1.0 : 0.0;
    const int new_n = robot.addNeuron(std::move(new_neuron));

    // addNeuron() always zeros the new column in every existing neuron
    // (existing[i].synapse_weights[new_n] = 0).  Randomise those now so
    // existing neurons can also receive signal from the new one — again
    // matching RobotFactory's fully-connected random initialisation.
    for (int i = 0; i < N; ++i)
        robot.neurons[i].synapse_weights[new_n] =
            uniformReal(rng, params.new_synapse_weight_min, params.new_synapse_weight_max);

    // If this is the only neuron and bars exist, wire an actuator so it has
    // physical influence (otherwise it would be permanently isolated).
    if (N == 0 && !robot.bars.empty()) {
        const int bi = uniformInt(rng, 0, static_cast<int>(robot.bars.size()) - 1);
        robot.addActuator(Actuator(bi, new_n, uniformReal(rng, 0.0, params.actuator_range_max)));
    }
}

// ── addBarNew() ──────────────────────────────────────────────────────────────
//
// Strategy A: dangle a new vertex off an existing vertex at median-bar-length.
// Bootstraps from scratch if no vertices exist.  Always returns true.

bool Mutator::addBarNew(Robot& robot, std::mt19937& rng, const MutatorParams& params)
{
    const int nv = static_cast<int>(robot.vertices.size());
    const double len = std::clamp(medianBarLength(robot),
        params.bar_length_min, params.bar_length_max);

    if (nv == 0) {
        // Bootstrap: create first two vertices from scratch above the floor.
        robot.addVertex(Vertex(0.0, 0.0, kFloorZ));
        const Eigen::Vector3d dir = randomDirection(rng);
        Eigen::Vector3d p1 = dir * len;
        p1.z() = std::max(p1.z(), kFloorZ);
        robot.addVertex(Vertex(p1));
        const double k0 = uniformReal(rng, params.new_bar_stiffness_min, params.new_bar_stiffness_max);
        robot.addBar(Bar(0, 1, len, k0));
        return true;
    }
    const int v = uniformInt(rng, 0, nv - 1);
    Eigen::Vector3d new_pos = robot.vertices[v].pos + randomDirection(rng) * len;
    new_pos.z() = std::max(new_pos.z(), kFloorZ);
    const int v_new = robot.addVertex(Vertex(new_pos));
    const double k_a = uniformReal(rng, params.new_bar_stiffness_min, params.new_bar_stiffness_max);
    robot.addBar(Bar(v, v_new, len, k_a));
    return true;
}

// ── addBarBridge() ────────────────────────────────────────────────────────────
//
// Strategy B: connect two existing vertices that share no bar, using their
// Euclidean distance as rest_length.  Returns false if fully connected.

bool Mutator::addBarBridge(Robot& robot, std::mt19937& rng, const MutatorParams& params)
{
    const int nv = static_cast<int>(robot.vertices.size());
    if (nv < 2) return false;

    std::vector<std::pair<int,int>> candidates;
    for (int i = 0; i < nv; ++i)
        for (int j = i + 1; j < nv; ++j)
            if (!barExists(robot, i, j))
                candidates.emplace_back(i, j);
    if (candidates.empty()) return false;

    const int ci = uniformInt(rng, 0, static_cast<int>(candidates.size()) - 1);
    const auto [v1, v2] = candidates[ci];
    const double len = (robot.vertices[v1].pos - robot.vertices[v2].pos).norm();
    if (len < 1e-6) return false;
    const double k_b = uniformReal(rng, params.new_bar_stiffness_min, params.new_bar_stiffness_max);
    robot.addBar(Bar(v1, v2, len, k_b));
    return true;
}

// ── addNeuron() ───────────────────────────────────────────────────────────────
//
// Public wrapper around addConnectedNeuron.

void Mutator::addNeuron(Robot& robot, std::mt19937& rng, const MutatorParams& params)
{
    addConnectedNeuron(robot, rng, params);
}

// ── removeBar() ───────────────────────────────────────────────────────────────
//
// Connectivity-guarded bar removal.  Picks a random bar, removes it, prunes
// isolated vertices, then checks connectivity.  Restores and returns false if
// the removal splits the graph or no bars exist.

bool Mutator::removeBar(Robot& robot, std::mt19937& rng, const MutatorParams& /*params*/)
{
    if (robot.bars.empty()) return false;
    Robot saved = robot;
    robot.removeBar(uniformInt(rng, 0, static_cast<int>(robot.bars.size()) - 1));
    robot.pruneIsolatedVertices();
    if (!robot.isConnected()) {
        robot = std::move(saved);
        return false;
    }
    return true;
}

// ── removeNeuron() ────────────────────────────────────────────────────────────
//
// Removes a randomly chosen neuron.  Returns false if no neurons exist.

bool Mutator::removeNeuron(Robot& robot, std::mt19937& rng, const MutatorParams& /*params*/)
{
    if (robot.neurons.empty()) return false;
    robot.removeNeuron(uniformInt(rng, 0,
        static_cast<int>(robot.neurons.size()) - 1));
    return true;
}

// ── removeBarEdge() ───────────────────────────────────────────────────────────
//
// Reverse of addBarNew (Strategy A).
// Finds all leaf bars — bars where at least one endpoint has degree == 1.
// Picks one at random, removes it, then prunes the now-isolated leaf vertex.
// A leaf edge can never be a structural bridge, so connectivity is guaranteed.
// Returns false when no leaf bars exist (all vertices have degree >= 2).

bool Mutator::removeBarEdge(Robot& robot, std::mt19937& rng, const MutatorParams& /*params*/)
{
    // A single-bar robot would become empty after removal — degenerate.
    if (robot.bars.size() <= 1) return false;

    // Compute per-vertex degree.
    std::vector<int> degree(robot.vertices.size(), 0);
    for (const auto& b : robot.bars) {
        ++degree[b.v1];
        ++degree[b.v2];
    }

    // Collect bars with at least one degree-1 endpoint.
    std::vector<int> candidates;
    for (int i = 0; i < static_cast<int>(robot.bars.size()); ++i) {
        const auto& b = robot.bars[i];
        if (degree[b.v1] == 1 || degree[b.v2] == 1)
            candidates.push_back(i);
    }
    if (candidates.empty()) return false;

    const int bar_idx = candidates[
        uniformInt(rng, 0, static_cast<int>(candidates.size()) - 1)];
    robot.removeBar(bar_idx);
    robot.pruneIsolatedVertices();   // removes the now-isolated leaf vertex
    return true;
}

// ── removeBarBridge() ─────────────────────────────────────────────────────────
//
// Reverse of addBarBridge (Strategy B).
// Finds bars that are (a) not a graph-theory bridge (removal doesn't disconnect)
// and (b) have both endpoints at degree >= 2 (neither becomes isolated).
// Picks one at random and removes it; no vertex is removed.
// "Bridge" here matches addBarBridge's everyday sense: a bar creating a cycle.
// Returns false when no such bar exists.

bool Mutator::removeBarBridge(Robot& robot, std::mt19937& rng, const MutatorParams& /*params*/)
{
    if (robot.bars.empty()) return false;

    std::vector<int> degree(robot.vertices.size(), 0);
    for (const auto& b : robot.bars) {
        ++degree[b.v1];
        ++degree[b.v2];
    }

    std::vector<int> candidates;
    for (int i = 0; i < static_cast<int>(robot.bars.size()); ++i) {
        const auto& b = robot.bars[i];
        // Both endpoints must retain degree >= 1 after removal.
        if (degree[b.v1] < 2 || degree[b.v2] < 2) continue;
        // Must not be a structural bridge (removal must leave graph connected).
        Robot test = robot;
        test.removeBar(i);
        if (test.isConnected())
            candidates.push_back(i);
    }
    if (candidates.empty()) return false;

    const int bar_idx = candidates[
        uniformInt(rng, 0, static_cast<int>(candidates.size()) - 1)];
    robot.removeBar(bar_idx);
    return true;
}

// ── joinElement() ────────────────────────────────────────────────────────────
//
// Reverse of splitElement (bar-bisect mode).
// Finds internal degree-2 vertices w where:
//   - both neighbours u, v have degree >= 2 (interior-only rule)
//   - u and v are not already directly connected
//   - the Euclidean distance |p_u - p_v| >= bar_length_min
// Picks one at random, removes w and its two half-bars, inserts bar(u,v) with
//   rest_length = |p_u - p_v|  (geometry-driven, not sum of halves)
//   stiffness   = (k_b1 + k_b2) / 2
// Actuators from both half-bars are migrated to the new bar.
// Returns false when no eligible vertex exists.

bool Mutator::joinElement(Robot& robot, std::mt19937& rng, const MutatorParams& params)
{
    if (robot.bars.size() < 2) return false;

    const int nv = static_cast<int>(robot.vertices.size());

    // Compute per-vertex degree.
    std::vector<int> degree(nv, 0);
    for (const auto& b : robot.bars) { ++degree[b.v1]; ++degree[b.v2]; }

    // Returns true when a bar directly connects vertices a and b.
    auto barExists = [&](int a, int b_v) -> bool {
        for (const auto& b : robot.bars)
            if ((b.v1 == a && b.v2 == b_v) || (b.v1 == b_v && b.v2 == a))
                return true;
        return false;
    };

    struct Candidate {
        int w;              // vertex to collapse
        int u, v;           // its two neighbours
        int b1_idx, b2_idx; // indices of the two incident bars
    };
    std::vector<Candidate> candidates;

    for (int w = 0; w < nv; ++w) {
        if (degree[w] != 2) continue;

        // Identify the two incident bars and their far endpoints.
        int b1_idx = -1, b2_idx = -1, u = -1, v_vert = -1;
        for (int i = 0; i < static_cast<int>(robot.bars.size()); ++i) {
            const auto& b = robot.bars[i];
            if (b.v1 == w || b.v2 == w) {
                const int other = (b.v1 == w) ? b.v2 : b.v1;
                if (b1_idx == -1) { b1_idx = i; u        = other; }
                else              { b2_idx = i; v_vert   = other; }
            }
        }
        if (b1_idx < 0 || b2_idx < 0) continue;

        // Interior-only: both neighbours must have degree >= 2.
        if (degree[u] < 2 || degree[v_vert] < 2) continue;

        // Reject if u and v are already connected.
        if (barExists(u, v_vert)) continue;

        // Reject if the merged bar would be too short.
        const double len =
            (robot.vertices[u].pos - robot.vertices[v_vert].pos).norm();
        if (len < params.bar_length_min) continue;

        candidates.push_back({w, u, v_vert, b1_idx, b2_idx});
    }
    if (candidates.empty()) return false;

    const Candidate c =
        candidates[uniformInt(rng, 0, static_cast<int>(candidates.size()) - 1)];

    // Save everything we need before any structural modification.
    const Eigen::Vector3d p_u   = robot.vertices[c.u].pos;
    const Eigen::Vector3d p_v   = robot.vertices[c.v].pos;
    const double k1             = robot.bars[c.b1_idx].stiffness;
    const double k2             = robot.bars[c.b2_idx].stiffness;
    const double new_len        = (p_u - p_v).norm();
    const double new_k          = (k1 + k2) * 0.5;

    // Save actuators from both half-bars (bar_idx is about to be invalidated).
    struct SavedAct { int neuron_idx; double bar_range; };
    std::vector<SavedAct> saved_acts;
    for (const auto& a : robot.actuators)
        if (a.bar_idx == c.b1_idx || a.bar_idx == c.b2_idx)
            saved_acts.push_back({a.neuron_idx, a.bar_range});

    // Remove bars higher-index first so the lower index stays stable.
    const int hi = std::max(c.b1_idx, c.b2_idx);
    const int lo = std::min(c.b1_idx, c.b2_idx);
    robot.removeBar(hi);
    robot.removeBar(lo);

    // w is now isolated; prune it.  All vertices with original index > w
    // shift down by 1 — remap u and v accordingly.
    robot.pruneIsolatedVertices();
    int u_ = c.u;     if (u_ > c.w) --u_;
    int v_ = c.v;     if (v_ > c.w) --v_;

    // Add the merged bar and re-attach saved actuators.
    // If both half-bars were actuated keep only the first (one-per-bar invariant).
    const int new_bar_idx = static_cast<int>(robot.bars.size());
    robot.addBar(Bar(u_, v_, new_len, new_k));
    if (!saved_acts.empty())
        robot.addActuator(Actuator(new_bar_idx, saved_acts[0].neuron_idx, saved_acts[0].bar_range));

    return true;
}

// ── splitElement() ───────────────────────────────────────────────────────────

bool Mutator::splitElement(Robot& robot, std::mt19937& rng, const MutatorParams& params)
{
    const bool has_vertex = !robot.vertices.empty();
    const bool has_bar    = !robot.bars.empty();
    if (!has_vertex && !has_bar) return false;

    // Exclude bars that are too short to bisect; if none remain, force vertex-split.
    std::vector<int> splittable_bars;
    for (int i = 0; i < static_cast<int>(robot.bars.size()); ++i)
        if (robot.bars[i].rest_length >= params.split_bar_min_length)
            splittable_bars.push_back(i);
    const bool has_splittable_bar = !splittable_bars.empty();

    // Nothing useful to do if no vertices and all bars are too short.
    if (!has_vertex && !has_splittable_bar) return false;

    bool split_vertex;
    if (has_vertex && has_splittable_bar)
        split_vertex = (uniform01(rng) < 0.5);
    else
        split_vertex = has_vertex;  // forced vertex-split (or false if no vertices)

    if (split_vertex) {
        const int v = uniformInt(rng, 0,
            static_cast<int>(robot.vertices.size()) - 1);
        const double o = params.split_vertex_offset;
        const Eigen::Vector3d offset(
            uniformReal(rng, -o, o),
            uniformReal(rng, -o, o),
            uniformReal(rng, -o, o));
        Eigen::Vector3d new_pos = robot.vertices[v].pos + offset;
        new_pos.z() = std::max(new_pos.z(), kFloorZ);
        const int v_new = robot.addVertex(Vertex(new_pos));
        const double len = (offset.norm() < 1e-6) ? 0.01 : offset.norm();
        const double k_s = uniformReal(rng, params.new_bar_stiffness_min, params.new_bar_stiffness_max);
        robot.addBar(Bar(v, v_new, len, k_s));
        return true;
    }
    else {
        const int b = splittable_bars[
            uniformInt(rng, 0, static_cast<int>(splittable_bars.size()) - 1)];

        const Bar    B    = robot.bars[b];
        const int    v1   = B.v1;
        const int    v2   = B.v2;
        const double half = B.rest_length * 0.5;
        const double k    = B.stiffness;

        const Eigen::Vector3d mid =
            (robot.vertices[v1].pos + robot.vertices[v2].pos) * 0.5;
        Eigen::Vector3d safe_mid = mid;
        safe_mid.z() = std::max(safe_mid.z(), kFloorZ);
        const int v_mid = robot.addVertex(Vertex(safe_mid));

        std::vector<std::pair<int, double>> saved;
        for (const Actuator& a : robot.actuators)
            if (a.bar_idx == b)
                saved.emplace_back(a.neuron_idx, a.bar_range);

        robot.removeBar(b);

        const int b1 = robot.addBar(Bar(v1, v_mid, half, k));
        /* b2 = */     robot.addBar(Bar(v_mid, v2, half, k));

        for (auto& [n_idx, br] : saved)
            robot.addActuator(Actuator(b1, n_idx, br));

        return true;
    }
}

// ── attachDetach() ───────────────────────────────────────────────────────────
//
// Picks a random spring element — which may currently be a structural Bar or an
// actuated Bar (Actuator) — and toggles its neural connection:
//
//   structural bar  → actuated (Actuator):  add Actuator(bar, random_neuron, random_range)
//   actuated bar    → structural (Bar):     remove ALL Actuators targeting that bar
//
// NOTE: this is the canonical place where Bar ↔ Actuator conversion happens.
// The candidate pool is robot.bars, which holds ALL spring elements regardless
// of whether they are currently actuated.  Whether a bar_idx appears in
// robot.actuators determines its "actuated" status.  This whole toggle dance
// is a direct consequence of the awkward Bar/Actuator split — see Bar.h for
// the TODO on collapsing the two classes.

bool Mutator::attachNeuron(Robot& robot, std::mt19937& rng, const MutatorParams& params)
{
    if (robot.bars.empty()) return false;

    // Collect bars that currently have no actuators.
    std::vector<int> unactuated;
    for (int i = 0; i < static_cast<int>(robot.bars.size()); ++i) {
        bool has_act = false;
        for (const auto& a : robot.actuators)
            if (a.bar_idx == i) { has_act = true; break; }
        if (!has_act) unactuated.push_back(i);
    }
    if (unactuated.empty()) return false;  // all bars already actuated

    const int bar_idx = unactuated[uniformInt(rng, 0, (int)unactuated.size() - 1)];

    // Wire to a random neuron, creating one if none exist.
    if (robot.neurons.empty())
        addConnectedNeuron(robot, rng, params);
    const int neuron_idx = uniformInt(rng, 0,
        static_cast<int>(robot.neurons.size()) - 1);
    robot.addActuator(Actuator(bar_idx, neuron_idx,
                               uniformReal(rng, 0.0, params.actuator_range_max)));
    return true;
}

// ── detachNeuron() ───────────────────────────────────────────────────────────
//
// Picks a bar that has at least one actuator and removes all actuators on it.
// Returns false if no bars are currently actuated.

bool Mutator::detachNeuron(Robot& robot, std::mt19937& rng, const MutatorParams& /*params*/)
{
    if (robot.actuators.empty()) return false;

    // Collect bars that have at least one actuator.
    std::vector<int> actuated;
    for (const auto& a : robot.actuators) {
        if (std::find(actuated.begin(), actuated.end(), a.bar_idx) == actuated.end())
            actuated.push_back(a.bar_idx);
    }

    const int bar_idx = actuated[uniformInt(rng, 0, (int)actuated.size() - 1)];

    // Remove all actuators targeting this bar (iterate in reverse to keep indices stable).
    std::vector<int> to_remove;
    for (int i = 0; i < static_cast<int>(robot.actuators.size()); ++i)
        if (robot.actuators[i].bar_idx == bar_idx)
            to_remove.push_back(i);
    for (int i = static_cast<int>(to_remove.size()) - 1; i >= 0; --i)
        robot.removeActuator(to_remove[i]);
    return true;
}

// ── rewireNeuron() ───────────────────────────────────────────────────────────
//
// Topological rewiring: pick a random actuator and reassign either its
// bar_idx (physical target) or its neuron_idx (neural source) to a new,
// different index.  50/50 which end gets rewired.

bool Mutator::rewireNeuron(Robot& robot, std::mt19937& rng, const MutatorParams& /*params*/)
{
    if (robot.actuators.empty()) return false;

    const int a_idx = uniformInt(rng, 0,
        static_cast<int>(robot.actuators.size()) - 1);
    Actuator& a = robot.actuators[a_idx];

    const bool can_rewire_bar    = (static_cast<int>(robot.bars.size())    > 1);
    const bool can_rewire_neuron = (static_cast<int>(robot.neurons.size()) > 1);
    if (!can_rewire_bar && !can_rewire_neuron) return false;

    bool rewire_bar;
    if (can_rewire_bar && can_rewire_neuron)
        rewire_bar = (uniform01(rng) < 0.5);
    else
        rewire_bar = can_rewire_bar;

    if (rewire_bar) {
        int new_bar;
        do { new_bar = uniformInt(rng, 0,
                 static_cast<int>(robot.bars.size()) - 1); }
        while (new_bar == a.bar_idx);
        a.bar_idx = new_bar;
    } else {
        int new_neuron;
        do { new_neuron = uniformInt(rng, 0,
                 static_cast<int>(robot.neurons.size()) - 1); }
        while (new_neuron == a.neuron_idx);
        a.neuron_idx = new_neuron;
    }
    return true;
}
