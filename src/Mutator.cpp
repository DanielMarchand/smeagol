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

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
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

// ── mutate() ─────────────────────────────────────────────────────────────────

void Mutator::mutate(Robot& robot, std::mt19937& rng)
{
    constexpr double P_PERTURB    = 0.10;
    constexpr double P_ADD_REMOVE = 0.01;
    constexpr double P_SPLIT      = 0.03;
    constexpr double P_ATTACH     = 0.03;
    constexpr double P_REWIRE     = 0.03;

    bool any_fired = false;

    if (uniform01(rng) < P_PERTURB)
        any_fired |= perturbElement(robot, rng);

    if (uniform01(rng) < P_ADD_REMOVE)
        any_fired |= addRemoveElement(robot, rng);

    if (uniform01(rng) < P_SPLIT)
        any_fired |= splitElement(robot, rng);

    if (uniform01(rng) < P_ATTACH)
        any_fired |= attachDetach(robot, rng);

    if (uniform01(rng) < P_REWIRE)
        any_fired |= rewireNeuron(robot, rng);

    if (!any_fired) {
        const int op = uniformInt(rng, 0, 4);
        switch (op) {
            case 0: any_fired = perturbElement(robot, rng);    break;
            case 1: any_fired = addRemoveElement(robot, rng);  break;
            case 2: any_fired = splitElement(robot, rng);      break;
            case 3: any_fired = attachDetach(robot, rng);      break;
            case 4: any_fired = rewireNeuron(robot, rng);      break;
        }
        if (!any_fired)
            addConnectedNeuron(robot, rng);   // always succeeds
    }
}

// ── perturbElement() ─────────────────────────────────────────────────────────

bool Mutator::perturbElement(Robot& robot, std::mt19937& rng)
{
    bool fired = false;

    // ── Bars: nudge rest_length by ±10% ─────────────────────────────────────
    if (!robot.bars.empty()) {
        const int b = uniformInt(rng, 0, static_cast<int>(robot.bars.size()) - 1);
        robot.bars[b].rest_length *= uniformReal(rng, 0.9, 1.1);
        robot.bars[b].rest_length  = std::clamp(robot.bars[b].rest_length, 0.01, 1.0);
        fired = true;
    }

    // ── Neurons: perturb threshold OR a synapse weight ───────────────────────
    if (!robot.neurons.empty()) {
        const int n = uniformInt(rng, 0, static_cast<int>(robot.neurons.size()) - 1);

        const bool no_weights = (robot.neurons[n].synapse_weights.size() == 0);
        if (uniform01(rng) < 0.5 || no_weights) {
            robot.neurons[n].threshold += uniformReal(rng, -0.5, 0.5);
            robot.neurons[n].threshold  = std::clamp(robot.neurons[n].threshold, 0.0, 2.0);
            fired = true;
        } else {
            const int s = uniformInt(rng, 0,
                static_cast<int>(robot.neurons[n].synapse_weights.size()) - 1);
            robot.neurons[n].synapse_weights[s] += uniformReal(rng, -0.5, 0.5);
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

void Mutator::addConnectedNeuron(Robot& robot, std::mt19937& rng)
{
    const int N = static_cast<int>(robot.neurons.size());

    Eigen::VectorXd w = Eigen::VectorXd::Zero(N);
    if (N > 0) {
        const int src = uniformInt(rng, 0, N - 1);
        w[src] = uniformReal(rng, 0.5, 1.5) * (uniform01(rng) < 0.5 ? 1.0 : -1.0);
    }

    const int new_n = robot.addNeuron(
        Neuron(uniformReal(rng, 0.0, 1.0), w));

    // If this is the only neuron and bars exist, wire an actuator so it has
    // physical influence (otherwise it would be permanently isolated).
    if (N == 0 && !robot.bars.empty()) {
        const int bi = uniformInt(rng, 0, static_cast<int>(robot.bars.size()) - 1);
        robot.addActuator(Actuator(bi, new_n, uniformReal(rng, 0.0, 0.01)));
    }
}

// ── addBarMutation() ──────────────────────────────────────────────────────────
//
// Two strategies chosen 50/50 when both are available:
//
//  A) New-vertex bar (always possible):
//     Pick an existing vertex (or create two from scratch if none exist).
//     Sample bar length from |N(median, median)| clamped to [0.01, 1.0].
//     Place a new vertex at a random unit direction × length, add the bar.
//
//  B) Existing-vertex bar:
//     Pick two distinct vertices with no existing bar between them.
//     Use their actual Euclidean distance as rest_length.
//     Enforces uniqueness: no duplicate vertex pairs.

bool Mutator::addBarMutation(Robot& robot, std::mt19937& rng)
{
    const int nv = static_cast<int>(robot.vertices.size());
    const double median_len = medianBarLength(robot);

    // Strategy A ─────────────────────────────────────────────────────────────
    auto strategyA = [&]() -> bool {
        const double len = std::clamp(median_len, 0.01, 1.0);
        if (nv == 0) {
            // Bootstrap: first two vertices from scratch, both above the floor
            robot.addVertex(Vertex(0.0, 0.0, kFloorZ));
            const Eigen::Vector3d dir = randomDirection(rng);
            Eigen::Vector3d p1 = dir * len;
            p1.z() = std::max(p1.z(), kFloorZ);
            robot.addVertex(Vertex(p1));
            robot.addBar(Bar(0, 1, len));
            return true;
        }
        const int v = uniformInt(rng, 0, nv - 1);
        Eigen::Vector3d new_pos =
            robot.vertices[v].pos + randomDirection(rng) * len;
        new_pos.z() = std::max(new_pos.z(), kFloorZ);
        const int v_new = robot.addVertex(Vertex(new_pos));
        robot.addBar(Bar(v, v_new, len));
        return true;
    };

    // Strategy B ─────────────────────────────────────────────────────────────
    auto strategyB = [&]() -> bool {
        if (nv < 2) return false;
        // Collect all unconnected vertex pairs
        std::vector<std::pair<int,int>> candidates;
        for (int i = 0; i < nv; ++i)
            for (int j = i + 1; j < nv; ++j)
                if (!barExists(robot, i, j))
                    candidates.emplace_back(i, j);
        if (candidates.empty()) return false;

        const int ci = uniformInt(rng, 0,
            static_cast<int>(candidates.size()) - 1);
        const auto [v1, v2] = candidates[ci];
        const double len =
            (robot.vertices[v1].pos - robot.vertices[v2].pos).norm();
        if (len < 1e-6) return false;
        robot.addBar(Bar(v1, v2, len));
        return true;
    };

    if (nv < 2) return strategyA();
    if (uniform01(rng) < 0.5) return strategyA();
    bool ok = strategyB();
    if (!ok) ok = strategyA();
    return ok;
}

// ── addRemoveElement() ───────────────────────────────────────────────────────

bool Mutator::addRemoveElement(Robot& robot, std::mt19937& rng)
{
    if (uniform01(rng) < 0.5) {
        // ADD
        if (uniform01(rng) < 0.5)
            return addBarMutation(robot, rng);
        addConnectedNeuron(robot, rng);
        return true;
    }
    else {
        // REMOVE
        const bool has_bar    = !robot.bars.empty();
        const bool has_neuron = !robot.neurons.empty();
        if (!has_bar && !has_neuron) return false;

        bool remove_bar;
        if (has_bar && has_neuron)
            remove_bar = (uniform01(rng) < 0.5);
        else
            remove_bar = has_bar;

        if (remove_bar) {
            robot.removeBar(uniformInt(rng, 0,
                static_cast<int>(robot.bars.size()) - 1));
        } else {
            robot.removeNeuron(uniformInt(rng, 0,
                static_cast<int>(robot.neurons.size()) - 1));
        }
        return true;
    }
}

// ── splitElement() ───────────────────────────────────────────────────────────

bool Mutator::splitElement(Robot& robot, std::mt19937& rng)
{
    const bool has_vertex = !robot.vertices.empty();
    const bool has_bar    = !robot.bars.empty();
    if (!has_vertex && !has_bar) return false;

    bool split_vertex;
    if (has_vertex && has_bar)
        split_vertex = (uniform01(rng) < 0.5);
    else
        split_vertex = has_vertex;

    if (split_vertex) {
        const int v = uniformInt(rng, 0,
            static_cast<int>(robot.vertices.size()) - 1);
        const Eigen::Vector3d offset(
            uniformReal(rng, -0.01, 0.01),
            uniformReal(rng, -0.01, 0.01),
            uniformReal(rng, -0.01, 0.01));
        Eigen::Vector3d new_pos = robot.vertices[v].pos + offset;
        new_pos.z() = std::max(new_pos.z(), kFloorZ);
        const int v_new = robot.addVertex(Vertex(new_pos));
        const double len = (offset.norm() < 1e-6) ? 0.01 : offset.norm();
        robot.addBar(Bar(v, v_new, len));
        return true;
    }
    else {
        const int b = uniformInt(rng, 0,
            static_cast<int>(robot.bars.size()) - 1);

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
// Flips a random bar between structural (no actuator) and actuated.
//   structural → actuated:  add Actuator(bar, random_neuron, random_range)
//   actuated   → structural: remove ALL actuators targeting that bar

bool Mutator::attachDetach(Robot& robot, std::mt19937& rng)
{
    if (robot.bars.empty()) return false;

    const int bar_idx = uniformInt(rng, 0,
        static_cast<int>(robot.bars.size()) - 1);

    // Is this bar currently actuated?
    bool is_actuated = false;
    for (const auto& a : robot.actuators)
        if (a.bar_idx == bar_idx) { is_actuated = true; break; }

    if (is_actuated) {
        // Detach: remove all actuators on this bar (reverse order)
        std::vector<int> to_remove;
        for (int i = 0; i < static_cast<int>(robot.actuators.size()); ++i)
            if (robot.actuators[i].bar_idx == bar_idx)
                to_remove.push_back(i);
        for (int i = static_cast<int>(to_remove.size()) - 1; i >= 0; --i)
            robot.removeActuator(to_remove[i]);
        return true;
    }
    else {
        // Attach: wire to a random neuron (create one if needed)
        if (robot.neurons.empty())
            addConnectedNeuron(robot, rng);
        const int neuron_idx = uniformInt(rng, 0,
            static_cast<int>(robot.neurons.size()) - 1);
        robot.addActuator(Actuator(bar_idx, neuron_idx,
                                   uniformReal(rng, 0.0, 0.01)));
        return true;
    }
}

// ── rewireNeuron() ───────────────────────────────────────────────────────────
//
// Topological rewiring: pick a random actuator and reassign either its
// bar_idx (physical target) or its neuron_idx (neural source) to a new,
// different index.  50/50 which end gets rewired.

bool Mutator::rewireNeuron(Robot& robot, std::mt19937& rng)
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
