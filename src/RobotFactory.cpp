#include "RobotFactory.h"
#include "Vertex.h"
#include "Bar.h"
#include "Neuron.h"
#include "Actuator.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// helpers
// ─────────────────────────────────────────────────────────────────────────────

namespace {

// Uniform real in [lo, hi]
double ureal(std::mt19937& rng, double lo, double hi)
{
    return std::uniform_real_distribution<double>(lo, hi)(rng);
}

// Uniform int in [lo, hi] inclusive
int uint_(std::mt19937& rng, int lo, int hi)
{
    return std::uniform_int_distribution<int>(lo, hi)(rng);
}

// Bernoulli trial
bool coin(std::mt19937& rng, double p)
{
    return std::bernoulli_distribution(p)(rng);
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// RobotFactory::randomRobot
// ─────────────────────────────────────────────────────────────────────────────

Robot RobotFactory::randomRobot(std::mt19937& rng, const Params& p)
{
    Robot robot;

    // ── 1. Vertices ───────────────────────────────────────────────────────
    const int nv = uint_(rng, p.min_vertices, p.max_vertices);

    for (int i = 0; i < nv; ++i) {
        robot.addVertex(Vertex(
            ureal(rng, -p.bbox_xy,    +p.bbox_xy),
            ureal(rng, -p.bbox_xy,    +p.bbox_xy),
            ureal(rng,  p.bbox_z_min,  p.bbox_z_max)
        ));
    }

    // ── 2. Bars — random spanning tree first ──────────────────────────────
    // For each vertex i (1..nv-1), connect to a random earlier vertex j.
    // This guarantees full connectivity with exactly nv-1 bars.

    auto bar_between = [&](int v1, int v2) {
        const double L = (robot.vertices[v1].pos -
                          robot.vertices[v2].pos).norm();
        const double k = ureal(rng, p.stiffness_min, p.stiffness_max);
        // Guard: degenerate bar (coincident vertices) — use minimum length
        robot.addBar(Bar(v1, v2, std::max(L, 0.001), k));
    };

    for (int i = 1; i < nv; ++i) {
        const int j = uint_(rng, 0, i - 1);
        bar_between(i, j);
    }

    // ── 3. Extra bars (optional density enrichment) ───────────────────────
    // Consider every remaining vertex pair; add with probability p_extra_bar.
    for (int i = 0; i < nv; ++i) {
        for (int j = i + 1; j < nv; ++j) {
            // Check if bar already exists in spanning tree
            bool exists = false;
            for (const auto& b : robot.bars) {
                if ((b.v1 == i && b.v2 == j) || (b.v1 == j && b.v2 == i)) {
                    exists = true;
                    break;
                }
            }
            if (!exists && coin(rng, p.p_extra_bar))
                bar_between(i, j);
        }
    }

    // ── 4. Neurons ────────────────────────────────────────────────────────
    const int nn = uint_(rng, p.min_neurons, p.max_neurons);

    for (int i = 0; i < nn; ++i) {
        Neuron n;
        n.threshold = ureal(rng, p.threshold_min, p.threshold_max);
        // synapse_weights resized to nn by addNeuron; set values afterward
        robot.addNeuron(n);
    }

    // Set random synapse weights now that all neurons exist
    for (int i = 0; i < nn; ++i) {
        for (int j = 0; j < nn; ++j) {
            robot.neurons[i].synapse_weights(j) =
                ureal(rng, -p.weight_range, +p.weight_range);
        }
        // Seed initial activation randomly (0 or 1)
        robot.neurons[i].activation = coin(rng, 0.5) ? 1.0 : 0.0;
    }

    // ── 5. Actuators ──────────────────────────────────────────────────────
    // Walk every bar; attach to a random neuron with probability p_actuate_bar.
    // Guarantee at least one actuator.
    const int nb = static_cast<int>(robot.bars.size());
    bool any_actuated = false;

    for (int b = 0; b < nb; ++b) {
        if (coin(rng, p.p_actuate_bar)) {
            const int   n_idx     = uint_(rng, 0, nn - 1);
            const double bar_range = ureal(rng, 0.0, p.bar_range_max);
            robot.addActuator(Actuator(b, n_idx, bar_range));
            any_actuated = true;
        }
    }

    // Force at least one actuator (pick bar 0, neuron 0 if needed)
    if (!any_actuated) {
        const double bar_range = ureal(rng, 0.0, p.bar_range_max);
        robot.addActuator(Actuator(0, 0, bar_range == 0.0 ? 0.005 : bar_range));
    }

    return robot;
}

Robot RobotFactory::randomRobot(std::mt19937& rng)
{
    return randomRobot(rng, Params{});
}
