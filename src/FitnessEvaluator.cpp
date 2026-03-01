#include "FitnessEvaluator.h"
#include "Simulator.h"

#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// helpers
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Compute the mass-weighted centre-of-mass XY position from the current
 * Simulator state.
 */
static Eigen::Vector2d com_xy(const Simulator& sim)
{
    const double total_mass = sim.vertex_masses.sum();
    if (total_mass <= 0.0)
        return Eigen::Vector2d::Zero();

    // positions is (N, 3); columns 0 and 1 are X and Y
    const double cx = (sim.vertex_masses.array() * sim.positions.col(0).array()).sum() / total_mass;
    const double cy = (sim.vertex_masses.array() * sim.positions.col(1).array()).sum() / total_mass;
    return {cx, cy};
}

// ─────────────────────────────────────────────────────────────────────────────
// FitnessEvaluator
// ─────────────────────────────────────────────────────────────────────────────

FitnessEvaluator::FitnessEvaluator(Params params)
    : params_(params)
{}

double FitnessEvaluator::evaluate(const Robot&                  robot,
                                  std::vector<Eigen::Vector2d>* trajectory) const
{
    Simulator sim(robot);
    sim.wind = params_.wind;

    const Eigen::Vector2d start = com_xy(sim);

    if (trajectory) {
        trajectory->clear();
        trajectory->reserve(params_.cycles + 1);
        trajectory->push_back(start);   // position at t=0
    }

    for (int c = 0; c < params_.cycles; ++c) {
        sim.tickNeural();
        sim.applyActuators(params_.steps_per_cycle);
        sim.relax(params_.steps_per_cycle, params_.step_size,
                  /*noise=*/0.0, /*tol=*/0.0);

        if (trajectory)
            trajectory->push_back(com_xy(sim));
    }

    return (com_xy(sim) - start).norm();
}
