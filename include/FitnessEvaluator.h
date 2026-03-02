#pragma once

#include "Robot.h"

#include <Eigen/Dense>
#include <vector>

/**
 * @brief Parameters controlling a FitnessEvaluator run.
 *
 * Kept outside FitnessEvaluator so it can be used as a default argument
 * in the constructor without hitting GCC's incomplete-class restriction.
 */
struct FitnessParams
{
    /// Number of neural tick + relax cycles to run.
    int    cycles          = 12;

    /// Gradient-descent steps inside each relax() call.
    int    steps_per_cycle = 5000;

    /// Gradient step size [m/N].  Must satisfy Ds < 1/(2·k_max).
    double step_size       = 1.0e-7;

    /// Optional wind acceleration [m/s²] in +X applied to all vertices.
    /// Set > 0 to verify the fitness evaluator without a locomotion controller.
    double wind            = 0.0;

    /// Static friction coefficient (Coulomb model).
    /// Lateral force must exceed mu_static × normal_force to slide a grounded vertex.
    double mu_static       = 0.5;
};

/**
 * @brief Evaluates the locomotion fitness of a Robot.
 *
 * Fitness is defined as the horizontal (XY-plane) displacement of the
 * centre of mass over a fixed number of neural cycles — matching the
 * evaluation criterion described in Lipson & Pollack (2000).
 *
 * One "cycle" consists of:
 *   1. tickNeural()      — synchronous parallel update of all neurons
 *   2. applyActuators()  — apply neuron outputs to bar rest-lengths
 *   3. relax()           — quasi-static gradient descent to the nearest
 *                          energy minimum (steps_per_cycle steps)
 *
 * The robot is evaluated statelessly: a fresh Simulator is constructed
 * from the Robot on every call to evaluate(), so repeated calls on the
 * same Robot are deterministic.
 *
 * Usage
 * -----
 * @code
 *   FitnessEvaluator eval;                   // default params
 *   double score = eval.evaluate(robot);      // metres of XY displacement
 *
 *   // record per-cycle CoM positions for analysis / rendering
 *   std::vector<Eigen::Vector2d> traj;
 *   double score = eval.evaluate(robot, &traj);
 * @endcode
 */
class FitnessEvaluator
{
public:
    // Expose the params type under a nested alias for convenience
    using Params = FitnessParams;

    explicit FitnessEvaluator(Params params = Params());

    /**
     * @brief Evaluate a robot's locomotion fitness.
     *
     * Constructs a fresh Simulator, runs @p params.cycles neural–relax
     * cycles, and returns the Euclidean distance in the XY plane between
     * the initial and final centre-of-mass positions.
     *
     * @param robot      Robot to evaluate.  Not modified.
     * @param trajectory Optional output: one (x, y) point per cycle
     *                   (including the initial position at index 0).
     *                   Pass nullptr to skip recording.
     * @return           Horizontal displacement [m].  ≥ 0.
     */
    double evaluate(const Robot&                   robot,
                    std::vector<Eigen::Vector2d>*  trajectory = nullptr) const;

    const Params& params() const { return params_; }

private:
    Params params_;
};

