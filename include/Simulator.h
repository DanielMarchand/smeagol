#pragma once

#include "Robot.h"
#include "Materials.h"

#include <Eigen/Dense>
#include <random>

/**
 * @brief Physics simulator for a single Robot individual.
 *
 * The Simulator decouples the evolving geometric *state* (vertex positions)
 * from the fixed *topology* (bar connectivity, rest-lengths, neuron weights)
 * stored in the Robot genotype.  This lets the evolver keep a clean Robot
 * object while the Simulator freely relaxes positions during evaluation.
 *
 * Coordinate convention
 * ---------------------
 * Z is up.  Vertex height h_j = positions(j, 2).
 *
 * Energy function (§3.1)
 * ----------------------
 * Total energy:
 *
 *   H = H_elastic + H_gravity
 *
 * Elastic (spring) energy, summed over all bars i:
 *
 *   H_elastic = Σ k_i · δ_i²
 *
 * where:
 *   k_i   = E · A_i / L0_i          (axial stiffness)
 *   δ_i   = ‖p_v2 − p_v1‖ − L0_i   (extension from rest length)
 *
 * Gravitational potential energy, summed over all vertices j:
 *
 *   H_gravity = Σ m_j · g · h_j
 *
 * where m_j is the lumped vertex mass (each bar's mass split equally between
 * its two endpoints) and h_j = z_j.
 *
 * Usage
 * -----
 * @code
 *   Simulator sim(robot);
 *   double H = sim.totalEnergy();
 *
 *   // Access / modify positions directly for 3.2 relaxation:
 *   sim.positions(i, 2) += 0.001;  // nudge vertex i upward
 * @endcode
 */
class Simulator
{
public:
    /**
     * @brief Construct a Simulator from a Robot genotype.
     *
     * Copies vertex positions into the internal Eigen matrix and precomputes
     * lumped vertex masses.  The Robot reference is retained for topology
     * (bars, rest-lengths, stiffness) but its vertex positions are NOT
     * updated by the simulator — call copyPositionsBack() if needed.
     *
     * @param robot  The robot to simulate.  Must remain valid for the
     *               lifetime of this Simulator.
     */
    explicit Simulator(const Robot& robot);

    // ── §3.1  Energy function ─────────────────────────────────────────────

    /**
     * @brief Elastic (spring) energy of the current configuration.
     *
     * H_elastic = Σ_i  k_i · δ_i²
     *
     * Uses Eigen to compute per-bar extension vectors efficiently.
     */
    [[nodiscard]] double elasticEnergy() const;

    /**
     * @brief Gravitational potential energy of the current configuration.
     *
     * H_gravity = Σ_j  m_j · g · h_j
     *
     * Implemented as a single Eigen coefficient-wise product + sum.
     */
    [[nodiscard]] double gravitationalEnergy() const;

    /**
     * @brief Total energy H = H_elastic + H_gravity + H_collision.
     */
    [[nodiscard]] double totalEnergy() const;

    // ── §3.3  Environment physics ───────────────────────────────────────────

    /**
     * @brief Floor collision penalty energy.
     *
     * For each vertex j below the ground plane (z_j < 0):
     *
     *   H_collision = Σ_j  k_floor · z_j²
     *
     * The gradient contribution pushes the vertex back up:
     *   ∂H_collision/∂z_j = 2 · k_floor · z_j  (negative z → upward force)
     */
    [[nodiscard]] double collisionEnergy() const;

    // ── §3.2  Quasi-static relaxation ─────────────────────────────────────

    /**
     * Result returned by relax().
     */
    struct RelaxResult
    {
        int    iterations;    ///< Number of gradient steps actually taken
        double final_energy;  ///< H_total after relaxation
        bool   converged;     ///< true = stopped by tolerance; false = hit max_iter
    };

    /**
     * @brief Run gradient-descent quasi-static relaxation.
     *
     * Iteratively moves vertex positions down the energy gradient:
     *
     *   ∂H/∂p_j  =  Σ_{bars i incident on j}  (−1)^side · 2·k_i·δ_i·û_i
     *             +  [0, 0, m_j·g]
     *
     *   p_j  ←  p_j  −  step_size · ∂H/∂p_j  +  noise
     *
     * A small uniform noise term is added each iteration to prevent
     * the solver from settling in unstable equilibria (saddle points).
     *
     * Convergence is declared when the Frobenius norm of the gradient
     * matrix falls below @p convergence_tol.
     *
     * @param max_iterations    Hard cap on gradient steps.
     * @param step_size         Ds in the update rule [m/N].
     *                          Must satisfy Ds < 1/(2·k_max·degree_max)
     *                          for stability.  Default 1e-8 is conservative
     *                          for steel bars with r=0.01 m.
     * @param noise_amplitude   Half-width of the uniform noise added to each
     *                          coordinate per iteration [m].  Default 1e-6 m.
     * @param convergence_tol   Stop when ‖∇H‖_F < this value.
     * @return                  RelaxResult with iteration count, final energy,
     *                          and convergence flag.
     */
    RelaxResult relax(int    max_iterations   = 1000,
                      double step_size        = 1e-8,
                      double noise_amplitude  = 1e-6,
                      double convergence_tol  = 1e-10);

    /**
     * @brief Apply static friction lock to a gradient matrix in-place.
     *
     * For each vertex j currently in contact with the floor (z_j ≤ 0):
     *   - Normal force:  N_j = 2 · k_floor · |z_j|
     *   - Lateral force: F_lat = ‖(grad_x, grad_y)‖
     *   - If F_lat ≤ mu_static · N_j: zero out x,y components of grad row j
     *
     * Called by relax() after computeGradient() and before the position step.
     * Exposed publicly for unit testing.
     */
    void applyFriction(Eigen::MatrixX3d& grad) const;

    /**
     * @brief Apply all DebugActuators in the robot for the given sim time.
     *
     * Updates the internal rest-length overrides used by elasticEnergy() and
     * computeGradient().  Call once per frame before relax().
     *
     * @param sim_time  Simulation time [s] passed to DebugActuator::deltaLength().
     */
    void applyDebugActuators(double sim_time);

    // ── §3.4  Neural tick ─────────────────────────────────────────────────

    /**
     * @brief Run one discrete update of all neuron activations.
     *
     * For each neuron i:
     *
     *   weighted_sum = Σ_j  synapse_weights[j] · activations_[j]
     *   activations_[i] = (weighted_sum ≥ threshold) ? 1.0 : 0.0
     *
     * All neurons are evaluated from the *previous* activation vector
     * (parallel update), so the order of neurons does not matter.
     * Neurons with fewer synapse_weights entries than the network size
     * have the missing weights treated as 0.
     *
     * No-op when the robot has no neurons.
     */
    void tickNeural();

    // ── §3.5  Actuator coupling ───────────────────────────────────────────

    /**
     * @brief Update bar rest-lengths from the current neuron activations.
     *
     * For each Actuator a:
     *
     *   Δl = clamp(activations_[a.neuron_idx] · a.bar_range, −0.01, +0.01)
     *   rest_lengths_[a.bar_idx] += Δl
     *
     * The ±1 cm clamp enforces the paper's hard limit on length change per
     * neural cycle.  Must be called after tickNeural() to use the freshest
     * activations.  No-op when the robot has no actuators.
     */
    void applyActuators();

    // ── State ─────────────────────────────────────────────────────────────

    /**
     * Current vertex positions, shape (N, 3).
     * Row i = [x, y, z] of vertex i.  Modified in-place by the relaxation
     * solver (§3.2) and the actuator coupling (§3.5).
     */
    Eigen::MatrixX3d positions;

    /**
     * Lumped mass of each vertex [kg], shape (N,).
     * Precomputed at construction; does not change during simulation.
     */
    Eigen::VectorXd vertex_masses;

    /**
     * Runtime neuron activations, length == robot.neurons.size().
     * Each entry is 0.0 (quiescent) or 1.0 (firing).  Initialised to 0;
     * updated by tickNeural() and readable/settable by external callers
     * (e.g. evolver, tests) to prime a network's initial state.
     */
    std::vector<double> activations_;

    /**
     * Optional wind acceleration [m/s²] applied to all vertices in +X.
     * Set to a positive value before calling relax() to simulate a
     * constant horizontal force (useful for sanity-checking the fitness
     * evaluator without needing a working locomotion controller).
     * Default 0.0 = no wind.
     */
    double wind = 0.0;

    /**
     * Per-bar rest-length overrides, length == robot.bars.size().
     * Initialised from robot_.bars at construction; updated each frame by
     * applyDebugActuators() and applyActuators().  Exposed publicly so that
     * external callers can inspect or reset the lengths between evaluations.
     */
    std::vector<double> rest_lengths_;

    /**
     * Copy current positions back into the Robot's vertex list.
     * Useful for post-simulation inspection or rendering.
     */
    void copyPositionsBack(Robot& robot) const;

private:
    const Robot& robot_;   ///< Topology reference (bars, stiffness, etc.)

    /**
     * @brief Compute the N×3 energy gradient matrix.
     *
     * Row j = ∂H/∂p_j = elastic contributions from all bars incident on j
     *                   + gravitational gradient [0, 0, m_j·g].
     */
    [[nodiscard]] Eigen::MatrixX3d computeGradient() const;

    std::mt19937 rng_;   ///< PRNG for relaxation noise
};
