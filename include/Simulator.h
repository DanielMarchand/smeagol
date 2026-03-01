#pragma once

#include "Robot.h"
#include "Materials.h"

#include <Eigen/Dense>

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
     * @brief Total energy H = H_elastic + H_gravity.
     */
    [[nodiscard]] double totalEnergy() const;

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
     * Copy current positions back into the Robot's vertex list.
     * Useful for post-simulation inspection or rendering.
     */
    void copyPositionsBack(Robot& robot) const;

private:
    const Robot& robot_;   ///< Topology reference (bars, stiffness, etc.)
};
