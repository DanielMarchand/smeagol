#pragma once

#include "Robot.h"

#include <random>

/**
 * @brief Factory for generating random Robot configurations.
 *
 * Used both by the Evolver to seed the initial population and by diagnostic
 * tools that need plausible starting genotypes.
 *
 * Design goals
 * ------------
 * - Every generated robot passes robot.isValid().
 * - The structure is connected (no isolated vertices) and physically
 *   plausible (vertices above the floor, realistic bar dimensions).
 * - The neural controller has at least one actuated bar if neurons exist.
 * - Full determinism: same seed → same robot.
 *
 * Geometry conventions
 * --------------------
 * Vertices are scattered in a small bounding box:
 *   x, y  ∈ [-0.05, +0.05] m   (10 cm lateral footprint)
 *   z     ∈ [ 0.02,  0.15] m   (start above the floor)
 *
 * Connectivity
 * ------------
 * A random spanning tree is built first (N−1 bars) to guarantee connectivity.
 * Additional bars are added with probability p_extra_bar (default 0.4) for
 * each remaining possible vertex pair, up to a maximum density.
 *
 * Usage
 * -----
 * @code
 *   std::mt19937 rng(42);
 *   Robot r = RobotFactory::randomRobot(rng);
 *   assert(r.isValid());
 * @endcode
 */
class RobotFactory
{
public:
    /**
     * @brief Parameters controlling random robot generation.
     */
    struct Params
    {
        // Morphology
        int    min_vertices    = 3;      ///< Minimum vertex count
        int    max_vertices    = 6;      ///< Maximum vertex count
        double bbox_xy         = 0.05;   ///< Half-width of lateral bounding box [m]
        double bbox_z_min      = 0.02;   ///< Minimum vertex height [m]
        double bbox_z_max      = 0.15;   ///< Maximum vertex height [m]
        double p_extra_bar     = 0.40;   ///< Probability of adding each non-tree edge
        double stiffness_min   = 10'000.0;  ///< Min bar spring stiffness [N/m]
        double stiffness_max   = 100'000.0; ///< Max bar spring stiffness [N/m]

        // Neural controller
        int    min_neurons     = 1;      ///< Minimum number of neurons
        int    max_neurons     = 3;      ///< Maximum number of neurons
        double threshold_min   = 0.2;   ///< Min neuron firing threshold
        double threshold_max   = 0.8;   ///< Max neuron firing threshold
        double weight_range    = 1.0;   ///< Synapse weights drawn from [-w, +w]

        // Actuation
        double p_actuate_bar   = 0.50;   ///< Probability each bar gets an actuator
        double bar_range_max   = 0.010;  ///< Upper bound for bar_range [m]; drawn from [0, bar_range_max]
    };

    /**
     * @brief Generate one random Robot.
     *
     * @param rng     Seeded PRNG — consumed in place, fully deterministic.
     * @param params  Generation parameters (defaults give plausible creatures).
     * @return        A valid Robot with at least min_vertices, one spanning tree
     *                of bars, at least one neuron, and at least one actuator.
     */
    [[nodiscard]] static Robot randomRobot(std::mt19937& rng,
                                           const Params& params);
    /** Overload using default Params. */
    [[nodiscard]] static Robot randomRobot(std::mt19937& rng);
};
