#pragma once

#include "RobotPart.h"
#include <Eigen/Core>
#include <memory>
#include <string>

/**
 * @brief A discrete computational node in the robot's neural controller.
 *
 * At each neural tick the neuron sums its weighted inputs, applies a
 * threshold, and outputs a binary activation (0 or 1).
 *
 *   activation = (weighted_sum >= threshold) ? 1.0 : 0.0
 *
 * The synapse_weights vector has one entry per other neuron in the network
 * (ordered by neuron index in Robot::neurons).  During mutation the vector
 * grows or shrinks via Eigen::VectorXd::conservativeResize() to stay
 * consistent with the network size.  The neural tick evaluates:
 *
 *   weighted_sum = synapse_weights.dot(activation_vector)
 *   activation   = (weighted_sum >= threshold) ? 1.0 : 0.0
 */
class Neuron : public RobotPart
{
public:
    /**
     * @param threshold       Firing threshold.
     * @param synapse_weights Weights from each other neuron.  Pass a
     *                        zero-length VectorXd (default) for an isolated
     *                        (unconnected) neuron.
     */
    explicit Neuron(double threshold = 0.5,
                    Eigen::VectorXd synapse_weights = Eigen::VectorXd{})
        : threshold(threshold),
          synapse_weights(std::move(synapse_weights)),
          activation(0.0)
    {}

    // ── RobotPart interface ───────────────────────────────────────────────
    [[nodiscard]] std::unique_ptr<RobotPart> clone() const override
    {
        return std::make_unique<Neuron>(*this);
    }

    [[nodiscard]] Type type() const override { return Type::Neuron; }

    [[nodiscard]] std::string typeName() const override { return "neuron"; }

    // ── Data ─────────────────────────────────────────────────────────────
    double          threshold;        ///< Firing threshold
    Eigen::VectorXd synapse_weights;  ///< One weight per neuron in network
    double          activation;       ///< Current output (0 or 1), runtime only
};
