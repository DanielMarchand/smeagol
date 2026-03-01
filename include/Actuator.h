#pragma once

#include "RobotPart.h"
#include <memory>
#include <string>

/**
 * @brief Couples one Neuron to one Bar, driving length changes.
 *
 * During each neural cycle the Simulator checks the linked neuron's activation
 * and scales bar_range to compute a delta applied to Bar::rest_length.
 * The change is clamped to ≤ 1 cm per cycle (per the paper).
 *
 *   Δl = neuron.activation * bar_range
 *   Δl = clamp(Δl, -0.01, +0.01)      // 1 cm limit
 */
class Actuator : public RobotPart
{
public:
    /**
     * @param bar_idx    Index into Robot::bars.
     * @param neuron_idx Index into Robot::neurons.
     * @param bar_range  Maximum length change per cycle [m].
     *                   Typically small (e.g. 0.01 m = 1 cm).
     */
    Actuator(int bar_idx, int neuron_idx, double bar_range = 0.01)
        : bar_idx(bar_idx),
          neuron_idx(neuron_idx),
          bar_range(bar_range)
    {}

    // ── RobotPart interface ───────────────────────────────────────────────
    [[nodiscard]] std::unique_ptr<RobotPart> clone() const override
    {
        return std::make_unique<Actuator>(*this);
    }

    [[nodiscard]] Type type() const override { return Type::Actuator; }

    [[nodiscard]] std::string typeName() const override { return "actuator"; }

    // ── Data ─────────────────────────────────────────────────────────────
    int    bar_idx;    ///< Index into Robot::bars
    int    neuron_idx; ///< Index into Robot::neurons
    double bar_range;  ///< Max Δl per neural cycle [m]
};
