#pragma once

#include "RobotPart.h"
#include <memory>
#include <string>

/**
 * @brief A spring element that is driven by a Neuron.
 *
 * During each neural cycle the Simulator checks the linked neuron's activation
 * and scales bar_range to compute a delta applied to the bar's rest_length.
 * The change is clamped to ≤ 1 cm per cycle (per the paper).
 *
 *   Δl = neuron.activation * bar_range
 *   Δl = clamp(Δl, 0, 0.01)           // extension-only, 1 cm limit
 *
 * @note DESIGN SMELL: In the paper every spring is a "bar" that optionally has
 * a neural connection.  The current split — Bar (no neuron) vs Actuator (has
 * neuron) — is a historical artefact.  Attaching a neuron to a Bar converts it
 * into an Actuator; detaching the last neuron reverts it to a bare Bar.  This
 * is awkward.  TODO: remove Bar and rename this class to Bar, carrying an
 * optional neuron_idx field.
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
