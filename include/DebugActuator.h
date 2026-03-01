#pragma once

#include "RobotPart.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

/**
 * @brief Sine-wave driven bar actuator; no neuron required.
 *
 * Inherits RobotPart like every other part.  Behaves identically to Actuator
 * during simulation, except the length delta is computed from a user-specified
 * sine wave rather than a neuron activation:
 *
 *   Δl = amplitude · sin(2π · frequency · t + phase)
 *
 * The result is clamped to [-bar_range, +bar_range], preserving the paper's
 * 1 cm/cycle hard limit.  Not serialised to YAML (debug/test use only).
 */
class DebugActuator : public RobotPart
{
public:
    /**
     * @param bar_idx   Index into Robot::bars.
     * @param amplitude Peak displacement [m]. Clamped to bar_range after evaluation.
     * @param frequency Oscillation frequency [Hz].
     * @param phase     Phase offset [rad].
     * @param bar_range Maximum allowed Δl per tick [m]. Defaults to 0.01 m (1 cm).
     */
    DebugActuator(int    bar_idx,
                  double amplitude,
                  double frequency,
                  double phase     = 0.0,
                  double bar_range = 0.01)
        : bar_idx(bar_idx),
          amplitude(amplitude),
          frequency(frequency),
          phase(phase),
          bar_range(bar_range)
    {}

    // ── RobotPart interface ───────────────────────────────────────────────
    [[nodiscard]] std::unique_ptr<RobotPart> clone() const override
    {
        return std::make_unique<DebugActuator>(*this);
    }

    [[nodiscard]] Type type() const override { return Type::DebugActuator; }

    [[nodiscard]] std::string typeName() const override { return "debug_actuator"; }

    // ── Sine-wave output ──────────────────────────────────────────────────

    /**
     * @brief Compute the instantaneous rest-length delta at time @p t.
     * @param t  Simulation time [s].
     * @return   Δl in metres, clamped to [-bar_range, +bar_range].
     */
    [[nodiscard]] double deltaLength(double t) const
    {
        const double raw = amplitude * std::sin(
            2.0 * M_PI * frequency * t + phase);
        return std::clamp(raw, -bar_range, +bar_range);
    }

    // ── Data ─────────────────────────────────────────────────────────────
    int    bar_idx;    ///< Index into Robot::bars
    double amplitude;  ///< Peak displacement [m] (pre-clamp)
    double frequency;  ///< Oscillation frequency [Hz]
    double phase;      ///< Phase offset [rad]
    double bar_range;  ///< Clamp limit [m]
};
