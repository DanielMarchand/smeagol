#pragma once

#include "RobotPart.h"
#include "Materials.h"
#include <memory>
#include <string>
#include <cmath>

/**
 * @brief A spring element connecting two vertices in the truss.
 *
 * Stiffness k is stored directly as a genotype parameter (N/m), decoupling
 * spring behaviour from bar geometry.  This prevents the numerical instability
 * that arises when k = EA/L -> inf for short or thick bars.
 *
 * Mass is back-calculated as m = rho*(k/E)*L0^2  (equivalent to a circular rod
 * with cross-sectional area A = kL0/E under the paper's material constants).
 */
class Bar : public RobotPart
{
public:
    /**
     * @param v1_idx      Index of the first vertex in Robot::vertices.
     * @param v2_idx      Index of the second vertex in Robot::vertices.
     * @param rest_length Relaxed (target) length [m].
     * @param stiffness   Spring stiffness k [N/m].  Defaults to Materials::k_default.
     */
    Bar(int v1_idx, int v2_idx,
        double rest_length,
        double stiffness = Materials::k_default)
        : v1(v1_idx), v2(v2_idx),
          rest_length(rest_length),
          stiffness(stiffness)
    {}

    // ── RobotPart interface ───────────────────────────────────────────────
    [[nodiscard]] std::unique_ptr<RobotPart> clone() const override
    {
        return std::make_unique<Bar>(*this);
    }

    [[nodiscard]] Type type() const override { return Type::Bar; }

    [[nodiscard]] std::string typeName() const override { return "bar"; }

    // ── Data ─────────────────────────────────────────────────────────────
    int    v1;           ///< Index into Robot::vertices
    int    v2;           ///< Index into Robot::vertices
    double rest_length;  ///< Relaxed length L0 [m]
    double stiffness;    ///< Spring stiffness k [N/m]
};
