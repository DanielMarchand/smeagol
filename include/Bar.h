#pragma once

#include "RobotPart.h"
#include "Materials.h"
#include <memory>
#include <string>
#include <cmath>

/**
 * @brief A rigid rod connecting two vertices in the truss.
 *
 * The bar's cross-sectional area is derived from the paper's uniform circular
 * cross-section assumption.  Stiffness k = E*A / rest_length is computed on
 * demand so it automatically reflects any mutation to rest_length.
 */
class Bar : public RobotPart
{
public:
    /**
     * @param v1_idx      Index of the first vertex in Robot::vertices.
     * @param v2_idx      Index of the second vertex in Robot::vertices.
     * @param rest_length Relaxed (target) length [m].
     * @param radius      Cross-section radius [m].  Defaults to 0.01 m (1 cm).
     */
    Bar(int v1_idx, int v2_idx,
        double rest_length,
        double radius = 0.01)
        : v1(v1_idx), v2(v2_idx),
          rest_length(rest_length),
          radius(radius)
    {}

    // ── RobotPart interface ───────────────────────────────────────────────
    [[nodiscard]] std::unique_ptr<RobotPart> clone() const override
    {
        return std::make_unique<Bar>(*this);
    }

    [[nodiscard]] Type type() const override { return Type::Bar; }

    [[nodiscard]] std::string typeName() const override { return "bar"; }

    // ── Derived physics quantities ────────────────────────────────────────

    /// Cross-sectional area [m²]
    [[nodiscard]] double area() const
    {
        return M_PI * radius * radius;
    }

    /// Axial stiffness k = E·A / L₀  [N/m]
    [[nodiscard]] double stiffness() const
    {
        return (Materials::E * area()) / rest_length;
    }

    // ── Data ─────────────────────────────────────────────────────────────
    int    v1;           ///< Index into Robot::vertices
    int    v2;           ///< Index into Robot::vertices
    double rest_length;  ///< Relaxed length L₀ [m]
    double radius;       ///< Cross-section radius [m]
};
