#pragma once

#include "RobotPart.h"
#include <Eigen/Core>
#include <memory>
#include <string>

/**
 * @brief A ball-joint node in the robot's truss structure.
 *
 * Stores a 3-D position in world space.  During physics simulation the
 * position is updated in-place by the quasi-static relaxation solver.
 */
class Vertex : public RobotPart
{
public:
    explicit Vertex(double x = 0.0, double y = 0.0, double z = 0.0)
        : pos(x, y, z) {}

    explicit Vertex(const Eigen::Vector3d& position)
        : pos(position) {}

    // ── RobotPart interface ───────────────────────────────────────────────
    [[nodiscard]] std::unique_ptr<RobotPart> clone() const override
    {
        return std::make_unique<Vertex>(*this);
    }

    [[nodiscard]] Type type() const override { return Type::Vertex; }

    [[nodiscard]] std::string typeName() const override { return "vertex"; }

    // ── Data ─────────────────────────────────────────────────────────────
    Eigen::Vector3d pos;  ///< World-space position [m]
};
