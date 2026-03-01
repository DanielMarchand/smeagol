#pragma once

#include <memory>
#include <string>

/**
 * @brief Abstract base class for all physical and neural components of a Robot.
 *
 * Every component that can be stored in a Robot's part lists inherits from
 * RobotPart.  The class provides a uniform interface for cloning (needed by
 * the genetic mutation operators) and type identification.
 */
class RobotPart
{
public:
    /// Stable integer tag – avoids dynamic_cast in hot paths.
    enum class Type { Vertex, Bar, Neuron, Actuator };

    virtual ~RobotPart() = default;

    /// Deep-copy this part (used by mutation operators).
    [[nodiscard]] virtual std::unique_ptr<RobotPart> clone() const = 0;

    /// Return the concrete type of this part.
    [[nodiscard]] virtual Type type() const = 0;

    /// Human-readable label (useful for debugging / YAML keys).
    [[nodiscard]] virtual std::string typeName() const = 0;
};
