#pragma once

/**
 * @brief Global material constants from Lipson & Pollack (2000).
 *
 * All values are in SI units.
 */
namespace Materials
{
    /// Young's modulus [Pa]  (0.896 GPa)
    inline constexpr double E       = 0.896e9;

    /// Mass density [kg/m³]
    inline constexpr double rho     = 1000.0;

    /// Yield strength [Pa]  (19 MPa)
    inline constexpr double S_yield = 19.0e6;

    /// Gravitational acceleration [m/s²]
    inline constexpr double g       = 9.81;

    /// Floor penalty stiffness [N/m].
    /// Set equal to a typical bar stiffness (E·A/L₀ for r=1 cm, L₀=0.2 m ≈ 1.4e6 N/m).
    /// A soft floor is acceptable for quasi-static gradient descent — the
    /// robot will penetrate slightly but the gradient always pushes it back up.
    inline constexpr double k_floor  = 1.4e6;

    /// Static friction coefficient (dimensionless).
    /// Lateral force must exceed mu_static * normal_force to move a grounded vertex.
    inline constexpr double mu_static = 0.5;
}
