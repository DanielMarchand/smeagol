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
}
