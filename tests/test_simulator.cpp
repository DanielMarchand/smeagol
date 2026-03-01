/**
 * test_simulator.cpp
 *
 * Tests for §3.1 (energy function) and §3.2 (quasi-static relaxation).
 * All expected values are derived analytically from Materials constants to
 * ensure the test stays correct if constants change.
 */

#include "Simulator.h"
#include "Robot.h"
#include "Bar.h"
#include "Vertex.h"
#include "Materials.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

static int g_failures = 0;

#define CHECK(cond)                                                               \
    do {                                                                          \
        if (!(cond)) {                                                            \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__               \
                      << "  (" #cond ")\n";                                       \
            ++g_failures;                                                         \
        }                                                                         \
    } while (false)

#define CHECK_NEAR(a, b, eps)                                                     \
    do {                                                                          \
        if (std::abs((a) - (b)) > (eps)) {                                        \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__               \
                      << "  |" << (a) << " - " << (b) << "| > " << (eps) << "\n";\
            ++g_failures;                                                         \
        }                                                                         \
    } while (false)

static constexpr double EPS = 1e-9;

// ── helpers ───────────────────────────────────────────────────────────────────

// One horizontal bar at its rest length, both vertices on the ground plane.
static Robot make_single_bar_horizontal()
{
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(1.0, 0.0, 0.0));
    r.addBar(Bar(0, 1, /*rest_length=*/1.0, /*stiffness=*/Materials::k_default));
    return r;
}

// ── tests ─────────────────────────────────────────────────────────────────────

static void test_positions_initialised_from_robot()
{
    Robot r = make_single_bar_horizontal();
    Simulator sim(r);

    CHECK(sim.positions.rows() == 2);
    CHECK(sim.positions.cols() == 3);
    CHECK_NEAR(sim.positions(0, 0), 0.0, EPS);
    CHECK_NEAR(sim.positions(1, 0), 1.0, EPS);
    CHECK_NEAR(sim.positions(0, 2), 0.0, EPS); // z=0
    CHECK_NEAR(sim.positions(1, 2), 0.0, EPS); // z=0
}

static void test_vertex_masses_precomputed()
{
    // Single bar: mass = rho * (k/E) * L0^2, split equally between two vertices.
    const double L0   = 1.0;
    const double k    = Materials::k_default;
    const double half = 0.5 * Materials::rho * (k / Materials::E) * L0 * L0;

    Robot robot = make_single_bar_horizontal();
    Simulator sim(robot);

    CHECK(sim.vertex_masses.size() == 2);
    CHECK_NEAR(sim.vertex_masses(0), half, EPS);
    CHECK_NEAR(sim.vertex_masses(1), half, EPS);
}

static void test_elastic_energy_at_rest_is_zero()
{
    // When bar length == rest_length, δ = 0, H_elastic = 0.
    Robot r = make_single_bar_horizontal();
    Simulator sim(r);
    CHECK_NEAR(sim.elasticEnergy(), 0.0, EPS);
}

static void test_elastic_energy_stretched_bar()
{
    // Bar rest_length = 1.0 m, actual distance = 1.1 m → δ = 0.1 m.
    //   k = bar.stiffness
    //   H = k · δ²
    const double k0    = 50000.0;
    const double L0    = 1.0;
    const double k     = k0;
    const double delta = 0.1;
    const double expected = k * delta * delta;

    Robot robot;
    robot.addVertex(Vertex(0.0, 0.0, 0.0));
    robot.addVertex(Vertex(1.1, 0.0, 0.0));   // 1.1 m apart
    robot.addBar(Bar(0, 1, L0, k0));

    Simulator sim(robot);
    CHECK_NEAR(sim.elasticEnergy(), expected, 1e-6);
}

static void test_elastic_energy_compressed_bar()
{
    // Compression: bar is shorter than rest length, δ < 0, H = k·δ² > 0.
    const double k0    = 50000.0;
    const double L0    = 1.0;
    const double k     = k0;
    const double delta = -0.05;   // compressed by 5 cm
    const double expected = k * delta * delta;

    Robot robot;
    robot.addVertex(Vertex(0.0, 0.0, 0.0));
    robot.addVertex(Vertex(0.95, 0.0, 0.0));
    robot.addBar(Bar(0, 1, L0, k0));

    Simulator sim(robot);
    CHECK_NEAR(sim.elasticEnergy(), expected, 1e-6);
}

static void test_elastic_energy_multiple_bars()
{
    // Two bars: one at rest (ΔE=0), one stretched by δ.
    const double k0    = 50000.0;
    const double L0    = 1.0;
    const double k     = k0;
    const double delta = 0.2;

    Robot robot;
    robot.addVertex(Vertex(0.0, 0.0, 0.0));  // 0
    robot.addVertex(Vertex(1.0, 0.0, 0.0));  // 1  — at rest from 0
    robot.addVertex(Vertex(2.2, 0.0, 0.0));  // 2  — 1.2 m from 1, rest=1.0
    robot.addBar(Bar(0, 1, L0, k0));         // at rest
    robot.addBar(Bar(1, 2, L0, k0));         // stretched by 0.2

    Simulator sim(robot);
    CHECK_NEAR(sim.elasticEnergy(), k * delta * delta, 1e-6);
}

static void test_gravitational_energy_ground_plane()
{
    // Both vertices at z=0 → H_gravity = 0 regardless of mass.
    Robot r = make_single_bar_horizontal();
    Simulator sim(r);
    CHECK_NEAR(sim.gravitationalEnergy(), 0.0, EPS);
}

static void test_gravitational_energy_vertical_bar()
{
    // Vertical bar: v0 at z=0, v1 at z=1.
    // Bar mass: rho * (k/E) * L0^2, split equally.
    // m_each = 0.5 * rho * (k/E) * L0^2
    // H_gravity = m0*g*0 + m1*g*1 = m_each * g * 1.0
    const double k0     = 50000.0;
    const double L0     = 1.0;
    const double m_each = 0.5 * Materials::rho * (k0 / Materials::E) * L0 * L0;
    const double expected = m_each * Materials::g * 1.0;

    Robot robot;
    robot.addVertex(Vertex(0.0, 0.0, 0.0));
    robot.addVertex(Vertex(0.0, 0.0, 1.0));  // z = 1 m
    robot.addBar(Bar(0, 1, L0, k0));

    Simulator sim(robot);
    CHECK_NEAR(sim.gravitationalEnergy(), expected, 1e-9);
}

static void test_total_energy_is_sum()
{
    // totalEnergy() must equal elasticEnergy() + gravitationalEnergy().
    Robot robot;
    robot.addVertex(Vertex(0.0, 0.0, 0.0));
    robot.addVertex(Vertex(1.1, 0.0, 0.5));  // stretched + elevated
    robot.addBar(Bar(0, 1, 1.0));

    Simulator sim(robot);
    CHECK_NEAR(sim.totalEnergy(),
               sim.elasticEnergy() + sim.gravitationalEnergy(),
               EPS);
}

static void test_energy_recomputed_after_position_change()
{
    // Modify positions directly then recheck energy.
    Robot robot = make_single_bar_horizontal();
    Simulator sim(robot);

    // Stretch bar by moving v1 to x=1.2
    sim.positions(1, 0) = 1.2;
    const double delta = 0.2;
    const double k = Materials::k_default;  // stiffness stored directly on Bar

    CHECK_NEAR(sim.elasticEnergy(), k * delta * delta, 1e-6);
}

static void test_copy_positions_back()
{
    Robot robot = make_single_bar_horizontal();
    Simulator sim(robot);

    // Move both vertices up by 1 m
    sim.positions(0, 2) = 1.0;
    sim.positions(1, 2) = 1.0;

    sim.copyPositionsBack(robot);

    CHECK_NEAR(robot.vertices[0].pos.z(), 1.0, EPS);
    CHECK_NEAR(robot.vertices[1].pos.z(), 1.0, EPS);
}

// ── §3.2  Quasi-static relaxation tests ──────────────────────────────────────

static void test_gradient_direction_stretched_bar()
{
    // Horizontal bar stretched in +x: v0 at origin, v1 at x=1.1, rest=1.0.
    // Analytical gradient:
    //   g_contrib = 2k * delta / length * dp
    //   grad[v0] -= g_contrib  →  negative x  (v0 pulled toward v1, update moves +x)
    //   grad[v1] += g_contrib  →  positive x  (v1 pulled toward v0, update moves -x)
    // Gradient descent: p -= step * grad.
    // After one step v0 should have moved in +x and v1 in -x.
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));   // v0
    r.addVertex(Vertex(1.1, 0.0, 0.0));   // v1 (stretched)
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);
    const double x0_before = sim.positions(0, 0);
    const double x1_before = sim.positions(1, 0);
    const double len_before = (sim.positions.row(1) - sim.positions.row(0)).norm();

    // Single relaxation step with no noise
    sim.relax(1, 1e-8, /*noise=*/0.0);

    const double len_after = (sim.positions.row(1) - sim.positions.row(0)).norm();

    // v0 should move in +x (toward v1), v1 in -x (toward v0)
    CHECK(sim.positions(0, 0) > x0_before);
    CHECK(sim.positions(1, 0) < x1_before);
    // Bar length should have decreased toward rest_length=1.0
    CHECK(len_after < len_before);
}

static void test_gradient_gravity_only()
{
    // Single vertex with known mass, no bars.  Only gravitational gradient.
    // ∂H/∂p = [0, 0, m*g]
    // After one step (step_size=s): z → z - s*m*g  (vertex falls)
    const double k0     = Materials::k_default;
    const double L0     = 1.0;
    const double half_mass = 0.5 * Materials::rho * (k0 / Materials::E) * L0 * L0;

    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 1.0));  // elevated
    r.addVertex(Vertex(1.0, 0.0, 1.0));
    r.addBar(Bar(0, 1, L0, k0));     // bar at rest length (no elastic gradient)

    const double step = 1e-8;
    Simulator sim(r);
    const double z0_before = sim.positions(0, 2);

    sim.relax(1, step, /*noise=*/0.0);

    // Expected z after one step: z - step * m * g
    const double expected_z = z0_before - step * half_mass * Materials::g;
    CHECK_NEAR(sim.positions(0, 2), expected_z, 1e-14);
}

static void test_relax_reduces_elastic_energy()
{
    // Stretch a bar, confirm that elastic energy decreases monotonically
    // across several relaxation steps.
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(1.5, 0.0, 0.0));   // stretched from rest=1.0
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);
    const double He_before = sim.elasticEnergy();

    sim.relax(50, 1e-8, /*noise=*/0.0);

    CHECK(sim.elasticEnergy() < He_before);
}

static void test_relax_bar_approaches_rest_length()
{
    // After sufficient iterations a stretched bar should approach rest length.
    const double rest = 1.0;
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(1.2, 0.0, 0.0));   // 20% stretch
    r.addBar(Bar(0, 1, rest));

    Simulator sim(r);
    const double len_before = (sim.positions.row(1) - sim.positions.row(0)).norm();

    sim.relax(500, 1e-8, /*noise=*/0.0);

    const double len_after = (sim.positions.row(1) - sim.positions.row(0)).norm();
    CHECK(std::abs(len_after - rest) < std::abs(len_before - rest));
}

static void test_relax_result_struct()
{
    // Verify RelaxResult fields are sensibly populated.
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(1.1, 0.0, 0.0));
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);
    auto result = sim.relax(200, 1e-8, 0.0);

    CHECK(result.iterations >  0);
    CHECK(result.iterations <= 200);
    CHECK(std::isfinite(result.final_energy));
}

static void test_relax_at_rest_does_not_increase_elastic_energy()
{
    // Robot exactly at rest geometry. Elastic energy should remain ~0
    // even after relaxation (gravity moves vertices but doesn't stretch bars).
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(1.0, 0.0, 0.0));   // exactly at rest length
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);
    CHECK_NEAR(sim.elasticEnergy(), 0.0, EPS);

    sim.relax(100, 1e-8, /*noise=*/0.0);

    // Bar is horizontal; gravity only moves z. Horizontal distance unchanged.
    // Elastic energy should stay effectively zero.
    CHECK(sim.elasticEnergy() < 1.0);  // loose bound — mainly ensures no explosion
}

// ── §3.3  Environment physics tests ──────────────────────────────────────────

static void test_collision_energy_zero_above_floor()
{
    // All vertices at z >= 0: collision energy must be exactly zero.
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(1.0, 0.0, 1.0));
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);
    CHECK_NEAR(sim.collisionEnergy(), 0.0, EPS);
}

static void test_collision_energy_below_floor()
{
    // One vertex at z = -0.1: H_collision = k_floor * 0.1^2
    const double z      = -0.1;
    const double expected = Materials::k_floor * z * z;

    Robot r;
    r.addVertex(Vertex(0.0, 0.0, z));
    r.addVertex(Vertex(1.0, 0.0, 0.0));
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);
    CHECK_NEAR(sim.collisionEnergy(), expected, 1e-6);
}

static void test_total_energy_includes_collision()
{
    // totalEnergy() must equal elastic + gravity + collision.
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, -0.05));   // below floor
    r.addVertex(Vertex(1.0, 0.0,  0.00));
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);
    CHECK_NEAR(sim.totalEnergy(),
               sim.elasticEnergy() +
               sim.gravitationalEnergy() +
               sim.collisionEnergy(),
               EPS);
}

static void test_floor_gradient_pushes_vertex_up()
{
    // Vertex below floor: after one relaxation step its z should increase.
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, -0.01));
    r.addVertex(Vertex(1.0, 0.0, -0.01));
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);
    const double z_before = sim.positions(0, 2);

    sim.relax(1, 1e-8, /*noise=*/0.0);

    CHECK(sim.positions(0, 2) > z_before);
}

static void test_friction_locks_lateral_when_small_force()
{
    // Grounded vertex (z <= 0) with tiny lateral gradient → x,y should be
    // locked by static friction.
    // We test applyFriction() directly: construct a grad with a known small
    // lateral component and confirm it gets zeroed.
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, -0.001));  // just below floor
    r.addVertex(Vertex(1.0, 0.0,  0.000));
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);

    // Build a synthetic gradient: tiny lateral force on vertex 0
    Eigen::MatrixX3d grad = Eigen::MatrixX3d::Zero(2, 3);
    grad(0, 0) = 1e-5;   // tiny x force
    grad(0, 1) = 1e-5;   // tiny y force

    // Normal force at z=-0.001: N = 2 * k_floor * 0.001 = 2e6 N
    // Lateral: sqrt(2)*1e-5 ≈ 1.41e-5 << mu_static(0.5) * 2e6 = 1e6
    // → must be locked
    sim.applyFriction(grad);

    CHECK_NEAR(grad(0, 0), 0.0, EPS);
    CHECK_NEAR(grad(0, 1), 0.0, EPS);
}

static void test_friction_allows_lateral_when_large_force()
{
    // Grounded vertex with a lateral force exceeding mu_static * N → NOT locked.
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, -0.001));
    r.addVertex(Vertex(1.0, 0.0,  0.000));
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);

    // Normal force  ≈ 2 * 1e9 * 0.001 = 2e6 N
    // Need lateral > mu_static * 2e6 = 1e6 N
    Eigen::MatrixX3d grad = Eigen::MatrixX3d::Zero(2, 3);
    grad(0, 0) = 2e6;   // large x force

    sim.applyFriction(grad);

    // x should NOT be zeroed
    CHECK(grad(0, 0) != 0.0);
}

static void test_subsurface_vertex_rises_above_floor_after_relaxation()
{
    // After sufficient relaxation a vertex starting below z=0 should end ≥ 0.
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, -0.05));
    r.addVertex(Vertex(1.0, 0.0, -0.05));
    r.addBar(Bar(0, 1, 1.0));

    Simulator sim(r);
    sim.relax(500, 1e-8, /*noise=*/0.0);

    // Both vertices should be driven back to at or above z=0
    CHECK(sim.positions(0, 2) >= -1e-4);   // allow small numerical residual
    CHECK(sim.positions(1, 2) >= -1e-4);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    auto run = [](const char* name, void (*fn)()) {
        std::cout << "[ RUN  ] " << name << "\n";
        fn();
    };

    // §3.1 energy function
    run("Positions initialised from Robot",           test_positions_initialised_from_robot);
    run("Vertex masses precomputed",                  test_vertex_masses_precomputed);
    run("Elastic energy = 0 at rest length",          test_elastic_energy_at_rest_is_zero);
    run("Elastic energy: stretched bar",              test_elastic_energy_stretched_bar);
    run("Elastic energy: compressed bar",             test_elastic_energy_compressed_bar);
    run("Elastic energy: multiple bars",              test_elastic_energy_multiple_bars);
    run("Gravitational energy = 0 on ground plane",   test_gravitational_energy_ground_plane);
    run("Gravitational energy: vertical bar",         test_gravitational_energy_vertical_bar);
    run("Total energy = elastic + gravitational",     test_total_energy_is_sum);
    run("Energy recomputed after position change",    test_energy_recomputed_after_position_change);
    run("copyPositionsBack updates Robot vertices",   test_copy_positions_back);

    // §3.2 quasi-static relaxation
    run("Gradient direction: stretched bar",          test_gradient_direction_stretched_bar);
    run("Gradient: gravity-only one step",            test_gradient_gravity_only);
    run("relax() reduces elastic energy",             test_relax_reduces_elastic_energy);
    run("relax() bar approaches rest length",         test_relax_bar_approaches_rest_length);
    run("relax() result struct fields valid",         test_relax_result_struct);
    run("relax() at rest: elastic energy stays ~0",   test_relax_at_rest_does_not_increase_elastic_energy);

    // §3.3 environment physics
    run("Collision energy = 0 above floor",           test_collision_energy_zero_above_floor);
    run("Collision energy: analytical value",         test_collision_energy_below_floor);
    run("totalEnergy() includes collision term",      test_total_energy_includes_collision);
    run("Floor gradient pushes vertex upward",        test_floor_gradient_pushes_vertex_up);
    run("Friction locks lateral: small force",        test_friction_locks_lateral_when_small_force);
    run("Friction allows lateral: large force",       test_friction_allows_lateral_when_large_force);
    run("Sub-floor vertex rises above floor",         test_subsurface_vertex_rises_above_floor_after_relaxation);

    if (g_failures == 0) {
        std::cout << "\nAll Simulator tests passed.\n";
        return 0;
    }
    std::cout << "\n" << g_failures << " check(s) FAILED.\n";
    return 1;
}
