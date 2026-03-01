/**
 * test_simulator.cpp
 *
 * Tests for §3.1: Simulator energy function.
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
    r.addBar(Bar(0, 1, /*rest_length=*/1.0, /*radius=*/0.01));
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
    // Single bar: mass = rho * A * L0, split equally between two vertices.
    const double r     = 0.01;
    const double L0    = 1.0;
    const double A     = M_PI * r * r;
    const double half  = 0.5 * Materials::rho * A * L0;

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
    //   k = E·A / L0
    //   H = k · δ²
    const double r0    = 0.01;
    const double L0    = 1.0;
    const double A     = M_PI * r0 * r0;
    const double k     = Materials::E * A / L0;
    const double delta = 0.1;
    const double expected = k * delta * delta;

    Robot robot;
    robot.addVertex(Vertex(0.0, 0.0, 0.0));
    robot.addVertex(Vertex(1.1, 0.0, 0.0));   // 1.1 m apart
    robot.addBar(Bar(0, 1, L0, r0));

    Simulator sim(robot);
    CHECK_NEAR(sim.elasticEnergy(), expected, 1e-6);
}

static void test_elastic_energy_compressed_bar()
{
    // Compression: bar is shorter than rest length, δ < 0, H = k·δ² > 0.
    const double r0    = 0.01;
    const double L0    = 1.0;
    const double A     = M_PI * r0 * r0;
    const double k     = Materials::E * A / L0;
    const double delta = -0.05;   // compressed by 5 cm
    const double expected = k * delta * delta;

    Robot robot;
    robot.addVertex(Vertex(0.0, 0.0, 0.0));
    robot.addVertex(Vertex(0.95, 0.0, 0.0));
    robot.addBar(Bar(0, 1, L0, r0));

    Simulator sim(robot);
    CHECK_NEAR(sim.elasticEnergy(), expected, 1e-6);
}

static void test_elastic_energy_multiple_bars()
{
    // Two bars: one at rest (ΔE=0), one stretched by δ.
    const double r0    = 0.01;
    const double L0    = 1.0;
    const double A     = M_PI * r0 * r0;
    const double k     = Materials::E * A / L0;
    const double delta = 0.2;

    Robot robot;
    robot.addVertex(Vertex(0.0, 0.0, 0.0));  // 0
    robot.addVertex(Vertex(1.0, 0.0, 0.0));  // 1  — at rest from 0
    robot.addVertex(Vertex(2.2, 0.0, 0.0));  // 2  — 1.2 m from 1, rest=1.0
    robot.addBar(Bar(0, 1, L0, r0));         // at rest
    robot.addBar(Bar(1, 2, L0, r0));         // stretched by 0.2

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
    // Bar mass: rho * A * L0, split equally.
    // m_each = 0.5 * rho * pi * r² * L0
    // H_gravity = m0*g*0 + m1*g*1 = m_each * g * 1.0
    const double r0     = 0.01;
    const double L0     = 1.0;
    const double A      = M_PI * r0 * r0;
    const double m_each = 0.5 * Materials::rho * A * L0;
    const double expected = m_each * Materials::g * 1.0;

    Robot robot;
    robot.addVertex(Vertex(0.0, 0.0, 0.0));
    robot.addVertex(Vertex(0.0, 0.0, 1.0));  // z = 1 m
    robot.addBar(Bar(0, 1, L0, r0));

    Simulator sim(robot);
    CHECK_NEAR(sim.gravitationalEnergy(), expected, 1e-9);
}

static void test_total_energy_is_sum()
{
    // totalEnergy() must equal elasticEnergy() + gravitationalEnergy().
    Robot robot;
    robot.addVertex(Vertex(0.0, 0.0, 0.0));
    robot.addVertex(Vertex(1.1, 0.0, 0.5));  // stretched + elevated
    robot.addBar(Bar(0, 1, 1.0, 0.01));

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
    const double k = Bar(0, 1, 1.0, 0.01).stiffness();

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

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    auto run = [](const char* name, void (*fn)()) {
        std::cout << "[ RUN  ] " << name << "\n";
        fn();
    };

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

    if (g_failures == 0) {
        std::cout << "\nAll Simulator energy tests passed.\n";
        return 0;
    }
    std::cout << "\n" << g_failures << " check(s) FAILED.\n";
    return 1;
}
