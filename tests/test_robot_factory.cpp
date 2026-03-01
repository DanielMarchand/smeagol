/**
 * test_robot_factory.cpp
 *
 * Unit tests for RobotFactory::randomRobot.
 *
 * Tests
 * -----
 *  1. is_valid        — every generated robot passes isValid()
 *  2. part_counts     — vertex/bar/neuron/actuator counts within expected ranges
 *  3. has_actuator    — at least one actuator always present
 *  4. determinism     — same seed → identical YAML serialisation
 *  5. different_seeds — different seeds → different bar counts or positions
 *  6. floor_clearance — all vertices start at z > 0
 *  7. bar_lengths     — all rest lengths > 0 and physically plausible
 *  8. synapse_shape   — synapse_weights length == neuron count for every neuron
 *  9. stress_valid    — 500 random robots all pass isValid()
 */

#include "RobotFactory.h"
#include "Robot.h"

#include <cassert>
#include <iostream>
#include <random>
#include <sstream>

static int g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__            \
                      << "  (" #cond ")\n";                                    \
            ++g_failures;                                                      \
        }                                                                      \
    } while (false)

#define RUN(name)                                                              \
    do {                                                                       \
        const int before = g_failures;                                         \
        test_##name();                                                         \
        std::cout << (g_failures == before ? "  PASS  " : "  FAIL  ")         \
                  << #name << "\n";                                             \
    } while (false)

// ─────────────────────────────────────────────────────────────────────────────

static void test_is_valid()
{
    std::mt19937 rng(1);
    for (int i = 0; i < 20; ++i) {
        Robot r = RobotFactory::randomRobot(rng);
        CHECK(r.isValid());
    }
}

static void test_part_counts()
{
    RobotFactory::Params p;
    std::mt19937 rng(2);
    for (int i = 0; i < 50; ++i) {
        Robot r = RobotFactory::randomRobot(rng, p);
        CHECK(static_cast<int>(r.vertices.size()) >= p.min_vertices);
        CHECK(static_cast<int>(r.vertices.size()) <= p.max_vertices);
        CHECK(static_cast<int>(r.neurons.size())  >= p.min_neurons);
        CHECK(static_cast<int>(r.neurons.size())  <= p.max_neurons);
        // spanning tree gives at least nv-1 bars
        CHECK(static_cast<int>(r.bars.size()) >=
              static_cast<int>(r.vertices.size()) - 1);
    }
}

static void test_has_actuator()
{
    std::mt19937 rng(3);
    for (int i = 0; i < 100; ++i) {
        Robot r = RobotFactory::randomRobot(rng);
        CHECK(!r.actuators.empty());
    }
}

static void test_determinism()
{
    // Two separate generators seeded identically must produce the same robot.
    std::mt19937 rng_a(99);
    std::mt19937 rng_b(99);
    Robot a = RobotFactory::randomRobot(rng_a);
    Robot b = RobotFactory::randomRobot(rng_b);

    CHECK(a.vertices.size()  == b.vertices.size());
    CHECK(a.bars.size()      == b.bars.size());
    CHECK(a.neurons.size()   == b.neurons.size());
    CHECK(a.actuators.size() == b.actuators.size());

    for (int i = 0; i < static_cast<int>(a.vertices.size()); ++i)
        CHECK((a.vertices[i].pos - b.vertices[i].pos).norm() < 1e-15);

    for (int i = 0; i < static_cast<int>(a.bars.size()); ++i) {
        CHECK(a.bars[i].v1 == b.bars[i].v1);
        CHECK(a.bars[i].v2 == b.bars[i].v2);
        CHECK(std::abs(a.bars[i].rest_length - b.bars[i].rest_length) < 1e-15);
    }
}

static void test_different_seeds()
{
    std::mt19937 rng_a(1);
    std::mt19937 rng_b(2);
    Robot a = RobotFactory::randomRobot(rng_a);
    Robot b = RobotFactory::randomRobot(rng_b);
    // With overwhelming probability, two random robots differ in topology or geometry.
    bool differ = (a.vertices.size() != b.vertices.size()) ||
                  (a.bars.size()     != b.bars.size())     ||
                  (a.neurons.size()  != b.neurons.size());
    if (!differ && !a.vertices.empty() && !b.vertices.empty())
        differ = (a.vertices[0].pos - b.vertices[0].pos).norm() > 1e-9;
    CHECK(differ);
}

static void test_floor_clearance()
{
    std::mt19937 rng(4);
    for (int i = 0; i < 50; ++i) {
        Robot r = RobotFactory::randomRobot(rng);
        for (const auto& v : r.vertices)
            CHECK(v.pos.z() > 0.0);
    }
}

static void test_bar_lengths()
{
    std::mt19937 rng(5);
    for (int i = 0; i < 50; ++i) {
        Robot r = RobotFactory::randomRobot(rng);
        for (const auto& b : r.bars) {
            CHECK(b.rest_length > 0.0);
            CHECK(b.rest_length < 2.0);   // sanity: no bar > 2 m
            CHECK(b.stiffness > 0.0);
        }
    }
}

static void test_synapse_shape()
{
    std::mt19937 rng(6);
    for (int i = 0; i < 50; ++i) {
        Robot r = RobotFactory::randomRobot(rng);
        for (const auto& n : r.neurons)
            CHECK(n.synapse_weights.size() == static_cast<int>(r.neurons.size()));
    }
}

static void test_stress_valid()
{
    std::mt19937 rng(7);
    int failures = 0;
    for (int i = 0; i < 500; ++i) {
        Robot r = RobotFactory::randomRobot(rng);
        if (!r.isValid()) ++failures;
    }
    CHECK(failures == 0);
    if (failures > 0)
        std::cerr << "    " << failures << "/500 robots failed isValid()\n";
}

// ─────────────────────────────────────────────────────────────────────────────

int main()
{
    std::cout << "=== test_robot_factory ===\n";
    RUN(is_valid);
    RUN(part_counts);
    RUN(has_actuator);
    RUN(determinism);
    RUN(different_seeds);
    RUN(floor_clearance);
    RUN(bar_lengths);
    RUN(synapse_shape);
    RUN(stress_valid);

    if (g_failures == 0)
        std::cout << "\nAll tests passed.\n";
    else
        std::cout << "\n" << g_failures << " test(s) FAILED.\n";

    return g_failures > 0 ? 1 : 0;
}
