/**
 * test_robot_parts.cpp
 *
 * Unit tests for RobotPart and its four concrete subclasses.
 * Checks construction, data access, type tags, clone(), and derived physics.
 */

#include "RobotPart.h"
#include "Vertex.h"
#include "Bar.h"
#include "Neuron.h"
#include "Actuator.h"
#include "Materials.h"

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

// ── helpers ───────────────────────────────────────────────────────────────────

static int g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__            \
                      << "  (" #cond ")\n";                                    \
            ++g_failures;                                                      \
        }                                                                      \
    } while (false)

// ── individual test functions ─────────────────────────────────────────────────

static void test_materials()
{
    CHECK(Materials::E       > 0.0);
    CHECK(Materials::rho     > 0.0);
    CHECK(Materials::S_yield > 0.0);
    CHECK(Materials::g       > 0.0);

    // Values from the paper
    CHECK(std::abs(Materials::E       - 0.896e9) < 1.0);
    CHECK(std::abs(Materials::rho     - 1000.0)  < 1e-9);
    CHECK(std::abs(Materials::S_yield - 19.0e6)  < 1.0);
}

static void test_vertex()
{
    Vertex v(1.0, 2.0, 3.0);

    CHECK(v.type()     == RobotPart::Type::Vertex);
    CHECK(v.typeName() == "vertex");
    CHECK(v.pos.x()    == 1.0);
    CHECK(v.pos.y()    == 2.0);
    CHECK(v.pos.z()    == 3.0);

    // Default constructor
    Vertex v0;
    CHECK(v0.pos.isZero());

    // Eigen::Vector3d constructor
    Vertex v1(Eigen::Vector3d(4.0, 5.0, 6.0));
    CHECK(v1.pos.x() == 4.0);

    // clone() produces independent copy
    auto cloned = v.clone();
    CHECK(cloned->type() == RobotPart::Type::Vertex);
    auto* cv = static_cast<Vertex*>(cloned.get());
    cv->pos.x() = 99.0;
    CHECK(v.pos.x() == 1.0);  // original unaffected
}

static void test_bar()
{
    const double len      = 0.2;      // 20 cm
    const double stiffness = 50000.0; // N/m
    Bar b(0, 1, len, stiffness);

    CHECK(b.type()     == RobotPart::Type::Bar);
    CHECK(b.typeName() == "bar");
    CHECK(b.v1          == 0);
    CHECK(b.v2          == 1);
    CHECK(b.rest_length == len);
    CHECK(b.stiffness   == stiffness);

    // default-constructed bar should use k_default
    Bar b_default(0, 1, 1.0);
    CHECK(b_default.stiffness == Materials::k_default);

    // clone()
    auto cloned = b.clone();
    CHECK(cloned->type() == RobotPart::Type::Bar);
    auto* cb = static_cast<Bar*>(cloned.get());
    cb->rest_length = 999.0;
    CHECK(b.rest_length == len);  // original unaffected
}

static void test_neuron()
{
    Eigen::VectorXd weights(3);
    weights << 0.1, -0.3, 0.7;

    Neuron n(0.6, weights);

    CHECK(n.type()                   == RobotPart::Type::Neuron);
    CHECK(n.typeName()               == "neuron");
    CHECK(n.threshold                == 0.6);
    CHECK(n.synapse_weights.size()   == 3);
    CHECK(n.synapse_weights[0]       == 0.1);
    CHECK(n.synapse_weights[1]       == -0.3);
    CHECK(n.synapse_weights[2]       == 0.7);
    CHECK(n.activation               == 0.0);

    // Default constructor: zero-length weights
    Neuron n0;
    CHECK(n0.threshold               == 0.5);
    CHECK(n0.synapse_weights.size()  == 0);

    // dot-product works (the key operation in the neural tick)
    Eigen::VectorXd activations(3);
    activations << 1.0, 0.0, 1.0;
    double wsum = n.synapse_weights.dot(activations);
    CHECK(std::abs(wsum - (0.1 + 0.7)) < 1e-12);

    // conservativeResize preserves existing values
    n.synapse_weights.conservativeResize(4);
    CHECK(n.synapse_weights.size()   == 4);
    CHECK(n.synapse_weights[0]       == 0.1);

    // clone() produces independent copy
    Neuron n2(0.6, weights);
    auto cloned = n2.clone();
    CHECK(cloned->type() == RobotPart::Type::Neuron);
    auto* cn = static_cast<Neuron*>(cloned.get());
    cn->threshold = 999.0;
    CHECK(n2.threshold == 0.6);  // original unaffected
}

static void test_actuator()
{
    Actuator a(2, 3, 0.01);

    CHECK(a.type()       == RobotPart::Type::Actuator);
    CHECK(a.typeName()   == "actuator");
    CHECK(a.bar_idx      == 2);
    CHECK(a.neuron_idx   == 3);
    CHECK(a.bar_range    == 0.01);

    // Default bar_range
    Actuator a0(0, 0);
    CHECK(a0.bar_range == 0.01);

    // clone()
    auto cloned = a.clone();
    CHECK(cloned->type() == RobotPart::Type::Actuator);
    auto* ca = static_cast<Actuator*>(cloned.get());
    ca->bar_idx = 999;
    CHECK(a.bar_idx == 2);  // original unaffected
}

static void test_polymorphism()
{
    // Store all part types behind the base-class pointer
    std::vector<std::unique_ptr<RobotPart>> parts;
    parts.push_back(std::make_unique<Vertex>(0.0, 0.0, 0.0));
    parts.push_back(std::make_unique<Bar>(0, 1, 0.1));
    parts.push_back(std::make_unique<Neuron>(0.5));
    parts.push_back(std::make_unique<Actuator>(0, 0));

    CHECK(parts[0]->type() == RobotPart::Type::Vertex);
    CHECK(parts[1]->type() == RobotPart::Type::Bar);
    CHECK(parts[2]->type() == RobotPart::Type::Neuron);
    CHECK(parts[3]->type() == RobotPart::Type::Actuator);

    // Clone the whole heterogeneous list
    std::vector<std::unique_ptr<RobotPart>> copies;
    for (const auto& p : parts)
        copies.push_back(p->clone());

    for (size_t i = 0; i < parts.size(); ++i)
        CHECK(copies[i]->type() == parts[i]->type());
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    auto run = [](const char* name, void (*fn)()) {
        std::cout << "[ RUN  ] " << name << "\n";
        fn();
    };

    run("Materials constants",  test_materials);
    run("Vertex",               test_vertex);
    run("Bar",                  test_bar);
    run("Neuron",               test_neuron);
    run("Actuator",             test_actuator);
    run("Polymorphism / clone", test_polymorphism);

    if (g_failures == 0) {
        std::cout << "\nAll RobotPart tests passed.\n";
        return 0;
    } else {
        std::cout << "\n" << g_failures << " check(s) FAILED.\n";
        return 1;
    }
}
