/**
 * test_yaml.cpp
 *
 * Tests for Robot::toYAML / fromYAML.
 * Uses a temp file in /tmp so no workspace pollution.
 */

#include "Robot.h"
#include "Vertex.h"
#include "Bar.h"
#include "Neuron.h"
#include "Actuator.h"

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

static int g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__            \
                      << "  (" #cond ")\n";                                    \
            ++g_failures;                                                      \
        }                                                                      \
    } while (false)

#define CHECK_NEAR(a, b, eps) CHECK(std::abs((a) - (b)) < (eps))

// ── helpers ───────────────────────────────────────────────────────────────────

static Robot make_test_robot()
{
    Robot r(7);  // fixed ID so round-trip can assert it

    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(0.1, 0.2, 0.3));
    r.addVertex(Vertex(-0.05, 0.0, 0.15));

    r.addBar(Bar(0, 1, 0.12, 0.008));
    r.addBar(Bar(1, 2, 0.08, 0.010));

    // Two neurons so weight vector has 2 entries each
    Neuron n0(0.6);
    Neuron n1(0.4);
    r.addNeuron(n0);
    r.addNeuron(n1);
    // set some non-trivial weights
    r.neurons[0].synapse_weights << 0.25, -0.75;
    r.neurons[1].synapse_weights << 0.50,  0.10;

    r.addActuator(Actuator(0, 0, 0.009));
    r.addActuator(Actuator(1, 1, 0.005));

    return r;
}

// ── test functions ────────────────────────────────────────────────────────────

static void test_roundtrip_structure()
{
    const std::string path = "/tmp/golem_test_robot_yaml.yaml";

    Robot original = make_test_robot();
    original.toYAML(path);

    Robot loaded = Robot::fromYAML(path);

    CHECK(loaded.id == original.id);

    // vertices
    CHECK(loaded.vertices.size() == original.vertices.size());
    for (std::size_t i = 0; i < original.vertices.size(); ++i) {
        CHECK_NEAR(loaded.vertices[i].pos.x(), original.vertices[i].pos.x(), 1e-12);
        CHECK_NEAR(loaded.vertices[i].pos.y(), original.vertices[i].pos.y(), 1e-12);
        CHECK_NEAR(loaded.vertices[i].pos.z(), original.vertices[i].pos.z(), 1e-12);
    }

    // bars
    CHECK(loaded.bars.size() == original.bars.size());
    for (std::size_t i = 0; i < original.bars.size(); ++i) {
        CHECK(loaded.bars[i].v1 == original.bars[i].v1);
        CHECK(loaded.bars[i].v2 == original.bars[i].v2);
        CHECK_NEAR(loaded.bars[i].rest_length, original.bars[i].rest_length, 1e-12);
        CHECK_NEAR(loaded.bars[i].radius,      original.bars[i].radius,      1e-12);
    }

    // neurons
    CHECK(loaded.neurons.size() == original.neurons.size());
    for (std::size_t i = 0; i < original.neurons.size(); ++i) {
        CHECK_NEAR(loaded.neurons[i].threshold, original.neurons[i].threshold, 1e-12);
        CHECK(loaded.neurons[i].synapse_weights.size() ==
              original.neurons[i].synapse_weights.size());
        for (int j = 0; j < original.neurons[i].synapse_weights.size(); ++j) {
            CHECK_NEAR(loaded.neurons[i].synapse_weights[j],
                       original.neurons[i].synapse_weights[j], 1e-12);
        }
    }

    // actuators
    CHECK(loaded.actuators.size() == original.actuators.size());
    for (std::size_t i = 0; i < original.actuators.size(); ++i) {
        CHECK(loaded.actuators[i].bar_idx    == original.actuators[i].bar_idx);
        CHECK(loaded.actuators[i].neuron_idx == original.actuators[i].neuron_idx);
        CHECK_NEAR(loaded.actuators[i].bar_range, original.actuators[i].bar_range, 1e-12);
    }

    CHECK(loaded.isValid());
    fs::remove(path);
}

static void test_yaml_is_readable_text()
{
    // Serialize, load the raw YAML and spot-check keys are present
    const std::string path = "/tmp/golem_test_robot_keys.yaml";
    Robot r = make_test_robot();
    r.toYAML(path);

    YAML::Node doc = YAML::LoadFile(path);
    CHECK(doc["id"]);
    CHECK(doc["vertices"]);
    CHECK(doc["bars"]);
    CHECK(doc["neurons"]);
    CHECK(doc["actuators"]);

    // Vertex positions stored as sequences
    CHECK(doc["vertices"][0].IsSequence());
    CHECK(doc["vertices"][0].size() == 3);

    // Bar stored as map with expected keys
    CHECK(doc["bars"][0]["v1"]);
    CHECK(doc["bars"][0]["v2"]);
    CHECK(doc["bars"][0]["rest_length"]);
    CHECK(doc["bars"][0]["radius"]);

    // Neuron stored as map with threshold + weights
    CHECK(doc["neurons"][0]["threshold"]);
    CHECK(doc["neurons"][0]["weights"]);
    CHECK(doc["neurons"][0]["weights"].IsSequence());

    // Actuator stored as map
    CHECK(doc["actuators"][0]["bar_idx"]);
    CHECK(doc["actuators"][0]["neuron_idx"]);
    CHECK(doc["actuators"][0]["bar_range"]);

    fs::remove(path);
}

static void test_empty_robot_roundtrip()
{
    const std::string path = "/tmp/golem_test_empty.yaml";
    Robot r(0);
    r.toYAML(path);

    Robot loaded = Robot::fromYAML(path);
    CHECK(loaded.vertices.empty());
    CHECK(loaded.bars.empty());
    CHECK(loaded.neurons.empty());
    CHECK(loaded.actuators.empty());
    CHECK(loaded.isValid());

    fs::remove(path);
}

static void test_bad_path_throws()
{
    Robot r;
    bool threw = false;
    try {
        r.toYAML("/nonexistent_directory/robot.yaml");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    auto run = [](const char* name, void (*fn)()) {
        std::cout << "[ RUN  ] " << name << "\n";
        fn();
    };

    run("Round-trip structure",   test_roundtrip_structure);
    run("YAML has correct keys",  test_yaml_is_readable_text);
    run("Empty robot roundtrip",  test_empty_robot_roundtrip);
    run("Bad path throws",        test_bad_path_throws);

    if (g_failures == 0) {
        std::cout << "\nAll YAML tests passed.\n";
        return 0;
    } else {
        std::cout << "\n" << g_failures << " check(s) FAILED.\n";
        return 1;
    }
}
