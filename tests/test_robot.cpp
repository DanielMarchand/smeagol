/**
 * test_robot.cpp
 *
 * Unit tests for the Robot container class.
 * Covers construction, part mutators, index patching on removal,
 * centerOfMass(), vertexMass(), isValid(), and clone() independence.
 */

#include "Robot.h"
#include "Vertex.h"
#include "Bar.h"
#include "Neuron.h"
#include "Actuator.h"
#include "Materials.h"

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <iostream>

static int g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__            \
                      << "  (" #cond ")\n";                                    \
            ++g_failures;                                                      \
        }                                                                      \
    } while (false)

// ── helpers ───────────────────────────────────────────────────────────────────

/// Build a minimal 2-vertex / 1-bar / 1-neuron / 1-actuator robot.
static Robot make_simple_robot()
{
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(0.1, 0.0, 0.0));   // 10 cm apart
    r.addBar(Bar(0, 1, 0.1));
    r.addNeuron(Neuron(0.5));             // 1 neuron → weight vec size 1
    r.addActuator(Actuator(0, 0, 0.01));
    return r;
}

// ── test functions ────────────────────────────────────────────────────────────

static void test_construction()
{
    Robot r;
    CHECK(r.vertices.empty());
    CHECK(r.bars.empty());
    CHECK(r.neurons.empty());
    CHECK(r.actuators.empty());

    // IDs are unique
    Robot r2;
    CHECK(r2.id != r.id);

    // Explicit ID constructor
    Robot r3(42);
    CHECK(r3.id == 42);
}

static void test_add_parts()
{
    Robot r = make_simple_robot();

    CHECK(r.vertices.size()  == 2);
    CHECK(r.bars.size()      == 1);
    CHECK(r.neurons.size()   == 1);
    CHECK(r.actuators.size() == 1);

    CHECK(r.bars[0].v1 == 0);
    CHECK(r.bars[0].v2 == 1);
    CHECK(r.actuators[0].bar_idx    == 0);
    CHECK(r.actuators[0].neuron_idx == 0);
}

static void test_is_valid()
{
    Robot r = make_simple_robot();
    CHECK(r.isValid());

    // Corrupt a bar index
    r.bars[0].v1 = 99;
    CHECK(!r.isValid());
    r.bars[0].v1 = 0;
    CHECK(r.isValid());

    // Neuron weight vector size mismatch
    r.neurons[0].synapse_weights.conservativeResize(5);
    CHECK(!r.isValid());
    r.neurons[0].synapse_weights.conservativeResize(1);
    CHECK(r.isValid());

    // Duplicate actuator on the same bar must fail validation.
    // (Direct inject bypasses addActuator guard so we can test isValid independently.)
    r.actuators.push_back(Actuator(0, 0, 0.01));  // bar 0 already has one
    CHECK(!r.isValid());
}

static void test_neuron_weight_sizing()
{
    Robot r;
    r.addNeuron(Neuron(0.5));  // neuron 0 → weights size 1
    CHECK(r.neurons[0].synapse_weights.size() == 1);

    r.addNeuron(Neuron(0.3));  // neuron 1 → all neurons get size 2
    CHECK(r.neurons[0].synapse_weights.size() == 2);
    CHECK(r.neurons[1].synapse_weights.size() == 2);

    r.addNeuron(Neuron(0.7));  // neuron 2 → all get size 3
    CHECK(r.neurons[0].synapse_weights.size() == 3);
    CHECK(r.neurons[1].synapse_weights.size() == 3);
    CHECK(r.neurons[2].synapse_weights.size() == 3);

    CHECK(r.isValid());
}

static void test_remove_vertex()
{
    Robot r;
    r.addVertex(Vertex(0, 0, 0));  // idx 0
    r.addVertex(Vertex(1, 0, 0));  // idx 1
    r.addVertex(Vertex(2, 0, 0));  // idx 2
    r.addBar(Bar(0, 1, 1.0));      // bar 0: verts 0-1
    r.addBar(Bar(1, 2, 1.0));      // bar 1: verts 1-2

    // Remove vertex 1 → bars referencing it should disappear
    r.removeVertex(1);
    CHECK(r.vertices.size() == 2);
    // Both bars referenced vertex 1, so both should be gone
    CHECK(r.bars.empty());
}

static void test_remove_bar_patches_actuators()
{
    Robot r = make_simple_robot();
    r.addVertex(Vertex(0.2, 0.0, 0.0));
    r.addBar(Bar(1, 2, 0.1));   // bar 1
    r.addNeuron(Neuron(0.5));   // neuron 1
    r.addActuator(Actuator(1, 1, 0.01));  // references bar 1, neuron 1

    // Remove bar 0 → actuator referencing bar 0 is removed, bar_idx for
    // the remaining actuator should shift from 1 → 0.
    // Vertex 0 is no longer referenced by any bar after this removal, so
    // prune it first before checking isValid().
    r.removeBar(0);
    CHECK(r.bars.size()      == 1);
    CHECK(r.actuators.size() == 1);   // actuator for bar 0 removed
    CHECK(r.actuators[0].bar_idx == 0);  // was bar 1, now bar 0
    r.pruneIsolatedVertices();        // vertex 0 is now isolated; prune it
    CHECK(r.isValid());
}

static void test_remove_neuron_patches()
{
    Robot r;
    r.addNeuron(Neuron(0.5));  // neuron 0
    r.addNeuron(Neuron(0.5));  // neuron 1
    r.addNeuron(Neuron(0.5));  // neuron 2
    r.addVertex(Vertex(0, 0, 0));
    r.addVertex(Vertex(1, 0, 0));
    r.addBar(Bar(0, 1, 1.0));
    r.addActuator(Actuator(0, 2, 0.01));  // references neuron 2

    // Remove neuron 0 → weight vecs shrink from 3→2, neuron_idx 2→1
    r.removeNeuron(0);
    CHECK(r.neurons.size() == 2);
    CHECK(r.neurons[0].synapse_weights.size() == 2);
    CHECK(r.neurons[1].synapse_weights.size() == 2);
    CHECK(r.actuators[0].neuron_idx == 1);
    CHECK(r.isValid());
}

static void test_center_of_mass()
{
    // Two vertices along x, equal bar stiffness/length → symmetric CoM at midpoint
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(1.0, 0.0, 0.0));
    r.addBar(Bar(0, 1, 1.0));

    Eigen::Vector3d com = r.centerOfMass();
    CHECK(std::abs(com.x() - 0.5) < 1e-10);
    CHECK(std::abs(com.y())       < 1e-10);
    CHECK(std::abs(com.z())       < 1e-10);
}

static void test_vertex_mass()
{
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(1.0, 0.0, 0.0));
    const double stiffness = 50000.0;   // N/m  (k = bar.stiffness)
    r.addBar(Bar(0, 1, 1.0, stiffness));

    // mass = rho * (k/E) * L0^2, split equally between two vertices
    const double bar_mass = Materials::rho * (stiffness / Materials::E) * 1.0 * 1.0;
    const double half     = bar_mass / 2.0;

    CHECK(std::abs(r.vertexMass(0) - half) < 1e-10);
    CHECK(std::abs(r.vertexMass(1) - half) < 1e-10);
}

static void test_clone_independence()
{
    Robot original = make_simple_robot();
    Robot copy     = original.clone();

    // Same id (lineage tracking)
    CHECK(copy.id == original.id);

    // Same structure
    CHECK(copy.vertices.size()  == original.vertices.size());
    CHECK(copy.bars.size()      == original.bars.size());

    // Mutating copy does not affect original
    copy.vertices[0].pos.x() = 999.0;
    CHECK(original.vertices[0].pos.x() == 0.0);

    copy.bars[0].rest_length = 999.0;
    CHECK(std::abs(original.bars[0].rest_length - 0.1) < 1e-12);
}

static void test_clear()
{
    Robot r = make_simple_robot();
    r.clear();
    CHECK(r.vertices.empty());
    CHECK(r.bars.empty());
    CHECK(r.neurons.empty());
    CHECK(r.actuators.empty());
    CHECK(r.isValid());
}

static void test_empty_com()
{
    Robot r;
    Eigen::Vector3d com = r.centerOfMass();
    CHECK(com.isZero());
}

static void test_is_connected()
{
    // --- Empty robot (0 vertices): trivially connected ---
    {
        Robot r;
        CHECK(r.isConnected());
        CHECK(r.isValid());
    }

    // --- Single vertex, no bars: trivially connected ---
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        CHECK(r.isConnected());
    }

    // --- Linear chain A-B-C: connected ---
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.1));
        r.addVertex(Vertex(0.1, 0.0, 0.1));
        r.addVertex(Vertex(0.2, 0.0, 0.1));
        r.addBar(Bar(0, 1, 0.1));
        r.addBar(Bar(1, 2, 0.1));
        CHECK(r.isConnected());
        CHECK(r.isValid());
    }

    // --- Triangle (fully connected): connected ---
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.1));
        r.addVertex(Vertex(0.1, 0.0, 0.1));
        r.addVertex(Vertex(0.05, 0.1, 0.1));
        r.addBar(Bar(0, 1, 0.1));
        r.addBar(Bar(1, 2, 0.1));
        r.addBar(Bar(0, 2, 0.1));
        CHECK(r.isConnected());
        CHECK(r.isValid());
    }

    // --- Two disjoint bars A-B and C-D: disconnected ---
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.1));  // 0
        r.addVertex(Vertex(0.1, 0.0, 0.1));  // 1
        r.addVertex(Vertex(1.0, 0.0, 0.1));  // 2  (far away, no connection)
        r.addVertex(Vertex(1.1, 0.0, 0.1));  // 3
        r.addBar(Bar(0, 1, 0.1));  // cluster 1
        r.addBar(Bar(2, 3, 0.1));  // cluster 2 — no link to cluster 1
        CHECK(!r.isConnected());
        CHECK(!r.isValid());  // isValid() must also reject this
    }

    // --- Star topology (all spokes meet at vertex 0): connected ---
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.1));  // hub
        r.addVertex(Vertex(0.1, 0.0, 0.1));
        r.addVertex(Vertex(0.0, 0.1, 0.1));
        r.addVertex(Vertex(-0.1, 0.0, 0.1));
        r.addBar(Bar(0, 1, 0.1));
        r.addBar(Bar(0, 2, 0.1));
        r.addBar(Bar(0, 3, 0.1));
        CHECK(r.isConnected());
    }

    // --- Bridge: two triangles joined by one bar — remove the bridge → split ---
    {
        // Build: 0-1-2 triangle + 3-4-5 triangle, bridged by bar 2-3
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.1));  // 0
        r.addVertex(Vertex(0.1, 0.0, 0.1));  // 1
        r.addVertex(Vertex(0.05, 0.1, 0.1)); // 2
        r.addVertex(Vertex(0.2, 0.0, 0.1));  // 3
        r.addVertex(Vertex(0.3, 0.0, 0.1));  // 4
        r.addVertex(Vertex(0.25, 0.1, 0.1)); // 5
        r.addBar(Bar(0, 1, 0.1));  // triangle 1
        r.addBar(Bar(1, 2, 0.1));
        r.addBar(Bar(0, 2, 0.1));
        r.addBar(Bar(3, 4, 0.1));  // triangle 2
        r.addBar(Bar(4, 5, 0.1));
        r.addBar(Bar(3, 5, 0.1));
        r.addBar(Bar(2, 3, 0.15)); // bridge (index 6)
        CHECK(r.isConnected());

        // Remove the bridge bar — the two triangles become disconnected
        r.removeBar(6);
        CHECK(!r.isConnected());
    }
}

static void test_probe_neural_activity(){
    // --- No actuators: probe returns false regardless of neurons ---
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        r.addVertex(Vertex(0.1, 0.0, 0.0));
        r.addBar(Bar(0, 1, 0.1));
        Neuron n(0.5);
        r.addNeuron(n);
        r.addNeuron(n);
        CHECK(!r.probeNeuralActivity());
    }

    // --- Permanently OFF: single isolated neuron (no weights), stays at 0 ---
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        r.addVertex(Vertex(0.1, 0.0, 0.0));
        r.addBar(Bar(0, 1, 0.1));
        Neuron n(0.5);
        r.addNeuron(n);
        r.addActuator(Actuator(0, 0, 0.01));
        CHECK(!r.probeNeuralActivity());   // stuck OFF
    }

    // --- Permanently ON: self-excitatory neuron, always fires ---
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        r.addVertex(Vertex(0.1, 0.0, 0.0));
        r.addBar(Bar(0, 1, 0.1));
        Neuron n(0.5);
        r.addNeuron(n);
        r.neurons[0].synapse_weights[0] = 1.0;  // self-excitatory
        r.neurons[0].activation = 1.0;           // primed ON
        r.addActuator(Actuator(0, 0, 0.01));
        CHECK(!r.probeNeuralActivity());   // stuck ON
    }

    // --- Anti-phase oscillator (period 2): should be detected as ALIVE ---
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        r.addVertex(Vertex(0.1, 0.0, 0.0));
        r.addBar(Bar(0, 1, 0.1));
        Neuron n0(0.5), n1(0.5);
        n0.activation = 0.0;
        n1.activation = 1.0;
        r.addNeuron(n0);
        r.addNeuron(n1);
        r.neurons[0].synapse_weights << 0.0, 1.0;  // n0 ← n1
        r.neurons[1].synapse_weights << 1.0, 0.0;  // n1 ← n0
        r.addActuator(Actuator(0, 0, 0.01));
        CHECK(r.probeNeuralActivity());    // oscillates: ALIVE
    }

    // --- Actuator driven by a non-oscillating neuron: DEAD even with 2 neurons ---
    // n0 is permanently OFF (default); n1 oscillates but doesn't drive the actuator.
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        r.addVertex(Vertex(0.1, 0.0, 0.0));
        r.addBar(Bar(0, 1, 0.1));
        Neuron n0(0.5), n1(0.5);
        n0.activation = 0.0;  // stays 0 forever (no inputs)
        n1.activation = 1.0;  // will fire, but it's not driving the actuator
        r.addNeuron(n0);
        r.addNeuron(n1);
        // n1 self-excites; n0 has no inputs
        r.neurons[1].synapse_weights[1] = 1.0;
        r.addActuator(Actuator(0, /*neuron_idx=*/0, 0.01));  // driven by n0 (stuck OFF)
        CHECK(!r.probeNeuralActivity());   // actuator neuron is stuck OFF
    }
}

static void test_add_actuator_dedup()
{
    // Adding a second actuator to a bar that already has one must replace
    // the first — the robot must never have two actuators for the same bar.
    Robot r = make_simple_robot();          // bar 0 already has actuator (0,0)
    CHECK(r.actuators.size() == 1);
    CHECK(r.actuators[0].neuron_idx == 0);

    // Re-add with a different neuron_idx (bar 0 again).
    r.addNeuron(Neuron(0.5));               // neuron 1
    const int ret = r.addActuator(Actuator(/*bar*/0, /*neuron*/1, 0.02));
    CHECK(r.actuators.size() == 1);         // still only one
    CHECK(r.actuators[0].bar_idx    == 0);
    CHECK(r.actuators[0].neuron_idx == 1);  // replaced with new one
    CHECK(r.actuators[0].bar_range  == 0.02);
    CHECK(ret == 0);                        // index of the replaced slot
    CHECK(r.isValid());
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    auto run = [](const char* name, void (*fn)()) {
        std::cout << "[ RUN  ] " << name << "\n";
        fn();
    };

    run("Construction",               test_construction);
    run("Add parts",                  test_add_parts);
    run("isValid",                    test_is_valid);
    run("Neuron weight auto-sizing",  test_neuron_weight_sizing);
    run("Remove vertex",              test_remove_vertex);
    run("Remove bar patches actuators", test_remove_bar_patches_actuators);
    run("Remove neuron patches",      test_remove_neuron_patches);
    run("Centre of mass",             test_center_of_mass);
    run("Vertex mass",                test_vertex_mass);
    run("Clone independence",         test_clone_independence);
    run("Clear",                      test_clear);
    run("Empty CoM",                  test_empty_com);
    run("Graph connectivity",         test_is_connected);
    run("Probe neural activity",      test_probe_neural_activity);
    run("addActuator dedup",          test_add_actuator_dedup);

    if (g_failures == 0) {
        std::cout << "\nAll Robot tests passed.\n";
        return 0;
    } else {
        std::cout << "\n" << g_failures << " check(s) FAILED.\n";
        return 1;
    }
}
