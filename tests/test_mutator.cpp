/**
 * test_mutator.cpp
 *
 * Unit tests for the Mutator class.  Each of the 7 sub-tests runs the
 * relevant operator hundreds of times and verifies:
 *   - robot.isValid() after every call
 *   - all cross-reference indices stay in-bounds
 *   - structural changes actually occur (counts grow/shrink)
 */

#include "Mutator.h"
#include "Robot.h"
#include "Bar.h"
#include "Vertex.h"
#include "Neuron.h"
#include "Actuator.h"
#include "Materials.h"

#include <Eigen/Core>
#include <cassert>
#include <iostream>
#include <random>

static int g_failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__              \
                      << "  (" #cond ")\n";                                      \
            ++g_failures;                                                        \
        }                                                                        \
    } while (false)

// ── shared seed robot (tetrahedron + 2-neuron oscillator + 1 actuator) ────────

static Robot make_seed_robot()
{
    Robot r;
    r.addVertex(Vertex(0.00, 0.00, 0.00));
    r.addVertex(Vertex(0.20, 0.00, 0.00));
    r.addVertex(Vertex(0.10, 0.20, 0.00));
    r.addVertex(Vertex(0.10, 0.10, 0.20));

    r.addBar(Bar(0, 1, 0.200));
    r.addBar(Bar(0, 2, 0.224));
    r.addBar(Bar(1, 2, 0.224));
    r.addBar(Bar(0, 3, 0.245));
    r.addBar(Bar(1, 3, 0.245));
    r.addBar(Bar(2, 3, 0.224));

    Eigen::VectorXd w0 = Eigen::VectorXd::Zero(0);
    r.addNeuron(Neuron(0.5, w0));
    r.addNeuron(Neuron(0.5, w0));

    r.addActuator(Actuator(3, 0, 0.01));
    return r;
}

// ── sub-tests ─────────────────────────────────────────────────────────────────

static void test_perturb_bar()
{
    std::mt19937 rng(42);
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(0.1, 0.0, 0.0));
    r.addBar(Bar(0, 1, 0.1));

    for (int i = 0; i < 1000; ++i) {
        Robot copy = r.clone();
        Mutator::perturbElement(copy, rng);
        CHECK(copy.isValid());
        CHECK(copy.bars[0].rest_length >= 0.01);
        CHECK(copy.bars[0].rest_length <= 1.0);
    }
    std::cout << "PASS  test_perturb_bar\n";
}

static void test_perturb_neuron()
{
    std::mt19937 rng(42);
    Robot r;
    Eigen::VectorXd w = Eigen::VectorXd::Zero(0);
    r.addNeuron(Neuron(0.5, w));
    r.addNeuron(Neuron(0.5, w));

    for (int i = 0; i < 1000; ++i) {
        Robot copy = r.clone();
        Mutator::perturbElement(copy, rng);
        CHECK(copy.isValid());
        for (const auto& n : copy.neurons) {
            CHECK(n.threshold >= 0.0);
            CHECK(n.threshold <= 2.0);
        }
    }
    std::cout << "PASS  test_perturb_neuron\n";
}

static void test_add_bar()
{
    std::mt19937 rng(42);

    // Strategy A: zero vertices – bar can only be added via new-vertex strategy
    {
        Robot base;  // intentionally empty
        bool got_bar = false;
        for (int i = 0; i < 500; ++i) {
            Robot copy = base.clone();
            Mutator::addRemoveElement(copy, rng);
            CHECK(copy.isValid());
            if (!copy.bars.empty()) got_bar = true;
        }
        CHECK(got_bar);
    }

    // Strategy B: two vertices, no bars – bar can be added via existing-vertices
    {
        Robot base;
        base.addVertex(Vertex(0.0, 0.0, 0.0));
        base.addVertex(Vertex(0.1, 0.0, 0.0));
        bool got_bar = false;
        for (int i = 0; i < 500; ++i) {
            Robot copy = base.clone();
            Mutator::addRemoveElement(copy, rng);
            CHECK(copy.isValid());
            if (!copy.bars.empty()) {
                got_bar = true;
                // All bar vertex indices must be in-bounds and distinct
                for (const auto& b : copy.bars)
                    CHECK(b.v1 != b.v2);
            }
        }
        CHECK(got_bar);
    }
    std::cout << "PASS  test_add_bar\n";
}

static void test_remove_bar()
{
    std::mt19937 rng(42);
    // 3 bars, 1 neuron, 2 actuators (bar 0 and bar 2)
    Robot r;
    r.addVertex(Vertex(0.00, 0.0, 0.0));
    r.addVertex(Vertex(0.10, 0.0, 0.0));
    r.addVertex(Vertex(0.20, 0.0, 0.0));
    r.addVertex(Vertex(0.15, 0.1, 0.0));
    r.addBar(Bar(0, 1, 0.10));
    r.addBar(Bar(1, 2, 0.10));
    r.addBar(Bar(0, 3, 0.18));
    Eigen::VectorXd w = Eigen::VectorXd::Zero(0);
    r.addNeuron(Neuron(0.5, w));
    r.addActuator(Actuator(0, 0, 0.01));
    r.addActuator(Actuator(2, 0, 0.01));

    for (int i = 0; i < 100; ++i) {
        Robot copy = r.clone();
        Mutator::addRemoveElement(copy, rng);
        CHECK(copy.isValid());
        for (const auto& a : copy.actuators) {
            CHECK(a.bar_idx    >= 0 && a.bar_idx    < static_cast<int>(copy.bars.size()));
            CHECK(a.neuron_idx >= 0 && a.neuron_idx < static_cast<int>(copy.neurons.size()));
        }
    }
    std::cout << "PASS  test_remove_bar\n";
}

static void test_split_vertex()
{
    std::mt19937 rng(42);
    Robot r;
    r.addVertex(Vertex(0.00, 0.00, 0.00));
    r.addVertex(Vertex(0.10, 0.00, 0.00));
    r.addVertex(Vertex(0.05, 0.10, 0.00));
    r.addVertex(Vertex(0.05, 0.05, 0.10));
    r.addBar(Bar(0, 1, 0.100));
    r.addBar(Bar(0, 2, 0.111));
    r.addBar(Bar(1, 2, 0.111));
    r.addBar(Bar(0, 3, 0.122));

    const int initial_verts = static_cast<int>(r.vertices.size());
    bool vertex_count_grew = false;

    for (int i = 0; i < 100; ++i) {
        Robot copy = r.clone();
        Mutator::splitElement(copy, rng);
        CHECK(copy.isValid());
        // Any split always adds exactly one new vertex
        if (static_cast<int>(copy.vertices.size()) > initial_verts)
            vertex_count_grew = true;
    }
    CHECK(vertex_count_grew);
    std::cout << "PASS  test_split_vertex\n";
}

static void test_split_bar()
{
    std::mt19937 rng(42);
    Robot r = make_seed_robot();  // 6 bars, 1 actuator on bar 3
    const int initial_bars = static_cast<int>(r.bars.size());
    bool bar_count_grew = false;

    for (int i = 0; i < 100; ++i) {
        Robot copy = r.clone();
        Mutator::splitElement(copy, rng);
        CHECK(copy.isValid());
        if (static_cast<int>(copy.bars.size()) > initial_bars)
            bar_count_grew = true;
        for (const auto& a : copy.actuators) {
            CHECK(a.bar_idx    >= 0 && a.bar_idx    < static_cast<int>(copy.bars.size()));
            CHECK(a.neuron_idx >= 0 && a.neuron_idx < static_cast<int>(copy.neurons.size()));
        }
    }
    CHECK(bar_count_grew);
    std::cout << "PASS  test_split_bar\n";
}

static void test_attach_detach()
{
    std::mt19937 rng(42);
    Robot r = make_seed_robot();  // 6 bars, 2 neurons, 1 actuator (bar 3 → neuron 0)

    for (int i = 0; i < 500; ++i) {
        Robot copy = r.clone();
        Mutator::attachDetach(copy, rng);
        CHECK(copy.isValid());
        for (const auto& a : copy.actuators) {
            CHECK(a.bar_idx    >= 0 && a.bar_idx    < static_cast<int>(copy.bars.size()));
            CHECK(a.neuron_idx >= 0 && a.neuron_idx < static_cast<int>(copy.neurons.size()));
        }
    }
    // Flip-toggle: bar 3 is actuated → calling attachDetach on the seed with
    // seed rng=99 should sometimes produce 0 actuators (detach) and sometimes
    // more (another bar gets attached).
    bool ever_zero = false, ever_two = false;
    std::mt19937 rng2(99);
    for (int i = 0; i < 200; ++i) {
        Robot copy = r.clone();
        Mutator::attachDetach(copy, rng2);
        if (copy.actuators.empty())  ever_zero = true;
        if (copy.actuators.size() > 1) ever_two  = true;
    }
    CHECK(ever_zero || ever_two);   // at least one direction must have fired
    std::cout << "PASS  test_attach_detach\n";
}

static void test_add_neuron_connected()
{
    std::mt19937 rng(77);

    // Robot with one bar but no neurons.  Call addRemoveElement until a
    // neuron is added; verify that the NEW neuron is immediately connected
    // (≥1 nonzero synapse weight OR an actuator pointing to it).
    Robot base;
    base.addVertex(Vertex(0.0, 0.0, 0.0));
    base.addVertex(Vertex(0.2, 0.0, 0.0));
    base.addBar(Bar(0, 1, 0.2));

    int verified = 0;
    for (int i = 0; i < 1000 && verified < 50; ++i) {
        Robot copy = base.clone();
        const int n_before = static_cast<int>(copy.neurons.size());
        Mutator::addRemoveElement(copy, rng);
        CHECK(copy.isValid());

        if (static_cast<int>(copy.neurons.size()) > n_before) {
            // A neuron was added; check the newest one (highest index)
            const int new_idx = static_cast<int>(copy.neurons.size()) - 1;
            const auto& n     = copy.neurons[new_idx];
            bool connected = false;
            for (int k = 0; k < n.synapse_weights.size(); ++k)
                if (std::abs(n.synapse_weights[k]) > 1e-9) { connected = true; break; }
            if (!connected) {
                for (const auto& a : copy.actuators)
                    if (a.neuron_idx == new_idx) { connected = true; break; }
            }
            CHECK(connected);
            ++verified;
        }
    }
    CHECK(verified >= 10);   // must have observed the "add neuron" path at least 10×
    std::cout << "PASS  test_add_neuron_connected\n";
}

static void test_rewire_neuron()
{
    std::mt19937 rng(55);
    // Build robot with 2 bars, 2 neurons, 1 actuator
    Robot r;
    r.addVertex(Vertex(0.00, 0.0, 0.0));
    r.addVertex(Vertex(0.10, 0.0, 0.0));
    r.addVertex(Vertex(0.20, 0.0, 0.0));
    r.addBar(Bar(0, 1, 0.10));
    r.addBar(Bar(1, 2, 0.10));
    Eigen::VectorXd w0 = Eigen::VectorXd::Zero(0);
    r.addNeuron(Neuron(0.5, w0));
    r.addNeuron(Neuron(0.5, w0));
    r.addActuator(Actuator(0, 0, 0.01));

    bool bar_changed    = false;
    bool neuron_changed = false;

    for (int i = 0; i < 500; ++i) {
        Robot copy = r.clone();
        bool changed = Mutator::rewireNeuron(copy, rng);
        CHECK(copy.isValid());
        CHECK(changed);
        for (const auto& a : copy.actuators) {
            CHECK(a.bar_idx    >= 0 && a.bar_idx    < static_cast<int>(copy.bars.size()));
            CHECK(a.neuron_idx >= 0 && a.neuron_idx < static_cast<int>(copy.neurons.size()));
        }
        if (copy.actuators[0].bar_idx    != r.actuators[0].bar_idx)    bar_changed    = true;
        if (copy.actuators[0].neuron_idx != r.actuators[0].neuron_idx) neuron_changed = true;
    }
    CHECK(bar_changed);
    CHECK(neuron_changed);
    std::cout << "PASS  test_rewire_neuron\n";
}

// ── MutatorParams YAML round-trip ─────────────────────────────────────────────

static void test_mutator_params_yaml()
{
    // Build a non-default MutatorParams.
    MutatorParams mp;
    mp.p_perturb            = 0.20;
    mp.p_add_remove         = 0.05;
    mp.perturb_bar_frac     = 0.25;
    mp.bar_length_min       = 0.02;
    mp.new_synapse_weight_max = 2.0;

    // Serialise to a YAML string.
    YAML::Emitter out;
    out << YAML::BeginMap;
    mp.toYAML(out);
    out << YAML::EndMap;
    CHECK(out.good());

    // Parse back and round-trip check.
    const YAML::Node node = YAML::Load(out.c_str());
    const MutatorParams mp2 = MutatorParams::fromYAML(node);

    CHECK(mp2.p_perturb            == mp.p_perturb);
    CHECK(mp2.p_add_remove         == mp.p_add_remove);
    CHECK(mp2.p_split              == mp.p_split);        // default preserved
    CHECK(mp2.perturb_bar_frac     == mp.perturb_bar_frac);
    CHECK(mp2.bar_length_min       == mp.bar_length_min);
    CHECK(mp2.new_synapse_weight_max == mp.new_synapse_weight_max);

    // fromYAML on an empty node must produce defaults.
    const YAML::Node empty = YAML::Load("{}");
    const MutatorParams def = MutatorParams::fromYAML(empty);
    CHECK(def.p_perturb    == 0.10);
    CHECK(def.p_add_remove == 0.01);
    CHECK(def.p_split      == 0.03);

    std::cout << "PASS  test_mutator_params_yaml\n";
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    test_perturb_bar();
    test_perturb_neuron();
    test_add_bar();
    test_remove_bar();
    test_split_vertex();
    test_split_bar();
    test_attach_detach();
    test_add_neuron_connected();
    test_rewire_neuron();
    test_mutator_params_yaml();

    if (g_failures == 0) {
        std::cout << "\nAll mutator tests passed.\n";
        return 0;
    }
    std::cerr << "\n" << g_failures << " test(s) FAILED.\n";
    return 1;
}
