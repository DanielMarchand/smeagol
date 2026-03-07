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

    // addBarNew (Strategy A): zero vertices – bootstraps from scratch.
    {
        Robot base;
        bool got_bar = false;
        for (int i = 0; i < 200; ++i) {
            Robot copy = base.clone();
            Mutator::addBarNew(copy, rng);
            CHECK(copy.isValid());
            if (!copy.bars.empty()) got_bar = true;
        }
        CHECK(got_bar);
    }

    // addBarNew (Strategy A): existing robot – always adds a new vertex + bar.
    {
        Robot base = make_seed_robot();
        const int b_before = static_cast<int>(base.bars.size());
        for (int i = 0; i < 100; ++i) {
            Robot copy = base.clone();
            Mutator::addBarNew(copy, rng);
            CHECK(copy.isValid());
            CHECK(static_cast<int>(copy.bars.size()) == b_before + 1);
        }
    }

    // addBarBridge (Strategy B): chain (0-1-2) with (0,2) pair missing.
    // Must close the triangle when called.
    {
        Robot base;
        base.addVertex(Vertex(0.0, 0.0, 0.0));
        base.addVertex(Vertex(0.1, 0.0, 0.0));
        base.addVertex(Vertex(0.2, 0.0, 0.0));
        base.addBar(Bar(0, 1, 0.1));
        base.addBar(Bar(1, 2, 0.1));
        bool got_bar = false;
        for (int i = 0; i < 200; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::addBarBridge(copy, rng);
            CHECK(copy.isValid());
            if (ok && copy.bars.size() > base.bars.size()) {
                got_bar = true;
                for (const auto& b : copy.bars)
                    CHECK(b.v1 != b.v2);
            }
        }
        CHECK(got_bar);
    }

    // addBarBridge on a fully-connected graph returns false.
    {
        Robot base;
        base.addVertex(Vertex(0.0, 0.0, 0.0));
        base.addVertex(Vertex(0.1, 0.0, 0.0));
        base.addBar(Bar(0, 1, 0.1));  // only pair is already connected
        Robot copy = base.clone();
        bool ok = Mutator::addBarBridge(copy, rng);
        CHECK(!ok);
        CHECK(copy.bars.size() == base.bars.size());
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
        Mutator::removeBar(copy, rng);
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

static void test_attach_neuron()
{
    std::mt19937 rng(42);
    Robot r = make_seed_robot();  // 6 bars, 2 neurons, 1 actuator (bar 3 → neuron 0)

    // attachNeuron only attaches: actuator count never decreases.
    for (int i = 0; i < 200; ++i) {
        Robot copy = r.clone();
        const int a_before = static_cast<int>(copy.actuators.size());
        bool ok = Mutator::attachNeuron(copy, rng);
        CHECK(copy.isValid());
        if (ok) {
            CHECK(static_cast<int>(copy.actuators.size()) > a_before);
        }
        for (const auto& a : copy.actuators) {
            CHECK(a.bar_idx    >= 0 && a.bar_idx    < static_cast<int>(copy.bars.size()));
            CHECK(a.neuron_idx >= 0 && a.neuron_idx < static_cast<int>(copy.neurons.size()));
        }
    }

    // Returns false when all bars are already actuated.
    Robot fully_wired = make_seed_robot();
    for (int b = 0; b < static_cast<int>(fully_wired.bars.size()); ++b)
        fully_wired.addActuator(Actuator(b, 0, 0.01));
    bool result = Mutator::attachNeuron(fully_wired, rng);
    CHECK(!result);   // all bars actuated — must return false

    std::cout << "PASS  test_attach_neuron\n";
}

static void test_detach_neuron()
{
    std::mt19937 rng(77);

    // Returns false when no actuated bars.
    Robot empty_acts = make_seed_robot();
    // Remove all actuators from seed robot.
    while (!empty_acts.actuators.empty())
        empty_acts.removeActuator(0);
    bool result = Mutator::detachNeuron(empty_acts, rng);
    CHECK(!result);   // no actuated bars — must return false

    // Apply detachNeuron repeatedly to a robot with all bars actuated;
    // robot should eventually become fully structural (0 actuators).
    Robot r = make_seed_robot();
    // Clear existing actuators, then attach exactly one per bar.
    while (!r.actuators.empty()) r.removeActuator(0);
    for (int b = 0; b < static_cast<int>(r.bars.size()); ++b)
        r.addActuator(Actuator(b, 0, 0.01));
    CHECK(r.actuators.size() == static_cast<size_t>(r.bars.size()));
    int prev_size = static_cast<int>(r.actuators.size());
    for (int i = 0; i < 100; ++i) {
        if (r.actuators.empty()) break;
        bool ok = Mutator::detachNeuron(r, rng);
        CHECK(ok);
        CHECK(static_cast<int>(r.actuators.size()) < prev_size);
        prev_size = static_cast<int>(r.actuators.size());
        CHECK(r.isValid());
    }
    CHECK(r.actuators.empty());   // all bars should be unactuated after N iterations

    std::cout << "PASS  test_detach_neuron\n";
}

static void test_add_neuron_connected()
{
    std::mt19937 rng(77);

    // Robot with one bar but no neurons.  addNeuron() must always produce a
    // neuron that is immediately connected (≥1 nonzero synapse weight OR an
    // actuator pointing to it).
    Robot base;
    base.addVertex(Vertex(0.0, 0.0, 0.0));
    base.addVertex(Vertex(0.2, 0.0, 0.0));
    base.addBar(Bar(0, 1, 0.2));

    for (int i = 0; i < 50; ++i) {
        Robot copy = base.clone();
        const int n_before = static_cast<int>(copy.neurons.size());
        Mutator::addNeuron(copy, rng);
        CHECK(copy.isValid());
        CHECK(static_cast<int>(copy.neurons.size()) == n_before + 1);

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
    }
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

// ── Shared topology helpers ───────────────────────────────────────────────────

// Per-vertex degree (number of bars incident to each vertex).
static std::vector<int> computeDegrees(const Robot& r)
{
    std::vector<int> deg(r.vertices.size(), 0);
    for (const auto& b : r.bars) { ++deg[b.v1]; ++deg[b.v2]; }
    return deg;
}

// Caterpillar: triangle {v0,v1,v2} with leaf v3 off v0 and leaf v4 off v1.
// Degrees: v0=3, v1=3, v2=2, v3=1, v4=1.
// removeBarEdge candidates: bars b3 (v0–v3) and b4 (v1–v4).
static Robot make_caterpillar()
{
    Robot r;
    r.addVertex(Vertex( 0.00, 0.00, 0.00));   // v0
    r.addVertex(Vertex( 0.20, 0.00, 0.00));   // v1
    r.addVertex(Vertex( 0.10, 0.20, 0.00));   // v2
    r.addVertex(Vertex(-0.10, 0.00, 0.00));   // v3 — leaf off v0
    r.addVertex(Vertex( 0.30, 0.00, 0.00));   // v4 — leaf off v1
    r.addBar(Bar(0, 1, 0.200));  // b0 interior
    r.addBar(Bar(1, 2, 0.224));  // b1 interior
    r.addBar(Bar(0, 2, 0.224));  // b2 interior
    r.addBar(Bar(0, 3, 0.100));  // b3 leaf (v3 degree-1)
    r.addBar(Bar(1, 4, 0.100));  // b4 leaf (v4 degree-1)
    return r;
}

// Path graph v0–v1–v2–v3: a tree; every bar is a structural bridge.
// removeBarBridge must return false here.
static Robot make_path_graph()
{
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(0.1, 0.0, 0.0));
    r.addVertex(Vertex(0.2, 0.0, 0.0));
    r.addVertex(Vertex(0.3, 0.0, 0.0));
    r.addBar(Bar(0, 1, 0.1));
    r.addBar(Bar(1, 2, 0.1));
    r.addBar(Bar(2, 3, 0.1));
    return r;
}

// Diamond: v0–v1–v2 and v0–v3–v2 (two independent paths, no bridges).
// All vertices have degree 2.
// joinElement candidates: v1 (would merge v0↔v2) and v3 (same endpoints).
static Robot make_diamond()
{
    Robot r;
    r.addVertex(Vertex(0.00, 0.00, 0.00));   // v0
    r.addVertex(Vertex(0.10, 0.00, 0.00));   // v1 — midpoint on lower path
    r.addVertex(Vertex(0.20, 0.00, 0.00));   // v2
    r.addVertex(Vertex(0.10, 0.20, 0.00));   // v3 — midpoint on upper path
    r.addBar(Bar(0, 1, 0.100));
    r.addBar(Bar(1, 2, 0.100));
    r.addBar(Bar(0, 3, 0.224));
    r.addBar(Bar(3, 2, 0.224));
    return r;
}

// Square with diagonal v0–v2.
// Degree-2 vertices v1 and v3 both have endpoints {v0, v2} which already share
// the diagonal bar → joinElement must reject every candidate → return false.
static Robot make_square_with_diagonal()
{
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));   // v0
    r.addVertex(Vertex(0.1, 0.0, 0.0));   // v1
    r.addVertex(Vertex(0.1, 0.1, 0.0));   // v2
    r.addVertex(Vertex(0.0, 0.1, 0.0));   // v3
    r.addBar(Bar(0, 1, 0.100));
    r.addBar(Bar(1, 2, 0.100));
    r.addBar(Bar(2, 3, 0.100));
    r.addBar(Bar(3, 0, 0.100));
    r.addBar(Bar(0, 2, 0.141));   // diagonal — v0 and v2 are already connected
    return r;
}

// ── test_remove_bar_edge ──────────────────────────────────────────────────────
//
// removeBarEdge is the reverse of addBarNew (Strategy A).
// It removes a leaf bar (one endpoint has degree 1) and prunes the isolated
// leaf vertex.  Net: –1 bar, –1 vertex; the rest of the graph is untouched.

static void test_remove_bar_edge()
{
    std::mt19937 rng(42);

    // ── 1. Basic: caterpillar always has leaf bars; succeeds every time ───
    //    Each call must remove exactly –1 bar and –1 vertex; graph stays
    //    valid and connected.
    {
        const Robot base = make_caterpillar();
        const int v0 = (int)base.vertices.size();   // 5
        const int b0 = (int)base.bars.size();        // 5
        for (int i = 0; i < 100; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::removeBarEdge(copy, rng);
            CHECK(ok);
            CHECK(copy.isValid());
            CHECK(copy.isConnected());
            CHECK((int)copy.vertices.size() == v0 - 1);
            CHECK((int)copy.bars.size()     == b0 - 1);
        }
    }

    // ── 2. Triangle (all degree ≥ 2): no leaf bars → must return false ────
    {
        Robot tri;
        tri.addVertex(Vertex(0.00, 0.00, 0.0));
        tri.addVertex(Vertex(0.10, 0.00, 0.0));
        tri.addVertex(Vertex(0.05, 0.10, 0.0));
        tri.addBar(Bar(0, 1, 0.100));
        tri.addBar(Bar(1, 2, 0.112));
        tri.addBar(Bar(0, 2, 0.112));
        Robot copy = tri.clone();
        bool ok = Mutator::removeBarEdge(copy, rng);
        CHECK(!ok);
        CHECK((int)copy.bars.size()     == (int)tri.bars.size());
        CHECK((int)copy.vertices.size() == (int)tri.vertices.size());
    }

    // ── 3. PATHOLOGY — single-bar robot ──────────────────────────────────
    //    Removing the only bar yields an empty robot (0 bars, 0 vertices).
    //    An empty robot is technically isValid() = true but is degenerate.
    //    Expected behaviour: guard fires → return false, robot unchanged.
    //    THIS TEST WILL FAIL on the current implementation (guard missing).
    {
        Robot single;
        single.addVertex(Vertex(0.0, 0.0, 0.0));
        single.addVertex(Vertex(0.1, 0.0, 0.0));
        single.addBar(Bar(0, 1, 0.1));
        Robot copy = single.clone();
        bool ok = Mutator::removeBarEdge(copy, rng);
        CHECK(!ok);                             // ← FAILS today
        CHECK((int)copy.bars.size()     == 1);  // ← FAILS today
        CHECK((int)copy.vertices.size() == 2);  // ← FAILS today
    }

    // ── 4. Actuators on the leaf bar are cleaned up after removal ─────────
    //    Attach an actuator to b3 (the v0–v3 leaf bar, index 3).
    //    After any successful removeBarEdge, all actuator refs must be valid.
    {
        Robot base = make_caterpillar();
        Eigen::VectorXd w = Eigen::VectorXd::Zero(0);
        base.addNeuron(Neuron(0.5, w));
        base.addActuator(Actuator(3, 0, 0.01));   // actuator on leaf bar b3
        for (int i = 0; i < 60; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::removeBarEdge(copy, rng);
            if (!ok) continue;
            CHECK(copy.isValid());
            const int nb = (int)copy.bars.size();
            const int nn = (int)copy.neurons.size();
            for (const auto& a : copy.actuators) {
                CHECK(a.bar_idx    >= 0 && a.bar_idx    < nb);
                CHECK(a.neuron_idx >= 0 && a.neuron_idx < nn);
            }
        }
    }

    // ── 5. No isolated vertices survive ──────────────────────────────────
    //    pruneIsolatedVertices should always run; every remaining vertex
    //    must have degree ≥ 1 in the result.
    {
        const Robot base = make_caterpillar();
        for (int i = 0; i < 50; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::removeBarEdge(copy, rng);
            CHECK(ok);   // caterpillar always has candidates
            const auto deg = computeDegrees(copy);
            for (int d : deg) CHECK(d >= 1);
        }
    }

    std::cout << "PASS  test_remove_bar_edge\n";
}

// ── test_remove_bar_bridge ────────────────────────────────────────────────────
//
// removeBarBridge is the reverse of addBarBridge (Strategy B).
// It removes a bar whose both endpoints have degree ≥ 2 and whose removal
// does not disconnect the graph.  Net: –1 bar, 0 vertex change.

static void test_remove_bar_bridge()
{
    std::mt19937 rng(77);

    // ── 1. Basic: diamond (no bridges, all degree 2) → always succeeds ────
    //    Each call removes exactly –1 bar; vertex count unchanged; valid.
    {
        const Robot base = make_diamond();
        const int v0 = (int)base.vertices.size();   // 4
        const int b0 = (int)base.bars.size();        // 4
        for (int i = 0; i < 100; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::removeBarBridge(copy, rng);
            CHECK(ok);
            CHECK(copy.isValid());
            CHECK(copy.isConnected());
            CHECK((int)copy.vertices.size() == v0);       // no vertex removed
            CHECK((int)copy.bars.size()     == b0 - 1);   // exactly one bar gone
        }
    }

    // ── 2. Path graph (tree): every bar is a bridge → must return false ───
    {
        const Robot base = make_path_graph();
        Robot copy = base.clone();
        bool ok = Mutator::removeBarBridge(copy, rng);
        CHECK(!ok);
        CHECK((int)copy.bars.size()     == (int)base.bars.size());
        CHECK((int)copy.vertices.size() == (int)base.vertices.size());
    }

    // ── 3. No isolated vertices created ───────────────────────────────────
    //    The degree-≥ 2 guard ensures neither endpoint drops to degree 0.
    //    Verify explicitly: after every removal, all vertices have degree ≥ 1.
    {
        const Robot base = make_diamond();
        for (int i = 0; i < 50; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::removeBarBridge(copy, rng);
            if (!ok) continue;
            const auto deg = computeDegrees(copy);
            for (int d : deg) CHECK(d >= 1);
        }
    }

    // ── 4. Actuators on the removed bar are cleaned up correctly ──────────
    //    Attach an actuator to bar b0 (one of the diamond's cycle bars).
    //    After any successful removal, all actuator refs must remain valid.
    {
        Robot base = make_diamond();
        Eigen::VectorXd w = Eigen::VectorXd::Zero(0);
        base.addNeuron(Neuron(0.5, w));
        base.addActuator(Actuator(0, 0, 0.01));   // actuator on bar b0 (v0–v1)
        for (int i = 0; i < 60; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::removeBarBridge(copy, rng);
            if (!ok) continue;
            CHECK(copy.isValid());
            const int nb = (int)copy.bars.size();
            const int nn = (int)copy.neurons.size();
            for (const auto& a : copy.actuators) {
                CHECK(a.bar_idx    >= 0 && a.bar_idx    < nb);
                CHECK(a.neuron_idx >= 0 && a.neuron_idx < nn);
            }
        }
    }

    // ── 5. PATHOLOGY — bridge bar is never removed ────────────────────────
    //    Build a triangle with one pendant edge (v0–v3).
    //    v0–v3 is a bridge (only connection to v3); the triangle bars are not.
    //    After each call, v3 must still be present (the pendant is safe).
    {
        Robot base;
        base.addVertex(Vertex(0.00, 0.00, 0.00));  // v0 (degree 3)
        base.addVertex(Vertex(0.10, 0.00, 0.00));  // v1 (degree 2)
        base.addVertex(Vertex(0.05, 0.10, 0.00));  // v2 (degree 2)
        base.addVertex(Vertex(-0.10, 0.00, 0.00)); // v3 (degree 1 — pendant)
        base.addBar(Bar(0, 1, 0.100));
        base.addBar(Bar(1, 2, 0.112));
        base.addBar(Bar(0, 2, 0.112));
        base.addBar(Bar(0, 3, 0.100));   // bridge pendant
        const int v_before = (int)base.vertices.size();
        const int b_before = (int)base.bars.size();
        for (int i = 0; i < 50; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::removeBarBridge(copy, rng);
            CHECK(ok);                                              // triangle bars are always candidates
            CHECK(copy.isValid());
            CHECK(copy.isConnected());
            CHECK((int)copy.vertices.size() == v_before);           // v3 must NOT disappear
            CHECK((int)copy.bars.size()     == b_before - 1);
        }
    }

    std::cout << "PASS  test_remove_bar_bridge\n";
}

// ── test_join_element ─────────────────────────────────────────────────────────
//
// joinElement is the reverse of splitElement (bar-bisect mode).
// It merges a degree-2 internal vertex w (both neighbours u, v have degree ≥ 2
// and are not yet directly connected) into a single bar (u, v).
//
// Contract:
//   rest_length = |p_u – p_v|  (Euclidean — matches un-stressed geometry)
//   stiffness   = (k_b1 + k_b2) / 2
//   actuators on both half-bars migrate to the new bar
//   reject if |p_u – p_v| < bar_length_min
//
// ALL tests that expect joinElement to return true WILL FAIL because the
// stub always returns false.  Tests that expect false pass trivially; they
// verify the guard contracts once the real implementation exists.

static void test_join_element()
{
    std::mt19937 rng(11);
    MutatorParams params;   // bar_length_min = 0.01

    // ── 1. Basic: diamond has two joinable vertices → should return true ──
    //    After join: –1 vertex, –1 bar; graph valid and connected.
    //    FAILS: stub returns false.
    {
        const Robot base = make_diamond();
        const int v0 = (int)base.vertices.size();
        const int b0 = (int)base.bars.size();
        bool ever_joined = false;
        for (int i = 0; i < 50; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::joinElement(copy, rng, params);
            if (ok) {
                CHECK(copy.isValid());
                CHECK(copy.isConnected());
                CHECK((int)copy.vertices.size() == v0 - 1);
                CHECK((int)copy.bars.size()     == b0 - 1);
                ever_joined = true;
            }
        }
        CHECK(ever_joined);   // ← FAILS: stub never fires
    }

    // ── 2. rest_length of merged bar == Euclidean |p_u – p_v| ────────────
    //    Both join candidates in the diamond collapse to the same endpoint
    //    pair v0(0,0,0)–v2(0.2,0,0), so expected length is 0.2.
    //    FAILS: stub makes no change.
    {
        const Robot base = make_diamond();
        const double expected =
            (base.vertices[0].pos - base.vertices[2].pos).norm();   // 0.2
        for (int i = 0; i < 20; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::joinElement(copy, rng, params);
            if (!ok) continue;   // ← FAILS: stub never enters block
            bool found = false;
            for (const auto& b : copy.bars)
                if (std::abs(b.rest_length - expected) < 1e-9) { found = true; break; }
            CHECK(found);
        }
    }

    // ── 3. Stiffness of merged bar == average of the two half-bars ────────
    //    FAILS: stub makes no change.
    {
        Robot base = make_diamond();
        const double k1 = 20000.0, k2 = 80000.0;
        base.bars[0].stiffness = k1;   // bar v0–v1
        base.bars[1].stiffness = k2;   // bar v1–v2
        const double expected_k = (k1 + k2) / 2.0;
        for (int i = 0; i < 20; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::joinElement(copy, rng, params);
            if (!ok) continue;   // ← FAILS: stub never enters block
            bool found = false;
            for (const auto& b : copy.bars)
                if (std::abs(b.stiffness - expected_k) < 1.0) { found = true; break; }
            CHECK(found);
        }
    }

    // ── 4. No candidates: tetrahedron (all vertices degree ≥ 3) ──────────
    //    Passes trivially with stub; verifies guard contract when real.
    {
        const Robot base = make_seed_robot();
        for (int i = 0; i < 20; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::joinElement(copy, rng, params);
            CHECK(!ok);
            CHECK((int)copy.bars.size()     == (int)base.bars.size());
            CHECK((int)copy.vertices.size() == (int)base.vertices.size());
        }
    }

    // ── 5. No candidates: path graph (interior-only rule) ─────────────────
    //    v1 and v2 are degree-2 but their respective outer neighbours (v0, v3)
    //    are degree-1, which violates the "both neighbours ≥ 2" rule.
    //    Passes trivially with stub.
    {
        const Robot base = make_path_graph();
        for (int i = 0; i < 20; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::joinElement(copy, rng, params);
            CHECK(!ok);
            CHECK((int)copy.bars.size()     == (int)base.bars.size());
            CHECK((int)copy.vertices.size() == (int)base.vertices.size());
        }
    }

    // ── 6. No candidates: u–v pair already connected (square + diagonal) ──
    //    Every degree-2 vertex's endpoint pair already shares the diagonal bar.
    //    Passes trivially with stub.
    {
        const Robot base = make_square_with_diagonal();
        for (int i = 0; i < 20; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::joinElement(copy, rng, params);
            CHECK(!ok);
        }
    }

    // ── 7. Reject when merged bar would be shorter than bar_length_min ────
    //    v0(0,0,0) – v1(0.0025,0,0) – v2(0.005,0,0); v0 and v2 also have
    //    pendant neighbours to satisfy the degree-≥ 2 rule.
    //    |p_v0 – p_v2| = 0.005 < 0.01 → must return false.
    //    Passes trivially with stub.
    {
        Robot base;
        base.addVertex(Vertex(0.0000, 0.00, 0.0));  // v0
        base.addVertex(Vertex(0.0025, 0.00, 0.0));  // v1 — join candidate
        base.addVertex(Vertex(0.0050, 0.00, 0.0));  // v2
        base.addVertex(Vertex(0.0000, 0.20, 0.0));  // v3 — gives v0 degree 2
        base.addVertex(Vertex(0.0050, 0.20, 0.0));  // v4 — gives v2 degree 2
        base.addBar(Bar(0, 1, 0.0025));
        base.addBar(Bar(1, 2, 0.0025));
        base.addBar(Bar(0, 3, 0.2000));
        base.addBar(Bar(2, 4, 0.2000));
        // v1: degree-2, u=v0 (deg 2), v=v2 (deg 2), not connected.
        // |p_v0 – p_v2| = 0.005 < bar_length_min=0.01 → reject.
        Robot copy = base.clone();
        bool ok = Mutator::joinElement(copy, rng, params);
        CHECK(!ok);
        CHECK((int)copy.bars.size() == (int)base.bars.size());
    }

    // ── 8. Actuator migration: refs must be valid after join ──────────────
    //    Attach actuators to both half-bars of the diamond.  After a
    //    successful join the new bar's index must be within bounds and
    //    all actuator refs must be valid.
    //    FAILS: stub never performs a join.
    {
        Robot base = make_diamond();
        Eigen::VectorXd w = Eigen::VectorXd::Zero(0);
        base.addNeuron(Neuron(0.5, w));
        base.addActuator(Actuator(0, 0, 0.01));   // on bar b0 (v0–v1)
        base.addActuator(Actuator(1, 0, 0.01));   // on bar b1 (v1–v2)
        bool any_joined = false;
        for (int i = 0; i < 50; ++i) {
            Robot copy = base.clone();
            bool ok = Mutator::joinElement(copy, rng, params);
            if (!ok) continue;   // ← FAILS: stub never fires
            any_joined = true;
            CHECK(copy.isValid());
            const int nb = (int)copy.bars.size();
            const int nn = (int)copy.neurons.size();
            for (const auto& a : copy.actuators) {
                CHECK(a.bar_idx    >= 0 && a.bar_idx    < nb);
                CHECK(a.neuron_idx >= 0 && a.neuron_idx < nn);
            }
        }
        CHECK(any_joined);   // ← FAILS: stub returns false
    }

    // ── 9. Vertex index remap: merged bar endpoints match original positions
    //    After joining, we identify the new bar by checking that its two
    //    endpoint positions correspond to the original p_v0 and p_v2.
    //    (Both candidates in the diamond map to these same two positions.)
    //    FAILS: stub does not modify the robot.
    {
        Robot copy = make_diamond();
        const Eigen::Vector3d p_v0 = copy.vertices[0].pos;   // (0,  0,0)
        const Eigen::Vector3d p_v2 = copy.vertices[2].pos;   // (0.2,0,0)
        bool ok = Mutator::joinElement(copy, rng, params);
        if (ok) {
            bool found = false;
            for (const auto& b : copy.bars) {
                const Eigen::Vector3d& pa = copy.vertices[b.v1].pos;
                const Eigen::Vector3d& pb = copy.vertices[b.v2].pos;
                if ((pa-p_v0).norm() < 1e-9 && (pb-p_v2).norm() < 1e-9) { found = true; break; }
                if ((pa-p_v2).norm() < 1e-9 && (pb-p_v0).norm() < 1e-9) { found = true; break; }
            }
            CHECK(found);
        } else {
            ++g_failures;   // explicitly fail: stub didn't enter the block
        }
    }

    // ── 10. Interior-only: 3-vertex chain (degree-2 vertex has degree-1 ─
    //     neighbours) must be rejected.
    //     v0(deg-1) – v1(deg-2) – v2(deg-1): both neighbours of v1 are
    //     degree-1 → fails the "degree ≥ 2" check → returns false.
    //     Passes trivially with stub.
    {
        Robot chain;
        chain.addVertex(Vertex(0.0, 0.0, 0.0));
        chain.addVertex(Vertex(0.1, 0.0, 0.0));
        chain.addVertex(Vertex(0.2, 0.0, 0.0));
        chain.addBar(Bar(0, 1, 0.1));
        chain.addBar(Bar(1, 2, 0.1));
        Robot copy = chain.clone();
        bool ok = Mutator::joinElement(copy, rng, params);
        CHECK(!ok);
    }

    std::cout << "PASS  test_join_element\n";
}

// ── MutatorParams YAML round-trip ─────────────────────────────────────────────

static void test_mutator_params_yaml()
{
    // Build a non-default MutatorParams.
    MutatorParams mp;
    mp.p_perturb            = 0.20;
    mp.p_add_bar_new        = 0.02;
    mp.p_add_bar_bridge     = 0.03;
    mp.p_add_neuron         = 0.01;
    mp.p_remove_bar        = 0.01;
    mp.p_remove_neuron     = 0.01;
    mp.p_remove_bar_edge   = 0.01;
    mp.p_remove_bar_bridge = 0.01;
    mp.p_join_vertex        = 0.01;
    mp.num_method_retries   = 5;
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
    CHECK(mp2.p_add_bar_new        == mp.p_add_bar_new);
    CHECK(mp2.p_add_bar_bridge     == mp.p_add_bar_bridge);
    CHECK(mp2.p_add_neuron         == mp.p_add_neuron);
    CHECK(mp2.p_remove_bar        == mp.p_remove_bar);
    CHECK(mp2.p_remove_neuron     == mp.p_remove_neuron);
    CHECK(mp2.p_remove_bar_edge   == mp.p_remove_bar_edge);
    CHECK(mp2.p_remove_bar_bridge == mp.p_remove_bar_bridge);
    CHECK(mp2.p_join_vertex       == mp.p_join_vertex);
    CHECK(mp2.num_method_retries  == mp.num_method_retries);
    CHECK(mp2.p_split_vertex       == mp.p_split_vertex);        // default preserved
    CHECK(mp2.perturb_bar_frac     == mp.perturb_bar_frac);
    CHECK(mp2.bar_length_min       == mp.bar_length_min);
    CHECK(mp2.new_synapse_weight_max == mp.new_synapse_weight_max);

    // fromYAML on an empty node must produce defaults.
    const YAML::Node empty = YAML::Load("{}");
    const MutatorParams def = MutatorParams::fromYAML(empty);
    CHECK(def.p_perturb        == 0.10);
    CHECK(def.p_add_bar_new    == 0.005);
    CHECK(def.p_add_bar_bridge == 0.010);
    CHECK(def.p_add_neuron        == 0.005);
    CHECK(def.p_remove_bar        == 0.005);
    CHECK(def.p_remove_neuron     == 0.005);
    CHECK(def.p_remove_bar_edge   == 0.005);
    CHECK(def.p_remove_bar_bridge == 0.005);
    CHECK(def.p_join_vertex        == 0.005);
    CHECK(def.num_method_retries   == 10);
    CHECK(def.p_split_vertex       == 0.03);

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
    test_attach_neuron();
    test_detach_neuron();
    test_add_neuron_connected();
    test_rewire_neuron();
    test_remove_bar_edge();
    test_remove_bar_bridge();
    test_join_element();
    test_mutator_params_yaml();

    if (g_failures == 0) {
        std::cout << "\nAll mutator tests passed.\n";
        return 0;
    }
    std::cerr << "\n" << g_failures << " test(s) FAILED.\n";
    return 1;
}
