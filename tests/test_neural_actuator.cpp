/**
 * test_neural_actuator.cpp
 *
 * Tests for §3.4 (neural tick) and §3.5 (actuator coupling).
 *
 * Each test builds a minimal Robot, constructs a Simulator, then calls
 * tickNeural() and/or applyActuators() and checks the resulting
 * activations_ / rest_lengths_ values directly.
 */

#include "Robot.h"
#include "Simulator.h"
#include "Neuron.h"
#include "Actuator.h"
#include "Bar.h"
#include "Vertex.h"

#include <Eigen/Core>
#include <cmath>
#include <iostream>

static int g_failures = 0;

#define CHECK(cond)                                                             \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__             \
                      << "  (" #cond ")\n";                                     \
            ++g_failures;                                                       \
        }                                                                       \
    } while (false)

#define CHECK_NEAR(a, b, eps)                                                   \
    do {                                                                        \
        if (std::abs((a) - (b)) > (eps)) {                                      \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__             \
                      << "  |" << (a) << " - " << (b) << "| > " << (eps) << "\n"; \
            ++g_failures;                                                       \
        }                                                                       \
    } while (false)

static constexpr double EPS = 1e-12;

// ── helpers ───────────────────────────────────────────────────────────────────

/** Minimal robot: two vertices, one bar, no neural wiring. */
static Robot make_base_robot()
{
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(0.1, 0.0, 0.0));
    r.addBar(Bar(0, 1, 0.1));
    return r;
}

// ── §3.4  Neural tick tests ───────────────────────────────────────────────────

/** An isolated neuron (zero synapse weights) never fires regardless of
 *  initial state, because its weighted sum is always 0 < threshold. */
static void test_isolated_neuron_never_fires()
{
    Robot r = make_base_robot();
    // addNeuron sizes weights to match the growing network;
    // a fresh neuron gets one zero-weight entry (pointing to itself).
    r.addNeuron(Neuron(0.5));   // threshold=0.5, weight auto-set to 0

    Simulator sim(r);

    // Case 1: starts silent, should remain silent
    sim.activations_[0] = 0.0;
    sim.tickNeural();
    CHECK_NEAR(sim.activations_[0], 0.0, EPS);

    // Case 2: prime it active — still should not fire (sum = 0 * 1.0 = 0 < 0.5)
    sim.activations_[0] = 1.0;
    sim.tickNeural();
    CHECK_NEAR(sim.activations_[0], 0.0, EPS);
}

/** A self-excitatory neuron (weight_self ≥ threshold) latches once primed. */
static void test_self_excitatory_neuron_latches()
{
    Robot r = make_base_robot();
    Neuron n(0.5);
    r.addNeuron(n);
    // Set self-weight to 2.0 so that when active, weighted_sum = 2.0 ≥ 0.5
    r.neurons[0].synapse_weights(0) = 2.0;

    Simulator sim(r);

    // Prime to firing state
    sim.activations_[0] = 1.0;
    sim.tickNeural();
    CHECK_NEAR(sim.activations_[0], 1.0, EPS);   // latched on

    // Re-check: remains on
    sim.tickNeural();
    CHECK_NEAR(sim.activations_[0], 1.0, EPS);
}

/** Two neurons with cross-connections form a period-2 oscillator.
 *
 *  Network: neuron 0 is driven by neuron 1; neuron 1 is driven by neuron 0.
 *  Both thresholds = 0.5, both cross-weights = 1.0, self-weights = 0.
 *
 *  Tick sequence (parallel evaluation):
 *    t=0: [1, 0]
 *    t=1: neuron-0 sees prev[1]=0 → silent; neuron-1 sees prev[0]=1 → fires
 *         → [0, 1]
 *    t=2: neuron-0 sees prev[1]=1 → fires; neuron-1 sees prev[0]=0 → silent
 *         → [1, 0]  (back to start)
 */
static void test_two_neuron_oscillator()
{
    Robot r = make_base_robot();
    r.addNeuron(Neuron(0.5));   // index 0
    r.addNeuron(Neuron(0.5));   // index 1
    // Each addNeuron call grows synapse_weights; both now have size 2.
    // Weights: [self, from-other] — set cross-coupling only
    r.neurons[0].synapse_weights(0) = 0.0;   // no self
    r.neurons[0].synapse_weights(1) = 1.0;   // driven by neuron 1
    r.neurons[1].synapse_weights(0) = 1.0;   // driven by neuron 0
    r.neurons[1].synapse_weights(1) = 0.0;   // no self

    Simulator sim(r);
    sim.activations_[0] = 1.0;
    sim.activations_[1] = 0.0;

    // Tick 1: [1,0] → [0,1]
    sim.tickNeural();
    CHECK_NEAR(sim.activations_[0], 0.0, EPS);
    CHECK_NEAR(sim.activations_[1], 1.0, EPS);

    // Tick 2: [0,1] → [1,0]
    sim.tickNeural();
    CHECK_NEAR(sim.activations_[0], 1.0, EPS);
    CHECK_NEAR(sim.activations_[1], 0.0, EPS);

    // Tick 3: periodic — back to [0,1]
    sim.tickNeural();
    CHECK_NEAR(sim.activations_[0], 0.0, EPS);
    CHECK_NEAR(sim.activations_[1], 1.0, EPS);
}

/** tickNeural is a no-op when the robot has no neurons. */
static void test_tickneural_noop_no_neurons()
{
    Robot r = make_base_robot();
    Simulator sim(r);
    CHECK(sim.activations_.empty());
    sim.tickNeural();   // must not throw or crash
}

// ── §3.5  Actuator coupling tests ────────────────────────────────────────────

// Helper: apply actuators and immediately complete the ramp in one step.
// With steps_per_cycle = 1 the per-step delta equals the full length change,
// so after one relax iteration rest_lengths_ == target_rest_lengths_.
static void apply_and_ramp(Simulator& sim)
{
    sim.applyActuators(1);
    sim.relax(1, 1e-8, 0.0, 0.0);
}

/** When the driving neuron is silent, applyActuators must not change
 *  rest_lengths_ (Δl = 0 * bar_range = 0). */
static void test_actuator_silent_neuron_no_change()
{
    Robot r = make_base_robot();
    r.addNeuron(Neuron(0.5));
    r.addActuator(Actuator(0, 0, 0.005));   // bar 0, neuron 0, range 5 mm

    Simulator sim(r);
    const double L0_before = sim.rest_lengths_[0];

    // Neuron silent (default)
    sim.activations_[0] = 0.0;
    apply_and_ramp(sim);

    CHECK_NEAR(sim.rest_lengths_[0], L0_before, EPS);
}

/** When the driving neuron fires, rest_length increases by bar_range. */
static void test_actuator_fires_extends_bar()
{
    Robot r = make_base_robot();
    r.addNeuron(Neuron(0.5));
    r.addActuator(Actuator(0, 0, 0.005));

    Simulator sim(r);
    const double L0_before = sim.rest_lengths_[0];

    sim.activations_[0] = 1.0;
    apply_and_ramp(sim);

    CHECK_NEAR(sim.rest_lengths_[0], L0_before + 0.005, EPS);
}

/** A negative bar_range is clamped to zero (extension-only): no change. */
static void test_actuator_negative_range_clamps_to_zero()
{
    Robot r = make_base_robot();
    r.addNeuron(Neuron(0.5));
    r.addActuator(Actuator(0, 0, -0.005));   // negative — clamped to 0

    Simulator sim(r);
    const double L0_before = sim.rest_lengths_[0];

    sim.activations_[0] = 1.0;
    apply_and_ramp(sim);

    // Extension-only: negative range → extension = 0 → no change
    CHECK_NEAR(sim.rest_lengths_[0], L0_before, EPS);
}

/** bar_range > 0.01 m is clamped to ≤ 0.01 m. */
static void test_actuator_clamp_positive()
{
    Robot r = make_base_robot();
    r.addNeuron(Neuron(0.5));
    r.addActuator(Actuator(0, 0, 0.05));   // 5 cm — exceeds the 1 cm limit

    Simulator sim(r);
    const double L0_before = sim.rest_lengths_[0];

    sim.activations_[0] = 1.0;
    apply_and_ramp(sim);

    CHECK_NEAR(sim.rest_lengths_[0], L0_before + 0.01, EPS);   // clamped to 1 cm
}

/** applyActuators is a no-op when the robot has no actuators. */
static void test_applyactuators_noop_no_actuators()
{
    Robot r = make_base_robot();
    Simulator sim(r);
    const double L0_before = sim.rest_lengths_[0];
    apply_and_ramp(sim);
    CHECK_NEAR(sim.rest_lengths_[0], L0_before, EPS);
}

/** tickNeural followed by applyActuators: two-neuron, two-bar oscillating drive.
 *
 *  Neuron 0 → extends bar 0 (+0.005 m per cycle when firing)
 *  Neuron 1 → shortens bar 1 (-0.005 m per cycle when firing)
 *  Network oscillates with period 2 (as in test_two_neuron_oscillator).
 *
 *  Cycle 1 (start [1,0]):
 *    tick → [0,1]   (neuron 0 silent, neuron 1 fires)
 *    apply → bar 1 shortens by 0.005
 *  Cycle 2 (start [0,1]):
 *    tick → [1,0]   (neuron 0 fires, neuron 1 silent)
 *    apply → bar 0 extends by 0.005
 */
static void test_combined_tick_and_apply()
{
    Robot r = make_base_robot();
    // Add a second bar (bar index 1)
    r.addVertex(Vertex(0.2, 0.0, 0.0));
    r.addBar(Bar(1, 2, 0.1));

    r.addNeuron(Neuron(0.5));   // neuron 0
    r.addNeuron(Neuron(0.5));   // neuron 1

    // Cross-couple: 0 driven by 1, 1 driven by 0
    r.neurons[0].synapse_weights(0) = 0.0;
    r.neurons[0].synapse_weights(1) = 1.0;
    r.neurons[1].synapse_weights(0) = 1.0;
    r.neurons[1].synapse_weights(1) = 0.0;

    r.addActuator(Actuator(0, 0,  0.005));   // neuron 0 → bar 0 extends
    r.addActuator(Actuator(1, 1,  0.005));   // neuron 1 → bar 1 extends

    Simulator sim(r);
    const double L0_bar0 = sim.rest_lengths_[0];
    const double L0_bar1 = sim.rest_lengths_[1];

    // ── Cycle 1 ──────────────────────────────────────────────────────────
    sim.activations_[0] = 1.0;
    sim.activations_[1] = 0.0;

    sim.tickNeural();
    CHECK_NEAR(sim.activations_[0], 0.0, EPS);   // silenced
    CHECK_NEAR(sim.activations_[1], 1.0, EPS);   // fired

    apply_and_ramp(sim);
    // Neuron 0 was silent → bar 0 unchanged
    CHECK_NEAR(sim.rest_lengths_[0], L0_bar0, EPS);
    // Neuron 1 fired → bar 1 extends by 0.005
    CHECK_NEAR(sim.rest_lengths_[1], L0_bar1 + 0.005, EPS);

    // ── Cycle 2 ──────────────────────────────────────────────────────────
    sim.tickNeural();
    CHECK_NEAR(sim.activations_[0], 1.0, EPS);   // refired
    CHECK_NEAR(sim.activations_[1], 0.0, EPS);   // silenced

    apply_and_ramp(sim);
    // Neuron 0 fired → bar 0 extends by 0.005
    CHECK_NEAR(sim.rest_lengths_[0], L0_bar0 + 0.005, EPS);
    // Neuron 1 silent → bar 1 retracted back to base
    CHECK_NEAR(sim.rest_lengths_[1], L0_bar1, EPS);
}

// ── main ──────────────────────────────────────────────────────────────────────

static void run(const char* name, void (*fn)())
{
    std::cout << "  " << name << "\n";
    fn();
}

int main()
{
    std::cout << "test_neural_actuator\n";

    run("isolated neuron never fires",           test_isolated_neuron_never_fires);
    run("self-excitatory neuron latches",         test_self_excitatory_neuron_latches);
    run("two-neuron period-2 oscillator",         test_two_neuron_oscillator);
    run("tickNeural no-op with no neurons",       test_tickneural_noop_no_neurons);
    run("actuator: silent neuron → no change",    test_actuator_silent_neuron_no_change);
    run("actuator: firing neuron extends bar",    test_actuator_fires_extends_bar);
    run("actuator: negative range clamps to 0",  test_actuator_negative_range_clamps_to_zero);
    run("actuator: positive clamp at 1 cm",       test_actuator_clamp_positive);
    run("applyActuators no-op with no actuators", test_applyactuators_noop_no_actuators);
    run("combined tick + apply over 2 cycles",    test_combined_tick_and_apply);

    if (g_failures == 0)
        std::cout << "PASS (all tests)\n";
    else
        std::cout << "FAIL (" << g_failures << " failure(s))\n";

    return g_failures > 0 ? 1 : 0;
}
