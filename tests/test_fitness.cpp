/**
 * test_fitness.cpp
 *
 * Unit tests for §3.6 FitnessEvaluator.
 *
 * Tests are purely physics-level (no display/rendering required).
 */

#include "FitnessEvaluator.h"
#include "Robot.h"
#include "Vertex.h"
#include "Bar.h"
#include "Neuron.h"
#include "Actuator.h"

#include <cmath>
#include <iostream>

static int g_failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__              \
                      << "  (" #cond ")\n";                                      \
            ++g_failures;                                                        \
        }                                                                        \
    } while (false)

#define CHECK_NEAR(a, b, eps)                                                    \
    do {                                                                         \
        if (std::abs((a) - (b)) > (eps)) {                                       \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__              \
                      << "  |" << (a) << " - " << (b) << "| > " << (eps) << "\n"; \
            ++g_failures;                                                         \
        }                                                                         \
    } while (false)

// ── helpers ───────────────────────────────────────────────────────────────────

/** Tetrahedron resting on the floor with no neural wiring. */
static Robot make_passive_tetra()
{
    Robot r(1);
    r.addVertex(Vertex(0.00, 0.00, 0.00));
    r.addVertex(Vertex(0.20, 0.00, 0.00));
    r.addVertex(Vertex(0.10, 0.20, 0.00));
    r.addVertex(Vertex(0.10, 0.10, 0.20));
    // rest lengths auto-computed from positions by Bar constructor
    r.addBar(Bar(0, 1, 0.20, 0.010));
    r.addBar(Bar(0, 2, 0.22, 0.010));
    r.addBar(Bar(1, 2, 0.22, 0.010));
    r.addBar(Bar(0, 3, 0.24, 0.008));
    r.addBar(Bar(1, 3, 0.24, 0.008));
    r.addBar(Bar(2, 3, 0.22, 0.008));
    return r;
}

/**
 * Same tetrahedron with a two-neuron anti-phase oscillator: n0 and n1
 * alternate firing each cycle (period 2).  Only n0 drives an actuator, so
 * bar 3 pulses extend/retract every two ticks.  The asymmetric floor
 * contact produces net XY locomotion, and probeNeuralActivity() correctly
 * identifies this robot as a mover.
 */
static Robot make_driven_tetra()
{
    Robot r = make_passive_tetra();

    // Add both neurons first with default zero weights; set cross-excitation
    // weights afterwards so addNeuron's auto-sizing does not clobber them.
    Neuron n0(0.5), n1(0.5);
    n0.activation = 0.0;   // n0 starts quiet
    n1.activation = 1.0;   // n1 primed so n0 fires on the very first tick
    r.addNeuron(n0);
    r.addNeuron(n1);

    // Cross-excitation: n0 fires when n1 was active, and vice-versa.
    r.neurons[0].synapse_weights << 0.0, 1.0;
    r.neurons[1].synapse_weights << 1.0, 0.0;

    r.addActuator(Actuator(/*bar_idx=*/3, /*neuron_idx=*/0, /*bar_range=*/+0.005));
    return r;
}

/**
 * Two-neuron anti-phase oscillator (period 2) wired with equal but opposite
 * bar_ranges.  Over an even number of cycles the elongation exactly cancels,
 * so this serves as a near-zero-drift symmetry check.
 */
static Robot make_antiphase_tetra()
{
    Robot r = make_passive_tetra();

    Neuron n0;
    n0.threshold = 0.5;
    n0.synapse_weights.resize(2); n0.synapse_weights << 0.0, 1.0;  // fires when n1 was active
    n0.activation = 1.0;               // primed
    r.addNeuron(n0);

    Neuron n1;
    n1.threshold = 0.5;
    n1.synapse_weights.resize(2); n1.synapse_weights << 1.0, 0.0;  // fires when n0 was active
    n1.activation = 0.0;
    r.addNeuron(n1);

    // opposing actuators on the same bar → perfectly cancel over 2-tick period
    r.addActuator(Actuator(3, 0, +0.005));
    r.addActuator(Actuator(3, 1, -0.005));
    return r;
}

// ── tests ─────────────────────────────────────────────────────────────────────

static void test_passive_robot_near_zero_fitness()
{
    std::cout << "[ RUN ] passive robot fitness ≈ 0\n";
    // A robot with no actuators cannot self-locomote.
    // After relaxation gravity may settle it slightly but XY CoM must not drift.
    FitnessEvaluator eval;
    double score = eval.evaluate(make_passive_tetra());
    // Allow for floating-point accumulation over 60 000 integration steps (~0.5 mm threshold)
    CHECK(score < 5e-4);
    std::cout << "        score = " << score << "\n";
}

static void test_driven_robot_positive_fitness()
{
    std::cout << "[ RUN ] driven robot has positive fitness\n";
    // A robot with a monotonically elongating strut should drift in XY.
    FitnessEvaluator eval;
    double score = eval.evaluate(make_driven_tetra());
    CHECK(score > 0.0);
    std::cout << "        score = " << score << "\n";
}

static void test_more_cycles_increases_fitness()
{
    std::cout << "[ RUN ] more steps \u2265 fewer steps for driven robot\n";
    FitnessEvaluator::Params p6;  p6.num_steps = 30000;
    FitnessEvaluator::Params p12; p12.num_steps = 60000;

    double s6  = FitnessEvaluator(p6 ).evaluate(make_driven_tetra());
    double s12 = FitnessEvaluator(p12).evaluate(make_driven_tetra());

    CHECK(s12 >= s6);
    std::cout << "        6-frame-equiv steps=" << s6 << "  12-frame-equiv steps=" << s12 << "\n";
}

static void test_trajectory_length()
{
    std::cout << "[ RUN ] trajectory has num_steps/spc+1 entries\n";
    FitnessEvaluator::Params p; p.num_steps = 25000;
    FitnessEvaluator eval(p);

    std::vector<Eigen::Vector2d> traj;
    eval.evaluate(make_driven_tetra(), &traj);

    CHECK(static_cast<int>(traj.size()) == p.num_steps / p.steps_per_frame + 1);
}

static void test_trajectory_start_matches_fitness_start()
{
    std::cout << "[ RUN ] trajectory[0] is initial CoM\n";
    FitnessEvaluator eval;

    std::vector<Eigen::Vector2d> traj;
    double score = eval.evaluate(make_driven_tetra(), &traj);

    // The score equals ‖traj.back() - traj.front()‖
    double traj_dist = (traj.back() - traj.front()).norm();
    CHECK_NEAR(score, traj_dist, 1e-10);
}

static void test_deterministic()
{
    std::cout << "[ RUN ] evaluate is deterministic (no noise)\n";
    FitnessEvaluator eval;
    Robot r = make_driven_tetra();

    double s1 = eval.evaluate(r);
    double s2 = eval.evaluate(r);

    CHECK_NEAR(s1, s2, 1e-15);
}

static void test_antiphase_low_drift()
{
    std::cout << "[ RUN ] anti-phase oscillator has low long-term drift\n";
    // Over 12 even cycles the bar oscillates ±0.5 cm and returns to L0 each
    // second cycle — the CoM should stay very close to its starting position.
    FitnessEvaluator eval;
    double score = eval.evaluate(make_antiphase_tetra());
    std::cout << "        score = " << score << "\n";
    // Not asserting == 0 because friction and floor may break symmetry slightly,
    // but it should be well below what a driven robot achieves.
    CHECK(score < 0.01);
}

static void test_zero_cycles_returns_zero()
{
    std::cout << "[ RUN ] 0 steps → fitness == 0\n";
    FitnessEvaluator::Params p; p.num_steps = 0;
    double score = FitnessEvaluator(p).evaluate(make_driven_tetra());
    CHECK_NEAR(score, 0.0, 1e-15);
}

// ─────────────────────────────────────────────────────────────────────────────

int main()
{
    test_passive_robot_near_zero_fitness();
    test_driven_robot_positive_fitness();
    test_more_cycles_increases_fitness();
    test_trajectory_length();
    test_trajectory_start_matches_fitness_start();
    test_deterministic();
    test_antiphase_low_drift();
    test_zero_cycles_returns_zero();

    if (g_failures == 0) {
        std::cout << "\nAll fitness tests passed.\n";
        return 0;
    }
    std::cerr << "\n" << g_failures << " test(s) FAILED.\n";
    return 1;
}
