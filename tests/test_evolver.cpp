/**
 * test_evolver.cpp
 *
 * Unit tests for EvolverParams, Evolver construction, and the
 * steady-state evolutionary loop.
 *
 * Tests:
 *   1. Default params have sensible values
 *   2. YAML round-trip preserves all fields
 *   3. Population is initialised with the right size, all empty + valid
 *   4. Fitnesses are all zero
 *   5. run_config.yaml is written to the run directory
 *   6. Same seed → same initial mutation (reproducibility)
 */

#include "Evolver.h"
#include "FitnessEvaluator.h"
#include "Mutator.h"
#include "Actuator.h"
#include "Neuron.h"
#include "Bar.h"
#include "Vertex.h"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

static int g_failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__              \
                      << "  (" #cond ")\n";                                      \
            ++g_failures;                                                        \
        }                                                                        \
    } while (false)

// ── helpers ───────────────────────────────────────────────────────────────────

static EvolverParams makeTestParams(int pop_size, const std::string& run_dir)
{
    EvolverParams p;
    p.population_size          = pop_size;
    p.max_evaluations          = 10;
    p.seed                     = 99;
    p.run_dir                  = run_dir;
    p.record_min_improvement   = 0.0;
    p.video.enabled            = false;   // skip PNG/MP4 in tests
    return p;
}

// ── tests ─────────────────────────────────────────────────────────────────────

static void test_default_params()
{
    EvolverParams p;
    CHECK(p.population_size == 200);
    CHECK(p.max_evaluations == 100'000);
    CHECK(p.seed            == 42);
    CHECK(p.output_base_dir == "runs/");
    CHECK(p.run_dir.empty());
    CHECK(p.fitness.num_steps        == 60000);
    CHECK(p.fitness.steps_per_frame  == 5000);
    CHECK(p.fitness.steps_per_cycle  == 0);
    CHECK(std::abs(p.fitness.mu_static - 0.5) < 1e-10);
    CHECK(p.video.fps             == 30);
    CHECK(std::abs(p.record_min_improvement - 0.01) < 1e-10);
    std::cout << "PASS  test_default_params\n";
}

static void test_yaml_roundtrip()
{
    const std::string tmp_dir  = "/tmp/test_evolver_yaml/";
    const std::string tmp_file = tmp_dir + "params.yaml";
    fs::create_directories(tmp_dir);

    EvolverParams out;
    out.population_size     = 50;
    out.max_evaluations     = 500;
    out.seed                = 7;
    out.output_base_dir     = "/tmp/base/";
    out.run_dir             = "/tmp/test_run/";
    out.fitness.num_steps        = 15000;  // 5 ticks × spc=3000
    out.fitness.steps_per_frame  = 1000;
    out.fitness.steps_per_cycle  = 3000;
    out.fitness.step_size   = 2.5e-7;
    out.fitness.wind        = 0.3;
    out.fitness.mu_static   = 0.8;
    out.video.fps             = 24;
    out.record_min_improvement = 0.005;
    out.toYAML(tmp_file);

    EvolverParams in = EvolverParams::fromYAML(tmp_file);
    CHECK(in.population_size          == 50);
    CHECK(in.max_evaluations          == 500);
    CHECK(in.seed                     == 7);
    CHECK(in.output_base_dir          == "/tmp/base/");
    CHECK(in.run_dir                  == "/tmp/test_run/");
    CHECK(in.fitness.num_steps         == 15000);
    CHECK(in.fitness.steps_per_frame   == 1000);
    CHECK(in.fitness.steps_per_cycle   == 3000);
    CHECK(std::abs(in.fitness.step_size  - 2.5e-7) < 1e-15);
    CHECK(std::abs(in.fitness.wind       - 0.3)    < 1e-10);
    CHECK(std::abs(in.fitness.mu_static  - 0.8)    < 1e-10);
    CHECK(in.video.fps             == 24);
    CHECK(std::abs(in.record_min_improvement - 0.005) < 1e-10);
    std::cout << "PASS  test_yaml_roundtrip\n";
}

static void test_population_initialised()
{
    const std::string run_dir = "/tmp/test_evolver_pop/";
    Evolver ev(makeTestParams(7, run_dir));

    CHECK(ev.evalCount() == 0);

    // bestRobot() should return a valid (empty) robot
    const Robot& best = ev.bestRobot();
    CHECK(best.isValid());
    CHECK(ev.bestFitness() == 0.0);
    std::cout << "PASS  test_population_initialised\n";
}

static void test_fitnesses_zero()
{
    // Build a small evolver and confirm bestFitness is 0
    const std::string run_dir = "/tmp/test_evolver_fit/";
    Evolver ev(makeTestParams(5, run_dir));
    CHECK(ev.bestFitness() == 0.0);
    std::cout << "PASS  test_fitnesses_zero\n";
}

static void test_run_config_written()
{
    const std::string run_dir = "/tmp/test_evolver_cfg/";
    Evolver ev(makeTestParams(3, run_dir));

    const std::string cfg = run_dir + "run_config.yaml";
    CHECK(fs::exists(cfg));
    CHECK(fs::file_size(cfg) > 0);

    // Round-trip: reload and verify seed came through
    EvolverParams back = EvolverParams::fromYAML(cfg);
    CHECK(back.seed == 99);
    CHECK(back.population_size == 3);
    std::cout << "PASS  test_run_config_written\n";
}

static void test_seed_reproducibility()
{
    // Two evolvers with the same seed should produce identical mutations
    // on a clone of the same robot.
    const std::string run_dir_a = "/tmp/test_evolver_rep_a/";
    const std::string run_dir_b = "/tmp/test_evolver_rep_b/";
    Evolver ev_a(makeTestParams(3, run_dir_a));
    Evolver ev_b(makeTestParams(3, run_dir_b));

    // Both evolvers seeded with 99 — snag a RNG-derived mutation
    // by mutating a shared seed robot twice.
    Robot base;
    base.addVertex(Vertex(0.0, 0.0, 0.0));
    base.addVertex(Vertex(0.2, 0.0, 0.0));
    base.addBar(Bar(0, 1, 0.2));

    // Grab the RNG state by mutating through the Mutator directly
    // (Evolver doesn't expose rng_, so we regenerate from the same seed)
    std::mt19937 rng_a(99), rng_b(99);
    Robot r_a = base.clone();
    Robot r_b = base.clone();
    Mutator::mutate(r_a, rng_a);
    Mutator::mutate(r_b, rng_b);

    CHECK(r_a.bars.size()     == r_b.bars.size());
    CHECK(r_a.vertices.size() == r_b.vertices.size());
    if (!r_a.bars.empty() && !r_b.bars.empty())
        CHECK(std::abs(r_a.bars[0].rest_length - r_b.bars[0].rest_length) < 1e-15);
    std::cout << "PASS  test_seed_reproducibility\n";
}

// Integration test: run a short evolution with wind-only fitness so
// even simple bar structures accumulate non-zero fitness quickly.
static void test_run_loop()
{
    const std::string run_dir = "/tmp/test_evolver_run/";

    EvolverParams p;
    p.population_size   = 10;
    p.max_evaluations   = 50;
    p.seed              = 1;
    p.run_dir           = run_dir;
    // Wind mode: all vertices experience a constant +X acceleration,
    // so any robot with vertices gets non-zero fitness without needing
    // a working neural oscillator.
    p.fitness.wind            = 9.8;
    p.fitness.num_steps        = 1500;  // 3 ticks × spf=500
    p.fitness.steps_per_frame  = 500;
    p.record_min_improvement  = 0.0;
    p.video.enabled           = false;

    Evolver ev(p);
    ev.run();

    CHECK(ev.evalCount() == 50);
    CHECK(ev.bestFitness() >= 0.0);
    CHECK(ev.bestRobot().isValid());

    // fitness_log.csv should exist and have 51 lines (header + 50 data rows)
    {
        std::ifstream log(run_dir + "fitness_log.csv");
        CHECK(log.is_open());
        std::string header;
        std::getline(log, header);
        CHECK(header == "eval,generation,best_fitness,mean_fitness,"
                        "best_robot_id,best_v,best_b,best_n,best_a");
        int lines = 1;  // already consumed header
        std::string line;
        while (std::getline(log, line)) ++lines;
        CHECK(lines == 51);
    }

    // lineage.csv should exist and have 51 lines (header + 50 data rows)
    {
        std::ifstream lin(run_dir + "lineage.csv");
        CHECK(lin.is_open());
        std::string header;
        std::getline(lin, header);
        CHECK(header == "eval,child_id,parent_id,child_fitness,"
                        "replaced_id,replaced_fitness");
        int lines = 1;
        std::string line;
        while (std::getline(lin, line)) ++lines;
        CHECK(lines == 51);
    }

    // best_robot_final.yaml must be written
    CHECK(fs::exists(run_dir + "best_robot_final.yaml"));

    std::cout << "PASS  test_run_loop  (best_fitness="
              << ev.bestFitness() << "m)\n";
}

// Test that isNonMover() catches every case that can't produce locomotion:
//   • no actuators (the original check)
//   • no neurons  (activations stay 0 forever — added after observing dead evals)
//   • both present → is a mover and should run the full sim
static void test_nonmover_fast_path()
{
    // Case 1: empty robot — no actuators, no neurons
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        r.addVertex(Vertex(0.1, 0.0, 0.0));
        r.addBar(Bar(0, 1, 0.1));
        CHECK(r.isNonMover());
    }

    // Case 2: has actuators but neuron is self-excitatory (permanently ON) —
    // the probe sees a constant output, no delta → dead network → non-mover.
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        r.addVertex(Vertex(0.1, 0.0, 0.0));
        r.addBar(Bar(0, 1, 0.1));
        Neuron n(0.5);
        r.addNeuron(n);
        r.neurons[0].synapse_weights[0] = 1.0;  // self-excitatory (set after addNeuron)
        r.neurons[0].activation = 1.0;           // primed ON
        r.addActuator(Actuator(0, 0, 0.01));
        CHECK(r.isNonMover());  // permanently ON → probe returns false
    }

    // Case 3: has neurons but no actuators — nothing to extend
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        r.addVertex(Vertex(0.1, 0.0, 0.0));
        r.addBar(Bar(0, 1, 0.1));
        Neuron n(0.5);
        r.addNeuron(n);
        r.addNeuron(n);
        CHECK(r.isNonMover());
    }

    // Case 4: 2-neuron anti-phase oscillator + actuator — IS a mover.
    // n0 fires when n1 was active, and vice-versa → period-2 oscillation.
    // The probe sees a delta on the very first probe tick → returns true.
    {
        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        r.addVertex(Vertex(0.1, 0.0, 0.0));
        r.addBar(Bar(0, 1, 0.1));
        Neuron n0(0.5), n1(0.5);
        n0.activation = 0.0;  // n0 starts quiet
        n1.activation = 1.0;  // n1 primed so n0 fires on first tick
        r.addNeuron(n0);
        r.addNeuron(n1);
        r.neurons[0].synapse_weights << 0.0, 1.0;  // n0 ← n1
        r.neurons[1].synapse_weights << 1.0, 0.0;  // n1 ← n0
        r.addActuator(Actuator(0, 0, 0.01));
        CHECK(!r.isNonMover());  // oscillates → probe returns true
    }

    // FitnessEvaluator should return 0 instantly for non-movers
    {
        FitnessParams fp;
        fp.num_steps       = 10000;
        fp.steps_per_frame = 1000;
        fp.wind            = 0.0;

        Robot r;
        r.addVertex(Vertex(0.0, 0.0, 0.0));
        r.addVertex(Vertex(0.1, 0.0, 0.0));
        r.addBar(Bar(0, 1, 0.1));
        // No neurons → isNonMover() → evaluate() should return 0 without running sim
        CHECK(r.isNonMover());
        CHECK(FitnessEvaluator(fp).evaluate(r) == 0.0);
    }

    std::cout << "PASS  test_nonmover_fast_path\n";
}

// Integration test: run with a mix of non-movers (instant) and movers (slow).
// The out-of-order collection fix ensures the fast jobs don't stall waiting
// for the one slow mover at the front of the old FIFO queue.
// Verifies:
//   • run() completes with exactly max_evaluations evals
//   • fitness_log and lineage_log have the correct number of rows
//   • at least one robot was treated as a mover (fitness > 0 possible)
static void test_run_out_of_order_collection()
{
    const std::string run_dir = "/tmp/test_evolver_ooo/";
    fs::remove_all(run_dir);

    EvolverParams p;
    p.population_size          = 6;    // small population
    p.max_evaluations          = 30;
    p.seed                     = 7;
    p.run_dir                  = run_dir;
    p.record_min_improvement   = 0.0;
    p.video.enabled            = false;

    // Use wind so any robot with vertices drifts — even non-neural ones get
    // non-zero fitness, which exercises the replacement and best_idx machinery.
    p.fitness.wind             = 9.8;
    p.fitness.num_steps        = 600;   // 2 ticks × spf=300  — short enough to be fast
    p.fitness.steps_per_frame  = 300;
    p.fitness.steps_per_cycle  = 300;

    // High p_attach so the mutation loop will produce robots with both
    // neurons and actuators within the first few evals, creating the
    // fast/slow mix that exercises waitForAny.
    p.mutation.p_attach_neuron = 0.5;
    p.mutation.p_add_bar_new    = 0.5;

    Evolver ev(p);
    ev.run();

    // Core correctness: every submitted job must be collected exactly once.
    CHECK(ev.evalCount() == 30);
    CHECK(ev.bestRobot().isValid());

    // fitness_log: header + 30 data rows
    {
        std::ifstream log(run_dir + "fitness_log.csv");
        CHECK(log.is_open());
        int lines = 0;
        std::string line;
        while (std::getline(log, line)) ++lines;
        CHECK(lines == 31);
    }

    // lineage.csv: header + 30 data rows
    {
        std::ifstream lin(run_dir + "lineage.csv");
        CHECK(lin.is_open());
        int lines = 0;
        std::string line;
        while (std::getline(lin, line)) ++lines;
        CHECK(lines == 31);
    }

    std::cout << "PASS  test_run_out_of_order_collection  (best_fitness="
              << ev.bestFitness() << "m)\n";
}

// Integration test: resume from a checkpoint after a short run.
static void test_resume()
{
    const std::string run_dir = "/tmp/test_evolver_resume/";

    // Remove any leftovers from a previous run of this test.
    fs::remove_all(run_dir);

    // ── Phase A: initial run, 30 evals ────────────────────────────────────────
    {
        EvolverParams p;
        p.population_size   = 10;
        p.max_evaluations   = 30;
        p.seed              = 42;
        p.run_dir           = run_dir;
        p.fitness.wind            = 9.8;
        p.fitness.num_steps        = 1500;  // 3 ticks × spf=500
        p.fitness.steps_per_frame  = 500;
        p.record_min_improvement  = 0.0;
        p.video.enabled           = false;

        Evolver ev(p);
        ev.run();
        CHECK(ev.evalCount() == 30);
    }

    // Both checkpoint files must exist after phase A.
    CHECK(fs::exists(run_dir + "checkpoint_population.yaml"));
    CHECK(fs::exists(run_dir + "checkpoint_latest.yaml"));

    // ── Phase B: resume and run 20 more evals ─────────────────────────────────
    {
        EvolverParams p;
        p.max_evaluations   = 50;   // total budget; picks up from eval 30
        p.seed              = 42;
        p.run_dir           = run_dir;
        p.resume            = true;
        p.fitness.wind            = 9.8;
        p.fitness.num_steps        = 1500;  // 3 ticks × spf=500
        p.fitness.steps_per_frame  = 500;
        p.record_min_improvement  = 0.0;
        p.video.enabled           = false;

        Evolver ev(p);
        CHECK(ev.evalCount() == 30);   // restored from checkpoint
        ev.run();
        CHECK(ev.evalCount() == 50);   // ran 20 more
    }

    // fitness_log.csv should have header + 50 data rows (30 + 20)
    {
        std::ifstream log(run_dir + "fitness_log.csv");
        CHECK(log.is_open());
        int lines = 0;
        std::string line;
        while (std::getline(log, line)) ++lines;
        CHECK(lines == 51);  // header + 50 data rows
    }

    std::cout << "PASS  test_resume\n";
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    test_default_params();
    test_yaml_roundtrip();
    test_population_initialised();
    test_fitnesses_zero();
    test_run_config_written();
    test_seed_reproducibility();
    test_nonmover_fast_path();
    test_run_out_of_order_collection();
    test_run_loop();
    test_resume();

    if (g_failures == 0) {
        std::cout << "\nAll evolver tests passed.\n";
        return 0;
    }
    std::cerr << "\n" << g_failures << " test(s) FAILED.\n";
    return 1;
}
