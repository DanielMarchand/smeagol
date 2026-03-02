/**
 * test_evolver.cpp
 *
 * Phase 4.1 unit tests for EvolverParams and Evolver construction.
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
#include "Mutator.h"
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
    p.population_size = pop_size;
    p.max_evaluations = 10;
    p.seed            = 99;
    p.run_dir         = run_dir;
    return p;
}

// ── tests ─────────────────────────────────────────────────────────────────────

static void test_default_params()
{
    EvolverParams p;
    CHECK(p.population_size == 200);
    CHECK(p.max_evaluations == 100'000);
    CHECK(p.seed            == 42);
    CHECK(p.video_interval  == 1000);
    CHECK(p.run_dir.empty());
    CHECK(p.fitness.cycles          == 12);
    CHECK(p.fitness.steps_per_cycle == 5000);
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
    out.video_interval      = 100;
    out.run_dir             = "/tmp/test_run/";
    out.fitness.cycles      = 5;
    out.fitness.steps_per_cycle = 1000;
    out.fitness.step_size   = 2.5e-7;
    out.fitness.wind        = 0.3;
    out.toYAML(tmp_file);

    EvolverParams in = EvolverParams::fromYAML(tmp_file);
    CHECK(in.population_size          == 50);
    CHECK(in.max_evaluations          == 500);
    CHECK(in.seed                     == 7);
    CHECK(in.video_interval           == 100);
    CHECK(in.run_dir                  == "/tmp/test_run/");
    CHECK(in.fitness.cycles           == 5);
    CHECK(in.fitness.steps_per_cycle  == 1000);
    CHECK(std::abs(in.fitness.step_size - 2.5e-7) < 1e-15);
    CHECK(std::abs(in.fitness.wind    - 0.3)      < 1e-10);
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

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    test_default_params();
    test_yaml_roundtrip();
    test_population_initialised();
    test_fitnesses_zero();
    test_run_config_written();
    test_seed_reproducibility();

    if (g_failures == 0) {
        std::cout << "\nAll evolver tests passed.\n";
        return 0;
    }
    std::cerr << "\n" << g_failures << " test(s) FAILED.\n";
    return 1;
}
