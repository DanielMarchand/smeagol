#pragma once

#include "FitnessEvaluator.h"
#include "Robot.h"

#include <fstream>
#include <random>
#include <string>
#include <vector>

/**
 * @brief Configuration for a single evolutionary run.
 *
 * All fields have sensible defaults matching Lipson & Pollack (2000).
 * Load from YAML with EvolverParams::fromYAML().
 *
 * YAML format (all fields optional):
 * @code
 * population_size: 200
 * max_evaluations: 100000
 * seed: 42          # 0 = seed from std::random_device
 * video_interval: 1000
 * run_dir: ""       # auto-set to runs/run_<timestamp>/ if empty
 * fitness:
 *   cycles: 12
 *   steps_per_cycle: 5000
 *   step_size: 1.0e-7
 *   wind: 0.0
 * @endcode
 */
struct EvolverParams
{
    int         population_size = 200;
    int         max_evaluations = 100'000;
    int         seed            = 42;   ///< 0 → seed from std::random_device
    FitnessParams fitness;
    int         video_interval  = 1000; ///< save best-robot video every N evals
    std::string run_dir         = "";   ///< auto-set to runs/run_<timestamp>/

    /**
     * Load from a YAML file.  Any field not present in the file keeps its
     * default value, so a minimal config (e.g. just `seed: 7`) is valid.
     */
    static EvolverParams fromYAML(const std::string& path);

    /** Write the full resolved params (including auto-chosen run_dir and seed)
     *  to @p path for run reproducibility. */
    void toYAML(const std::string& path) const;
};

/**
 * @brief Steady-state evolutionary loop — Lipson & Pollack (2000) §2.
 *
 * Construction initialises the population (all empty robots, all zero fitness)
 * and creates the run directory.  Call run() to start the evolutionary loop.
 *
 * Typical usage:
 * @code
 *   EvolverParams p = EvolverParams::fromYAML("config.yaml");
 *   Evolver ev(p);
 *   ev.run();
 * @endcode
 */
class Evolver
{
public:
    explicit Evolver(EvolverParams params);

    /**
     * Run the evolutionary loop until eval_count_ >= params_.max_evaluations.
     * Each iteration: select parent → clone → mutate → evaluate → replace.
     */
    void run();

    const Robot& bestRobot()  const;
    double       bestFitness() const;
    int          evalCount()   const { return eval_count_; }

private:
    // ── selection & replacement ────────────────────────────────────────────
    int selectParent()      const;   ///< fitness-proportionate roulette
    int selectReplacement() const;   ///< uniform random over entire population

    // ── inner loop helpers ─────────────────────────────────────────────────
    void evaluateOne(int idx);              ///< run fitness eval, update fitnesses_[idx]
    void maybeSaveSnapshot(int eval_num);   ///< write video if on video_interval

    // ── state ──────────────────────────────────────────────────────────────
    EvolverParams           params_;
    std::vector<Robot>      population_;
    std::vector<double>     fitnesses_;
    mutable std::mt19937    rng_;
    int                     eval_count_ = 0;
    int                     best_idx_   = 0;

    std::ofstream           fitness_log_;    ///< eval_count, best_fitness per row
};
