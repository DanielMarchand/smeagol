#pragma once

#include "FitnessEvaluator.h"
#include "Mutator.h"
#include "Robot.h"

#include <fstream>
#include <random>
#include <string>
#include <vector>

/**
 * @brief Video rendering parameters for record-checkpoint snapshots.
 */
struct VideoParams
{
    bool enabled        = true;     ///< set false to skip PNG+MP4 (e.g. in unit tests)
    int fps             = 30;       ///< output framerate passed to ffmpeg
    int steps_per_frame = 2000000;  ///< physics steps between captured frames
};

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
 * seed: 42               # 0 = seed from std::random_device
 * output_base_dir: runs/ # timestamped run dirs land here
 * run_dir: ""            # overrides output_base_dir when non-empty
 * fitness:
 *   cycles: 12
 *   steps_per_cycle: 5000
 *   step_size: 1.0e-7
 *   wind: 0.0
 *   mu_static: 0.5       # static Coulomb friction coefficient
 * video:
 *   fps:             30
 *   steps_per_frame: 2000000
 * @endcode
 */
struct EvolverParams
{
    int         population_size  = 200;
    int         max_evaluations  = 100'000;
    int         seed             = 42;        ///< 0 → seed from std::random_device
    FitnessParams fitness;
    MutatorParams mutation;                   ///< mutation knobs (all have defaults)
    std::string output_base_dir  = "runs/";   ///< timestamped run dirs are created here
    std::string run_dir          = "";        ///< overrides output_base_dir when non-empty
    bool        resume           = false;     ///< if true, restore from checkpoint_population.yaml
    VideoParams video;                        ///< video rendering knobs for record snapshots
    double      record_min_improvement = 0.01; ///< new best must exceed old by at least this [m]

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
 * and creates the run directory with subdirectories:
 *   robots/       — YAML archive of every child placed in the population
 *   checkpoints/  — periodic best-robot YAML, PNG, and MP4 snapshots
 *
 * Call run() to start the evolutionary loop.
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
     * Every placed child is saved to robots/robot_<id>.yaml.
     * YAML + PNG + MP4 written to checkpoints/ on every new fitness record.
     */
    void run();

    const Robot& bestRobot()    const;
    double       bestFitness()  const;
    int          evalCount()    const { return eval_count_; }

    /// Returns true if run() was interrupted by SIGINT/SIGTERM.
    bool wasInterrupted() const;

private:
    // ── selection & replacement ────────────────────────────────────────────
    int selectParent()      const;   ///< fitness-proportionate roulette
    int selectReplacement() const;   ///< uniform random over entire population

    // ── inner loop helpers ─────────────────────────────────────────────────
    void evaluateOne(int idx);              ///< run fitness eval, update fitnesses_[idx]
    void maybeSaveSnapshot(int eval_num);   ///< write YAML+PNG+MP4 to checkpoints/ on new fitness record

    // ── state ──────────────────────────────────────────────────────────────
    EvolverParams           params_;
    std::vector<Robot>      population_;
    std::vector<double>     fitnesses_;
    mutable std::mt19937    rng_;
    int                     eval_count_ = 0;
    int                     best_idx_   = 0;

    std::ofstream           fitness_log_;    ///< eval,generation,best_fitness,... per row
    std::ofstream           lineage_log_;    ///< eval,child_id,parent_id,... per row
    double                  record_fitness_ = -1.0; ///< all-time best; snapshot fires on improvement
};
