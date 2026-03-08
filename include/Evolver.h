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
    bool   enabled     = true;     ///< set false to skip PNG+MP4 (e.g. in unit tests)
    int    fps         = 30;       ///< output framerate passed to ffmpeg
    int    width       = 1280;     ///< render width in pixels
    int    height      = 720;      ///< render height in pixels
    bool   verbose     = false;    ///< if false (default) suppress raylib INFO and ffmpeg output
    double min_fitness = 0.01;     ///< skip MP4 render if best fitness is below this [m]
    float  render_vertex_radius = 0.010f; ///< visual sphere radius for vertices [m]
    float  render_bar_radius    = 0.010f; ///< visual cylinder radius for bars/actuators [m]
    float  camera_distance  = 0.8f;  ///< distance from target [m]
    float  camera_fov       = 45.0f; ///< field of view [degrees]
    float  camera_elevation = 0.8f;  ///< vertical offset ratio relative to lateral distance
    bool   camera_follow    = true;  ///< if true, camera tracks robot CoM each frame; if false, camera fixed at origin
};

// ── SelectionParams ───────────────────────────────────────────────────────────

/**
 * @brief All knobs that control how parents and replacement slots are chosen.
 *
 * Three parent-selection schemes are available.  Each uses the `pressure`
 * field differently — this is intentional; one float is simpler than three
 * scheme-specific fields:
 *
 * | scheme         | what `pressure` means                                   | pressure range |
 * |----------------|---------------------------------------------------------|----------------|
 * | proportionate  | exponent applied to raw fitness before weighting:       | ≥ 0            |
 * |                | weight_i = fitness_i ^ pressure.  1.0 = linear roulette|                |
 * |                | (current behaviour).  2.0 = squares the gaps;  4.0 =   |                |
 * |                | fourth-power.  0.0 = uniform random (all equal weight). |                |
 * | tournament     | tournament size (cast to int ≥ 2):                      | 2 .. pop_size  |
 * |                | sample `pressure` individuals at random and return the  |                |
 * |                | fittest.  2 = mild pressure, 20 = aggressive.           |                |
 * | rank           | η_max in linear rank selection ∈ [1.0, 2.0]:            | 1.0 .. 2.0     |
 * |                | the best-ranked individual gets η_max/(N/2) times the   |                |
 * |                | average weight; 2.0 = maximum linear pressure; 1.0 =   |                |
 * |                | uniform (all equal weight).  Completely insensitive to  |                |
 * |                | the absolute scale of fitness values.                   |                |
 *
 * Replacement controls which population slot the child overwrites:
 *
 * | replacement    | behaviour                                               |
 * |----------------|---------------------------------------------------------|
 * | uniform_random | any slot chosen uniformly at random (Lipson & Pollack  |
 * |                | 2000 original).  Can overwrite good robots.             |
 * | worst          | always overwrites the current worst-fitness slot.       |
 * |                | Monotonically improves worst-case fitness; combined with|
 * |                | strong selection pressure this is the most aggressive   |
 * |                | setting.                                                |
 *
 * `max_rerolls` caps the number of times a mutated child is thrown away and
 * re-mutated because it failed `robot.isValid()`.  On the 100th failure the
 * evolver falls back to an unmutated clone (guaranteed valid).  Rare in
 * practice; only relevant during rapid topology changes early in a run.
 */
struct SelectionParams
{
    std::string scheme       = "proportionate"; ///< proportionate | tournament | rank
    double      pressure     = 1.0;             ///< scheme-specific pressure knob (see table above)
    std::string replacement  = "uniform_random";///< uniform_random | worst
    int         max_rerolls  = 100;             ///< mutation retry limit before accepting a clone

    static SelectionParams fromYAML(const YAML::Node& node);
    void toYAML(YAML::Emitter& out) const;
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
 * seed: 42                  # 0 = seed from std::random_device
 * output_base_dir: runs/    # timestamped run dirs land here
 * run_dir: ""               # overrides output_base_dir when non-empty
 * report_interval: 200      # print fitness digest to stdout every N evals
 * periodic_video_interval: 200  # save best-robot video every N evals (0 = off)
 * selection:
 *   scheme:      proportionate  # proportionate | tournament | rank
 *   pressure:    1.0            # scheme-specific knob; see SelectionParams docs
 *   replacement: uniform_random # uniform_random | worst
 *   max_rerolls: 100
 * fitness:
 *   cycles: 12
 *   steps_per_cycle: 5000
 *   step_size: 1.0e-7
 *   wind: 0.0
 *   mu_static: 0.5           # static Coulomb friction coefficient
 * video:
 *   fps:             30
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
    VideoParams     video;                    ///< video rendering knobs for record snapshots
    SelectionParams selection;                ///< parent selection + replacement scheme
    double      record_min_improvement  = 0.01; ///< new best must exceed old by at least this [m]
    int         report_interval         = 200;  ///< print fitness digest to stdout every N evals
    int         periodic_video_interval = 200;  ///< save best-robot video/YAML every N evals (0=off)

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
    void maybeSavePeriodicVideo(int eval_num); ///< write checkpoints/periodic_eval_N.{yaml,mp4} every periodic_video_interval evals
    void printSelectionReport(double mean, double stddev); ///< print population/mutation stats to stdout

    /** Write checkpoint_population.yaml atomically with the current eval_count_. */
    void writePopulationCheckpoint();

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

    // ── mutation operator counters (reset every report_interval) ──────────
    int mc_perturb_          = 0;
    int mc_add_bar_new_      = 0;  ///< bars added via Strategy A (new vertex)
    int mc_add_bar_bridge_   = 0;  ///< bars added via Strategy B (bridge)
    int mc_add_neuron_          = 0;  ///< neurons added
    int mc_remove_bar_         = 0;  ///< bars removed
    int mc_remove_neuron_      = 0;  ///< neurons removed
    int mc_remove_bar_edge_    = 0;  ///< leaf bars removed (reverse of add_bar_new)
    int mc_remove_bar_bridge_  = 0;  ///< cycle bars removed (reverse of add_bar_bridge)
    int mc_join_vertex_        = 0;  ///< degree-2 vertices merged (reverse of split)
    int mc_split_vertex_       = 0;
    int mc_attach_neuron_      = 0;
    int mc_detach_neuron_      = 0;
    int mc_rewire_             = 0;
    int mc_stoch_rounds_       = 0;  ///< total stochastic-loop iterations (1/mutation = normal; >1 = low-prob regime)
    int mc_cloned_             = 0;  ///< mutations aborted (retry exhaustion)
    int mc_rerolls_          = 0;  ///< total invalid-robot rerolls
    double mc_vertex_repulse_us_ = 0.0; ///< cumulative vertex-repulsion µs this window
    double mc_bar_repulse_us_    = 0.0; ///< cumulative bar-repulsion µs this window
    int    mc_repulse_evals_     = 0;   ///< evals that had repulsion timing data
};
