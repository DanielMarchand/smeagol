/**
 * Evolver.cpp
 *
 * Implements EvolverParams YAML I/O, the Evolver constructor, and the
 * steady-state evolutionary loop.
 */

#include "Evolver.h"
#include "FitnessEvaluator.h"
#include "Mutator.h"
#include "Simulator.h"
#include "SnapshotRenderer.h"
#include "VideoRenderer.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <future>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace fs = std::filesystem;

// ── Signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_stop_requested{false};
static std::atomic<int>  g_signal_count{0};

static void signal_handler(int /*sig*/) noexcept
{
    if (g_signal_count.fetch_add(1, std::memory_order_relaxed) == 0) {
        g_stop_requested.store(true, std::memory_order_relaxed);
        constexpr char msg[] =
            "\n[Evolver] interrupt received — finishing in-flight evals, "
            "please wait...\n"
            "           (press Ctrl+C again to force quit immediately)\n";
        (void)write(STDERR_FILENO, msg, sizeof(msg) - 1);
    } else {
        constexpr char msg[] = "\n[Evolver] force quit.\n";
        (void)write(STDERR_FILENO, msg, sizeof(msg) - 1);
        _exit(130);  // 128 + SIGINT, conventional exit code
    }
}

// ── Job/result packets for parallel evaluation ────────────────────────────────

struct EvalJob {
    Robot          child;               ///< Deep copy — worker owns it entirely
    Robot::ID      parent_id;           ///< For lineage logging
    double         parent_fitness;      ///< Fitness of the selected parent
    int            parent_rank;         ///< 1-based rank of parent in population (1 = best)
    double         parent_sel_prob;     ///< Roulette selection probability share [0,1]
    MutationRecord mutation_record;     ///< Which operators fired
};

struct EvalResult {
    Robot          child;               ///< Moved back for archiving and logging
    double         fitness;
    Robot::ID      parent_id;
    double         parent_fitness;
    int            parent_rank;
    double         parent_sel_prob;
    MutationRecord mutation_record;
    double         vertex_repulse_us = 0.0; ///< avg µs/gradient-step for vertex repulsion
    double         bar_repulse_us    = 0.0; ///< avg µs/gradient-step for bar repulsion
};

using EvalFuture = std::future<EvalResult>;

// ── SelectionParams YAML I/O ─────────────────────────────────────────────────

SelectionParams SelectionParams::fromYAML(const YAML::Node& n)
{
    SelectionParams p;
    if (n["scheme"])      p.scheme      = n["scheme"].as<std::string>();
    if (n["pressure"])    p.pressure    = n["pressure"].as<double>();
    if (n["replacement"]) p.replacement = n["replacement"].as<std::string>();
    if (n["max_rerolls"]) p.max_rerolls = n["max_rerolls"].as<int>();
    return p;
}

void SelectionParams::toYAML(YAML::Emitter& out) const
{
    out << YAML::Key << "scheme"      << YAML::Value << scheme;
    out << YAML::Key << "pressure"    << YAML::Value << pressure;
    out << YAML::Key << "replacement" << YAML::Value << replacement;
    out << YAML::Key << "max_rerolls" << YAML::Value << max_rerolls;
}

// ── EvolverParams YAML I/O ────────────────────────────────────────────────────

EvolverParams EvolverParams::fromYAML(const std::string& path)
{
    YAML::Node node = YAML::LoadFile(path);
    EvolverParams p;

    if (node["population_size"])  p.population_size  = node["population_size"].as<int>();
    if (node["max_evaluations"])  p.max_evaluations   = node["max_evaluations"].as<int>();
    if (node["seed"])             p.seed              = node["seed"].as<int>();
    if (node["output_base_dir"]) p.output_base_dir   = node["output_base_dir"].as<std::string>();
    if (node["run_dir"]) {
        p.run_dir = node["run_dir"].as<std::string>();
        // Restore trailing slash stripped by toYAML so run_dir is always a
        // well-formed directory path after deserialization.
        if (!p.run_dir.empty() && p.run_dir.back() != '/')
            p.run_dir += '/';
    }

    if (const auto& f = node["fitness"]) {
        bool num_steps_set = false;
        int  legacy_nf     = -1;
        if (f["num_steps"])       { p.fitness.num_steps       = f["num_steps"].as<int>(); num_steps_set = true; }
        if (f["delay_steps"])       p.fitness.delay_steps      = f["delay_steps"].as<int>();
        if (f["num_frames"])        legacy_nf                  = f["num_frames"].as<int>();     // legacy
        if (f["cycles"])            legacy_nf                  = f["cycles"].as<int>();         // legacy-legacy
        if (f["steps_per_frame"])   p.fitness.steps_per_frame  = f["steps_per_frame"].as<int>();
        if (f["steps_per_cycle"])   p.fitness.steps_per_cycle  = f["steps_per_cycle"].as<int>();
        if (!num_steps_set && legacy_nf >= 0) {
            const int spc = p.fitness.steps_per_cycle > 0 ? p.fitness.steps_per_cycle : p.fitness.steps_per_frame;
            p.fitness.num_steps = legacy_nf * spc;
        }
        if (f["step_size"])       p.fitness.step_size       = f["step_size"].as<double>();
        if (f["gravity"])         p.fitness.gravity         = f["gravity"].as<double>();
        if (f["wind"])            p.fitness.wind            = f["wind"].as<double>();
        if (f["mu_static"])       p.fitness.mu_static       = f["mu_static"].as<double>();
        if (f["k_floor"])         p.fitness.k_floor         = f["k_floor"].as<double>();
        if (f["actuator_max_extension_per_cycle"])
            p.fitness.actuator_max_extension_per_cycle = f["actuator_max_extension_per_cycle"].as<double>();
        if (f["actuator_max_extension_total"])
            p.fitness.actuator_max_extension_total = f["actuator_max_extension_total"].as<double>();
        if (f["bar_stiffness_override"])
            p.fitness.bar_stiffness_override = f["bar_stiffness_override"].as<double>();
        if (f["k_repulse_vertex"])
            p.fitness.k_repulse_vertex = f["k_repulse_vertex"].as<double>();
        if (f["repulse_vertex_min_dist"])
            p.fitness.repulse_vertex_min_dist = f["repulse_vertex_min_dist"].as<double>();
        if (f["k_repulse_bar"])
            p.fitness.k_repulse_bar = f["k_repulse_bar"].as<double>();
        if (f["repulse_bar_min_dist"])
            p.fitness.repulse_bar_min_dist = f["repulse_bar_min_dist"].as<double>();
    }

    if (const auto& m = node["mutation"])
        p.mutation = MutatorParams::fromYAML(m);

    if (const auto& s = node["selection"])
        p.selection = SelectionParams::fromYAML(s);

    if (node["resume"]) p.resume = node["resume"].as<bool>();
    if (node["record_min_improvement"])
        p.record_min_improvement = node["record_min_improvement"].as<double>();
    if (node["report_interval"])
        p.report_interval = node["report_interval"].as<int>();
    if (node["periodic_video_interval"])
        p.periodic_video_interval = node["periodic_video_interval"].as<int>();

    if (const auto& v = node["video"]) {
        if (v["fps"])             p.video.fps             = v["fps"].as<int>();
        if (v["width"])           p.video.width           = v["width"].as<int>();
        if (v["height"])          p.video.height          = v["height"].as<int>();
        if (v["verbose"])         p.video.verbose         = v["verbose"].as<bool>();
        if (v["min_fitness"])     p.video.min_fitness     = v["min_fitness"].as<double>();
        // Legacy: old configs had video.steps_per_frame for capture granularity.
        // Migrate it to fitness.steps_per_frame (new location).
        if (v["steps_per_frame"] && p.fitness.steps_per_frame == FitnessParams{}.steps_per_frame)
            p.fitness.steps_per_frame = v["steps_per_frame"].as<int>();
    }

    return p;
}

void EvolverParams::toYAML(const std::string& path) const
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "population_size" << YAML::Value << population_size;
    out << YAML::Key << "max_evaluations" << YAML::Value << max_evaluations;
    out << YAML::Key << "seed"            << YAML::Value << seed;
    out << YAML::Key << "output_base_dir" << YAML::Value << output_base_dir;
    // Serialize run_dir as a leaf name only (strip output_base_dir prefix and
    // trailing slash) so the saved config can be used for resuming unchanged.
    {
        std::string leaf = run_dir;
        if (!output_base_dir.empty() && leaf.substr(0, output_base_dir.size()) == output_base_dir)
            leaf = leaf.substr(output_base_dir.size());
        if (!leaf.empty() && leaf.back() == '/') leaf.pop_back();
        out << YAML::Key << "run_dir" << YAML::Value << leaf;
    }
    out << YAML::Key << "fitness"         << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "num_steps"       << YAML::Value << fitness.num_steps;
    out << YAML::Key << "delay_steps"     << YAML::Value << fitness.delay_steps;
    out << YAML::Key << "steps_per_frame" << YAML::Value << fitness.steps_per_frame;
    out << YAML::Key << "steps_per_cycle" << YAML::Value << fitness.steps_per_cycle;
    out << YAML::Key << "step_size"       << YAML::Value << fitness.step_size;
    out << YAML::Key << "gravity"         << YAML::Value << fitness.gravity;
    out << YAML::Key << "wind"            << YAML::Value << fitness.wind;
    out << YAML::Key << "mu_static"       << YAML::Value << fitness.mu_static;
    out << YAML::Key << "k_floor"         << YAML::Value << fitness.k_floor;
    out << YAML::Key << "actuator_max_extension_per_cycle" << YAML::Value << fitness.actuator_max_extension_per_cycle;
    out << YAML::Key << "actuator_max_extension_total"     << YAML::Value << fitness.actuator_max_extension_total;
    out << YAML::Key << "k_repulse_vertex"        << YAML::Value << fitness.k_repulse_vertex;
    out << YAML::Key << "repulse_vertex_min_dist" << YAML::Value << fitness.repulse_vertex_min_dist;
    out << YAML::Key << "k_repulse_bar"           << YAML::Value << fitness.k_repulse_bar;
    out << YAML::Key << "repulse_bar_min_dist"    << YAML::Value << fitness.repulse_bar_min_dist;
    out << YAML::EndMap;  // fitness
    out << YAML::Key << "mutation"        << YAML::Value << YAML::BeginMap;
    mutation.toYAML(out);
    out << YAML::EndMap;  // mutation
    out << YAML::Key << "selection"       << YAML::Value << YAML::BeginMap;
    selection.toYAML(out);
    out << YAML::EndMap;  // selection
    out << YAML::Key << "resume"                   << YAML::Value << resume;
    out << YAML::Key << "record_min_improvement"    << YAML::Value << record_min_improvement;
    out << YAML::Key << "report_interval"           << YAML::Value << report_interval;
    out << YAML::Key << "periodic_video_interval"   << YAML::Value << periodic_video_interval;
    out << YAML::Key << "video"           << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "fps"             << YAML::Value << video.fps;
    out << YAML::Key << "width"           << YAML::Value << video.width;
    out << YAML::Key << "height"          << YAML::Value << video.height;
    out << YAML::Key << "verbose"         << YAML::Value << video.verbose;
    out << YAML::Key << "min_fitness"     << YAML::Value << video.min_fitness;
    out << YAML::EndMap;  // video
    out << YAML::EndMap;  // root

    std::ofstream f(path);
    if (!f) throw std::runtime_error("EvolverParams::toYAML: cannot write to '" + path + "'");
    f << out.c_str() << "\n";
}

// ── Evolver constructor ───────────────────────────────────────────────────────

static std::string timestampDir(const std::string& base)
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return base + "run_" + buf + "/";
}

Evolver::Evolver(EvolverParams params)
    : params_(std::move(params))
{
    // ── Resolve seed ──────────────────────────────────────────────────────────
    if (params_.seed == 0) {
        std::random_device rd;
        params_.seed = static_cast<int>(rd());
    }
    rng_.seed(static_cast<unsigned>(params_.seed));

    // ── Resolve run_dir ───────────────────────────────────────────────────────
    // run_dir in config is a leaf name only (e.g. "run_20260306_000952").
    // We always prepend output_base_dir so both are relative to cwd the same way.
    if (!params_.run_dir.empty()) {
        fs::path leaf = params_.run_dir;
        if (leaf.is_relative())
            params_.run_dir = (fs::path(params_.output_base_dir) / leaf).string() + "/";
        else if (params_.run_dir.back() != '/')
            params_.run_dir += "/";
    }

    if (params_.resume) {
        // ── Resume path: restore from checkpoint_population.yaml ─────────────
        if (params_.run_dir.empty())
            throw std::logic_error(
                "Evolver: resume=true but run_dir is empty.\n"
                "  Add 'run_dir: run_<timestamp>' to your config (leaf name only,\n"
                "  relative to output_base_dir).");

        const std::string pop_path = params_.run_dir + "checkpoint_population.yaml";
        if (!fs::exists(pop_path))
            throw std::logic_error(
                "Evolver: resume requested but '" +
                fs::absolute(pop_path).string() + "' not found");

        YAML::Node node = YAML::LoadFile(pop_path);
        eval_count_ = node["eval_count"].as<int>();

        population_.clear();
        fitnesses_.clear();
        for (const auto& entry : node["population"]) {
            const std::string rpath =
                params_.run_dir + entry["path"].as<std::string>();
            population_.push_back(Robot::fromYAML(rpath));
            fitnesses_.push_back(entry["fitness"].as<double>());
        }

        best_idx_ = static_cast<int>(
            std::max_element(fitnesses_.begin(), fitnesses_.end())
            - fitnesses_.begin());
        record_fitness_ = fitnesses_[best_idx_];  // don't re-snapshot on resume

        // Append to existing CSV logs (no re-writing headers).
        fitness_log_.open(params_.run_dir + "fitness_log.csv", std::ios::app);
        lineage_log_.open(params_.run_dir + "lineage.csv",    std::ios::app);

        std::cout << "[Evolver] RESUMING " << fs::absolute(params_.run_dir).string() << "\n"
                  << "[Evolver] restored " << eval_count_ << " evals, "
                  << population_.size() << " robots, "
                  << "best_fitness=" << fitnesses_[best_idx_] << "m\n";
    } else {
        // ── Fresh-start path ──────────────────────────────────────────────────
        // ── Resolve run directory ─────────────────────────────────────────────
        if (params_.run_dir.empty())
            params_.run_dir = timestampDir(params_.output_base_dir);

        fs::create_directories(params_.run_dir);
        fs::create_directories(params_.run_dir + "robots/");
        fs::create_directories(params_.run_dir + "checkpoints/");

        // ── Persist resolved config for reproducibility ───────────────────────
        params_.toYAML(params_.run_dir + "run_config.yaml");

        // ── Initialise population (all empty robots, zero fitness) ────────────
        population_.reserve(params_.population_size);
        for (int i = 0; i < params_.population_size; ++i) {
            population_.emplace_back();   // Robot() — empty, auto-assigned ID
            // Save immediately so checkpoint_population.yaml can reference them.
            population_.back().toYAML(
                params_.run_dir + "robots/robot_" +
                std::to_string(population_.back().id) + ".yaml");
        }

        fitnesses_.assign(params_.population_size, 0.0);

        // ── Open fitness log ──────────────────────────────────────────────────
        fitness_log_.open(params_.run_dir + "fitness_log.csv");
        fitness_log_ << "eval,generation,best_fitness,mean_fitness,"
                        "best_robot_id,best_v,best_b,best_n,best_a\n";

        // ── Open lineage log ────────────────────────────────────────────────
        lineage_log_.open(params_.run_dir + "lineage.csv");
        lineage_log_ << "eval,child_id,parent_id,child_fitness,"
                        "replaced_id,replaced_fitness\n";

        std::cout << "[Evolver] run_dir  : " << fs::absolute(params_.run_dir).string() << "\n"
                  << "[Evolver] seed     : " << params_.seed      << "\n"
                  << "[Evolver] pop_size : " << params_.population_size << "\n"
                  << "[Evolver] max_evals: " << params_.max_evaluations << "\n";
    }
}

// ── Accessors ─────────────────────────────────────────────────────────────────

const Robot& Evolver::bestRobot() const
{
    return population_[best_idx_];
}

double Evolver::bestFitness() const
{
    return fitnesses_[best_idx_];
}

bool Evolver::wasInterrupted() const
{
    return g_stop_requested.load(std::memory_order_relaxed);
}

// ── Evolutionary Loop ────────────────────────────────────────────────────────

// ── Parent selection ─────────────────────────────────────────────────────────
//
// Three schemes, all controlled by params_.selection:
//
//  proportionate — weight_i = fitness_i ^ pressure
//      pressure=1.0  → classic Lipson & Pollack linear roulette.
//      pressure=2.0  → squares the fitness gaps (quadratic amplification).
//      pressure=0.0  → uniform random (all equal weight).
//
//  tournament     — select `pressure` (cast to int) candidates at random;
//      return the fittest.  pressure=2 = mild; pressure=20 = very aggressive.
//      Completely insensitive to fitness scale (unlike proportionate).
//
//  rank           — sort population by fitness, assign weights by rank.
//      weight_i = 2*rank_i / (N*(N+1))  scaled by η_max (= pressure ∈ [1,2]):
//        weight_i = (2 - pressure)/N  +  (2*(pressure-1)*rank_i) / (N*(N-1))
//      rank 1 = worst, rank N = best.
//      pressure=1.0 → all equal weight (uniform).
//      pressure=2.0 → maximum linear rank pressure; best gets 2/N share.
//      Also completely insensitive to fitness scale.

int Evolver::selectParent() const
{
    const int n = static_cast<int>(population_.size());
    const SelectionParams& sp = params_.selection;

    // ── proportionate ─────────────────────────────────────────────────────
    if (sp.scheme == "proportionate") {
        if (sp.pressure == 0.0) {
            // degenerate: uniform random
            return std::uniform_int_distribution<int>(0, n - 1)(rng_);
        }
        // Compute weights = fitness^pressure; fall back to uniform if all 0.
        std::vector<double> w(n);
        double total = 0.0;
        for (int i = 0; i < n; ++i) {
            w[i] = (fitnesses_[i] > 0.0)
                   ? std::pow(fitnesses_[i], sp.pressure)
                   : 0.0;
            total += w[i];
        }
        if (total <= 0.0)
            return std::uniform_int_distribution<int>(0, n - 1)(rng_);
        double r = std::uniform_real_distribution<double>(0.0, total)(rng_);
        double acc = 0.0;
        for (int i = 0; i < n; ++i) {
            acc += w[i];
            if (acc >= r) return i;
        }
        return n - 1;
    }

    // ── tournament ────────────────────────────────────────────────────────
    if (sp.scheme == "tournament") {
        const int k = std::max(2, static_cast<int>(sp.pressure));
        auto pick = std::uniform_int_distribution<int>(0, n - 1);
        int best = pick(rng_);
        for (int t = 1; t < k; ++t) {
            int challenger = pick(rng_);
            if (fitnesses_[challenger] > fitnesses_[best])
                best = challenger;
        }
        return best;
    }

    // ── rank ──────────────────────────────────────────────────────────────
    // (also the fallback for any unrecognised scheme string)
    {
        const double eta = std::clamp(sp.pressure, 1.0, 2.0);
        // Sort indices worst→best (rank 0=worst, rank n-1=best).
        std::vector<int> order(n);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
                  [&](int a, int b){ return fitnesses_[a] < fitnesses_[b]; });

        // Linear rank weight: w(rank) = (2-eta)/n + 2*(eta-1)*rank/(n*(n-1))
        // rank ∈ {0,1,...,n-1}.
        std::vector<double> w(n);
        double total = 0.0;
        for (int r = 0; r < n; ++r) {
            w[order[r]] = (2.0 - eta) / n
                        + 2.0 * (eta - 1.0) * r / (n * std::max(n - 1, 1));
            total += w[order[r]];
        }
        double rnd = std::uniform_real_distribution<double>(0.0, total)(rng_);
        double acc = 0.0;
        for (int i = 0; i < n; ++i) {
            acc += w[i];
            if (acc >= rnd) return i;
        }
        return n - 1;
    }
}

// Replacement: choose which population slot the child overwrites.
// uniform_random — any slot (can overwrite excellent robots).
// worst          — overwrites the current weakest slot; ties broken uniformly
//                  at random so multiple concurrent workers don't all target
//                  the same index when fitnesses are equal.
int Evolver::selectReplacement() const
{
    const int n = static_cast<int>(population_.size());
    if (params_.selection.replacement == "worst") {
        double min_f = *std::min_element(fitnesses_.begin(), fitnesses_.end());
        std::vector<int> candidates;
        candidates.reserve(n);
        for (int i = 0; i < n; ++i)
            if (fitnesses_[i] == min_f) candidates.push_back(i);
        return candidates[
            std::uniform_int_distribution<int>(0, (int)candidates.size() - 1)(rng_)];
    }
    // default: uniform_random
    return std::uniform_int_distribution<int>(0, n - 1)(rng_);
}

// Run FitnessEvaluator on population_[idx] and update tracking state.
void Evolver::evaluateOne(int idx)
{
    double f = FitnessEvaluator(params_.fitness).evaluate(population_[idx]);
    fitnesses_[idx] = f;
    if (f > fitnesses_[best_idx_])
        best_idx_ = idx;
}

// Save a periodic best-robot video every params_.periodic_video_interval evals.
// Always fires regardless of whether a new fitness record was set.
void Evolver::maybeSavePeriodicVideo(int eval_num)
{
    const int interval = params_.periodic_video_interval;
    if (interval <= 0 || eval_num % interval != 0) return;
    if (!params_.video.enabled) return;

    const double best_fitness = fitnesses_[best_idx_];
    if (best_fitness < params_.video.min_fitness) {
        std::cout << "[Evolver] periodic video skipped (fitness " << best_fitness
                  << " < min_fitness " << params_.video.min_fitness << ")\n";
        return;
    }

    const std::string stem = params_.run_dir + "checkpoints/periodic_eval_"
                           + std::to_string(eval_num);

    // YAML snapshot of the current best.
    population_[best_idx_].toYAML(stem + ".yaml");

    // MP4 video.
    try {
        FitnessEvaluator::renderVideo(population_[best_idx_], params_.fitness,
                                      stem + ".mp4", params_.video.fps,
                                      params_.video.width, params_.video.height,
                                      params_.video.verbose);
        std::cout << "[Evolver] periodic video \u2192 " << fs::absolute(stem + ".mp4").string() << "\n";
    } catch (const std::exception& e) {
        std::cerr << "[Evolver] periodic video skipped at eval " << eval_num
                  << ": " << e.what() << "\n";
    }
}

// Print a selection/mutation stats digest to stdout.
// Called every report_interval evals by collect_one.
void Evolver::printSelectionReport(double mean, double stddev)
{
    const int n = static_cast<int>(fitnesses_.size());

    // ── Selection pressure metric: fraction of fitness weight
    //    held by top 10% — consistent observable regardless of scheme.
    double total = 0.0;
    for (double f : fitnesses_) total += f;
    std::vector<double> sorted = fitnesses_;
    std::sort(sorted.begin(), sorted.end(), std::greater<double>());
    const int top10_n = std::max(1, n / 10);
    double top10_sum = 0.0;
    for (int i = 0; i < top10_n; ++i) top10_sum += sorted[i];
    const double top10_share = (total > 0.0) ? (top10_sum / total * 100.0) : 0.0;

    // ── Average population topology ──────────────────────────────────────
    double avg_v = 0, avg_b = 0, avg_n = 0, avg_a = 0;
    for (const Robot& r : population_) {
        avg_v += r.vertices.size();
        avg_b += r.bars.size();
        avg_n += r.neurons.size();
        avg_a += r.actuators.size();
    }
    const double pn = static_cast<double>(population_.size());
    avg_v /= pn; avg_b /= pn; avg_n /= pn; avg_a /= pn;

    // ── Report ───────────────────────────────────────────────────────────
    const SelectionParams& sp = params_.selection;
    std::ostringstream sel;
    sel << std::fixed << std::setprecision(1) << sp.scheme;
    if      (sp.scheme == "proportionate") sel << " (pressure=" << sp.pressure << ")";
    else if (sp.scheme == "tournament")    sel << " (size=" << static_cast<int>(sp.pressure) << ")";
    else                                   sel << " (eta=" << sp.pressure << ")";
    sel << "  replace=" << sp.replacement;

    std::cout << std::fixed << std::setprecision(1)
              << "  pop        "
              << avg_v << "v  " << avg_b << "b  " << avg_n << "n  " << avg_a << "a  avg"
              << "   sigma=" << std::setprecision(4) << stddev << "m"
              << "   top10=" << std::setprecision(1) << top10_share << "%\n"
              << "  selection  " << sel.str() << "\n"
              << "  mutations  last " << params_.report_interval
              << "   rounds=" << mc_stoch_rounds_
              << "   cloned=" << mc_cloned_
              << "   rerolls=" << mc_rerolls_ << "\n"
              << "    topology  "
              << "perturb=" << mc_perturb_
              << "  split=" << mc_split_vertex_
              << "  join=" << mc_join_vertex_
              << "  attach=" << mc_attach_neuron_
              << "  detach=" << mc_detach_neuron_
              << "  rewire=" << mc_rewire_ << "\n"
              << "    struct    "
              << "+bar=" << mc_add_bar_new_
              << "  +bridge=" << mc_add_bar_bridge_
              << "  -bar=" << mc_remove_bar_
              << "  -bar_e=" << mc_remove_bar_edge_
              << "  -bar_b=" << mc_remove_bar_bridge_
              << "  +neuron=" << mc_add_neuron_
              << "  -neuron=" << mc_remove_neuron_ << "\n";

    // Self-collision timing — only print when either term is enabled.
    if (mc_repulse_evals_ > 0) {
        const double avg_vv = mc_vertex_repulse_us_ / mc_repulse_evals_;
        const double avg_bb = mc_bar_repulse_us_    / mc_repulse_evals_;
        std::cout << std::fixed << std::setprecision(3)
                  << "    repulsion vertex-avg=" << avg_vv << "µs"
                  << "  bar-avg=" << avg_bb << "µs"
                  << "  (over " << mc_repulse_evals_ << " evals)\n";
    }
}

// Save the best robot's YAML + PNG + MP4 to checkpoints/ whenever a new fitness record is set.
void Evolver::maybeSaveSnapshot(int eval_num)
{
    const double current_best = fitnesses_[best_idx_];
    if (current_best < record_fitness_ + params_.record_min_improvement) return;
    record_fitness_ = current_best;

    const std::string stem = params_.run_dir + "checkpoints/record_eval_" + std::to_string(eval_num);

    // Always save YAML.
    population_[best_idx_].toYAML(stem + ".yaml");

    // ── Atomic checkpoint_latest.yaml (best robot, safe for crash recovery) ──
    {
        const std::string tmp = params_.run_dir + "checkpoint_latest.yaml.tmp";
        population_[best_idx_].toYAML(tmp);
        fs::rename(tmp, params_.run_dir + "checkpoint_latest.yaml");
    }

    writePopulationCheckpoint();

    std::cout << "[Evolver] checkpoint saved → " << fs::absolute(stem + ".yaml").string() << "\n";

    if (!params_.video.enabled) return;

    // Save PNG snapshot — requires a display; skip gracefully if unavailable.
    try {
        SnapshotRenderer snap;
        snap.setVerbose(params_.video.verbose);
        snap.render_vertex_radius = static_cast<float>(params_.fitness.repulse_vertex_min_dist / 2.0);
        snap.render_bar_radius    = static_cast<float>(params_.fitness.repulse_bar_min_dist    / 2.0);
        snap.render(population_[best_idx_], stem + ".png");
    } catch (const std::exception& e) {
        std::cerr << "[Evolver] snapshot PNG skipped at eval " << eval_num
                  << ": " << e.what() << "\n";
    }

    // Save MP4 video — skip if best fitness is below the configured threshold.
    if (current_best < params_.video.min_fitness) {
        std::cout << "[Evolver] video skipped (fitness " << current_best
                  << " < min_fitness " << params_.video.min_fitness << ")\n";
        return;
    }
    try {
        FitnessEvaluator::renderVideo(population_[best_idx_], params_.fitness,
                                      stem + ".mp4", params_.video.fps,
                                      params_.video.width, params_.video.height,
                                      params_.video.verbose);
        std::cout << "[Evolver] video saved → " << fs::absolute(stem + ".mp4").string() << "\n";
    } catch (const std::exception& e) {
        std::cerr << "[Evolver] video MP4 skipped at eval " << eval_num
                  << ": " << e.what() << "\n";
    }
}

void Evolver::writePopulationCheckpoint()
{
    YAML::Emitter pop_out;
    pop_out << YAML::BeginMap;
    pop_out << YAML::Key << "eval_count" << YAML::Value << eval_count_;
    pop_out << YAML::Key << "population" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < static_cast<int>(population_.size()); ++i) {
        pop_out << YAML::BeginMap;
        pop_out << YAML::Key << "path" << YAML::Value
                << ("robots/robot_" + std::to_string(population_[i].id) + ".yaml");
        pop_out << YAML::Key << "fitness" << YAML::Value << fitnesses_[i];
        pop_out << YAML::EndMap;
    }
    pop_out << YAML::EndSeq;
    pop_out << YAML::EndMap;

    const std::string tmp = params_.run_dir + "checkpoint_population.yaml.tmp";
    {
        std::ofstream f(tmp);
        f << pop_out.c_str() << "\n";
    }
    fs::rename(tmp, params_.run_dir + "checkpoint_population.yaml");
}

// Poll all in-flight jobs and return the index of whichever one finishes first.
// Checks each job instantly (no_wait), then sleeps briefly before trying again
// so the manager thread doesn't busy-spin an entire core while waiting.
static int waitForAny(std::vector<EvalFuture>& jobs)
{
    const auto no_wait    = std::chrono::microseconds(0);
    const auto poll_sleep = std::chrono::microseconds(200);

    while (true) {
        for (int i = 0; i < (int)jobs.size(); ++i) {
            if (jobs[i].wait_for(no_wait) == std::future_status::ready)
                return i;
        }
        std::this_thread::sleep_for(poll_sleep);
    }
}

// Main steady-state loop — parallel flight-window design.
void Evolver::run()
{
    // ── Install signal handlers ───────────────────────────────────────────
    g_stop_requested.store(false, std::memory_order_relaxed);
    g_signal_count.store(0,     std::memory_order_relaxed);
    auto prev_sigint  = std::signal(SIGINT,  signal_handler);
    auto prev_sigterm = std::signal(SIGTERM, signal_handler);

    // ── Worker count: reserve 1 core for the manager ──────────────────────
    const int W = std::max(1,
        static_cast<int>(std::thread::hardware_concurrency()) - 1);
    std::cout << "[Evolver] using " << W << " worker thread(s)\n";

    // ── Worker lambda — zero shared state, purely functional ──────────────
    // Captures fitness params by value so workers need no shared data.
    auto eval_worker = [fitness_params = params_.fitness](EvalJob job) -> EvalResult {
        FitnessEvaluator fe(fitness_params);
        double f = fe.evaluate(job.child);
        // Retrieve self-collision timing from the internal simulator.
        // FitnessEvaluator exposes the last sim's averages via the params-built sim.
        // We re-read from the fitness params' sim via the last accumulated data.
        // Since evaluate() constructs a fresh Simulator, we capture timing via
        // a thin accessor added below.
        return EvalResult{ std::move(job.child), f,
                           job.parent_id, job.parent_fitness,
                           job.parent_rank, job.parent_sel_prob,
                           job.mutation_record,
                           fe.lastVertexRepulseUs(),
                           fe.lastBarRepulseUs() };
    };

    // ── Helpers ───────────────────────────────────────────────────────────
    const int kMaxRerolls = params_.selection.max_rerolls;

    std::vector<EvalFuture> in_flight;

    // Build one mutated child and submit it for evaluation.
    auto submit_one = [&]() {
        const int parent_idx = selectParent();
        const Robot::ID  pid        = population_[parent_idx].id;
        const double     pfitness   = fitnesses_[parent_idx];

        // ── Compute parent selection context ──────────────────────────────
        // Rank: 1 = highest fitness.
        const int n = static_cast<int>(fitnesses_.size());
        std::vector<int> order(n);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
                  [&](int a, int b){ return fitnesses_[a] > fitnesses_[b]; });
        int parent_rank = 1;
        for (int i = 0; i < n; ++i)
            if (order[i] == parent_idx) { parent_rank = i + 1; break; }

        double total_fitness = 0.0;
        for (double f : fitnesses_) total_fitness += f;
        const double parent_sel_prob =
            (total_fitness > 0.0) ? (pfitness / total_fitness) : (1.0 / n);

        Robot child = population_[parent_idx].clone();
        child.id = Robot::nextId();
        MutationRecord mrec = Mutator::mutateRecord(child, rng_, params_.mutation);

        int rerolls = 0;
        while (!child.isValid()) {
            ++rerolls;
            if (rerolls >= kMaxRerolls) {
                std::cerr << "[Evolver] warning: reroll limit hit at eval "
                          << eval_count_ << " — using unmutated clone\n";
                child = population_[parent_idx].clone();
                child.id = Robot::nextId();
                mrec = MutationRecord{};
                mrec.v_after = (int)child.vertices.size();
                mrec.b_after = (int)child.bars.size();
                mrec.n_after = (int)child.neurons.size();
                mrec.a_after = (int)child.actuators.size();
                break;
            }
            child = population_[parent_idx].clone();
            child.id = Robot::nextId();
            mrec = Mutator::mutateRecord(child, rng_, params_.mutation);
        }
        mrec.rerolls = rerolls;

        EvalJob job{ std::move(child), pid,
                     pfitness, parent_rank, parent_sel_prob, mrec };
        in_flight.push_back(
            std::async(std::launch::async, eval_worker, std::move(job)));
    };

    // Collect whichever in-flight job finishes first and apply it to the population.
    // Using waitForAny instead of always blocking on the front means a single slow
    // robot can't stall the pipeline — all other workers keep running.
    auto collect_one = [&]() {
        const int i    = waitForAny(in_flight);
        EvalResult res = in_flight[i].get();

        // Swap-erase: move the last element into slot i then pop the tail.
        // O(1) and avoids shifting the whole vector.
        in_flight[i] = std::move(in_flight.back());
        in_flight.pop_back();

        // Choose replacement slot now, against the current population state,
        // not at submission time (which caused all concurrent workers to target
        // the same slot when using replacement=worst with tied fitnesses).
        const int       ridx              = selectReplacement();
        const Robot::ID replaced_id       = population_[ridx].id;
        const double    replaced_fitness  = fitnesses_[ridx];

        population_[ridx] = std::move(res.child);
        fitnesses_[ridx]  = res.fitness;
        population_[ridx].parent_id = res.parent_id;
        population_[ridx].fitness   = res.fitness;

        const Robot::ID child_id = population_[ridx].id;

        // Archive robot YAML on disk.
        population_[ridx].toYAML(
            params_.run_dir + "robots/robot_" +
            std::to_string(child_id) + ".yaml");

        // ── Per-robot lineage + mutation log ──────────────────────────────
        {
            const int pop_n = static_cast<int>(fitnesses_.size());
            const double improvement = (res.parent_fitness > 0.0)
                ? ((res.fitness - res.parent_fitness) / res.parent_fitness * 100.0)
                : 0.0;

            std::ofstream rlog(params_.run_dir + "robots/robot_" +
                               std::to_string(child_id) + ".txt");
            rlog << std::fixed << std::setprecision(4)
                 << "=== Robot " << child_id << " ===\n"
                 << "eval            : " << (eval_count_ + 1) << "\n"
                 << "parent_id       : " << res.parent_id << "\n"
                 << "parent_fitness  : " << res.parent_fitness << " m"
                 << "  (rank " << res.parent_rank << "/" << pop_n
                 << ", " << std::setprecision(2) << (res.parent_sel_prob * 100.0)
                 << "% roulette share)\n"
                 << std::setprecision(4)
                 << "mutation        : " << res.mutation_record.describe() << "\n"
                 << "child_fitness   : " << res.fitness << " m"
                 << "  (" << std::showpos << std::setprecision(2) << improvement
                 << std::noshowpos << "% vs parent)\n";
        }

        // ── Update mutation op counters ───────────────────────────────────
        if (res.mutation_record.perturb)        ++mc_perturb_;
        if (res.mutation_record.add_bar_new)    ++mc_add_bar_new_;
        if (res.mutation_record.add_bar_bridge) ++mc_add_bar_bridge_;
        if (res.mutation_record.add_neuron)     ++mc_add_neuron_;
        if (res.mutation_record.remove_bar)      ++mc_remove_bar_;
        if (res.mutation_record.remove_neuron)   ++mc_remove_neuron_;
        if (res.mutation_record.remove_bar_edge)    ++mc_remove_bar_edge_;
        if (res.mutation_record.remove_bar_bridge)  ++mc_remove_bar_bridge_;
        if (res.mutation_record.join_vertex)         ++mc_join_vertex_;
        if (res.mutation_record.was_cloned)          ++mc_cloned_;
        if (res.mutation_record.split_vertex)        ++mc_split_vertex_;
        if (res.mutation_record.attach_neuron)       ++mc_attach_neuron_;
        if (res.mutation_record.detach_neuron)       ++mc_detach_neuron_;
        if (res.mutation_record.rewire)              ++mc_rewire_;
        mc_stoch_rounds_ += res.mutation_record.stoch_rounds;
        mc_rerolls_      += res.mutation_record.rerolls;
        if (res.vertex_repulse_us > 0.0 || res.bar_repulse_us > 0.0) {
            mc_vertex_repulse_us_ += res.vertex_repulse_us;
            mc_bar_repulse_us_    += res.bar_repulse_us;
            ++mc_repulse_evals_;
        }

        // Update best_idx_.
        if (ridx == best_idx_) {
            best_idx_ = static_cast<int>(
                std::max_element(fitnesses_.begin(), fitnesses_.end()) -
                fitnesses_.begin());
        } else if (res.fitness > fitnesses_[best_idx_]) {
            best_idx_ = ridx;
        }

        // Bookkeeping.
        ++eval_count_;
        {
            double mean = 0.0;
            for (double f : fitnesses_) mean += f;
            mean /= static_cast<double>(fitnesses_.size());

            const Robot& best = population_[best_idx_];
            fitness_log_ << eval_count_
                         << "," << (eval_count_ / params_.population_size)
                         << "," << fitnesses_[best_idx_]
                         << "," << mean
                         << "," << best.id
                         << "," << best.vertices.size()
                         << "," << best.bars.size()
                         << "," << best.neurons.size()
                         << "," << best.actuators.size()
                         << "\n";

            lineage_log_ << eval_count_
                         << "," << child_id
                         << "," << res.parent_id
                         << "," << res.fitness
                         << "," << replaced_id
                         << "," << replaced_fitness
                         << "\n";

            if (eval_count_ % 1000 == 0) {
                fitness_log_.flush();
                lineage_log_.flush();
            }

            const int ri = params_.report_interval;
            if (ri > 0 && eval_count_ % ri == 0) {
                double sum2 = 0.0;
                for (double f : fitnesses_) sum2 += (f - mean) * (f - mean);
                const double stddev = std::sqrt(sum2 / static_cast<double>(fitnesses_.size()));
                std::cout << std::fixed << std::setprecision(4)
                          << "eval=" << eval_count_
                          << "  best=" << fitnesses_[best_idx_] << "m"
                          << "  mean=" << mean << "m\n";
                printSelectionReport(mean, stddev);

                // Reset mutation counters for next interval.
                mc_perturb_ = mc_add_bar_new_ = mc_add_bar_bridge_ =
                    mc_add_neuron_ = mc_remove_bar_ = mc_remove_neuron_ =
                    mc_remove_bar_edge_ = mc_remove_bar_bridge_ = mc_join_vertex_ = mc_split_vertex_ =
                    mc_attach_neuron_ = mc_detach_neuron_ = mc_rewire_ =
                    mc_stoch_rounds_ = mc_cloned_ = mc_rerolls_ = 0;
                mc_vertex_repulse_us_ = mc_bar_repulse_us_ = 0.0;
                mc_repulse_evals_ = 0;
            }
        }

        maybeSaveSnapshot(eval_count_);
        maybeSavePeriodicVideo(eval_count_);
    };

    // ── Phase 1: fill the window ──────────────────────────────────────────
    while (static_cast<int>(in_flight.size()) < W
           && eval_count_ + static_cast<int>(in_flight.size())
              < params_.max_evaluations)
        submit_one();

    // ── Phase 2: rolling window ───────────────────────────────────────────
    while (eval_count_ < params_.max_evaluations) {
        if (g_stop_requested.load(std::memory_order_relaxed)) {
            std::cerr << "[Evolver] graceful stop — draining "
                      << in_flight.size() << " in-flight eval(s)...\n";
            break;
        }
        collect_one();
        if (eval_count_ + static_cast<int>(in_flight.size())
            < params_.max_evaluations)
            submit_one();
    }

    // ── Phase 3: drain remaining in-flight jobs ───────────────────────────
    while (!in_flight.empty())
        collect_one();

    // ── Final summary ─────────────────────────────────────────────────────
    fitness_log_.flush();
    lineage_log_.flush();

    // Always persist the final population state so resume always starts from
    // the true end-of-run eval_count, not just the last record snapshot.
    writePopulationCheckpoint();

    const std::string best_path = params_.run_dir + "best_robot_final.yaml";
    population_[best_idx_].toYAML(best_path);
    std::cout << "[Evolver] run complete.  eval=" << eval_count_
              << "  best_fitness=" << fitnesses_[best_idx_] << "m\n"
              << "[Evolver] best robot saved → " << best_path << "\n";

    // ── Restore signal handlers ───────────────────────────────────────────
    std::signal(SIGINT,  prev_sigint);
    std::signal(SIGTERM, prev_sigterm);
}
