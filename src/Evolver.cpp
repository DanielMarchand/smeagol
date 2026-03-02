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
#include <csignal>
#include <ctime>
#include <deque>
#include <filesystem>
#include <future>
#include <iomanip>
#include <iostream>
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
    Robot        child;        ///< Deep copy — worker owns it entirely
    int          replace_idx;  ///< Replacement slot, chosen by manager at submission
    Robot::ID    parent_id;    ///< For lineage logging
};

struct EvalResult {
    Robot        child;        ///< Moved back for archiving and logging
    int          replace_idx;
    double       fitness;
    Robot::ID    parent_id;
};

// ── EvolverParams YAML I/O ────────────────────────────────────────────────────

EvolverParams EvolverParams::fromYAML(const std::string& path)
{
    YAML::Node node = YAML::LoadFile(path);
    EvolverParams p;

    if (node["population_size"])  p.population_size  = node["population_size"].as<int>();
    if (node["max_evaluations"])  p.max_evaluations   = node["max_evaluations"].as<int>();
    if (node["seed"])             p.seed              = node["seed"].as<int>();
    if (node["output_base_dir"]) p.output_base_dir   = node["output_base_dir"].as<std::string>();
    if (node["run_dir"])          p.run_dir           = node["run_dir"].as<std::string>();

    if (const auto& f = node["fitness"]) {
        if (f["cycles"])          p.fitness.cycles          = f["cycles"].as<int>();
        if (f["steps_per_cycle"]) p.fitness.steps_per_cycle = f["steps_per_cycle"].as<int>();
        if (f["step_size"])       p.fitness.step_size       = f["step_size"].as<double>();
        if (f["wind"])            p.fitness.wind            = f["wind"].as<double>();
        if (f["mu_static"])       p.fitness.mu_static       = f["mu_static"].as<double>();
    }

    if (const auto& m = node["mutation"])
        p.mutation = MutatorParams::fromYAML(m);

    if (node["resume"]) p.resume = node["resume"].as<bool>();

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
    out << YAML::Key << "run_dir"         << YAML::Value << run_dir;
    out << YAML::Key << "fitness"         << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "cycles"          << YAML::Value << fitness.cycles;
    out << YAML::Key << "steps_per_cycle" << YAML::Value << fitness.steps_per_cycle;
    out << YAML::Key << "step_size"       << YAML::Value << fitness.step_size;
    out << YAML::Key << "wind"            << YAML::Value << fitness.wind;
    out << YAML::Key << "mu_static"       << YAML::Value << fitness.mu_static;
    out << YAML::EndMap;  // fitness
    out << YAML::Key << "mutation"        << YAML::Value << YAML::BeginMap;
    mutation.toYAML(out);
    out << YAML::EndMap;  // mutation
    out << YAML::Key << "resume"          << YAML::Value << resume;
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

    if (params_.resume) {
        // ── Resume path: restore from checkpoint_population.yaml ─────────────
        if (params_.run_dir.empty())
            throw std::logic_error("Evolver: resume=true but run_dir is empty");

        const std::string pop_path = params_.run_dir + "checkpoint_population.yaml";
        if (!fs::exists(pop_path))
            throw std::logic_error(
                "Evolver: resume requested but '" + pop_path + "' not found");

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

        std::cout << "[Evolver] RESUMING " << params_.run_dir << "\n"
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

        std::cout << "[Evolver] run_dir  : " << params_.run_dir  << "\n"
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

// Fitness-proportionate roulette-wheel selection.
// Falls back to uniform random when all fitnesses are 0.
int Evolver::selectParent() const
{
    const int n = static_cast<int>(population_.size());
    double total = 0.0;
    for (double f : fitnesses_) total += f;

    if (total <= 0.0)
        return std::uniform_int_distribution<int>(0, n - 1)(rng_);

    double r = std::uniform_real_distribution<double>(0.0, total)(rng_);
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        sum += fitnesses_[i];
        if (sum >= r) return i;
    }
    return n - 1;  // rounding safety fallback
}

// Uniform-random replacement — any slot, including the parent's.
int Evolver::selectReplacement() const
{
    const int n = static_cast<int>(population_.size());
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

// Save the best robot's YAML + PNG + MP4 to checkpoints/ whenever a new fitness record is set.
void Evolver::maybeSaveSnapshot(int eval_num)
{
    const double current_best = fitnesses_[best_idx_];
    if (current_best <= record_fitness_) return;
    record_fitness_ = current_best;

    const std::string stem = params_.run_dir + "checkpoints/record_eval_" + std::to_string(eval_num);

    // Always save YAML.
    population_[best_idx_].toYAML(stem + ".yaml");

    // Save PNG snapshot — requires a display; skip gracefully if unavailable.
    try {
        SnapshotRenderer snap;
        snap.render(population_[best_idx_], stem + ".png");
    } catch (const std::exception& e) {
        std::cerr << "[Evolver] snapshot PNG skipped at eval " << eval_num
                  << ": " << e.what() << "\n";
    }

    // Save MP4 video — re-simulate best robot and record one frame per cycle.
    try {
        const FitnessParams& fp = params_.fitness;
        Robot robot_copy = population_[best_idx_];  // mutable copy for positions
        Simulator sim(robot_copy);
        sim.wind = fp.wind;

        VideoRenderer vid;
        for (int c = 0; c < fp.cycles; ++c) {
            sim.tickNeural();
            sim.applyActuators(fp.steps_per_cycle);
            sim.relax(fp.steps_per_cycle, fp.step_size, 0.0, 0.0);
            sim.copyPositionsBack(robot_copy);
            vid.addFrame(robot_copy,
                         static_cast<double>(c + 1) * fp.steps_per_cycle * fp.step_size);
        }
        vid.finish(stem + ".mp4");
    } catch (const std::exception& e) {
        std::cerr << "[Evolver] video MP4 skipped at eval " << eval_num
                  << ": " << e.what() << "\n";
    }

    std::cout << "[Evolver] checkpoint saved → " << stem << "\n";

    // ── Atomic checkpoint_latest.yaml (best robot, safe for crash recovery) ──
    {
        const std::string tmp = params_.run_dir + "checkpoint_latest.yaml.tmp";
        population_[best_idx_].toYAML(tmp);
        fs::rename(tmp, params_.run_dir + "checkpoint_latest.yaml");
    }

    // ── Atomic checkpoint_population.yaml (full pop + fitnesses for resume) ──
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
        double f = FitnessEvaluator(fitness_params).evaluate(job.child);
        return EvalResult{ std::move(job.child), job.replace_idx, f, job.parent_id };
    };

    // ── Helpers ───────────────────────────────────────────────────────────
    constexpr int kMaxRerolls = 100;

    std::deque<std::future<EvalResult>> in_flight;

    // Build one mutated child and submit it for evaluation.
    auto submit_one = [&]() {
        const int parent_idx = selectParent();
        const Robot::ID pid  = population_[parent_idx].id;

        Robot child = population_[parent_idx].clone();
        child.id = Robot::nextId();
        Mutator::mutate(child, rng_, params_.mutation);

        int rerolls = 0;
        while (!child.isValid()) {
            ++rerolls;
            if (rerolls >= kMaxRerolls) {
                std::cerr << "[Evolver] warning: reroll limit hit at eval "
                          << eval_count_ << " — using unmutated clone\n";
                child = population_[parent_idx].clone();
                child.id = Robot::nextId();
                break;
            }
            child = population_[parent_idx].clone();
            child.id = Robot::nextId();
            Mutator::mutate(child, rng_, params_.mutation);
        }

        EvalJob job{ std::move(child), selectReplacement(), pid };
        in_flight.push_back(
            std::async(std::launch::async, eval_worker, std::move(job)));
    };

    // Collect the oldest in-flight result and apply it to the population.
    auto collect_one = [&]() {
        EvalResult res = in_flight.front().get();
        in_flight.pop_front();

        const int       ridx              = res.replace_idx;
        const Robot::ID replaced_id       = population_[ridx].id;
        const double    replaced_fitness  = fitnesses_[ridx];

        population_[ridx] = std::move(res.child);
        fitnesses_[ridx]  = res.fitness;

        // Archive on disk.
        population_[ridx].toYAML(
            params_.run_dir + "robots/robot_" +
            std::to_string(population_[ridx].id) + ".yaml");

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
                         << "," << population_[ridx].id
                         << "," << res.parent_id
                         << "," << res.fitness
                         << "," << replaced_id
                         << "," << replaced_fitness
                         << "\n";

            if (eval_count_ % 1000 == 0) {
                fitness_log_.flush();
                lineage_log_.flush();
            }

            if (eval_count_ % 200 == 0) {
                std::cout << std::fixed << std::setprecision(4)
                          << "eval=" << eval_count_
                          << "  best=" << fitnesses_[best_idx_] << "m"
                          << "  mean=" << mean << "m\n";
            }
        }

        maybeSaveSnapshot(eval_count_);
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
    const std::string best_path = params_.run_dir + "best_robot_final.yaml";
    population_[best_idx_].toYAML(best_path);
    std::cout << "[Evolver] run complete.  eval=" << eval_count_
              << "  best_fitness=" << fitnesses_[best_idx_] << "m\n"
              << "[Evolver] best robot saved → " << best_path << "\n";

    // ── Restore signal handlers ───────────────────────────────────────────
    std::signal(SIGINT,  prev_sigint);
    std::signal(SIGTERM, prev_sigterm);
}
