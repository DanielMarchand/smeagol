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
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

// ── EvolverParams YAML I/O ────────────────────────────────────────────────────

EvolverParams EvolverParams::fromYAML(const std::string& path)
{
    YAML::Node node = YAML::LoadFile(path);
    EvolverParams p;

    if (node["population_size"]) p.population_size = node["population_size"].as<int>();
    if (node["max_evaluations"]) p.max_evaluations  = node["max_evaluations"].as<int>();
    if (node["seed"])            p.seed             = node["seed"].as<int>();
    if (node["video_interval"])  p.video_interval   = node["video_interval"].as<int>();
    if (node["run_dir"])         p.run_dir          = node["run_dir"].as<std::string>();

    if (const auto& f = node["fitness"]) {
        if (f["cycles"])          p.fitness.cycles          = f["cycles"].as<int>();
        if (f["steps_per_cycle"]) p.fitness.steps_per_cycle = f["steps_per_cycle"].as<int>();
        if (f["step_size"])       p.fitness.step_size       = f["step_size"].as<double>();
        if (f["wind"])            p.fitness.wind            = f["wind"].as<double>();
    }

    if (const auto& m = node["mutation"])
        p.mutation = MutatorParams::fromYAML(m);

    return p;
}

void EvolverParams::toYAML(const std::string& path) const
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "population_size" << YAML::Value << population_size;
    out << YAML::Key << "max_evaluations" << YAML::Value << max_evaluations;
    out << YAML::Key << "seed"            << YAML::Value << seed;
    out << YAML::Key << "video_interval"  << YAML::Value << video_interval;
    out << YAML::Key << "run_dir"         << YAML::Value << run_dir;
    out << YAML::Key << "fitness"         << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "cycles"          << YAML::Value << fitness.cycles;
    out << YAML::Key << "steps_per_cycle" << YAML::Value << fitness.steps_per_cycle;
    out << YAML::Key << "step_size"       << YAML::Value << fitness.step_size;
    out << YAML::Key << "wind"            << YAML::Value << fitness.wind;
    out << YAML::EndMap;  // fitness
    out << YAML::Key << "mutation"        << YAML::Value << YAML::BeginMap;
    mutation.toYAML(out);
    out << YAML::EndMap;  // mutation
    out << YAML::EndMap;  // root

    std::ofstream f(path);
    if (!f) throw std::runtime_error("EvolverParams::toYAML: cannot write to '" + path + "'");
    f << out.c_str() << "\n";
}

// ── Evolver constructor ───────────────────────────────────────────────────────

static std::string timestampDir()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return std::string("runs/run_") + buf + "/";
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

    // ── Resolve run directory ─────────────────────────────────────────────────
    if (params_.run_dir.empty())
        params_.run_dir = timestampDir();

    fs::create_directories(params_.run_dir);
    fs::create_directories(params_.run_dir + "robots/");
    fs::create_directories(params_.run_dir + "checkpoints/");

    // ── Persist resolved config for reproducibility ───────────────────────────
    params_.toYAML(params_.run_dir + "run_config.yaml");

    // ── Initialise population (all empty robots, zero fitness) ────────────────
    population_.reserve(params_.population_size);
    for (int i = 0; i < params_.population_size; ++i)
        population_.emplace_back();   // Robot() — empty, auto-assigned ID

    fitnesses_.assign(params_.population_size, 0.0);

    // ── Open fitness log ──────────────────────────────────────────────────────
    fitness_log_.open(params_.run_dir + "fitness_log.csv");
    fitness_log_ << "eval,generation,best_fitness,mean_fitness,"
                    "best_robot_id,best_v,best_b,best_n,best_a\n";

    // ── Open lineage log ────────────────────────────────────────────────────
    lineage_log_.open(params_.run_dir + "lineage.csv");
    lineage_log_ << "eval,child_id,parent_id,child_fitness,"
                    "replaced_id,replaced_fitness\n";

    std::cout << "[Evolver] run_dir  : " << params_.run_dir  << "\n"
              << "[Evolver] seed     : " << params_.seed      << "\n"
              << "[Evolver] pop_size : " << params_.population_size << "\n"
              << "[Evolver] max_evals: " << params_.max_evaluations << "\n";
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

// Save the best robot's YAML (and optionally a snapshot PNG) to run_dir.
void Evolver::maybeSaveSnapshot(int eval_num)
{
    if (eval_num % params_.video_interval != 0) return;

    const std::string stem = params_.run_dir + "best_eval_" + std::to_string(eval_num);

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

    std::cout << "[Evolver] checkpoint saved → " << stem << "\n";
}

// Main steady-state loop.
void Evolver::run()
{
    // Maximum rerolls before falling back to the unmutated parent clone.
    constexpr int kMaxRerolls = 100;

    while (eval_count_ < params_.max_evaluations) {

        // 1. Select parent (fitness-proportionate).
        const int parent_idx = selectParent();
        const Robot::ID parent_id = population_[parent_idx].id;

        // 2. Clone parent and assign a fresh unique ID.
        Robot child = population_[parent_idx].clone();
        child.id = Robot::nextId();

        // 3. Mutate — reroll until valid; fall back after kMaxRerolls attempts.
        Mutator::mutate(child, rng_, params_.mutation);
        int rerolls = 0;
        while (!child.isValid()) {
            ++rerolls;
            if (rerolls >= kMaxRerolls) {
                std::cerr << "[Evolver] warning: child still invalid after "
                          << kMaxRerolls << " rerolls at eval " << eval_count_
                          << " — using unmutated parent clone\n";
                child = population_[parent_idx].clone();
                child.id = Robot::nextId();
                break;
            }
            child = population_[parent_idx].clone();
            child.id = Robot::nextId();
            Mutator::mutate(child, rng_, params_.mutation);
        }

        // 4. Evaluate child.
        const double child_fitness =
            FitnessEvaluator(params_.fitness).evaluate(child);

        // 5. Replace a uniformly-random slot (including possibly the parent's).
        const int replace_idx = selectReplacement();
        const Robot::ID replaced_id       = population_[replace_idx].id;
        const double    replaced_fitness  = fitnesses_[replace_idx];
        population_[replace_idx] = std::move(child);
        fitnesses_[replace_idx]  = child_fitness;
        // Archive every placed robot so the full lineage is on disk.
        population_[replace_idx].toYAML(
            params_.run_dir + "robots/robot_" +
            std::to_string(population_[replace_idx].id) + ".yaml");

        // 6. Update best_idx_.
        if (replace_idx == best_idx_) {
            // We clobbered the best slot — rescan to find the new best.
            best_idx_ = static_cast<int>(
                std::max_element(fitnesses_.begin(), fitnesses_.end()) -
                fitnesses_.begin());
        } else if (child_fitness > fitnesses_[best_idx_]) {
            best_idx_ = replace_idx;
        }

        // 7. Bookkeeping.
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
                         << "," << population_[replace_idx].id
                         << "," << parent_id
                         << "," << child_fitness
                         << "," << replaced_id
                         << "," << replaced_fitness
                         << "\n";

            if (eval_count_ % 1000 == 0) {
                fitness_log_.flush();
                lineage_log_.flush();
            }

            // 8. Periodic progress print (every 200 evals).
            if (eval_count_ % 200 == 0) {
                std::cout << std::fixed << std::setprecision(4)
                          << "eval=" << eval_count_
                          << "  best=" << fitnesses_[best_idx_] << "m"
                          << "  mean=" << mean << "m\n";
            }
        }

        // 9. Maybe save checkpoint snapshot.
        maybeSaveSnapshot(eval_count_);
    }

    // ── Final summary ─────────────────────────────────────────────────────
    fitness_log_.flush();
    lineage_log_.flush();
    const std::string best_path = params_.run_dir + "best_robot_final.yaml";
    population_[best_idx_].toYAML(best_path);
    std::cout << "[Evolver] run complete.  eval=" << eval_count_
              << "  best_fitness=" << fitnesses_[best_idx_] << "m\n"
              << "[Evolver] best robot saved → " << best_path << "\n";
}
