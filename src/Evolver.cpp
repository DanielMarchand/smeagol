/**
 * Evolver.cpp
 *
 * Phase 4.1: Population Initialisation.
 *
 * Implements EvolverParams YAML I/O and the Evolver constructor.
 * The run() loop and selection helpers are stubbed and will be filled in
 * Phase 4.3.
 */

#include "Evolver.h"
#include "Mutator.h"

#include <yaml-cpp/yaml.h>

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

    // ── Persist resolved config for reproducibility ───────────────────────────
    params_.toYAML(params_.run_dir + "run_config.yaml");

    // ── Initialise population (all empty robots, zero fitness) ────────────────
    population_.reserve(params_.population_size);
    for (int i = 0; i < params_.population_size; ++i)
        population_.emplace_back();   // Robot() — empty, auto-assigned ID

    fitnesses_.assign(params_.population_size, 0.0);

    // ── Open fitness log ──────────────────────────────────────────────────────
    fitness_log_.open(params_.run_dir + "fitness_log.csv");
    fitness_log_ << "eval,best_fitness\n";

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

// ── Stubs (Phase 4.3) ─────────────────────────────────────────────────────────

void Evolver::run()
{
    throw std::logic_error("Evolver::run() — not yet implemented (Phase 4.3)");
}

int Evolver::selectParent() const
{
    throw std::logic_error("Evolver::selectParent() — not yet implemented (Phase 4.3)");
}

int Evolver::selectReplacement() const
{
    throw std::logic_error("Evolver::selectReplacement() — not yet implemented (Phase 4.3)");
}

void Evolver::evaluateOne(int /*idx*/)
{
    throw std::logic_error("Evolver::evaluateOne() — not yet implemented (Phase 4.3)");
}

void Evolver::maybeSaveSnapshot(int /*eval_num*/)
{
    throw std::logic_error("Evolver::maybeSaveSnapshot() — not yet implemented (Phase 4.3)");
}
