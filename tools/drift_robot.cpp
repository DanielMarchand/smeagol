/**
 * drift_robot.cpp
 *
 * Randomly walks a single robot through mutation space for N steps with NO
 * fitness evaluation, then optionally evaluates and records the final result.
 *
 * Purpose
 * -------
 * Fast parameter tuning.  Running 100–500 mutations takes seconds; it lets
 * you check that robots reliably grow neurons, maintain sensible bar lengths,
 * accumulate actuators, etc. — before committing to a slow full-evolution run.
 *
 * Usage
 * -----
 *   build/drift_robot <drift.yaml>
 *
 * All settings live in drift.yaml.  See examples/drift_robot/drift.yaml for
 * a fully-annotated template that mirrors prod/config.yaml mutation params.
 *
 * Output
 * ------
 * Per-step table to stdout:
 *
 *   step=  0  [initial]   v=6   b=9   n=12  a=4
 *              bars: count=9   min=0.021m  mean=0.057m  max=0.092m
 *   step=  1  perturb     v=6   b=9   n=12  a=4
 *   step=  5  add/remove  v=7   b=10  n=12  a=4  [snapshot]
 *   ...
 *   step=100  [final]     v=9   b=12  n=13  a=5
 *   fitness  = 0.0312 m
 *   video    → output/final.mp4
 */

#include "RobotFactory.h"
#include "Robot.h"
#include "Mutator.h"
#include "FitnessEvaluator.h"
#include "VideoRenderer.h"
#include "Simulator.h"

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>

namespace fs = std::filesystem;

// ── helpers ───────────────────────────────────────────────────────────────────

struct BarStats {
    double min_len, mean_len, max_len;
    int count;
};

static BarStats barStats(const Robot& r)
{
    if (r.bars.empty()) return {0.0, 0.0, 0.0, 0};
    double lo = r.bars[0].rest_length, hi = lo, sum = 0.0;
    for (const auto& b : r.bars) {
        lo  = std::min(lo,  b.rest_length);
        hi  = std::max(hi,  b.rest_length);
        sum += b.rest_length;
    }
    return { lo, sum / r.bars.size(), hi, static_cast<int>(r.bars.size()) };
}

static void printStep(int step, const std::string& op_label, const Robot& r,
                      bool snapshotted, bool bars_changed)
{
    const int nv = static_cast<int>(r.vertices.size());
    const int nb = static_cast<int>(r.bars.size());
    const int nn = static_cast<int>(r.neurons.size());
    const int na = static_cast<int>(r.actuators.size());

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "step=" << std::setw(4) << step
              << "  " << std::left << std::setw(22) << op_label << std::right
              << "  v=" << std::setw(3) << nv
              << "  b=" << std::setw(3) << nb
              << "  n=" << std::setw(3) << nn
              << "  a=" << std::setw(3) << na;

    if (!r.bars.empty()) {
        const auto bs = barStats(r);
        std::cout << "   bars: min=" << std::setw(7) << bs.min_len << "m"
                  << "  mean=" << std::setw(7) << bs.mean_len << "m"
                  << "  max=" << std::setw(7) << bs.max_len << "m";
    }

    if (snapshotted) std::cout << "  [snapshot]";
    std::cout << "\n";
}

// ── Load FitnessParams from a YAML node (mirrors Evolver::fromYAML) ───────────

static FitnessParams fitnessParamsFromYAML(const YAML::Node& n)
{
    FitnessParams p;
    bool num_steps_set = false;
    int  legacy_nf     = -1;
    if (n["num_steps"])       { p.num_steps       = n["num_steps"].as<int>(); num_steps_set = true; }
    if (n["delay_steps"])       p.delay_steps      = n["delay_steps"].as<int>();
    if (n["num_frames"])        legacy_nf          = n["num_frames"].as<int>();     // legacy
    if (n["cycles"])            legacy_nf          = n["cycles"].as<int>();         // legacy-legacy
    if (n["steps_per_frame"])   p.steps_per_frame  = n["steps_per_frame"].as<int>();
    if (n["steps_per_cycle"])   p.steps_per_cycle  = n["steps_per_cycle"].as<int>();
    if (!num_steps_set && legacy_nf >= 0) {
        const int spc = p.steps_per_cycle > 0 ? p.steps_per_cycle : p.steps_per_frame;
        p.num_steps = legacy_nf * spc;
    }
    if (n["step_size"])       p.step_size        = n["step_size"].as<double>();
    if (n["gravity"])         p.gravity          = n["gravity"].as<double>();
    if (n["wind"])            p.wind             = n["wind"].as<double>();
    if (n["mu_static"])       p.mu_static        = n["mu_static"].as<double>();
    if (n["k_floor"])         p.k_floor          = n["k_floor"].as<double>();
    if (n["actuator_max_extension_per_cycle"])
        p.actuator_max_extension_per_cycle = n["actuator_max_extension_per_cycle"].as<double>();
    if (n["actuator_max_extension_total"])
        p.actuator_max_extension_total = n["actuator_max_extension_total"].as<double>();
    if (n["bar_stiffness_override"])
        p.bar_stiffness_override = n["bar_stiffness_override"].as<double>();
    return p;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: drift_robot <drift.yaml>\n"
                  << "  See examples/drift_robot/drift.yaml for a template.\n";
        return 1;
    }

    // ── Load config ───────────────────────────────────────────────────────
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config '" << fs::absolute(argv[1]).string() << "': "
                  << e.what() << "\n"
                  << "  (cwd: " << fs::current_path().string() << ")\n";
        return 1;
    }

    const int      steps          = cfg["steps"]            ? cfg["steps"].as<int>()            : 100;
    const unsigned seed           = cfg["seed"]             ? static_cast<unsigned>(cfg["seed"].as<int>()) : 42u;
    const int      snapshot_every = cfg["snapshot_every"]   ? cfg["snapshot_every"].as<int>()   : 0;
    const bool     evaluate_final = cfg["evaluate_final"]   ? cfg["evaluate_final"].as<bool>()  : false;
    const int      max_rerolls    = cfg["max_rerolls"]      ? cfg["max_rerolls"].as<int>()      : 100;

    // Resolve output_dir relative to the config file
    const std::string raw_out = cfg["output_dir"] ? cfg["output_dir"].as<std::string>() : "drift_output";
    const fs::path cfg_dir    = fs::absolute(argv[1]).parent_path();
    const fs::path output_dir = fs::path(raw_out).is_relative()
                                ? cfg_dir / raw_out
                                : fs::path(raw_out);
    fs::create_directories(output_dir);

    // ── MutatorParams ──────────────────────────────────────────────────────
    MutatorParams mp;
    if (cfg["mutation"])
        mp = MutatorParams::fromYAML(cfg["mutation"]);

    // ── Starting robot ─────────────────────────────────────────────────────
    // Either load from YAML (start_yaml) or generate fresh (robot: section).
    std::mt19937 rng(seed);
    Robot robot;

    if (cfg["start_yaml"] && !cfg["start_yaml"].as<std::string>().empty()) {
        const std::string raw_yaml = cfg["start_yaml"].as<std::string>();
        const fs::path yaml_path = fs::path(raw_yaml).is_relative()
                                   ? cfg_dir / raw_yaml
                                   : fs::path(raw_yaml);
        const std::string yaml_path_str = yaml_path.string();
        try {
            robot = Robot::fromYAML(yaml_path_str);
            std::cout << "Loaded starting robot from '" << yaml_path_str << "'\n";
        } catch (const std::exception& e) {
            std::cerr << "Failed to load start_yaml '" << yaml_path_str
                      << "': " << e.what() << "\n";
            return 1;
        }
    } else {
        RobotFactory::Params rp;
        if (cfg["robot"]) {
            const YAML::Node& r = cfg["robot"];
            if (r["min_vertices"])  rp.min_vertices  = r["min_vertices"].as<int>();
            if (r["max_vertices"])  rp.max_vertices  = r["max_vertices"].as<int>();
            if (r["bbox_xy"])       rp.bbox_xy       = r["bbox_xy"].as<double>();
            if (r["bbox_z_min"])    rp.bbox_z_min    = r["bbox_z_min"].as<double>();
            if (r["bbox_z_max"])    rp.bbox_z_max    = r["bbox_z_max"].as<double>();
            if (r["stiffness_min"]) rp.stiffness_min = r["stiffness_min"].as<double>();
            if (r["stiffness_max"]) rp.stiffness_max = r["stiffness_max"].as<double>();
            if (r["p_extra_bar"])   rp.p_extra_bar   = r["p_extra_bar"].as<double>();
            if (r["min_neurons"])   rp.min_neurons   = r["min_neurons"].as<int>();
            if (r["max_neurons"])   rp.max_neurons   = r["max_neurons"].as<int>();
            if (r["threshold_min"]) rp.threshold_min = r["threshold_min"].as<double>();
            if (r["threshold_max"]) rp.threshold_max = r["threshold_max"].as<double>();
            if (r["weight_range"])  rp.weight_range  = r["weight_range"].as<double>();
            if (r["p_actuate_bar"]) rp.p_actuate_bar = r["p_actuate_bar"].as<double>();
            if (r["bar_range_max"]) rp.bar_range_max = r["bar_range_max"].as<double>();
        }
        robot = RobotFactory::randomRobot(rng, rp);
        std::cout << "Generated random starting robot (seed=" << seed << ")\n";
    }

    if (!robot.isValid()) {
        std::cerr << "Warning: starting robot failed isValid() — check params.\n";
    }

    std::cout << "\n";
    printStep(0, "[initial]", robot, false, true);
    std::cout << "\n";

    // ── Mutation walk ─────────────────────────────────────────────────────
    for (int s = 1; s <= steps; ++s) {
        // Try to mutate. Reroll up to max_rerolls on invalid result.
        Robot candidate  = robot;
        MutationRecord mrec;
        bool valid = false;

        for (int attempt = 0; attempt <= max_rerolls; ++attempt) {
            candidate = robot;
            mrec      = Mutator::mutateRecord(candidate, rng, mp);
            if (candidate.isValid()) { valid = true; break; }
        }

        if (!valid) {
            // Give up — keep the parent unchanged this step.
            candidate = robot;
            mrec      = MutationRecord{};
        }

        robot = std::move(candidate);

        // Build human-readable operator label.
        std::string label = mrec.describe();
        if (label.empty()) label = "(none)";

        // Snapshot?
        bool snapshotted = false;
        if (snapshot_every > 0 && (s % snapshot_every == 0)) {
            const std::string snap_path = (output_dir / ("step_" + std::to_string(s) + ".yaml")).string();
            robot.toYAML(snap_path);
            snapshotted = true;
        }

        printStep(s, label, robot, snapshotted, true);
    }

    // ── Save final robot ──────────────────────────────────────────────────
    const std::string final_yaml = (output_dir / "robot_final.yaml").string();
    robot.toYAML(final_yaml);
    std::cout << "\nFinal robot saved → " << final_yaml << "\n";

    // ── Summary ───────────────────────────────────────────────────────────
    {
        const auto bs = barStats(robot);
        std::cout << "\n── Summary ────────────────────────────────────────────\n";
        std::cout << "  Vertices  : " << robot.vertices.size()  << "\n";
        std::cout << "  Bars      : " << robot.bars.size()      << "\n";
        std::cout << "  Neurons   : " << robot.neurons.size()   << "\n";
        std::cout << "  Actuators : " << robot.actuators.size() << "\n";
        if (!robot.bars.empty()) {
            std::cout << "  Bar lengths: min=" << bs.min_len << "m"
                      << "  mean=" << bs.mean_len << "m"
                      << "  max=" << bs.max_len << "m\n";
        }
    }
    std::cout << "\n";

    // ── Optional fitness evaluation + video ───────────────────────────────
    if (evaluate_final) {
        FitnessParams fp;
        if (cfg["fitness"]) fp = fitnessParamsFromYAML(cfg["fitness"]);

        std::cout << "── Evaluating fitness ";
        std::cout.flush();
        const double fitness = FitnessEvaluator(fp).evaluate(robot);
        std::cout << "→ " << std::fixed << std::setprecision(4) << fitness << " m\n\n";

        // Video — requires video: section and enabled: true (or absent).
        bool vid_enabled = true;
        int  vid_fps     = 30;
        bool vid_verbose = false;
        if (cfg["video"]) {
            const YAML::Node& v = cfg["video"];
            if (v["enabled"]) vid_enabled = v["enabled"].as<bool>();
            if (v["fps"])     vid_fps     = v["fps"].as<int>();
            if (v["verbose"]) vid_verbose = v["verbose"].as<bool>();
        }

        if (vid_enabled) {
            const std::string vid_path = (output_dir / "final.mp4").string();
            try {
                FitnessEvaluator::renderVideo(robot, fp, vid_path, vid_fps, vid_verbose);
                std::cout << "Video saved → " << vid_path << "\n";
            } catch (const std::exception& e) {
                std::cerr << "Video skipped: " << e.what() << "\n";
            }
        }
    }

    return 0;
}
