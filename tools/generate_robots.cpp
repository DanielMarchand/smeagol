/**
 * generate_robots.cpp
 *
 * Reads a YAML config and writes N random robot YAML files to output_dir.
 * No simulation, no rendering — pure generation.
 *
 * Usage
 * -----
 *   build/generate_robots <config.yaml>
 *
 * The config controls how many robots to create, the RNG seed, the output
 * directory, and every RobotFactory::Params knob.  See
 * examples/random_robots/config.yaml for a fully-annotated example.
 */

#include "RobotFactory.h"
#include "Robot.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

namespace fs = std::filesystem;

// ─────────────────────────────────────────────────────────────────────────────

static void printUsage(const char* argv0)
{
    std::cerr << "Usage: " << argv0 << " <config.yaml>\n"
              << "\n"
              << "  config.yaml  generation config "
              << "(see examples/random_robots/config.yaml)\n";
}

static void printSummary(const Robot& r, int idx, const std::string& path)
{
    const int nv = static_cast<int>(r.vertices.size());
    const int nb = static_cast<int>(r.bars.size());
    const int nn = static_cast<int>(r.neurons.size());
    const int na = static_cast<int>(r.actuators.size());

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  robot_" << idx << "  →  " << path << "\n"
              << "    vertices=" << nv
              << "  bars="      << nb
              << "  neurons="   << nn
              << "  actuators=" << na << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    if (argc != 2) {
        printUsage(argv[0]);
        return 1;
    }

    // ── Load config ───────────────────────────────────────────────────────
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config '" << argv[1] << "': "
                  << e.what() << "\n";
        return 1;
    }

    // ── Top-level scalars ─────────────────────────────────────────────────
    const int count =
        cfg["count"] ? cfg["count"].as<int>() : 3;
    const unsigned seed =
        cfg["seed"] ? static_cast<unsigned>(cfg["seed"].as<int>()) : 42u;

    // Resolve output_dir relative to the config file's directory so that
    // configs can use short local paths (e.g. "robots") regardless of CWD.
    const std::string raw_output_dir =
        cfg["output_dir"] ? cfg["output_dir"].as<std::string>() : "robots";
    const fs::path cfg_dir = fs::absolute(argv[1]).parent_path();
    const fs::path output_path = fs::path(raw_output_dir).is_relative()
        ? cfg_dir / raw_output_dir
        : fs::path(raw_output_dir);
    const std::string output_dir = output_path.string();

    // ── RobotFactory::Params from config["robot"] ─────────────────────────
    RobotFactory::Params p;
    if (cfg["robot"]) {
        const YAML::Node& r = cfg["robot"];
        if (r["min_vertices"])   p.min_vertices   = r["min_vertices"].as<int>();
        if (r["max_vertices"])   p.max_vertices   = r["max_vertices"].as<int>();
        if (r["bbox_xy"])        p.bbox_xy        = r["bbox_xy"].as<double>();
        if (r["bbox_z_min"])     p.bbox_z_min     = r["bbox_z_min"].as<double>();
        if (r["bbox_z_max"])     p.bbox_z_max     = r["bbox_z_max"].as<double>();
        if (r["stiffness_min"]) p.stiffness_min = r["stiffness_min"].as<double>();
        if (r["stiffness_max"]) p.stiffness_max = r["stiffness_max"].as<double>();
        if (r["p_extra_bar"])    p.p_extra_bar    = r["p_extra_bar"].as<double>();
        if (r["min_neurons"])    p.min_neurons    = r["min_neurons"].as<int>();
        if (r["max_neurons"])    p.max_neurons    = r["max_neurons"].as<int>();
        if (r["threshold_min"])  p.threshold_min  = r["threshold_min"].as<double>();
        if (r["threshold_max"])  p.threshold_max  = r["threshold_max"].as<double>();
        if (r["weight_range"])   p.weight_range   = r["weight_range"].as<double>();
        if (r["p_actuate_bar"])  p.p_actuate_bar  = r["p_actuate_bar"].as<double>();
        if (r["bar_range_max"])  p.bar_range_max  = r["bar_range_max"].as<double>();
    }

    // ── Create output directory ───────────────────────────────────────────
    try {
        fs::create_directories(output_dir);
    } catch (const std::exception& e) {
        std::cerr << "Failed to create output_dir '" << output_dir
                  << "': " << e.what() << "\n";
        return 1;
    }

    // ── Generate ──────────────────────────────────────────────────────────
    std::mt19937 rng(seed);

    std::cout << "Generating " << count << " robot(s)  "
              << "(seed=" << seed << "  →  " << output_dir << "/)\n\n";

    for (int i = 0; i < count; ++i) {
        Robot robot = RobotFactory::randomRobot(rng, p);
        const std::string path =
            output_dir + "/robot_" + std::to_string(i) + ".yaml";
        robot.toYAML(path);
        printSummary(robot, i, path);
    }

    std::cout << "\nDone.\n";
    return 0;
}
