/**
 * evaluate_fitness.cpp
 *
 * Loads a robot YAML, runs the §3.6 FitnessEvaluator for a configurable
 * number of neural cycles, prints the per-cycle CoM trajectory, and
 * optionally records a video for visual inspection.
 *
 * Usage:
 *   evaluate_fitness <robot.yaml> [params.yaml]
 *
 * params.yaml (all fields optional, shown with defaults):
 *   cycles:          12
 *   steps_per_cycle: 5000
 *   step_size:       1.0e-7
 *   fps:             10
 *   width:           640
 *   height:          480
 *   output:          ""         # leave empty to skip video
 *
 * Environment:
 *   CI_NUMFRAMES  – if set, overrides cycles (keeps CI runs fast)
 */

#include "Robot.h"
#include "FitnessEvaluator.h"
#include "Simulator.h"
#include "VideoRenderer.h"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: evaluate_fitness <robot.yaml> [params.yaml]\n";
        return 1;
    }

    const std::string robot_path  = argv[1];
    const std::string params_path = (argc >= 3) ? argv[2] : "";

    // ── Load robot ────────────────────────────────────────────────────────
    Robot robot;
    try {
        robot = Robot::fromYAML(robot_path);
    } catch (const std::exception& e) {
        std::cerr << "Error loading robot: " << e.what() << "\n";
        return 1;
    }

    // ── Load / default params ─────────────────────────────────────────────
    FitnessEvaluator::Params fp;
    int         fps      = 10;
    int         width    = 640;
    int         height   = 480;
    std::string out_path;

    if (!params_path.empty()) {
        try {
            YAML::Node cfg = YAML::LoadFile(params_path);
            if (cfg["cycles"])          fp.cycles          = cfg["cycles"].as<int>();
            if (cfg["steps_per_cycle"]) fp.steps_per_cycle = cfg["steps_per_cycle"].as<int>();
            if (cfg["step_size"])       fp.step_size       = cfg["step_size"].as<double>();
            if (cfg["wind"])            fp.wind            = cfg["wind"].as<double>();
            if (cfg["fps"])             fps                = cfg["fps"].as<int>();
            if (cfg["width"])           width              = cfg["width"].as<int>();
            if (cfg["height"])          height             = cfg["height"].as<int>();
            if (cfg["output"])          out_path           = cfg["output"].as<std::string>();
        } catch (const std::exception& e) {
            std::cerr << "Error loading params: " << e.what() << "\n";
            return 1;
        }
    }

    // CI override
    if (const char* ci = std::getenv("CI_NUMFRAMES"))
        fp.cycles = std::stoi(ci);

    // ── Run evaluation ────────────────────────────────────────────────────
    std::cout << "Robot:  " << robot_path
              << "  (" << robot.vertices.size() << "v, "
              << robot.bars.size() << "b, "
              << robot.neurons.size() << "n)\n"
              << "Params: cycles=" << fp.cycles
              << "  steps_per_cycle=" << fp.steps_per_cycle
              << "  step_size=" << fp.step_size << "\n\n";

    std::vector<Eigen::Vector2d> traj;
    FitnessEvaluator eval(fp);
    const double fitness = eval.evaluate(robot, &traj);

    // ── Print CoM trajectory ──────────────────────────────────────────────
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "CoM trajectory (x, y) per cycle:\n";
    for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
        std::cout << "  cycle " << std::setw(3) << i
                  << ":  x=" << std::setw(10) << traj[i].x()
                  << "  y=" << std::setw(10) << traj[i].y() << "\n";
    }
    std::cout << "\nFitness (XY displacement): " << fitness << " m\n\n";

    // ── Optional video ────────────────────────────────────────────────────
    if (out_path.empty()) {
        std::cout << "(No output path specified — skipping video)\n";
        return 0;
    }

    std::cout << "Recording video → " << out_path << "\n";

    Simulator sim(robot);
    sim.wind = fp.wind;
    VideoRenderer vid(fps, width, height);

    vid.addFrame(robot, 0.0, sim.activations_);

    for (int c = 0; c < fp.cycles; ++c) {
        sim.tickNeural();
        sim.applyActuators();
        sim.relax(fp.steps_per_cycle, fp.step_size, 0.0, 0.0);
        sim.copyPositionsBack(robot);
        vid.addFrame(robot, (c + 1.0) / fps, sim.activations_);
    }

    vid.finish(out_path);
    std::cout << "Written: " << out_path << "\n";
    return 0;
}
