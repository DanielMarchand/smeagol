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
 *   steps_per_frame: 5000    # relaxation steps per frame = neural tick interval
 *   num_frames:      12      # total number of frames (neural ticks) to run
 *   step_size:       1.0e-7
 *   fps:             10
 *   width:           640
 *   height:          480
 *   output:          ""      # leave empty to skip video
 *
 * Derived internally:
 *   total_steps = num_frames × steps_per_frame
 *
 * Environment:
 *   CI_NUMFRAMES  – if set, overrides num_frames (keeps CI runs fast)
 */

#include "Robot.h"
#include "Simulator.h"
#include "VideoRenderer.h"
#include "FitnessEvaluator.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
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
    int         fps             = 10;
    int         width           = 640;
    int         height          = 480;
    std::string out_path;

    if (!params_path.empty()) {
        try {
            YAML::Node cfg = YAML::LoadFile(params_path);
            if (cfg["steps_per_frame"]) fp.steps_per_frame = cfg["steps_per_frame"].as<int>();
            if (cfg["steps_per_cycle"]) fp.steps_per_cycle = cfg["steps_per_cycle"].as<int>();
            if (cfg["step_size"])       fp.step_size       = cfg["step_size"].as<double>();
            if (cfg["wind"])            fp.wind            = cfg["wind"].as<double>();
            if (cfg["fps"])             fps                = cfg["fps"].as<int>();
            if (cfg["width"])           width              = cfg["width"].as<int>();
            if (cfg["height"])          height             = cfg["height"].as<int>();
            if (cfg["output"])          out_path           = cfg["output"].as<std::string>();
            if (cfg["delay_steps"])     fp.delay_steps     = cfg["delay_steps"].as<int>();
            if (cfg["num_steps"]) {
                fp.num_steps = cfg["num_steps"].as<int>();
            } else {
                int _legacy_nf = -1;
                if (cfg["num_frames"]) _legacy_nf = cfg["num_frames"].as<int>();
                if (_legacy_nf >= 0) {
                    const int _spc = (fp.steps_per_cycle > 0) ? fp.steps_per_cycle : fp.steps_per_frame;
                    fp.num_steps = _legacy_nf * _spc;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error loading params: " << e.what() << "\n";
            return 1;
        }
    }

    // CI override
    if (const char* ci = std::getenv("CI_NUMSTEPS"))
        fp.num_steps = std::stoi(ci);
    // Legacy CI env var
    if (const char* ci = std::getenv("CI_NUMFRAMES")) {
        const int _spc = (fp.steps_per_cycle > 0) ? fp.steps_per_cycle : fp.steps_per_frame;
        fp.num_steps = std::stoi(ci) * _spc;
    }

    const int spc         = (fp.steps_per_cycle > 0) ? fp.steps_per_cycle : fp.steps_per_frame;
    const int spf         = fp.steps_per_frame;
    const int total_steps = fp.num_steps;

    std::cout << "Robot:  " << robot_path
              << "  (" << robot.vertices.size() << "v, "
              << robot.bars.size() << "b, "
              << robot.neurons.size() << "n)\n"
              << "Params: num_steps=" << fp.num_steps
              << "  steps_per_cycle=" << spc
              << "  steps_per_frame=" << spf
              << "  step_size=" << fp.step_size << "\n\n";

    // Mass-weighted CoM in XY from the current Simulator state.
    auto com_xy = [](const Simulator& s) -> Eigen::Vector2d {
        const double total = s.vertex_masses.sum();
        if (total <= 0.0) return Eigen::Vector2d::Zero();
        const double cx = (s.vertex_masses.array() * s.positions.col(0).array()).sum() / total;
        const double cy = (s.vertex_masses.array() * s.positions.col(1).array()).sum() / total;
        return {cx, cy};
    };

    Simulator sim = FitnessEvaluator::makeSimulator(robot, fp);

    // Open video renderer once at the start (avoids a second simulator run).
    const bool make_video = !out_path.empty();
    std::unique_ptr<VideoRenderer> vid;
    if (make_video) {
        std::cout << "Recording video → " << out_path << "\n";
        vid = std::make_unique<VideoRenderer>(fps, width, height);
        sim.copyPositionsBack(robot);
        vid->addFrame(robot, 0.0, sim.activations_);
    }

    const Eigen::Vector2d start = com_xy(sim);
    std::vector<Eigen::Vector2d> traj;
    traj.reserve(fp.num_steps / spc + 2);
    traj.push_back(start);

    if (make_video) {
        std::cout << std::scientific << std::setprecision(4)
                  << "  [frame  0 / cycle  0]  elastic=" << sim.elasticEnergy()
                  << " J  total=" << sim.totalEnergy() << " J\n";
    } else {
        std::cout << std::fixed << std::setprecision(4)
                  << "  [cycle   0 / " << (fp.num_steps / spc)
                  << "]  CoM=(" << start.x() << ", " << start.y() << ")\n";
    }
    std::cout.flush();

    int    frame_idx     = 0;
    int    total_so_far  = 0;
    double max_elastic   = sim.elasticEnergy();

    for (int cycle_idx = 0; cycle_idx < fp.num_steps / spc; ++cycle_idx) {
        sim.tickNeural();
        int remaining = spc;
        while (remaining > 0) {
            const int chunk = std::min(remaining, spf);
            sim.applyActuators(chunk);
            sim.relax(chunk, fp.step_size, 0.0, 0.0);
            remaining    -= chunk;
            total_so_far += chunk;

            if (make_video) {
                ++frame_idx;
                const double E_el = sim.elasticEnergy();
                max_elastic = std::max(max_elastic, E_el);
                std::cout << std::scientific << std::setprecision(4)
                          << "  [frame " << std::setw(3) << frame_idx
                          << " / cycle " << std::setw(2) << (cycle_idx + 1)
                          << "]  elastic=" << E_el
                          << " J  total=" << sim.totalEnergy() << " J\n";
                std::cout.flush();
                sim.copyPositionsBack(robot);
                vid->addFrame(robot,
                              static_cast<double>(total_so_far) * fp.step_size,
                              sim.activations_);
            }
        }

        traj.push_back(com_xy(sim));
        if (!make_video) {
            const auto& c = traj.back();
            std::cout << std::fixed << std::setprecision(4)
                      << "  [cycle " << std::setw(3) << (cycle_idx + 1)
                      << " / " << (fp.num_steps / spc) << "]  CoM=("
                      << c.x() << ", " << c.y() << ")\n";
            std::cout.flush();
        }
    }

    // ── Print CoM trajectory ──────────────────────────────────────────────
    std::cout << "\n" << std::fixed << std::setprecision(6);
    std::cout << "CoM trajectory (x, y) per cycle:\n";
    for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
        std::cout << "  cycle " << std::setw(3) << i
                  << ":  x=" << std::setw(10) << traj[i].x()
                  << "  y=" << std::setw(10) << traj[i].y() << "\n";
    }
    const double fitness = (com_xy(sim) - start).norm();
    std::cout << "\nFitness (XY displacement): " << fitness << " m\n\n";

    if (make_video) {
        std::cout << "Max elastic energy: " << max_elastic << " J\n"
                  << "Frames recorded: " << frame_idx
                  << "  (" << (fp.num_steps / spc) << " cycles * " << (spc / spf)
                  << " frames/cycle * " << spf << " steps/frame = "
                  << total_steps << " total steps)\n";
        vid->finish(out_path);
    }
    return 0;
}
