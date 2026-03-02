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
 *   steps_per_cycle: 5000    # relaxation steps per neural tick
 *   steps_per_frame: 5000    # relaxation steps between video frames (default = steps_per_cycle)
 *   num_frames:      12      # total number of video frames to produce
 *   step_size:       1.0e-7
 *   fps:             10
 *   width:           640
 *   height:          480
 *   output:          ""      # leave empty to skip video
 *
 * Derived internally:
 *   total_steps = num_frames * steps_per_frame
 *   cycles      = ceil(total_steps / steps_per_cycle)  (for fitness evaluator)
 *
 * Environment:
 *   CI_NUMFRAMES  – if set, overrides cycles (keeps CI runs fast)
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
    int         num_frames      = 12;
    int         steps_per_frame = -1;  // -1 → same as steps_per_cycle
    std::string out_path;

    if (!params_path.empty()) {
        try {
            YAML::Node cfg = YAML::LoadFile(params_path);
            if (cfg["steps_per_cycle"]) fp.steps_per_cycle = cfg["steps_per_cycle"].as<int>();
            if (cfg["step_size"])       fp.step_size       = cfg["step_size"].as<double>();
            if (cfg["wind"])            fp.wind            = cfg["wind"].as<double>();
            if (cfg["fps"])             fps                = cfg["fps"].as<int>();
            if (cfg["width"])           width              = cfg["width"].as<int>();
            if (cfg["height"])          height             = cfg["height"].as<int>();
            if (cfg["output"])          out_path           = cfg["output"].as<std::string>();
            if (cfg["num_frames"])      num_frames         = cfg["num_frames"].as<int>();
            if (cfg["steps_per_frame"]) steps_per_frame    = cfg["steps_per_frame"].as<int>();
        } catch (const std::exception& e) {
            std::cerr << "Error loading params: " << e.what() << "\n";
            return 1;
        }
    }

    if (steps_per_frame <= 0)
        steps_per_frame = fp.steps_per_cycle;

    // Derive cycle count so the fitness evaluator covers the same total simulation time
    const int total_steps = num_frames * steps_per_frame;
    fp.cycles = std::max(1, (total_steps + fp.steps_per_cycle - 1) / fp.steps_per_cycle);

    // CI override
    if (const char* ci = std::getenv("CI_NUMFRAMES"))
        fp.cycles = std::stoi(ci);

    // ── Single-pass simulation: fitness tracking + optional video ─────────
    //
    // Previously two separate passes were used: one silent FitnessEvaluator
    // run followed by a video-rendering run.  This combined loop does both in
    // one pass, cutting total physics work in half and giving continuous
    // per-cycle (or per-frame) progress output throughout.
    std::cout << "Robot:  " << robot_path
              << "  (" << robot.vertices.size() << "v, "
              << robot.bars.size() << "b, "
              << robot.neurons.size() << "n)\n"
              << "Params: num_frames=" << num_frames
              << "  steps_per_frame=" << steps_per_frame
              << "  steps_per_cycle=" << fp.steps_per_cycle
              << "  step_size=" << fp.step_size << "\n\n";

    // Mass-weighted CoM in XY from the current Simulator state.
    auto com_xy = [](const Simulator& s) -> Eigen::Vector2d {
        const double total = s.vertex_masses.sum();
        if (total <= 0.0) return Eigen::Vector2d::Zero();
        const double cx = (s.vertex_masses.array() * s.positions.col(0).array()).sum() / total;
        const double cy = (s.vertex_masses.array() * s.positions.col(1).array()).sum() / total;
        return {cx, cy};
    };

    Simulator sim(robot);
    sim.wind = fp.wind;

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
    traj.reserve(fp.cycles + 1);
    traj.push_back(start);

    // Print t=0 status line in whichever style suits the run mode.
    if (make_video) {
        std::cout << std::scientific << std::setprecision(4)
                  << "  [frame  0 / cycle  0]  elastic=" << sim.elasticEnergy()
                  << " J  total=" << sim.totalEnergy() << " J\n";
    } else {
        std::cout << std::fixed << std::setprecision(4)
                  << "  [cycle   0 / " << fp.cycles
                  << "]  CoM=(" << start.x() << ", " << start.y() << ")\n";
    }
    std::cout.flush();

    const int spf           = steps_per_frame;
    int       global_step   = 0;
    int       steps_to_tick = fp.steps_per_cycle;
    int       frame_idx     = 0;
    int       cycle_idx     = 0;
    double    max_elastic   = sim.elasticEnergy();

    while (global_step < total_steps) {
        // Advance to the nearest event boundary:
        //   · video mode  → min(next frame edge, next neural tick, end)
        //   · fitness-only → min(next neural tick, end)
        const int chunk = make_video
            ? std::min({spf - global_step % spf, steps_to_tick,
                        total_steps - global_step})
            : std::min(steps_to_tick, total_steps - global_step);

        sim.relax(chunk, fp.step_size, 0.0, 0.0);
        global_step   += chunk;
        steps_to_tick -= chunk;

        // ── Neural tick ───────────────────────────────────────────────────
        if (steps_to_tick == 0) {
            sim.tickNeural();
            sim.applyActuators(fp.steps_per_cycle);
            steps_to_tick = fp.steps_per_cycle;
            ++cycle_idx;
            traj.push_back(com_xy(sim));
            if (!make_video) {
                const auto& c = traj.back();
                std::cout << std::fixed << std::setprecision(4)
                          << "  [cycle " << std::setw(3) << cycle_idx
                          << " / " << fp.cycles << "]  CoM=("
                          << c.x() << ", " << c.y() << ")\n";
                std::cout.flush();
            }
        }

        // ── Video frame capture ───────────────────────────────────────────
        if (make_video && (global_step % spf == 0 || global_step == total_steps)) {
            ++frame_idx;
            const double E_el = sim.elasticEnergy();
            max_elastic = std::max(max_elastic, E_el);
            std::cout << std::scientific << std::setprecision(4)
                      << "  [frame " << std::setw(2) << frame_idx
                      << " / cycle " << std::setw(2) << cycle_idx
                      << "]  elastic=" << E_el
                      << " J  total=" << sim.totalEnergy() << " J\n";
            std::cout.flush();
            sim.copyPositionsBack(robot);
            vid->addFrame(robot,
                          static_cast<double>(global_step) * fp.step_size,
                          sim.activations_);
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
                  << "Frames recorded: " << num_frames
                  << "  (" << num_frames << " frames * " << spf
                  << " steps_per_frame = " << total_steps << " total steps)\n";
        vid->finish(out_path);
    }
    return 0;
}
