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
#include "FitnessEvaluator.h"
#include "Simulator.h"
#include "VideoRenderer.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
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

    // ── Run evaluation ────────────────────────────────────────────────────
    std::cout << "Robot:  " << robot_path
              << "  (" << robot.vertices.size() << "v, "
              << robot.bars.size() << "b, "
              << robot.neurons.size() << "n)\n"
              << "Params: num_frames=" << num_frames
              << "  steps_per_frame=" << steps_per_frame
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

    // Video loop: advance total_steps in chunks.
    // Neural tick fires every steps_per_cycle steps; frame captured every steps_per_frame steps.
    const int spf = steps_per_frame;
    int global_step = 0;
    int steps_to_next_tick = fp.steps_per_cycle;  // steps until next neural tick

    vid.addFrame(robot, 0.0, sim.activations_);
    std::cout << std::scientific << std::setprecision(4);
    std::cout << "  [frame  0 / cycle  0]  elastic=" << sim.elasticEnergy() << " J"
              << "  total=" << sim.totalEnergy() << " J\n";

    int frame_idx = 0;
    int cycle_idx = 0;
    double max_elastic = sim.elasticEnergy();

    while (global_step < total_steps) {
        // Chunk size: advance to whichever boundary comes first
        const int steps_to_frame = spf - (global_step % spf);
        const int chunk = std::min({steps_to_frame, steps_to_next_tick,
                                    total_steps - global_step});

        sim.relax(chunk, fp.step_size, 0.0, 0.0);
        global_step        += chunk;
        steps_to_next_tick -= chunk;

        if (steps_to_next_tick == 0) {
            sim.tickNeural();
            sim.applyActuators(fp.steps_per_cycle);
            steps_to_next_tick = fp.steps_per_cycle;
            ++cycle_idx;
        }

        if (global_step % spf == 0 || global_step == total_steps) {
            ++frame_idx;
            const double E_el = sim.elasticEnergy();
            max_elastic = std::max(max_elastic, E_el);
            std::cout << "  [frame " << std::setw(2) << frame_idx
                      << " / cycle " << std::setw(2) << cycle_idx << "]  elastic="
                      << E_el << " J  total=" << sim.totalEnergy() << " J\n";
            sim.copyPositionsBack(robot);
            vid.addFrame(robot,
                         static_cast<double>(global_step) * fp.step_size,
                         sim.activations_);
        }
    }

    std::cout << "\nMax elastic energy: " << max_elastic << " J\n";

    std::cout << "Frames recorded: " << num_frames
              << "  (" << num_frames << " frames * " << spf << " steps_per_frame = "
              << total_steps << " total steps)\n";

    vid.finish(out_path);
    std::cout << "Written: " << out_path << "\n";
    return 0;
}
