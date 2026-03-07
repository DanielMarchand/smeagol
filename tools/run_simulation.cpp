/**
 * run_simulation.cpp
 *
 * Loads a robot and simulation parameters from YAML, runs quasi-static
 * gradient descent frame-by-frame, and produces an MP4.
 *
 * Usage:
 *   run_simulation <robot.yaml> <simulation.yaml> [output.mp4]
 *
 * Requires a display server.  On headless machines:
 *   Xvfb :99 -screen 0 1280x720x24 &
 *   DISPLAY=:99 run_simulation robot.yaml simulation.yaml out.mp4
 *
 * Physics note: quasi-static gradient descent (Lipson & Pollack 2000).
 * No velocity or momentum; motion traces the energy-minimisation path.
 */

#include "Robot.h"
#include "Simulator.h"
#include "VideoRenderer.h"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <cstdlib>
#include <string>

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: run_simulation <robot.yaml> <simulation.yaml> [output.mp4]\n";
        return 1;
    }

    const std::string robot_path = argv[1];
    const std::string sim_path   = argv[2];

    // ── Load robot ────────────────────────────────────────────────────────
    Robot robot;
    try {
        robot = Robot::fromYAML(robot_path);
    } catch (const std::exception& e) {
        std::cerr << "Error loading robot: " << e.what() << "\n";
        return 1;
    }

    // ── Load simulation params ────────────────────────────────────────────
    int         fps             = 30;
    int         num_steps       = 1000000;
    int         steps_per_frame = 5000;
    double      step_size       = 1e-7;
    int         width           = 640;
    int         height          = 480;
    std::string out_path        = "/tmp/tetrahedron_fall.mp4";

    // Physics overrides (all optional; defaults come from Simulator / Materials)
    double      gravity              = Materials::g;
    double      k_floor              = Materials::k_floor;
    double      mu_static            = Materials::mu_static;
    double      wind                 = 0.0;
    double      bar_stiffness_override = -1.0;  // <0 → leave robot values unchanged

    try {
        YAML::Node cfg = YAML::LoadFile(sim_path);
        if (cfg["fps"])             fps             = cfg["fps"].as<int>();
        if (cfg["num_steps"])       num_steps       = cfg["num_steps"].as<int>();
        else if (cfg["num_frames"]) {
            if (cfg["steps_per_frame"]) steps_per_frame = cfg["steps_per_frame"].as<int>();
            num_steps = cfg["num_frames"].as<int>() * steps_per_frame;  // legacy
        }
        if (cfg["steps_per_frame"]) steps_per_frame = cfg["steps_per_frame"].as<int>();
        if (cfg["step_size"])       step_size       = cfg["step_size"].as<double>();
        if (cfg["width"])           width           = cfg["width"].as<int>();
        if (cfg["height"])          height          = cfg["height"].as<int>();
        if (cfg["output"])          out_path        = cfg["output"].as<std::string>();
        if (cfg["gravity"])         gravity         = cfg["gravity"].as<double>();
        if (cfg["k_floor"])         k_floor         = cfg["k_floor"].as<double>();
        if (cfg["mu_static"])       mu_static       = cfg["mu_static"].as<double>();
        if (cfg["wind"])            wind            = cfg["wind"].as<double>();
        if (cfg["bar_stiffness"])   bar_stiffness_override = cfg["bar_stiffness"].as<double>();
    } catch (const std::exception& e) {
        std::cerr << "Error loading simulation config: " << e.what() << "\n";
        return 1;
    }

    // Apply bar stiffness override to all bars in the robot.
    if (bar_stiffness_override > 0.0)
        for (auto& bar : robot.bars)
            bar.stiffness = bar_stiffness_override;

    // CI_NUMSTEPS overrides num_steps from YAML (keeps CI runs fast)
    if (const char* ci_env = std::getenv("CI_NUMSTEPS")) {
        num_steps = std::stoi(ci_env);
    }
    // Legacy CI env var
    if (const char* ci_env = std::getenv("CI_NUMFRAMES")) {
        num_steps = std::stoi(ci_env) * steps_per_frame;
    }
    const int num_frames = num_steps / steps_per_frame;

    std::cout << "Robot:  " << robot_path
              << "  (" << robot.vertices.size() << "v, " << robot.bars.size() << "b)\n"
              << "Sim:    num_steps=" << num_steps
              << "  steps_per_frame=" << steps_per_frame
              << "  step_size=" << step_size << "\n"
              << "Physics: gravity=" << gravity
              << "  k_floor=" << k_floor
              << "  mu_static=" << mu_static
              << "  wind=" << wind;
    if (bar_stiffness_override > 0.0)
        std::cout << "  bar_stiffness=" << bar_stiffness_override << " (override)";
    std::cout << "\nOutput: " << num_frames << " frames @ " << fps
              << " fps → " << out_path << "\n\n";

    // ── Initial energy report ─────────────────────────────────────────────
    Simulator sim(robot);
    sim.gravity   = gravity;
    sim.k_floor   = k_floor;
    sim.mu_static = mu_static;
    sim.wind      = wind;
    std::cout << "Initial energies:\n"
              << "  H_elastic   = " << sim.elasticEnergy()       << " J\n"
              << "  H_gravity   = " << sim.gravitationalEnergy() << " J\n"
              << "  H_collision = " << sim.collisionEnergy()     << " J\n"
              << "  H_total     = " << sim.totalEnergy()         << " J\n\n";

    // ── Simulate + record ─────────────────────────────────────────────────
    VideoRenderer vid(fps, width, height);

    for (int f = 0; f < num_frames; ++f)
    {
        // Neural cycle: tick network, then update bar rest-lengths
        sim.tickNeural();
        sim.applyActuators(steps_per_frame);

        // Apply debug actuators (sine-wave driven, time-based)
        sim.applyDebugActuators(static_cast<double>(f) / fps);

        // tol=0 → never exit early; noise=0 → deterministic
        sim.relax(steps_per_frame, step_size, 0.0, 0.0);
        sim.copyPositionsBack(robot);
        vid.addFrame(robot, static_cast<double>(f) / fps, sim.activations_);

        if (f % fps == 0) {
            std::cout << "  t=" << static_cast<double>(f) / fps
                      << "s  E_elastic=" << sim.elasticEnergy()
                      << "  E_total="    << sim.totalEnergy();
            if (!sim.activations_.empty()) {
                std::cout << "  neurons=[";
                for (std::size_t i = 0; i < sim.activations_.size(); ++i)
                    std::cout << (i ? "," : "") << sim.activations_[i];
                std::cout << "]";
            }
            std::cout << "\n";
        }
    }

    const bool ok = vid.finish(out_path);
    return ok ? 0 : 1;
}
