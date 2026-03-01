/**
 * fall_animation.cpp
 *
 * Loads a robot and simulation parameters from YAML, runs quasi-static
 * gradient descent frame-by-frame, and produces an MP4.
 *
 * Usage:
 *   fall_animation <robot.yaml> <simulation.yaml> [output.mp4]
 *
 * Requires a display server.  On headless machines:
 *   Xvfb :99 -screen 0 1280x720x24 &
 *   DISPLAY=:99 fall_animation robot.yaml simulation.yaml out.mp4
 *
 * Physics note: quasi-static gradient descent (Lipson & Pollack 2000).
 * No velocity or momentum; motion traces the energy-minimisation path.
 */

#include "Robot.h"
#include "Simulator.h"
#include "VideoRenderer.h"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: fall_animation <robot.yaml> <simulation.yaml> [output.mp4]\n";
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
    int         num_frames      = 200;
    int         steps_per_frame = 5000;
    double      step_size       = 1e-7;
    int         width           = 640;
    int         height          = 480;
    std::string out_path        = "/tmp/tetrahedron_fall.mp4";

    try {
        YAML::Node cfg = YAML::LoadFile(sim_path);
        if (cfg["fps"])             fps             = cfg["fps"].as<int>();
        if (cfg["num_frames"])      num_frames      = cfg["num_frames"].as<int>();
        if (cfg["steps_per_frame"]) steps_per_frame = cfg["steps_per_frame"].as<int>();
        if (cfg["step_size"])       step_size       = cfg["step_size"].as<double>();
        if (cfg["width"])           width           = cfg["width"].as<int>();
        if (cfg["height"])          height          = cfg["height"].as<int>();
        if (cfg["output"])          out_path        = cfg["output"].as<std::string>();
    } catch (const std::exception& e) {
        std::cerr << "Error loading simulation config: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Robot:  " << robot_path
              << "  (" << robot.vertices.size() << "v, " << robot.bars.size() << "b)\n"
              << "Sim:    steps_per_frame=" << steps_per_frame
              << "  step_size=" << step_size << "\n"
              << "Output: " << num_frames << " frames @ " << fps
              << " fps → " << out_path << "\n\n";

    // ── Initial energy report ─────────────────────────────────────────────
    Simulator sim(robot);
    std::cout << "Initial energies:\n"
              << "  H_elastic   = " << sim.elasticEnergy()       << " J\n"
              << "  H_gravity   = " << sim.gravitationalEnergy() << " J\n"
              << "  H_collision = " << sim.collisionEnergy()     << " J\n"
              << "  H_total     = " << sim.totalEnergy()         << " J\n\n";

    // ── Simulate + record ─────────────────────────────────────────────────
    VideoRenderer vid(fps, width, height);

    for (int f = 0; f < num_frames; ++f)
    {
        // tol=0 → never exit early; noise=0 → deterministic
        sim.relax(steps_per_frame, step_size, 0.0, 0.0);
        sim.copyPositionsBack(robot);
        vid.addFrame(robot, static_cast<double>(f) / fps);

        if (f % fps == 0)
            std::cout << "  t=" << static_cast<double>(f) / fps
                      << "s  E_elastic=" << sim.elasticEnergy()
                      << "  E_total="    << sim.totalEnergy() << "\n";
    }

    const bool ok = vid.finish(out_path);
    return ok ? 0 : 1;
}
