/**
 * deformation_series.cpp
 *
 * Displaces one vertex of a Robot incrementally along a user-specified axis,
 * computes the total energy at each step via Simulator, saves a PNG snapshot
 * per step, and prints an energy table to stdout.
 *
 * Purpose: manual visual sanity-check of §3.1 (energy function).
 * A robot at rest should show near-zero elastic energy; stretching its bars
 * should produce a smoothly rising elastic energy curve.
 *
 * Usage:
 *   deformation_series <robot.yaml> <output_dir>
 *                      <vertex_idx>
 *                      <dir_x> <dir_y> <dir_z>
 *                      <step_size_m> <num_steps>
 *
 * Example (see examples/deformation_series/run.sh):
 *   deformation_series good_robot.yaml /tmp/deform_out  3  0 0 1  0.02 10
 *
 * Requires a display server.  On headless machines use Xvfb:
 *   Xvfb :99 -screen 0 1280x720x24 &
 *   DISPLAY=:99 deformation_series ...
 */

#include "Robot.h"
#include "Simulator.h"
#include "SnapshotRenderer.h"

#include <Eigen/Dense>

#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

static void usage()
{
    std::cerr <<
        "Usage: deformation_series <robot.yaml> <output_dir>\n"
        "                          <vertex_idx>\n"
        "                          <dir_x> <dir_y> <dir_z>\n"
        "                          <step_size_m> <num_steps>\n";
}

int main(int argc, char* argv[])
{
    if (argc != 9) { usage(); return 1; }

    const std::string yaml_path  = argv[1];
    const std::string output_dir = argv[2];
    const int    vertex_idx  = std::stoi(argv[3]);
    const double dir_x       = std::stod(argv[4]);
    const double dir_y       = std::stod(argv[5]);
    const double dir_z       = std::stod(argv[6]);
    const double step_size   = std::stod(argv[7]);
    const int    num_steps   = std::stoi(argv[8]);

    // Normalise direction
    Eigen::Vector3d dir(dir_x, dir_y, dir_z);
    if (dir.norm() < 1e-12) {
        std::cerr << "Error: direction vector must be non-zero.\n";
        return 1;
    }
    dir.normalize();

    // ── Load robot ────────────────────────────────────────────────────────
    Robot robot;
    try {
        robot = Robot::fromYAML(yaml_path);
    } catch (const std::exception& e) {
        std::cerr << "Error loading '" << yaml_path << "': " << e.what() << "\n";
        return 1;
    }

    if (vertex_idx < 0 ||
        vertex_idx >= static_cast<int>(robot.vertices.size()))
    {
        std::cerr << "Error: vertex_idx " << vertex_idx
                  << " out of range (robot has "
                  << robot.vertices.size() << " vertices).\n";
        return 1;
    }

    fs::create_directories(output_dir);

    // ── Header ────────────────────────────────────────────────────────────
    std::cout << "\nDeformation series: robot id=" << robot.id
              << "  vertex=" << vertex_idx
              << "  dir=[" << dir_x << "," << dir_y << "," << dir_z << "]"
              << "  step=" << step_size << " m"
              << "  steps=" << num_steps << "\n\n";

    std::cout << std::left
              << std::setw(6)  << "Step"
              << std::setw(12) << "Disp (m)"
              << std::setw(18) << "H_elastic (J)"
              << std::setw(18) << "H_gravity (J)"
              << std::setw(18) << "H_total (J)"
              << "\n"
              << std::string(72, '-') << "\n";

    // Store the original position to offset from
    const Eigen::Vector3d origin = robot.vertices[vertex_idx].pos;

    SnapshotRenderer snap(1280, 720);

    for (int step = 0; step <= num_steps; ++step)
    {
        const double disp = step * step_size;

        // Apply cumulative displacement from origin
        robot.vertices[vertex_idx].pos = origin + disp * dir;

        // Compute energy
        Simulator sim(robot);
        const double He = sim.elasticEnergy();
        const double Hg = sim.gravitationalEnergy();
        const double Ht = He + Hg;

        // Print row
        std::cout << std::left
                  << std::setw(6)  << step
                  << std::setw(12) << std::fixed << std::setprecision(4) << disp
                  << std::setw(18) << std::scientific << std::setprecision(4) << He
                  << std::setw(18) << Hg
                  << std::setw(18) << Ht
                  << "\n";

        // Save snapshot
        std::ostringstream fname;
        fname << output_dir << "/step_"
              << std::setw(2) << std::setfill('0') << step
              << ".png";
        snap.render(robot, fname.str());
    }

    std::cout << "\nSnapshots written to: " << output_dir << "/\n\n";
    std::cout << "Expected behaviour:\n"
              << "  H_elastic should be ~0 at step 0 (robot at rest)\n"
              << "  H_elastic should rise smoothly with displacement\n"
              << "  H_gravity should rise if displacement has +Z component\n";

    return 0;
}
