/**
 * visualize_robot.cpp
 *
 * Opens an interactive 3D window showing a Robot loaded from a YAML file.
 *
 * Usage:
 *   visualize_robot <robot.yaml>
 *
 * Controls (Raylib orbital camera):
 *   Left-drag    Orbit
 *   Scroll       Zoom in / out
 *   Right-drag   Pan
 *   ESC          Quit
 */

#include "Robot.h"
#include "SceneRenderer.h"

#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: visualize_robot <robot.yaml>\n";
        return 1;
    }

    const std::string path = argv[1];

    // ── Load ──────────────────────────────────────────────────────────────
    Robot robot;
    try {
        robot = Robot::fromYAML(path);
    } catch (const std::exception& e) {
        std::cerr << "Error loading '" << path << "': " << e.what() << "\n";
        return 1;
    }

    if (!robot.isValid()) {
        std::cerr << "Warning: robot loaded from '" << path
                  << "' has invalid cross-references. "
                  << "Display may be incomplete.\n";
        // Continue anyway – drawRobot skips out-of-range bars
    }

    std::cout << "Loaded robot id=" << robot.id
              << "  vertices="  << robot.vertices.size()
              << "  bars="      << robot.bars.size()
              << "  neurons="   << robot.neurons.size()
              << "  actuators=" << robot.actuators.size()
              << "\n";

    // ── Render ────────────────────────────────────────────────────────────
    SceneRenderer renderer("Golem 2000 – Robot Visualizer", 1280, 720, 60);
    renderer.runInteractive(robot);

    return 0;
}
