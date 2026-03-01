/**
 * snapshot_robot.cpp
 *
 * Renders a Robot loaded from a YAML file to a single PNG image and exits.
 * No interactive window is shown.
 *
 * Usage:
 *   snapshot_robot <robot.yaml> <output.png>
 *
 * Requires a display server (X11/Wayland) to be available so that Raylib
 * can open an off-screen buffer.  On a headless machine use Xvfb:
 *   Xvfb :99 -screen 0 1280x720x24 &
 *   DISPLAY=:99 snapshot_robot robot.yaml robot.png
 */

#include "Robot.h"
#include "SnapshotRenderer.h"

#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: snapshot_robot <robot.yaml> <output.png>\n";
        return 1;
    }

    const std::string yaml_path = argv[1];
    const std::string out_path  = argv[2];

    // ── Ensure output directory exists ────────────────────────────────────
    if (auto parent = fs::path(out_path).parent_path(); !parent.empty())
        fs::create_directories(parent);

    // ── Load ──────────────────────────────────────────────────────────────
    Robot robot;
    try {
        robot = Robot::fromYAML(yaml_path);
    } catch (const std::exception& e) {
        std::cerr << "Error loading '" << yaml_path << "': " << e.what() << "\n";
        return 1;
    }

    if (!robot.isValid()) {
        std::cerr << "Warning: robot has invalid cross-references. "
                  << "Image may be incomplete.\n";
    }

    std::cout << "Loaded robot id=" << robot.id
              << "  vertices="  << robot.vertices.size()
              << "  bars="      << robot.bars.size()
              << "  neurons="   << robot.neurons.size()
              << "  actuators=" << robot.actuators.size()
              << "\n";

    // ── Render ────────────────────────────────────────────────────────────
    SnapshotRenderer snap(1280, 720);
    snap.render(robot, out_path);

    std::cout << "Snapshot saved: " << out_path << "\n";
    return 0;
}
