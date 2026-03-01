#pragma once

#include "SceneRenderer.h"
#include "Robot.h"
#include <string>

/**
 * @brief Single-frame PNG renderer; no interactive window required.
 *
 * Opens an off-screen (hidden) Raylib window, renders one fully-lit frame
 * of the robot from a fixed viewpoint, exports it as a PNG, then closes
 * the window.  The whole operation is safe to call from non-interactive
 * environments (no cursor, no event loop) provided a display server is
 * available.
 *
 * Typical usage:
 * @code
 *   SnapshotRenderer snap;
 *   snap.render(robot, "debug/robot_42.png");
 *
 *   // Or via the Robot convenience method:
 *   robot.saveDebugImage("debug/robot_42.png");
 * @endcode
 *
 * Output file format
 * ------------------
 * The extension in `output_path` is ignored; the file is always written as
 * PNG.  Parent directories must exist before calling render().
 */
class SnapshotRenderer : public SceneRenderer
{
public:
    /**
     * @param width   Pixel width of the off-screen buffer.
     * @param height  Pixel height of the off-screen buffer.
     */
    explicit SnapshotRenderer(int width = 1280, int height = 720);

    /**
     * Render the robot and save the result to output_path.
     *
     * @param robot        Robot to draw.
     * @param output_path  Destination file path (PNG).
     * @throws std::runtime_error if the window cannot be opened or the
     *         image cannot be exported.
     */
    void render(const Robot& robot, const std::string& output_path);
};
