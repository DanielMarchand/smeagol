#pragma once

#include "Robot.h"
#include <raylib.h>
#include <string>

/**
 * @brief Base rendering class; owns the Raylib window and shared scene setup.
 *
 * Provides a fixed orbital 3D camera, an infinite ground grid, basic
 * point-lighting, and a drawRobot() routine that maps the Robot data
 * model to Raylib draw calls.
 *
 * Subclasses (SnapshotRenderer, VideoRenderer) control when the window
 * opens and closes, and what happens each frame.
 *
 * Window lifecycle
 * ----------------
 *   openWindow()   – call once before any drawing
 *   closeWindow()  – call when done (destructor calls it if still open)
 *
 * Coordinate system
 * -----------------
 *   Raylib uses right-handed Y-up. The physics simulation uses Z-up, so
 *   drawRobot() remaps  (x, y, z_sim) → (x, z_sim, -y)  before passing
 *   positions to Raylib, keeping the visual "floor" at Y=0.
 */
class SceneRenderer
{
public:
    // ── Construction / destruction ────────────────────────────────────────

    /**
     * @param window_title  Caption shown on the OS window.
     * @param width         Window width in pixels.
     * @param height        Window height in pixels.
     * @param target_fps    Vsync target (0 = unlocked).
     */
    explicit SceneRenderer(const std::string& window_title = "Golem 2000",
                           int width      = 1280,
                           int height     = 720,
                           int target_fps = 60);

    virtual ~SceneRenderer();

    // ── Window lifecycle (called by subclasses) ───────────────────────────

    /**
     * Open the OS window.  Pass hidden=true to create an off-screen context
     * (used by SnapshotRenderer / headless builds).
     */
    void openWindow(bool hidden = false);
    void closeWindow();

    [[nodiscard]] bool isOpen() const { return m_open; }

    // ── Camera control ────────────────────────────────────────────────────

    /**
     * Reset the camera to the default orbital position distance metres
     * above and behind the world origin.
     */
    void resetCamera(float distance = 1.5f);

    /**
     * Point the camera at a specific world position (Z-up coords).
     */
    void lookAt(float x, float y, float z);

    /**
     * Open a window, render the robot every frame until the user closes it.
     * Supports Raylib orbital camera controls (left-drag to orbit,
     * scroll to zoom, right-drag to pan).
     *
     * @param robot        The robot to display.
     * @param floor_slices Grid slices drawn around the robot.
     */
    void runInteractive(const Robot& robot, int floor_slices = 40);

    // ── Coordinate helper (public so subclasses / tests can use it) ───────

    /// Convert simulation Z-up position to Raylib Y-up Vector3.
    static Vector3 toRaylib(double x, double y, double z);

protected:
    // ── Frame helpers (call inside BeginDrawing / BeginMode3D blocks) ─────

    /// Clear background and set up 3D mode for one frame.
    void beginScene();

    /// End 3D mode and 2D overlay, flip buffer.
    void endScene();

    /**
     * Draw the infinite ground plane as a Raylib grid.
     * @param slices  Number of grid lines in each direction.
     * @param spacing Grid line spacing [m].
     */
    void drawFloor(int slices = 40, float spacing = 0.1f);

    /**
     * Draw all vertices as spheres and all bars as cylinders.
     * @param robot          The robot to draw.
     * @param vertex_radius  Visual sphere radius [m].
     * @param vertex_color   Colour for vertex spheres.
     * @param bar_color      Colour for bar cylinders.
     */
    void drawRobot(const Robot&  robot,
                   float         vertex_radius = 0.012f,
                   Color         vertex_color  = SKYBLUE,
                   Color         bar_color     = LIGHTGRAY);

    // ── Data ──────────────────────────────────────────────────────────────

    std::string m_title;
    int         m_width;
    int         m_height;
    int         m_target_fps;
    bool        m_open = false;

    Camera3D    m_camera{};
};
