#pragma once

#include "Robot.h"
#include <raylib.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

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

    // ── Render geometry (set before the first frame) ──────────────────────
    /// Visual sphere radius for vertices [m].  Set to repulse_vertex_min_dist/2
    /// so surfaces visually touch at the repulsion threshold.
    float render_vertex_radius = 0.010f;
    /// Visual cylinder radius for bars [m].  Set to repulse_bar_min_dist/2.
    float render_bar_radius    = 0.010f;
    /// Housing cylinder radius = render_bar_radius × this scale  (fat outer tube).
    float actuator_housing_radius_scale = 2.0f;
    /// Piston rod radius = render_bar_radius × this scale  (thin inner rod).
    float actuator_rod_radius_scale     = 0.65f;

    // ── Floor-contact / proximity effects (set before the first frame) ───
    /// Vertex centre height [m] at or below which the vertex is highlighted
    /// bright red (touching / clipping floor).  Defaults to render_vertex_radius
    /// so the highlight fires when the sphere surface reaches the floor.
    float floor_contact_threshold = 0.010f;
    /// Vertices whose centre is below this height [m] cast a blob shadow on the
    /// floor.  Shadow becomes more opaque and larger as the vertex descends.
    /// Set to 0 to disable shadows.
    float floor_shadow_height     = 0.05f;

    // ── Floor grid settings (set before the first frame) ─────────────────
    float floor_grid_spacing = 0.01f;  ///< spacing between ground-grid lines [m]  (default 1 cm)
    int   floor_grid_slices  = 100;    ///< grid lines per axis; total visible range = slices × spacing

    // ── Camera settings (set before the first frame) ──────────────────────
    float camera_distance  = 1.5f;  ///< distance from target [m]
    float camera_fov       = 45.0f; ///< perspective field of view [degrees]
    float camera_elevation = 0.8f;  ///< vertical offset as a fraction of lateral distance
    bool  camera_follow    = true;  ///< if true, camera tracks robot CoM; if false, fixed at origin

    // ── Camera control ────────────────────────────────────────────────────

    /**
     * Reset the camera to the orbital position defined by the camera_* fields.
     */
    void resetCamera();

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

    /** Set rendering verbosity.  Call before openWindow().
     *  false (default) = suppress raylib INFO/DEBUG trace output.
     *  true            = show full raylib trace log (useful for debugging). */
    void setVerbose(bool v) { verbose_ = v; }

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
     *
     * Each structural bar gets a distinct muted colour cycling through a
     * small palette (dusty blues, tans, sages, etc.).  Actuated bars
     * (those referenced by robot.actuators) are drawn in amber so they
     * stand out from the structural frame.
     *
     * @param robot          The robot to draw.
     * @param vertex_color   Colour for vertex spheres.
     * @param bar_color      Unused (kept for API compatibility).
     *
     * Bar and vertex radii are taken from render_bar_radius / render_vertex_radius.
     */
    /**
     * @param rest_lengths  Per-bar current rest lengths from Simulator::rest_lengths_.
     *                      When provided, actuated bars are drawn as a fat housing
     *                      cylinder (base rest length `robot.bars[i].rest_length`) plus
     *                      a thin piston rod (full v1→v2 geometry).  The rod protrudes
     *                      past the housing end to show any extension above base.
     *                      When empty (default), actuated bars are drawn as a single
     *                      bright cylinder (backwards-compatible).
     */
    void drawRobot(const Robot&               robot,
                   const std::vector<double>& rest_lengths = {},
                   Color                      vertex_color = SKYBLUE);

    /**
     * Draw a neural-network overlay above the robot.
     *
     * Neurons are laid out in a horizontal ring above the robot's centre of
     * mass.  Synapse lines are drawn first (so spheres appear on top):
     *   - Blue  = positive (excitatory) weight
     *   - Red   = negative (inhibitory) weight
     * Neuron spheres:
     *   - Bright red   = active  (activation value > 0.5)
     *   - Dark grey    = silent
     * Actuator connections:
     *   - Bright green line from neuron sphere to the midpoint of the
     *     actuated bar (using Robot::bars and Robot::actuators).
     *
     * Must be called inside a BeginMode3D / EndMode3D block.
     *
     * Projects all 3D positions to screen space so the overlay always
     * renders within the frame.  Neuron ring positions are clamped to the
     * screen edges; bar-midpoint connection lines are drawn to their true
     * projected position (may extend off-screen for off-frame bars).
     *
     * Must be called AFTER EndMode3D() but still inside BeginDrawing().
     *
     * @param robot       Robot topology (neurons, actuators, bar geometry).
     * @param activations Runtime activation values from Simulator::activations_.
     *                    If shorter than robot.neurons, missing entries = 0.
     */
    void drawNeuralOverlay(const Robot&               robot,
                           const std::vector<double>& activations);

    /**
     * Draw the historical centre-of-mass path as a 3D trail.
     *
     * Each entry in @p trail is a simulation-space (Z-up) CoM position.
     * Older positions are drawn as small dark spheres; the most recent
     * position is drawn as a bright green sphere.  Consecutive positions
     * are connected by gradient-coloured line segments.
     *
     * Must be called inside a BeginMode3D / EndMode3D block.
     *
     * @param trail  CoM positions in chronological order (Z-up coords).
     */
    void drawComTrail(const std::vector<Eigen::Vector3d>& trail);

    // ── Data ──────────────────────────────────────────────────────────────

    std::string m_title;
    int         m_width;
    int         m_height;
    int         m_target_fps;
    bool        m_open    = false;
    bool        verbose_  = false;  ///< when false, raylib trace log is silenced (default)

    Camera3D    m_camera{};
};
