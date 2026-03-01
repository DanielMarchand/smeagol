#include "SceneRenderer.h"
#include <rlgl.h>
#include <cmath>

// ── Construction / destruction ────────────────────────────────────────────────

SceneRenderer::SceneRenderer(const std::string& window_title,
                             int width, int height, int target_fps)
    : m_title(window_title)
    , m_width(width)
    , m_height(height)
    , m_target_fps(target_fps)
{
    resetCamera();
}

SceneRenderer::~SceneRenderer()
{
    if (m_open) closeWindow();
}

// ── Window lifecycle ──────────────────────────────────────────────────────────

void SceneRenderer::openWindow(bool hidden)
{
    if (m_open) return;

    unsigned int flags = FLAG_MSAA_4X_HINT;
    if (hidden) flags |= FLAG_WINDOW_HIDDEN;

    SetConfigFlags(flags);
    InitWindow(m_width, m_height, m_title.c_str());
    if (m_target_fps > 0)
        SetTargetFPS(m_target_fps);

    m_open = true;
}

void SceneRenderer::closeWindow()
{
    if (!m_open) return;
    CloseWindow();
    m_open = false;
}

// ── Camera ────────────────────────────────────────────────────────────────────

void SceneRenderer::resetCamera(float distance)
{
    m_camera.position   = { distance,  distance * 0.8f, distance };
    m_camera.target     = { 0.0f, 0.0f, 0.0f };
    m_camera.up         = { 0.0f, 1.0f, 0.0f };
    m_camera.fovy       = 45.0f;
    m_camera.projection = CAMERA_PERSPECTIVE;
}

void SceneRenderer::lookAt(float x, float y, float z)
{
    // Keep current offset vector, just translate target
    Vector3 offset = {
        m_camera.position.x - m_camera.target.x,
        m_camera.position.y - m_camera.target.y,
        m_camera.position.z - m_camera.target.z
    };
    m_camera.target   = { x, y, z };
    m_camera.position = { x + offset.x, y + offset.y, z + offset.z };
}

// ── Coordinate helper ─────────────────────────────────────────────────────────

Vector3 SceneRenderer::toRaylib(double x, double y, double z)
{
    // Simulation: Z-up   →   Raylib: Y-up
    //   sim x → ray x
    //   sim y → ray -z
    //   sim z → ray y
    return { static_cast<float>(x),
             static_cast<float>(z),
             static_cast<float>(-y) };
}

// ── Frame helpers ─────────────────────────────────────────────────────────────

void SceneRenderer::beginScene()
{
    BeginDrawing();
    ClearBackground({ 30, 30, 30, 255 });   // dark grey background
    BeginMode3D(m_camera);
}

void SceneRenderer::endScene()
{
    EndMode3D();
    EndDrawing();
}

void SceneRenderer::drawFloor(int slices, float spacing)
{
    // DrawGrid draws in the XZ plane (Y=0), which matches our remapped coords.
    DrawGrid(slices, spacing);

    // Thin solid plane so the grid is visible against a bright scene
    rlPushMatrix();
        rlTranslatef(0.0f, -0.001f, 0.0f);     // just below Y=0
        DrawPlane({ 0.0f, 0.0f, 0.0f },
                  { static_cast<float>(slices) * spacing,
                    static_cast<float>(slices) * spacing },
                  { 20, 20, 20, 200 });
    rlPopMatrix();
}

void SceneRenderer::runInteractive(const Robot& robot, int floor_slices)
{
    openWindow();

    // Centre the camera on the robot's centre of mass
    const Eigen::Vector3d com = robot.centerOfMass();
    lookAt(static_cast<float>(com.x()),
           static_cast<float>(com.z()),    // raylib Y = sim Z
           static_cast<float>(-com.y()));  // raylib Z = -sim Y

    while (!WindowShouldClose()) {
        UpdateCamera(&m_camera, CAMERA_ORBITAL);

        beginScene();
        drawFloor(floor_slices, 0.1f);
        drawRobot(robot);

        // HUD
        EndMode3D();
        DrawText("[ESC] quit  |  drag to orbit  |  scroll to zoom",
                 10, 10, 16, RAYWHITE);
        EndDrawing();
    }

    closeWindow();
}

void SceneRenderer::drawRobot(const Robot& robot,
                              float vertex_radius,
                              Color vertex_color,
                              Color bar_color)
{
    // ── bars ──────────────────────────────────────────────────────────────
    for (const auto& bar : robot.bars) {
        if (bar.v1 < 0 || bar.v1 >= static_cast<int>(robot.vertices.size())) continue;
        if (bar.v2 < 0 || bar.v2 >= static_cast<int>(robot.vertices.size())) continue;

        const auto& p1 = robot.vertices[bar.v1].pos;
        const auto& p2 = robot.vertices[bar.v2].pos;

        Vector3 rp1 = toRaylib(p1.x(), p1.y(), p1.z());
        Vector3 rp2 = toRaylib(p2.x(), p2.y(), p2.z());

        DrawCylinderEx(rp1, rp2,
                       static_cast<float>(bar.radius),
                       static_cast<float>(bar.radius),
                       8, bar_color);
    }

    // ── vertices (drawn on top of bars) ───────────────────────────────────
    for (const auto& vertex : robot.vertices) {
        Vector3 rp = toRaylib(vertex.pos.x(), vertex.pos.y(), vertex.pos.z());
        DrawSphere(rp, vertex_radius, vertex_color);
    }
}
