#include "SceneRenderer.h"
#include <rlgl.h>
#include <cmath>
#include <vector>

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

// ── Neural overlay ────────────────────────────────────────────────────────────

void SceneRenderer::drawNeuralOverlay(const Robot&               robot,
                                      const std::vector<double>& activations)
{
    const int N = static_cast<int>(robot.neurons.size());
    if (N == 0) return;

    // ── Layout: ring of neurons above the robot CoM ───────────────────────
    const Eigen::Vector3d com = robot.centerOfMass();
    const float ring_r        = std::max(0.07f, N * 0.022f);
    const float lift          = 0.30f;   // above robot CoM in sim-Z

    std::vector<Vector3> npos(N);
    for (int i = 0; i < N; ++i)
    {
        const float angle = (2.0f * 3.14159265f / static_cast<float>(N)) * i;
        const double nx = com.x() + ring_r * std::cos(angle);
        const double ny = com.y() + ring_r * std::sin(angle);
        const double nz = com.z() + lift;
        npos[i] = toRaylib(nx, ny, nz);
    }

    // ── Synapse connections (draw first so spheres sit on top) ────────────
    for (int i = 0; i < N; ++i)
    {
        const Eigen::VectorXd& w = robot.neurons[i].synapse_weights;
        const int sw = static_cast<int>(w.size());
        for (int j = 0; j < N && j < sw; ++j)
        {
            if (std::abs(w(j)) < 0.01) continue;
            Color c = (w(j) > 0.0)
                ? Color{100, 149, 237, 200}   // cornflower blue  = excitatory
                : Color{220,  70,  70, 200};  // soft red         = inhibitory
            DrawLine3D(npos[i], npos[j], c);
        }
    }

    // ── Actuator connections: neuron sphere → bar midpoint ────────────────
    for (const auto& a : robot.actuators)
    {
        if (a.neuron_idx < 0 || a.neuron_idx >= N) continue;
        if (a.bar_idx    < 0 || a.bar_idx    >= static_cast<int>(robot.bars.size())) continue;

        const Bar& bar = robot.bars[a.bar_idx];
        const Eigen::Vector3d mid =
            0.5 * (robot.vertices[bar.v1].pos + robot.vertices[bar.v2].pos);
        DrawLine3D(npos[a.neuron_idx],
                   toRaylib(mid.x(), mid.y(), mid.z()),
                   Color{50, 220, 50, 220});   // bright green
    }

    // ── Neuron spheres (drawn last → on top of all lines) ─────────────────
    for (int i = 0; i < N; ++i)
    {
        const bool active = (i < static_cast<int>(activations.size()))
                            && (activations[i] > 0.5);
        const Color c = active
            ? Color{255,  50,  50, 255}   // bright red  = firing
            : Color{ 70,  70,  70, 255};  // dark grey   = silent
        DrawSphere(npos[i], 0.016f, c);
    }
}

// ── CoM trail ─────────────────────────────────────────────────────────────────

void SceneRenderer::drawComTrail(const std::vector<Eigen::Vector3d>& trail)
{
    if (trail.empty()) return;

    const int n = static_cast<int>(trail.size());

    // Draw line segments connecting consecutive CoM positions.
    // Colour transitions from dark grey (oldest) to gold (newest).
    for (int i = 1; i < n; ++i)
    {
        const float t = static_cast<float>(i) / static_cast<float>(n);
        const Color c = {
            static_cast<unsigned char>(80  + static_cast<int>(175 * t)),   // R: 80→255
            static_cast<unsigned char>(80  + static_cast<int>( 90 * t)),   // G: 80→170
            static_cast<unsigned char>(static_cast<int>(80 * (1.0f - t))), // B: 80→0
            static_cast<unsigned char>(100 + static_cast<int>(155 * t))    // A: 100→255
        };
        DrawLine3D(
            toRaylib(trail[i-1].x(), trail[i-1].y(), trail[i-1].z()),
            toRaylib(trail[i  ].x(), trail[i  ].y(), trail[i  ].z()),
            c);
    }

    // Draw a small sphere at each past CoM; brightest/largest at the current end.
    for (int i = 0; i < n; ++i)
    {
        const float t = static_cast<float>(i) / static_cast<float>(n);
        const Vector3 pos = toRaylib(trail[i].x(), trail[i].y(), trail[i].z());

        if (i == n - 1)
        {
            // Current CoM — bright green, slightly larger
            DrawSphere(pos, 0.013f, Color{50, 255, 80, 255});
        }
        else
        {
            const float r = 0.005f + 0.004f * t;   // 5mm → 9mm as trail ages
            const Color c = {
                static_cast<unsigned char>(80  + static_cast<int>(175 * t)),
                static_cast<unsigned char>(80  + static_cast<int>( 90 * t)),
                static_cast<unsigned char>(static_cast<int>(80 * (1.0f - t))),
                static_cast<unsigned char>(80  + static_cast<int>(155 * t))
            };
            DrawSphere(pos, r, c);
        }
    }
}
