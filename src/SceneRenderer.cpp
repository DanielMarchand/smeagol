#include "SceneRenderer.h"
#include <rlgl.h>
#include <cmath>
#include <unordered_set>
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

    // Suppress raylib INFO/DEBUG trace unless verbose mode is on.
    // LOG_WARNING keeps actual warnings and errors visible.
    SetTraceLogLevel(verbose_ ? LOG_ALL : LOG_WARNING);

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

// ── Per-bar colour helpers ────────────────────────────────────────────────────
// Both use a Knuth multiplicative hash so each bar index maps to a distinct,
// deterministic colour that is consistent across every frame of a video.
//
// Hues deliberately avoided:
//   • ~200-250° blue  → neural firing/synapse overlay  {50,120,255} / {80,140,255}
//   • ~100-160° green → CoM trail                      {60,200,80}
//   • ~0° near-grey   → inactive synapse wires         {100,100,100}

static Color structuralBarColor(int idx)
{
    // Dark, muted warm/cool colours (L≈25–45).
    static const Color palette[] = {
        { 110,  35,  30, 255 },  // dark crimson
        { 120,  55,  15, 255 },  // dark amber-brown
        {  80,  20,  75, 255 },  // dark plum
        { 100,  60,  20, 255 },  // dark sienna
        {  90,  20,  50, 255 },  // dark wine-magenta
        {  70,  25,  85, 255 },  // dark purple
        { 115,  40,  25, 255 },  // dark terracotta
        {  85,  55,  12, 255 },  // dark umber (warm, not green)
        {  95,  15,  60, 255 },  // dark claret
        { 105,  35,  90, 255 },  // dark violet
        {  75,  40,  10, 255 },  // dark brown
        { 100,  18,  32, 255 },  // dark ruby
        {  65,  50,  72, 255 },  // dark muted mauve
        { 115,  62,  30, 255 },  // dark copper
        {  80,  14,  42, 255 },  // dark maroon
        {  92,  65,  22, 255 },  // dark gold-brown
    };
    static constexpr int N = static_cast<int>(sizeof(palette)/sizeof(palette[0]));
    return palette[(static_cast<uint32_t>(idx) * 2654435761u) % static_cast<uint32_t>(N)];
}

static Color actuatorBrightColor(int idx)
{
    // Vivid, high-saturation colours for actuated bars (L≈55–80).
    // Kept away from neural blues and trail greens.
    static const Color palette[] = {
        { 255,  88,  10, 255 },  // vivid orange
        { 255,  38,  52, 255 },  // neon red
        { 255, 172,   0, 255 },  // vivid amber
        { 255,  28, 165, 255 },  // hot magenta
        { 255, 122,   0, 255 },  // bright tangerine
        { 255,  58,  88, 255 },  // vivid coral-red
        { 238, 255,   0, 255 },  // electric yellow
        { 255,  48, 215, 255 },  // bright pink-magenta
        { 255, 202,  10, 255 },  // vivid gold
        { 228,  28,  28, 255 },  // pure red
        { 255, 142,  18, 255 },  // warm orange
        { 255,  78, 142, 255 },  // hot pink
        { 255, 212,  38, 255 },  // bright yellow
        { 198,   0, 255, 255 },  // electric purple
        { 255,  28, 102, 255 },  // vivid rose
        {   0, 228, 222, 255 },  // electric cyan (≠ neural blue {80,140,255})
    };
    static constexpr int N = static_cast<int>(sizeof(palette)/sizeof(palette[0]));
    return palette[(static_cast<uint32_t>(idx) * 2654435761u) % static_cast<uint32_t>(N)];
}

void SceneRenderer::drawRobot(const Robot& robot,
                              Color vertex_color,
                              Color /*bar_color*/)
{
    // Build set of actuated bar indices for O(1) lookup.
    std::unordered_set<int> actuated;
    actuated.reserve(robot.actuators.size());
    for (const auto& a : robot.actuators)
        actuated.insert(a.bar_idx);

    // ── bars ──────────────────────────────────────────────────────────────
    for (int bi = 0; bi < static_cast<int>(robot.bars.size()); ++bi) {
        const auto& bar = robot.bars[bi];
        if (bar.v1 < 0 || bar.v1 >= static_cast<int>(robot.vertices.size())) continue;
        if (bar.v2 < 0 || bar.v2 >= static_cast<int>(robot.vertices.size())) continue;

        const auto& p1 = robot.vertices[bar.v1].pos;
        const auto& p2 = robot.vertices[bar.v2].pos;

        Vector3 rp1 = toRaylib(p1.x(), p1.y(), p1.z());
        Vector3 rp2 = toRaylib(p2.x(), p2.y(), p2.z());

        if (actuated.count(bi)) {
            DrawCylinderEx(rp1, rp2,
                           render_bar_radius,
                           render_bar_radius,
                           8, actuatorBrightColor(bi));
        } else {
            // ── Structural bar: single dark muted cylinder ────────────────
            DrawCylinderEx(rp1, rp2,
                           render_bar_radius,
                           render_bar_radius,
                           8, structuralBarColor(bi));
        }
    }

    // ── vertices (drawn on top of bars) ───────────────────────────────────
    for (const auto& vertex : robot.vertices) {
        Vector3 rp = toRaylib(vertex.pos.x(), vertex.pos.y(), vertex.pos.z());
        DrawSphere(rp, render_vertex_radius, vertex_color);
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

    // Pre-compute per-neuron active flag for wire colouring.
    auto is_active = [&](int idx) -> bool {
        return idx >= 0 && idx < static_cast<int>(activations.size())
               && activations[idx] > 0.5;
    };

    // ── Synapse connections (draw first so spheres sit on top) ────────────
    // Wire colour: blue if the source neuron is firing, grey if silent.
    for (int i = 0; i < N; ++i)
    {
        const Eigen::VectorXd& w = robot.neurons[i].synapse_weights;
        const int sw = static_cast<int>(w.size());
        for (int j = 0; j < N && j < sw; ++j)
        {
            if (std::abs(w(j)) < 0.01) continue;
            const Color c = is_active(i)
                ? Color{ 80, 140, 255, 220}   // blue  = source neuron firing
                : Color{100, 100, 100, 160};  // grey  = source neuron silent
            DrawLine3D(npos[i], npos[j], c);
        }
    }

    // ── Actuator connections: neuron sphere → bar midpoint ────────────────
    // Wire colour: blue if the driving neuron is firing, grey if silent.
    for (const auto& a : robot.actuators)
    {
        if (a.neuron_idx < 0 || a.neuron_idx >= N) continue;
        if (a.bar_idx    < 0 || a.bar_idx    >= static_cast<int>(robot.bars.size())) continue;

        const Bar& bar = robot.bars[a.bar_idx];
        const Eigen::Vector3d mid =
            0.5 * (robot.vertices[bar.v1].pos + robot.vertices[bar.v2].pos);
        const Color wire_c = is_active(a.neuron_idx)
            ? Color{ 80, 140, 255, 220}   // blue = active
            : Color{100, 100, 100, 160};  // grey = silent
        DrawLine3D(npos[a.neuron_idx],
                   toRaylib(mid.x(), mid.y(), mid.z()),
                   wire_c);
    }

    // ── Neuron spheres (drawn last → on top of all lines) ─────────────────
    for (int i = 0; i < N; ++i)
    {
        const bool active = is_active(i);
        const Color c = active
            ? Color{ 50, 120, 255, 255}   // bright blue = firing
            : Color{ 70,  70,  70, 255};  // dark grey   = silent
        DrawSphere(npos[i], 0.016f, c);
    }
}

// ── CoM trail ─────────────────────────────────────────────────────────────────

void SceneRenderer::drawComTrail(const std::vector<Eigen::Vector3d>& trail)
{
    if (trail.empty()) return;

    const int n = static_cast<int>(trail.size());

    // Thin green line trail — no spheres.
    const Color trail_color = { 60, 200, 80, 180 };  // muted green
    for (int i = 1; i < n; ++i)
    {
        DrawLine3D(
            toRaylib(trail[i-1].x(), trail[i-1].y(), trail[i-1].z()),
            toRaylib(trail[i  ].x(), trail[i  ].y(), trail[i  ].z()),
            trail_color);
    }
}
