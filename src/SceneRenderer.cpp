#include "SceneRenderer.h"
#include <rlgl.h>
#include <algorithm>
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

void SceneRenderer::resetCamera()
{
    m_camera.position   = { camera_distance, camera_distance * camera_elevation, camera_distance };
    m_camera.target     = { 0.0f, 0.0f, 0.0f };
    m_camera.up         = { 0.0f, 1.0f, 0.0f };
    m_camera.fovy       = camera_fov;
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
        drawFloor(floor_grid_slices, floor_grid_spacing);
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

void SceneRenderer::drawRobot(const Robot&               robot,
                              const std::vector<double>& rest_lengths,
                              Color                      vertex_color)
{
    // Build map: bar_idx → actuator index, for O(1) lookup.
    std::unordered_set<int> actuated;
    actuated.reserve(robot.actuators.size());
    for (const auto& a : robot.actuators)
        actuated.insert(a.bar_idx);

    const bool have_rest = !rest_lengths.empty();

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
            const Color bright = actuatorBrightColor(bi);

            if (have_rest && bi < static_cast<int>(rest_lengths.size())) {
                // ── Two-cylinder actuator rendering ───────────────────────
                // Fat housing: starts at rp1, extends base_rest_length along
                // the bar axis.  Represents the unextended housing; never
                // changes shape regardless of neural activity.
                //
                // Thin piston rod: spans full v1→v2 geometry.  Protrudes past
                // the housing end when the actuator is extended above base.

                // Compute unit direction in Raylib space.
                const float dx = rp2.x - rp1.x;
                const float dy = rp2.y - rp1.y;
                const float dz = rp2.z - rp1.z;
                const float geom_len = std::sqrt(dx*dx + dy*dy + dz*dz);

                const float base_L = static_cast<float>(bar.rest_length);

                if (geom_len > 1e-6f) {
                    const float inv = base_L / geom_len;
                    const Vector3 housing_end = {
                        rp1.x + dx * inv,
                        rp1.y + dy * inv,
                        rp1.z + dz * inv
                    };

                    // Housing: thick, 50% darkened actuator colour.
                    const Color housing_c = {
                        static_cast<unsigned char>(bright.r / 2),
                        static_cast<unsigned char>(bright.g / 2),
                        static_cast<unsigned char>(bright.b / 2),
                        255
                    };
                    DrawCylinderEx(rp1, housing_end,
                                   render_bar_radius * actuator_housing_radius_scale,
                                   render_bar_radius * actuator_housing_radius_scale,
                                   8, housing_c);

                    // Piston rod: thin, full geometry, bright colour.
                    DrawCylinderEx(rp1, rp2,
                                   render_bar_radius * actuator_rod_radius_scale,
                                   render_bar_radius * actuator_rod_radius_scale,
                                   8, bright);
                } else {
                    // Degenerate bar: fall back to single cylinder.
                    DrawCylinderEx(rp1, rp2,
                                   render_bar_radius, render_bar_radius,
                                   8, bright);
                }
            } else {
                // No rest-length data: single bright cylinder.
                DrawCylinderEx(rp1, rp2,
                               render_bar_radius, render_bar_radius,
                               8, bright);
            }
        } else {
            // ── Structural bar: single dark muted cylinder ────────────────
            DrawCylinderEx(rp1, rp2,
                           render_bar_radius,
                           render_bar_radius,
                           8, structuralBarColor(bi));
        }
    }

    // ── vertices ──────────────────────────────────────────────────────────
    // Pass 1: blob shadows – drawn before spheres so they appear under them.
    if (floor_shadow_height > 0.0f) {
        for (const auto& vertex : robot.vertices) {
            const float h = static_cast<float>(vertex.pos.z());
            if (h >= floor_shadow_height) continue;
            // t = 1 when vertex is on the floor, 0 at threshold height.
            const float t     = std::max(0.0f, 1.0f - h / floor_shadow_height);
            const unsigned char alpha = static_cast<unsigned char>(150.0f * t);
            const float sr    = render_vertex_radius * (1.2f + 0.6f * t);
            const Vector3 sp  = { static_cast<float>(vertex.pos.x()),
                                  0.001f,
                                  static_cast<float>(-vertex.pos.y()) };
            const Vector3 sp2 = { static_cast<float>(vertex.pos.x()),
                                  0.002f,
                                  static_cast<float>(-vertex.pos.y()) };
            DrawCylinderEx(sp, sp2, sr, sr, 12, Color{ 0, 0, 0, alpha });
        }
    }

    // Pass 2: vertex spheres (always normal colour).
    for (const auto& vertex : robot.vertices) {
        Vector3 rp = toRaylib(vertex.pos.x(), vertex.pos.y(), vertex.pos.z());
        DrawSphere(rp, render_vertex_radius, vertex_color);
    }

    // Pass 3: floor-clip warning disks.
    // A solid red/orange disk is drawn flat on the floor under any vertex whose
    // centre has sunk below floor_contact_threshold.  The disk grows and
    // brightens as penetration deepens.
    for (const auto& vertex : robot.vertices) {
        const float h = static_cast<float>(vertex.pos.z());
        if (h >= floor_contact_threshold) continue;
        // t = 0 just touching, 1 fully buried (centre at -render_vertex_radius).
        const float t = std::clamp(
            -h / render_vertex_radius, 0.0f, 1.0f);
        const float disk_r = render_vertex_radius * (0.6f + 1.2f * t);
        const unsigned char alpha = static_cast<unsigned char>(180 + 75 * t);
        const Color disk_c = { 255,
                               static_cast<unsigned char>(160 - 130 * t),
                               0, alpha };
        const Vector3 centre = { static_cast<float>(vertex.pos.x()),
                                 0.0f,
                                 static_cast<float>(-vertex.pos.y()) };
        const Vector3 centre2 = { centre.x, 0.001f, centre.z };
        DrawCylinderEx(centre, centre2, disk_r, disk_r, 12, disk_c);
    }
}

// ── Neural overlay ────────────────────────────────────────────────────────────

void SceneRenderer::drawNeuralOverlay(const Robot&               robot,
                                      const std::vector<double>& activations)
{
    const int N = static_cast<int>(robot.neurons.size());
    if (N == 0) return;

    // ── Fixed ring layout in the top-right corner (pure screen space) ─────
    // The ring never moves regardless of camera or robot position.
    const float kRadius      = 8.0f;
    const float kPad         = 14.0f;               // inset from frame edge
    const float ring_r       = std::max(30.0f, N * 7.0f);  // px, scales with N
    const float cx           = static_cast<float>(m_width)  - kPad - ring_r;
    const float cy           = kPad + ring_r;

    std::vector<Vector2> npos(N);
    for (int i = 0; i < N; ++i)
    {
        // Start at top, go clockwise.
        const float angle = (2.0f * 3.14159265f / static_cast<float>(N)) * i
                            - 3.14159265f / 2.0f;
        npos[i] = { cx + ring_r * std::cos(angle),
                    cy + ring_r * std::sin(angle) };
    }

    // Pre-compute per-neuron active flag for wire colouring.
    auto is_active = [&](int idx) -> bool {
        return idx >= 0 && idx < static_cast<int>(activations.size())
               && activations[idx] > 0.5;
    };

    // Project a 3D sim-space point to a 2D screen position.
    auto project = [&](double x, double y, double z) -> Vector2 {
        return GetWorldToScreen(toRaylib(x, y, z), m_camera);
    };

    // ── Background panel so the ring is readable over any scene ──────────
    DrawRectangle(static_cast<int>(cx - ring_r - kRadius - 4),
                  static_cast<int>(cy - ring_r - kRadius - 4),
                  static_cast<int>(2 * (ring_r + kRadius) + 8),
                  static_cast<int>(2 * (ring_r + kRadius) + 8),
                  Color{ 0, 0, 0, 100 });

    // ── Synapse connections between neurons (drawn first, under circles) ──
    for (int i = 0; i < N; ++i)
    {
        const Eigen::VectorXd& w = robot.neurons[i].synapse_weights;
        const int sw = static_cast<int>(w.size());
        for (int j = 0; j < N && j < sw; ++j)
        {
            if (std::abs(w(j)) < 0.01) continue;
            const Color c = is_active(i)
                ? Color{ 80, 140, 255, 200}   // blue  = source neuron firing
                : Color{100, 100, 100, 130};  // grey  = source neuron silent
            DrawLineEx(npos[i], npos[j], 1.5f, c);
        }
    }

    // ── Actuator connections: neuron circle → projected bar midpoint ──────
    for (const auto& a : robot.actuators)
    {
        if (a.neuron_idx < 0 || a.neuron_idx >= N) continue;
        if (a.bar_idx    < 0 || a.bar_idx    >= static_cast<int>(robot.bars.size())) continue;

        const Bar& bar = robot.bars[a.bar_idx];
        const Eigen::Vector3d mid =
            0.5 * (robot.vertices[bar.v1].pos + robot.vertices[bar.v2].pos);
        const Vector2 bar2d   = project(mid.x(), mid.y(), mid.z());
        const Color   wire_c  = is_active(a.neuron_idx)
            ? Color{ 80, 140, 255, 180}
            : Color{100, 100, 100, 120};
        // Dashed-style: draw from neuron edge outward, then a dot at the bar.
        DrawLineEx(npos[a.neuron_idx], bar2d, 1.0f, wire_c);
        DrawCircleV(bar2d, 3.0f, wire_c);
    }

    // ── Neuron circles (drawn last → on top of all lines) ─────────────────
    for (int i = 0; i < N; ++i)
    {
        const bool active = is_active(i);
        const Color fill = active
            ? Color{ 50, 120, 255, 255}
            : Color{ 60,  60,  60, 255};
        DrawCircleV(npos[i], kRadius, fill);
        DrawCircleLinesV(npos[i], kRadius,
                         active ? Color{150, 200, 255, 200}
                                : Color{160, 160, 160, 120});
        // Neuron index label.
        DrawText(TextFormat("%d", i),
                 static_cast<int>(npos[i].x - (i < 10 ? 3 : 5)),
                 static_cast<int>(npos[i].y - 4),
                 8, active ? WHITE : Color{180, 180, 180, 200});
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
