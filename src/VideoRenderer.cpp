#include "VideoRenderer.h"

#include <raylib.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// ── Construction / destruction ────────────────────────────────────────────────

VideoRenderer::VideoRenderer(int fps, int width, int height,
                             std::string temp_dir_base)
    : SceneRenderer("Golem2000-Video", width, height, 0),
      fps_(fps)
{
    // Create a uniquely named temp directory to avoid collisions between
    // concurrent runs.
    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    frame_dir_ = fs::path(temp_dir_base)
                 / ("smeagol_video_" + std::to_string(now_ms));
    fs::create_directories(frame_dir_);

    openWindow(/*hidden=*/true);
    if (!isOpen())
        throw std::runtime_error(
            "VideoRenderer: failed to open off-screen window");
}

VideoRenderer::~VideoRenderer()
{
    if (!finished_)
    {
        try { finish(""); }
        catch (...) {}
    }
    if (isOpen()) closeWindow();
}

// ── Frame capture ─────────────────────────────────────────────────────────────

void VideoRenderer::addFrame(const Robot& robot, double sim_time)
{
    addFrame(robot, sim_time, {});
}

void VideoRenderer::addFrame(const Robot&               robot,
                             double                     sim_time,
                             const std::vector<double>& activations)
{
    // Record CoM for trail + displacement HUD (Z-up simulation coords)
    const Eigen::Vector3d com = robot.centerOfMass();
    com_trail_.push_back(com);

    // Centre camera on robot CoM (convert sim Z-up → Raylib Y-up)
    const Vector3 rl = toRaylib(com.x(), com.y(), com.z());
    lookAt(rl.x, rl.y, rl.z);
    resetCamera(0.8f);

    BeginDrawing();
        ClearBackground({ 30, 30, 30, 255 });
        BeginMode3D(m_camera);
            drawFloor(30, 0.1f);
            drawRobot(robot);
            drawComTrail(com_trail_);
            if (!activations.empty())
                drawNeuralOverlay(robot, activations);
        EndMode3D();

        // ── HUD ──────────────────────────────────────────────────────────
        const int active_n = [&]{
            int n = 0;
            for (double a : activations) if (a > 0.5) ++n;
            return n;
        }();
        const std::string hud = activations.empty()
            ? TextFormat("t=%.3fs  frame=%d  v=%d  b=%d",
                         sim_time, frame_count_,
                         (int)robot.vertices.size(),
                         (int)robot.bars.size())
            : TextFormat("t=%.3fs  frame=%d  v=%d  b=%d  neurons=%d/%d active",
                         sim_time, frame_count_,
                         (int)robot.vertices.size(),
                         (int)robot.bars.size(),
                         active_n, (int)activations.size());
        DrawText(hud.c_str(), 10, 10, 16, RAYWHITE);

        // ── Displacement banner (XY, from first frame) ───────────────────
        if (com_trail_.size() >= 2)
        {
            const double disp_xy = (com.head<2>() - com_trail_.front().head<2>()).norm();
            const std::string disp_str =
                TextFormat("displacement: %.4f m", disp_xy);

            // Draw a semi-transparent background strip, then the text centred on it
            const int font_size = 22;
            const int text_w    = MeasureText(disp_str.c_str(), font_size);
            const int strip_x   = m_width / 2 - text_w / 2 - 10;
            const int strip_y   = 36;
            DrawRectangle(strip_x, strip_y - 4, text_w + 20, font_size + 8,
                          Color{0, 0, 0, 160});
            DrawText(disp_str.c_str(),
                     m_width / 2 - text_w / 2, strip_y,
                     font_size, Color{255, 220, 50, 255});  // gold
        }
    EndDrawing();

    // Save frame as numbered PNG
    std::ostringstream fname;
    fname << (frame_dir_ / "frame_").string()
          << std::setw(4) << std::setfill('0') << frame_count_
          << ".png";

    Image img = LoadImageFromScreen();
    ExportImage(img, fname.str().c_str());
    UnloadImage(img);

    ++frame_count_;
}

// ── Video compilation ─────────────────────────────────────────────────────────

bool VideoRenderer::finish(const std::string& output_path)
{
    if (finished_) return true;
    finished_ = true;

    if (frame_count_ == 0)
    {
        std::cerr << "VideoRenderer::finish: no frames to compile.\n";
        return false;
    }

    // Resolve output path
    fs::path out = output_path.empty()
                   ? (frame_dir_ / "output.mp4")
                   : fs::path(output_path);
    fs::create_directories(out.parent_path().empty() ? "." : out.parent_path());

    // Build ffmpeg command
    // -y            overwrite without asking
    // -framerate    input frame rate
    // -i            numbered PNG pattern
    // -c:v libx264  H.264 codec (widely compatible)
    // -pix_fmt      required by libx264 for standard players
    std::ostringstream cmd;
    cmd << "ffmpeg -y"
        << " -framerate " << fps_
        << " -i \"" << (frame_dir_ / "frame_%04d.png").string() << "\""
        << " -c:v libx264 -pix_fmt yuv420p"
        << " \"" << out.string() << "\"";
    if (!verbose_)
        cmd << " -loglevel error";  // suppress ffmpeg progress/info spam
    cmd << " 2>&1";

    if (verbose_)
        std::cout << "VideoRenderer: compiling " << frame_count_
                  << " frames → " << out << "\n";

    const int rc = std::system(cmd.str().c_str());
    if (rc != 0)
    {
        std::cerr << "VideoRenderer: ffmpeg exited with code " << rc
                  << ".  Raw frames left in: " << frame_dir_ << "\n";
        return false;
    }

    // Clean up temp frames on success
    fs::remove_all(frame_dir_);
    if (verbose_)
        std::cout << "VideoRenderer: written " << out << "\n";
    return true;
}
