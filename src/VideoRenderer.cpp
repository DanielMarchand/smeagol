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

void VideoRenderer::markFitnessOrigin()
{
    // Point to the next frame that will be added — displacement will read 0
    // on that frame and grow from there.
    fitness_origin_idx_ = static_cast<int>(com_trail_.size());
}

void VideoRenderer::beginSettling()
{
    fitness_origin_idx_ = -1;  // activates the "settling..." banner
}

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

        // ── Displacement label (top-right, small, trail green) ───────────
        {
            static const Color kTrailGreen = { 60, 200, 80, 255 };
            static const int   kFont       = 14;
            static const int   kPad        = 8;   // px from right / top edge

            const char* label = nullptr;
            std::string disp_str;

            if (fitness_origin_idx_ < 0)
            {
                label = "settling...";
            }
            else if (static_cast<int>(com_trail_.size()) > fitness_origin_idx_)
            {
                const double disp_xy =
                    (com.head<2>() - com_trail_[fitness_origin_idx_].head<2>()).norm();
                disp_str = TextFormat("disp: %.4f m", disp_xy);
                label    = disp_str.c_str();
            }

            if (label)
            {
                const int text_w = MeasureText(label, kFont);
                DrawText(label, m_width - text_w - kPad, kPad, kFont, kTrailGreen);
            }
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
    std::cout << "\r  frame " << frame_count_ << std::flush;
}

// ── Video compilation ─────────────────────────────────────────────────────────

bool VideoRenderer::finish(const std::string& output_path)
{
    if (finished_) return true;
    finished_ = true;
    std::cout << "\n";  // end the \r frame-progress line
    std::cout.flush();

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
