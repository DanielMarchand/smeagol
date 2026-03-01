#pragma once

#include "SceneRenderer.h"
#include "Robot.h"

#include <filesystem>
#include <string>

/**
 * @brief Multi-frame video renderer; stitches frames into an MP4 via ffmpeg.
 *
 * Keeps a single off-screen Raylib window open for its entire lifetime.
 * Each call to addFrame() renders the current robot state and saves a
 * numbered PNG to a temporary directory.  Calling finish() (or allowing
 * the destructor to run) invokes ffmpeg to compile the frames into an MP4.
 *
 * Typical usage:
 * @code
 *   VideoRenderer vid(30, 1280, 720);   // 30 fps
 *
 *   for (int cycle = 0; cycle < 12; ++cycle) {
 *       sim.relax(...);
 *       // render several frames per physics cycle at the desired fps
 *       for (int sub = 0; sub < steps_per_cycle; ++sub)
 *           vid.addFrame(robot, cycle + sub * dt);
 *   }
 *
 *   vid.finish("runs/run_001/videos/robot_42.mp4");
 * @endcode
 *
 * ffmpeg dependency
 * -----------------
 * ffmpeg must be on PATH at the time finish() is called.  If it is absent
 * finish() prints a warning and returns false but does not throw.  The raw
 * PNG frames are left in the temp directory in that case.
 *
 * Temp directory
 * --------------
 * Frames are written to a sub-directory created inside @p temp_dir_base
 * (default: /tmp).  The directory is removed automatically by finish() on
 * success.  On failure it is left intact for debugging.
 */
class VideoRenderer : public SceneRenderer
{
public:
    /**
     * @param fps            Frames per second passed to ffmpeg.
     * @param width          Pixel width of the off-screen buffer.
     * @param height         Pixel height of the off-screen buffer.
     * @param temp_dir_base  Parent directory for the frame dump folder.
     */
    explicit VideoRenderer(int         fps            = 30,
                           int         width          = 1280,
                           int         height         = 720,
                           std::string temp_dir_base  = "/tmp");

    /**
     * Destructor: calls finish("") with a default output path if finish()
     * has not been called yet, then cleans up.  Errors are silently swallowed
     * (destructors must not throw).
     */
    ~VideoRenderer();

    // Non-copyable: owns a live Raylib window and a temp directory.
    VideoRenderer(const VideoRenderer&)            = delete;
    VideoRenderer& operator=(const VideoRenderer&) = delete;

    /**
     * @brief Render the current robot state and store the frame.
     *
     * @param robot     Robot to draw.
     * @param sim_time  Simulation time [s]; displayed as an HUD overlay.
     */
    void addFrame(const Robot& robot, double sim_time = 0.0);

    /**
     * @brief Compile accumulated frames into an MP4 and clean up.
     *
     * Invokes:
     *   ffmpeg -y -framerate <fps> -i <temp_dir>/frame_%04d.png
     *          -c:v libx264 -pix_fmt yuv420p <output_path>
     *
     * @param output_path  Destination .mp4 file.  Parent directories are
     *                     created if needed.  If empty a default path inside
     *                     the temp directory is used.
     * @return true if ffmpeg exited with code 0, false otherwise.
     */
    bool finish(const std::string& output_path = "");

    /// Number of frames accumulated so far.
    [[nodiscard]] int frameCount() const { return frame_count_; }

private:
    int                    fps_;
    std::filesystem::path  frame_dir_;   ///< temp dir holding numbered PNGs
    int                    frame_count_ = 0;
    bool                   finished_    = false;
};
