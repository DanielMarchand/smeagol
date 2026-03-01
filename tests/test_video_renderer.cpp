/**
 * test_video_renderer.cpp
 *
 * Tests for VideoRenderer.
 *
 * A display server (X11/Wayland) is required to open a Raylib window.
 * If DISPLAY is not set the render tests are skipped to keep CI green on
 * headless machines.
 *
 * ffmpeg must be on PATH for the video-compilation test to pass; the test
 * is skipped automatically if ffmpeg is absent.
 *
 * Run with a virtual display:
 *   Xvfb :99 -screen 0 1280x720x24 &
 *   DISPLAY=:99 ctest --test-dir build -R VideoRenderer
 */

#include "VideoRenderer.h"
#include "Robot.h"
#include "Vertex.h"
#include "Bar.h"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

static int g_failures = 0;

#define CHECK(cond)                                                             \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__             \
                      << "  (" #cond ")\n";                                     \
            ++g_failures;                                                       \
        }                                                                       \
    } while (false)

// ── helpers ───────────────────────────────────────────────────────────────────

static bool has_display()
{
    const char* d = std::getenv("DISPLAY");
    if (d && d[0] != '\0') return true;
    const char* w = std::getenv("WAYLAND_DISPLAY");
    return (w && w[0] != '\0');
}

static bool has_ffmpeg()
{
    return std::system("ffmpeg -version > /dev/null 2>&1") == 0;
}

static Robot make_tetra()
{
    Robot r(1);
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(0.2, 0.0, 0.0));
    r.addVertex(Vertex(0.1, 0.2, 0.0));
    r.addVertex(Vertex(0.1, 0.1, 0.2));
    r.addBar(Bar(0, 1, 0.20, 0.010));
    r.addBar(Bar(0, 2, 0.22, 0.010));
    r.addBar(Bar(1, 2, 0.22, 0.010));
    r.addBar(Bar(0, 3, 0.24, 0.008));
    r.addBar(Bar(1, 3, 0.24, 0.008));
    r.addBar(Bar(2, 3, 0.20, 0.008));
    return r;
}

// ── compile / link smoke test (always runs) ───────────────────────────────────

static void test_linkage()
{
    // frameCount() returning 0 proves constructor ran without throwing
    // Use a tiny resolution so even a slow machine handles it quickly
    VideoRenderer vid(30, 320, 240);
    CHECK(vid.frameCount() == 0);
}

// ── render tests (skipped without a display) ─────────────────────────────────

static void test_add_frame_increments_count()
{
    Robot robot = make_tetra();
    VideoRenderer vid(30, 320, 240);
    CHECK(vid.frameCount() == 0);

    vid.addFrame(robot, 0.0);
    CHECK(vid.frameCount() == 1);

    vid.addFrame(robot, 0.033);
    CHECK(vid.frameCount() == 2);

    // Destructor will call finish("") which produces a default .mp4 in
    // the temp dir; we don't check that file here, just that it doesn't crash.
}

static void test_finish_creates_mp4()
{
    if (!has_ffmpeg()) {
        std::cout << "[ SKIP ]   ffmpeg not found on PATH – skipping MP4 output test.\n";
        return;
    }

    const std::string out_path = "/tmp/golem_video_test.mp4";
    fs::remove(out_path);

    {
        Robot robot = make_tetra();
        VideoRenderer vid(10, 320, 240);  // 10 fps, tiny resolution

        // Render 5 frames at pseudo-time steps
        for (int i = 0; i < 5; ++i)
            vid.addFrame(robot, i * 0.1);

        CHECK(vid.frameCount() == 5);
        const bool ok = vid.finish(out_path);
        CHECK(ok);
    }

    CHECK(fs::exists(out_path));
    CHECK(fs::file_size(out_path) > 1000);  // a real MP4 is well above 1 KB
    fs::remove(out_path);
}

static void test_finish_idempotent()
{
    // Calling finish() twice must not crash or produce a second ffmpeg call
    const std::string out_path = "/tmp/golem_video_idem_test.mp4";

    Robot robot = make_tetra();
    VideoRenderer vid(10, 320, 240);
    vid.addFrame(robot, 0.0);

    // First call
    const bool ok1 = vid.finish(out_path);

    // Second call should quietly return true (already finished)
    const bool ok2 = vid.finish(out_path);

    if (has_ffmpeg())
    {
        CHECK(ok1);
        CHECK(ok2);  // idempotent
    }

    fs::remove(out_path);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    auto run = [](const char* name, void (*fn)()) {
        std::cout << "[ RUN  ] " << name << "\n";
        fn();
    };

    run("Linkage smoke test", test_linkage);

    if (!has_display()) {
        std::cout << "[ SKIP ] No display detected (DISPLAY / WAYLAND_DISPLAY not set).\n"
                  << "         Render tests skipped – run with Xvfb to exercise them.\n";
    } else {
        run("addFrame increments frameCount",  test_add_frame_increments_count);
        run("finish() creates MP4 file",       test_finish_creates_mp4);
        run("finish() is idempotent",          test_finish_idempotent);
    }

    if (g_failures == 0) {
        std::cout << "\nAll VideoRenderer tests passed.\n";
        return 0;
    } else {
        std::cout << "\n" << g_failures << " check(s) FAILED.\n";
        return 1;
    }
}
