/**
 * test_snapshot.cpp
 *
 * Tests for SnapshotRenderer and Robot::saveDebugImage().
 *
 * A display server (X11/Wayland) is required to open a Raylib window.
 * If DISPLAY is not set (or is empty), the render tests are skipped
 * so the CI suite stays green even on headless machines that do not
 * have Xvfb.
 *
 * With Xvfb the tests run fully:
 *   Xvfb :99 -screen 0 1280x720x24 &
 *   DISPLAY=:99 ctest --test-dir build -R SnapshotRenderer
 */

#include "SnapshotRenderer.h"
#include "Robot.h"
#include "Vertex.h"
#include "Bar.h"

#include <filesystem>
#include <iostream>
#include <string>
#include <cstdlib>

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
    // Just instantiating confirms the translation unit links correctly
    SnapshotRenderer snap(320, 240);
    CHECK(true);  // reached here → linked OK
}

// ── render tests (skipped without a display) ─────────────────────────────────

static void test_render_creates_file()
{
    const std::string path = "/tmp/golem_snapshot_test.png";
    fs::remove(path);  // clean up any leftover

    Robot robot = make_tetra();
    SnapshotRenderer snap(640, 480);
    snap.render(robot, path);

    CHECK(fs::exists(path));
    CHECK(fs::file_size(path) > 1000);  // a non-trivial PNG is >1 KB
    fs::remove(path);
}

static void test_save_debug_image()
{
    const std::string path = "/tmp/golem_debug_image_test.png";
    fs::remove(path);

    Robot robot = make_tetra();
    robot.saveDebugImage(path);

    CHECK(fs::exists(path));
    CHECK(fs::file_size(path) > 1000);
    fs::remove(path);
}

static void test_empty_robot_renders()
{
    // An empty robot should not crash – just draws the floor
    const std::string path = "/tmp/golem_empty_robot.png";
    Robot empty;
    SnapshotRenderer snap(320, 240);
    snap.render(empty, path);
    CHECK(fs::exists(path));
    fs::remove(path);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    auto run = [](const char* name, void (*fn)()) {
        std::cout << "[ RUN  ] " << name << "\n";
        fn();
    };

    run("Linkage smoke test",  test_linkage);

    if (!has_display()) {
        std::cout << "[ SKIP ] No display detected (DISPLAY / WAYLAND_DISPLAY not set).\n"
                  << "         Render tests skipped – run with Xvfb to exercise them.\n";
    } else {
        run("Render creates PNG file",    test_render_creates_file);
        run("Robot::saveDebugImage",      test_save_debug_image);
        run("Empty robot renders safely", test_empty_robot_renders);
    }

    if (g_failures == 0) {
        std::cout << "\nAll SnapshotRenderer tests passed.\n";
        return 0;
    } else {
        std::cout << "\n" << g_failures << " check(s) FAILED.\n";
        return 1;
    }
}
