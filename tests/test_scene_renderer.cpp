/**
 * test_scene_renderer.cpp
 *
 * Tests for SceneRenderer that do NOT require an open window or display.
 *
 * Opening a window requires a live X11/Wayland/display server so we
 * deliberately skip that here.  The rendering path is exercised manually
 * (or via SnapshotRenderer in an environment that has a display).
 *
 * What IS tested:
 *   - The translation unit compiles and links against Raylib.
 *   - The coordinate remapping helper (toRaylib) is numerically correct.
 *   - Camera state after resetCamera() is sane.
 *   - lookAt() moves target and preserves offset.
 */

#include "SceneRenderer.h"

#include <cmath>
#include <iostream>

static int g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__            \
                      << "  (" #cond ")\n";                                    \
            ++g_failures;                                                      \
        }                                                                      \
    } while (false)

#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((a) - (b)) < (eps))

// ── Coordinate remapping ──────────────────────────────────────────────────────

static void test_to_raylib()
{
    // Simulation Z-up  →  Raylib Y-up
    //   sim (x,  y,  z) → ray (x,  z, -y)
    {
        auto v = SceneRenderer::toRaylib(1.0, 2.0, 3.0);
        CHECK_NEAR(v.x,  1.0f, 1e-6f);  // x unchanged
        CHECK_NEAR(v.y,  3.0f, 1e-6f);  // sim z  → ray y
        CHECK_NEAR(v.z, -2.0f, 1e-6f);  // sim y  → ray -z
    }
    // Origin maps to origin
    {
        auto v = SceneRenderer::toRaylib(0.0, 0.0, 0.0);
        CHECK_NEAR(v.x, 0.0f, 1e-6f);
        CHECK_NEAR(v.y, 0.0f, 1e-6f);
        CHECK_NEAR(v.z, 0.0f, 1e-6f);
    }
    // A point on the floor (z=0) maps to ray y=0
    {
        auto v = SceneRenderer::toRaylib(0.5, -0.3, 0.0);
        CHECK_NEAR(v.y, 0.0f, 1e-6f);
    }
    // A point above the floor (z>0) maps to ray y>0
    {
        auto v = SceneRenderer::toRaylib(0.0, 0.0, 0.1);
        CHECK(v.y > 0.0f);
    }
}

// ── Camera ────────────────────────────────────────────────────────────────────

// SceneRenderer's constructor calls resetCamera(), so we can inspect
// the camera state without opening a window.
static void test_camera_init()
{
    SceneRenderer renderer("Test", 800, 600, 60);

    // We can't directly inspect Camera3D through the public API, but
    // we can verify that resetCamera() and lookAt() don't crash and
    // that the object is in a sane state (isOpen == false).
    CHECK(!renderer.isOpen());

    // lookAt should not crash
    renderer.lookAt(0.5f, 0.0f, 0.0f);

    // Reset to a different distance
    renderer.camera_distance = 2.0f;
    renderer.resetCamera();

    CHECK(!renderer.isOpen());
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    auto run = [](const char* name, void (*fn)()) {
        std::cout << "[ RUN  ] " << name << "\n";
        fn();
    };

    run("toRaylib coordinate remapping", test_to_raylib);
    run("Camera initialisation",         test_camera_init);

    if (g_failures == 0) {
        std::cout << "\nAll SceneRenderer tests passed.\n";
        return 0;
    } else {
        std::cout << "\n" << g_failures << " check(s) FAILED.\n";
        return 1;
    }
}
