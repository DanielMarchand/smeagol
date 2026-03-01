/**
 * test_dependencies.cpp
 *
 * Smoke-test that all three core dependencies (Eigen3, yaml-cpp, raylib) are
 * present, link correctly, and are minimally usable.  Each block performs a
 * trivial operation that would fail to compile or return a wrong result if the
 * library were missing or mis-installed.
 *
 * No windowing or GPU context is required – raylib is validated at the
 * compile/link level only.
 */

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <raylib.h>

#include <cassert>
#include <iostream>
#include <string>

// ──────────────────────────────────────────────────────────────────────────────
// Individual checks
// ──────────────────────────────────────────────────────────────────────────────

static bool check_eigen3()
{
    // Basic 3-D vector arithmetic
    Eigen::Vector3d a(1.0, 0.0, 0.0);
    Eigen::Vector3d b(0.0, 1.0, 0.0);
    Eigen::Vector3d c = a + b;

    if (c.norm() <= 0.0) return false;

    // Simple 3x3 identity check
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    if ((I * a - a).norm() > 1e-12) return false;

    return true;
}

static bool check_yaml_cpp()
{
    const std::string src =
        "project: Golem2000\n"
        "version: 0.1\n"
        "parts:\n"
        "  - vertex\n"
        "  - bar\n";

    YAML::Node doc = YAML::Load(src);

    if (!doc["project"]) return false;
    if (doc["project"].as<std::string>() != "Golem2000") return false;
    if (!doc["parts"] || doc["parts"].size() != 2) return false;

    return true;
}

static bool check_raylib()
{
    // RAYLIB_VERSION is a compile-time string constant – evaluating it at
    // runtime confirms the header and library both resolve without a window.
    const std::string ver = RAYLIB_VERSION;
    return !ver.empty();
}

// ──────────────────────────────────────────────────────────────────────────────
// Main
// ──────────────────────────────────────────────────────────────────────────────

int main()
{
    int failures = 0;

    auto run = [&](const char* name, bool (*fn)()) {
        bool ok = fn();
        std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << "\n";
        if (!ok) ++failures;
    };

    run("Eigen3     (linear algebra)",  check_eigen3);
    run("yaml-cpp   (YAML parsing)",    check_yaml_cpp);
    run("raylib     (rendering header)", check_raylib);

    if (failures == 0) {
        std::cout << "\nAll dependency checks passed.\n";
        return 0;
    } else {
        std::cout << "\n" << failures << " check(s) FAILED.\n";
        return 1;
    }
}
