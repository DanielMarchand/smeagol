/**
 * test_debug_actuator.cpp
 *
 * Tests for DebugActuator: sine-wave driven bar actuator with clamping.
 */

#include "DebugActuator.h"
#include "RobotPart.h"
#include "Robot.h"
#include "Bar.h"
#include "Vertex.h"

#include <cmath>
#include <iostream>

static int g_failures = 0;

#define CHECK(cond)                                                             \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__             \
                      << "  (" #cond ")\n";                                     \
            ++g_failures;                                                       \
        }                                                                       \
    } while (false)

#define CHECK_NEAR(a, b, eps)                                                   \
    do {                                                                        \
        if (std::abs((a) - (b)) > (eps)) {                                      \
            std::cerr << "  FAIL  " << __FILE__ << ":" << __LINE__             \
                      << "  |" << (a) << " - " << (b) << "| > " << (eps) << "\n"; \
            ++g_failures;                                                       \
        }                                                                       \
    } while (false)

// ── helpers ───────────────────────────────────────────────────────────────────

static constexpr double EPS = 1e-10;

// ── tests ────────────────────────────────────────────────────────────────────

static void test_zero_at_start()
{
    // sin(0 + 0) = 0 → Δl = 0
    DebugActuator a(0, 0.008, 1.0, /*phase=*/0.0);
    CHECK_NEAR(a.deltaLength(0.0), 0.0, EPS);
}

static void test_peak_at_quarter_period()
{
    // f=1 Hz, t=0.25 s → sin(π/2) = 1 → Δl = amplitude
    const double amp = 0.007;
    DebugActuator a(0, amp, 1.0, 0.0);
    CHECK_NEAR(a.deltaLength(0.25), amp, EPS);
}

static void test_trough_at_three_quarter_period()
{
    // f=2 Hz, t=0.375 s → sin(3π/2) = -1 → Δl = -amplitude
    const double amp = 0.005;
    DebugActuator a(1, amp, 2.0, 0.0);
    CHECK_NEAR(a.deltaLength(0.375), -amp, EPS);
}

static void test_phase_shift()
{
    // phase = π/2 → sin(0 + π/2) = 1 → starts at peak
    const double amp = 0.006;
    DebugActuator a(0, amp, 1.0, /*phase=*/M_PI / 2.0);
    CHECK_NEAR(a.deltaLength(0.0), amp, EPS);
}

static void test_clamp_positive()
{
    // amplitude > bar_range → output capped at +bar_range
    DebugActuator a(0, /*amplitude=*/0.05, 1.0, 0.0, /*bar_range=*/0.01);
    const double peak = a.deltaLength(0.25);   // sin(π/2)=1, raw=0.05
    CHECK_NEAR(peak, 0.01, EPS);
}

static void test_clamp_negative()
{
    DebugActuator a(0, 0.05, 1.0, 0.0, 0.01);
    const double trough = a.deltaLength(0.75);  // sin(3π/2)=-1, raw=-0.05
    CHECK_NEAR(trough, -0.01, EPS);
}

static void test_default_bar_range()
{
    // Default bar_range = 0.01 m
    DebugActuator a(0, 0.01, 1.0);
    CHECK_NEAR(a.bar_range, 0.01, EPS);
}

static void test_frequency_scaling()
{
    // At double frequency peak happens at half the time
    const double amp = 0.008;
    DebugActuator a(0, amp, 4.0, 0.0);   // 4 Hz → peak at t=0.0625 s
    CHECK_NEAR(a.deltaLength(1.0 / (4.0 * 4.0)), amp, EPS);
}

static void test_robot_has_debug_actuators_field()
{
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(0.1, 0.0, 0.0));
    r.addBar(Bar(0, 1, 0.1));

    r.debug_actuators.push_back(DebugActuator(0, 0.008, 0.5, 0.0));
    CHECK(r.debug_actuators.size() == 1);
    CHECK(r.debug_actuators[0].bar_idx == 0);
    CHECK_NEAR(r.debug_actuators[0].amplitude, 0.008, EPS);
    CHECK_NEAR(r.debug_actuators[0].frequency, 0.5,   EPS);
}

static void test_robotpart_interface()
{
    DebugActuator a(0, 0.005, 1.0);
    CHECK(a.type() == RobotPart::Type::DebugActuator);
    CHECK(a.typeName() == "debug_actuator");

    auto cloned = a.clone();
    CHECK(cloned->type() == RobotPart::Type::DebugActuator);
    auto* c = dynamic_cast<DebugActuator*>(cloned.get());
    CHECK(c != nullptr);
    CHECK_NEAR(c->amplitude, a.amplitude, EPS);
    CHECK_NEAR(c->frequency, a.frequency, EPS);
}

static void test_debug_actuators_not_serialised()
{
    // debug_actuators must survive a clone() but must NOT appear in YAML
    Robot r;
    r.addVertex(Vertex(0.0, 0.0, 0.0));
    r.addVertex(Vertex(0.1, 0.0, 0.0));
    r.addBar(Bar(0, 1, 0.1));
    r.debug_actuators.push_back(DebugActuator(0, 0.005, 1.0));

    const std::string tmp = "/tmp/golem_debug_act_serial_test.yaml";
    r.toYAML(tmp);
    Robot r2 = Robot::fromYAML(tmp);
    // debug_actuators should be empty after a round-trip through YAML
    CHECK(r2.debug_actuators.empty());
    std::remove(tmp.c_str());
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    auto run = [](const char* name, void (*fn)()) {
        std::cout << "[ RUN  ] " << name << "\n";
        fn();
    };

    run("Zero output at t=0 (no phase)",          test_zero_at_start);
    run("Peak at quarter period",                  test_peak_at_quarter_period);
    run("Trough at three-quarter period",          test_trough_at_three_quarter_period);
    run("Phase shift starts at peak",              test_phase_shift);
    run("Clamp to +bar_range",                     test_clamp_positive);
    run("Clamp to -bar_range",                     test_clamp_negative);
    run("Default bar_range is 0.01 m",             test_default_bar_range);
    run("Frequency scaling",                       test_frequency_scaling);
    run("Robot::debug_actuators field accessible", test_robot_has_debug_actuators_field);
    run("RobotPart interface (type/typeName/clone)",  test_robotpart_interface);
    run("debug_actuators not round-tripped in YAML", test_debug_actuators_not_serialised);

    if (g_failures == 0) {
        std::cout << "\nAll DebugActuator tests passed.\n";
        return 0;
    }
    std::cout << "\n" << g_failures << " check(s) FAILED.\n";
    return 1;
}
