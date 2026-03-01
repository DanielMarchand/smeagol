/**
 * validate_robot.cpp
 *
 * Command-line tool that loads one or more Robot YAML files and reports
 * whether each passes Robot::isValid().
 *
 * Usage:
 *   validate_robot <file1.yaml> [file2.yaml ...]
 *
 * Exit code:
 *   0  – every file is valid
 *   1  – at least one file is invalid or could not be loaded
 */

#include "Robot.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

static const char* RESET  = "\033[0m";
static const char* GREEN  = "\033[32m";
static const char* RED    = "\033[31m";
static const char* YELLOW = "\033[33m";

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: validate_robot <file.yaml> [file2.yaml ...]\n";
        return 1;
    }

    const std::vector<std::string> paths(argv + 1, argv + argc);
    int failures = 0;

    for (const auto& path : paths) {
        std::cout << "Checking: " << path << "\n";

        // ── try to parse ──────────────────────────────────────────────────
        Robot robot;
        try {
            robot = Robot::fromYAML(path);
        } catch (const std::exception& e) {
            std::cout << "  " << RED << "[LOAD ERROR]" << RESET
                      << " " << e.what() << "\n\n";
            ++failures;
            continue;
        }

        // ── structural summary ────────────────────────────────────────────
        std::cout << "  vertices : " << robot.vertices.size()  << "\n"
                  << "  bars     : " << robot.bars.size()      << "\n"
                  << "  neurons  : " << robot.neurons.size()   << "\n"
                  << "  actuators: " << robot.actuators.size() << "\n";

        // ── per-rule checks with diagnostic messages ──────────────────────
        const int nv = static_cast<int>(robot.vertices.size());
        const int nb = static_cast<int>(robot.bars.size());
        const int nn = static_cast<int>(robot.neurons.size());

        std::vector<std::string> errors;

        for (int i = 0; i < nb; ++i) {
            const auto& b = robot.bars[i];
            if (b.v1 < 0 || b.v1 >= nv)
                errors.push_back("bars[" + std::to_string(i) + "].v1=" +
                                 std::to_string(b.v1) + " out of range [0," +
                                 std::to_string(nv-1) + "]");
            if (b.v2 < 0 || b.v2 >= nv)
                errors.push_back("bars[" + std::to_string(i) + "].v2=" +
                                 std::to_string(b.v2) + " out of range [0," +
                                 std::to_string(nv-1) + "]");
            if (b.rest_length <= 0.0)
                errors.push_back("bars[" + std::to_string(i) +
                                 "].rest_length <= 0");
        }

        for (int i = 0; i < static_cast<int>(robot.actuators.size()); ++i) {
            const auto& a = robot.actuators[i];
            if (a.bar_idx < 0 || a.bar_idx >= nb)
                errors.push_back("actuators[" + std::to_string(i) +
                                 "].bar_idx=" + std::to_string(a.bar_idx) +
                                 " out of range [0," + std::to_string(nb-1) + "]");
            if (a.neuron_idx < 0 || a.neuron_idx >= nn)
                errors.push_back("actuators[" + std::to_string(i) +
                                 "].neuron_idx=" + std::to_string(a.neuron_idx) +
                                 " out of range [0," + std::to_string(nn-1) + "]");
        }

        for (int i = 0; i < nn; ++i) {
            if (robot.neurons[i].synapse_weights.size() != nn)
                errors.push_back("neurons[" + std::to_string(i) +
                                 "].weights size=" +
                                 std::to_string(robot.neurons[i].synapse_weights.size()) +
                                 " expected " + std::to_string(nn));
        }

        // ── result ────────────────────────────────────────────────────────
        if (errors.empty()) {
            std::cout << "  " << GREEN << "[VALID]" << RESET << "\n\n";
        } else {
            std::cout << "  " << RED << "[INVALID]" << RESET
                      << " – " << errors.size() << " error(s):\n";
            for (const auto& err : errors)
                std::cout << "    " << YELLOW << "• " << RESET << err << "\n";
            std::cout << "\n";
            ++failures;
        }
    }

    // ── summary ───────────────────────────────────────────────────────────
    const int total = static_cast<int>(paths.size());
    const int passed = total - failures;

    if (failures == 0)
        std::cout << GREEN << "All " << total << " file(s) valid." << RESET << "\n";
    else
        std::cout << RED << failures << "/" << total << " file(s) invalid."
                  << RESET << "\n";

    return (failures == 0) ? 0 : 1;
}
