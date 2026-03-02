/**
 * evolve.cpp
 *
 * CLI entry point for the evolutionary loop (Phase 4.3).
 *
 * Usage:
 *   evolve [config.yaml]
 *
 * If no config file is given, runs with all-default EvolverParams.
 * The run directory (runs/run_<timestamp>/) is created automatically and
 * contains run_config.yaml (resolved params), fitness_log.csv, and
 * periodic best-robot snapshots.
 *
 * Example config.yaml:
 *   population_size: 200
 *   max_evaluations: 100000
 *   seed: 42
 *   video_interval: 1000
 *   fitness:
 *     cycles: 12
 *     steps_per_cycle: 5000
 *     step_size: 1.0e-7
 */

#include "Evolver.h"

#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char* argv[])
{
    EvolverParams params;

    if (argc >= 2) {
        try {
            params = EvolverParams::fromYAML(argv[1]);
        } catch (const std::exception& e) {
            std::cerr << "Failed to load config '" << argv[1] << "': " << e.what() << "\n";
            return 1;
        }
    } else {
        std::cout << "[evolve] No config file given — using default EvolverParams.\n";
    }

    try {
        Evolver ev(params);
        ev.run();
    } catch (const std::logic_error& e) {
        // run() is a Phase 4.3 stub — expected until that phase is implemented
        std::cerr << "[evolve] " << e.what() << "\n";
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "[evolve] Fatal error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
