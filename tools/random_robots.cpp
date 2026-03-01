/**
 * random_robots.cpp
 *
 * Generates N random robots, writes each to a YAML file, simulates each for
 * a short run, and records the motion as an MP4 video.
 *
 * Usage
 * -----
 *   build/random_robots [output_dir] [N] [seed]
 *
 *   output_dir  Directory to write robot_0.yaml + robot_0.mp4 etc.
 *               (default: examples/random_robots/)
 *   N           Number of robots to generate (default: 3)
 *   seed        RNG seed (default: 42)
 *
 * Output
 * ------
 * For each robot the tool prints a human-readable summary to stdout and then
 * records  <output_dir>/robot_<i>.mp4  via VideoRenderer + ffmpeg.
 */

#include "RobotFactory.h"
#include "Robot.h"
#include "Simulator.h"
#include "VideoRenderer.h"
#include "FitnessEvaluator.h"

#include <Eigen/Core>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

namespace fs = std::filesystem;

static void printRobot(const Robot& r, int idx, const std::string& yaml_path)
{
    const int nv = static_cast<int>(r.vertices.size());
    const int nb = static_cast<int>(r.bars.size());
    const int nn = static_cast<int>(r.neurons.size());
    const int na = static_cast<int>(r.actuators.size());

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n── Robot " << idx << "  (id=" << r.id << ") ────────────────────\n";
    std::cout << "  Vertices  : " << nv << "\n";
    std::cout << "  Bars      : " << nb
              << "  (spanning-tree=" << (nv - 1) << " + extra=" << (nb - (nv - 1)) << ")\n";
    std::cout << "  Neurons   : " << nn << "\n";
    std::cout << "  Actuators : " << na << "\n";

    if (nv > 0) {
        const auto com = r.centerOfMass();
        std::cout << "  CoM       : [" << com.x() << ", " << com.y() << ", " << com.z() << "] m\n";
    }

    std::cout << "\n  Vertices:\n";
    for (int i = 0; i < nv; ++i) {
        const auto& p = r.vertices[i].pos;
        std::cout << "    [" << i << "]  x=" << std::setw(8) << p.x()
                  << "  y=" << std::setw(8) << p.y()
                  << "  z=" << std::setw(8) << p.z() << "\n";
    }

    std::cout << "\n  Bars:\n";
    for (int i = 0; i < nb; ++i) {
        const auto& b = r.bars[i];
        std::cout << "    [" << i << "]  v1=" << b.v1 << "  v2=" << b.v2
                  << "  L0=" << std::setw(8) << b.rest_length
                  << "  k=" << std::setw(8) << b.stiffness << "\n";
    }

    std::cout << "\n  Neurons:\n";
    for (int i = 0; i < nn; ++i) {
        const auto& n = r.neurons[i];
        std::cout << "    [" << i << "]  thresh=" << std::setw(6) << n.threshold
                  << "  init=" << static_cast<int>(n.activation)
                  << "  weights=[";
        for (int j = 0; j < n.synapse_weights.size(); ++j) {
            if (j) std::cout << " ";
            std::cout << std::setw(6) << n.synapse_weights(j);
        }
        std::cout << "]\n";
    }

    std::cout << "\n  Actuators:\n";
    for (int i = 0; i < na; ++i) {
        const auto& a = r.actuators[i];
        std::cout << "    [" << i << "]  bar=" << a.bar_idx
                  << "  neuron=" << a.neuron_idx
                  << "  range=" << std::showpos << a.bar_range << std::noshowpos << "\n";
    }

    std::cout << "\n  Saved → " << yaml_path << "\n";
}

/// Simulate robot for one FitnessParams run and write an MP4.
static void renderVideo(Robot r, const FitnessParams& fp,
                        const std::string& mp4_path,
                        int fps = 30)
{
    std::cout << "  Recording video → " << mp4_path << " ...\n";

    Simulator sim(r);
    sim.wind = fp.wind;

    VideoRenderer vid(fps, 1280, 720);
    vid.addFrame(r, 0.0, sim.activations_);

    for (int c = 0; c < fp.cycles; ++c) {
        sim.tickNeural();
        sim.applyActuators(fp.steps_per_cycle);
        sim.relax(fp.steps_per_cycle, fp.step_size, 0.0, 0.0);
        sim.copyPositionsBack(r);
        vid.addFrame(r, static_cast<double>(c + 1) / fps, sim.activations_);
    }

    if (vid.finish(mp4_path))
        std::cout << "  Written:  " << mp4_path << "\n";
    else
        std::cerr << "  WARNING: ffmpeg failed for " << mp4_path << "\n";
}

int main(int argc, char* argv[])
{
    const std::string out_dir = (argc > 1) ? argv[1] : "examples/random_robots";
    const int    N    = (argc > 2) ? std::stoi(argv[2]) : 3;
    const uint32_t seed = (argc > 3) ? static_cast<uint32_t>(std::stoul(argv[3])) : 42;

    // Create output directory
    fs::create_directories(out_dir);

    std::mt19937 rng(seed);
    const FitnessParams fp;   // default params: 12 cycles, 5000 steps/cycle

    std::cout << "Generating " << N << " random robots  (seed=" << seed << ")\n";
    std::cout << "Output dir: " << out_dir << "\n";
    std::cout << "Simulation: " << fp.cycles << " cycles  ("
              << fp.steps_per_cycle << " steps/cycle)\n";

    for (int i = 0; i < N; ++i) {
        Robot r = RobotFactory::randomRobot(rng);

        const std::string yaml_path = out_dir + "/robot_" + std::to_string(i) + ".yaml";
        const std::string mp4_path  = out_dir + "/robot_" + std::to_string(i) + ".mp4";

        r.toYAML(yaml_path);

        if (!r.isValid())
            std::cerr << "  WARNING: robot " << i << " failed isValid()!\n";

        printRobot(r, i, yaml_path);
        renderVideo(r, fp, mp4_path);
    }

    std::cout << "\nDone.\n";
    return 0;
}

