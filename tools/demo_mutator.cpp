/**
 * demo_mutator.cpp
 *
 * CLI tool that applies one mutation operator to a seed robot N times
 * (each with a deterministic per-index seed) and saves the results as YAML.
 * Used by examples/mutator_demo/run.sh to systematically showcase each
 * operator without relying on random chance.
 *
 * Usage:
 *   demo_mutator <input.yaml> <op> <count> <seed> <outdir>
 *
 *   op:    perturb | add_remove | split | attach | all
 *   count: number of output robots (each a fresh clone of the seed)
 *   seed:  base RNG seed; robot i uses seed+i for full reproducibility
 *   outdir: directory to write <op>_0.yaml … <op>_N-1.yaml
 */

#include "Robot.h"
#include "Mutator.h"

#include <Eigen/Core>
#include <filesystem>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

static void apply_op(const std::string& op, Robot& r, std::mt19937& rng)
{
    if      (op == "perturb")         Mutator::perturbElement(r, rng);
    else if (op == "add_bar_new")     Mutator::addBarNew(r, rng);
    else if (op == "add_bar_bridge")  Mutator::addBarBridge(r, rng);
    else if (op == "add_neuron")      Mutator::addNeuron(r, rng);
    else if (op == "remove_bar")      Mutator::removeBar(r, rng);
    else if (op == "remove_neuron")   Mutator::removeNeuron(r, rng);
    else if (op == "add_remove")      Mutator::mutate(r, rng);  // legacy alias
    else if (op == "split")           Mutator::splitElement(r, rng);
    else if (op == "attach")          Mutator::attachNeuron(r, rng);
    else if (op == "detach")          Mutator::detachNeuron(r, rng);
    else if (op == "rewire")          Mutator::rewireNeuron(r, rng);
    else if (op == "all")             Mutator::mutate(r, rng);
    else {
        std::cerr << "Unknown op '" << op
                  << "'. Valid: perturb | add_bar_new | add_bar_bridge | add_neuron | remove_bar | remove_neuron | split | attach | rewire | all\n";
        std::exit(1);
    }
}

// ── human-readable diff between two robots ───────────────────────────────────

static std::string describeDiff(const Robot& a, const Robot& b)
{
    std::ostringstream out;

    // Vertices
    const int dv = static_cast<int>(b.vertices.size()) - static_cast<int>(a.vertices.size());
    if (dv > 0)      out << "  + " << dv << " vertex(ices) added\n";
    else if (dv < 0) out << "  - " << -dv << " vertex(ices) removed\n";

    // Bars: count delta
    const int db = static_cast<int>(b.bars.size()) - static_cast<int>(a.bars.size());
    if (db > 0)      out << "  + " << db << " bar(s) added\n";
    else if (db < 0) out << "  - " << -db << " bar(s) removed\n";

    // Bars: rest_length changes on surviving bars
    const int common_bars = static_cast<int>(std::min(a.bars.size(), b.bars.size()));
    for (int i = 0; i < common_bars; ++i) {
        const double lo = a.bars[i].rest_length;
        const double ln = b.bars[i].rest_length;
        if (std::abs(ln - lo) > 1e-9) {
            out << "  ~ bar[" << i << "] rest_length  "
                << lo << " → " << ln
                << "  (" << (ln > lo ? "+" : "") << (ln - lo) << " m)\n";
        }
    }

    // Neurons: count delta
    const int dn = static_cast<int>(b.neurons.size()) - static_cast<int>(a.neurons.size());
    if (dn > 0)      out << "  + " << dn << " neuron(s) added\n";
    else if (dn < 0) out << "  - " << -dn << " neuron(s) removed\n";

    // Neurons: threshold and weight changes on surviving neurons
    const int common_n = static_cast<int>(std::min(a.neurons.size(), b.neurons.size()));
    for (int i = 0; i < common_n; ++i) {
        const double to = a.neurons[i].threshold;
        const double tn = b.neurons[i].threshold;
        if (std::abs(tn - to) > 1e-9)
            out << "  ~ neuron[" << i << "] threshold  " << to << " → " << tn << "\n";

        const int common_w = static_cast<int>(std::min(
            a.neurons[i].synapse_weights.size(),
            b.neurons[i].synapse_weights.size()));
        for (int s = 0; s < common_w; ++s) {
            const double wo = a.neurons[i].synapse_weights[s];
            const double wn = b.neurons[i].synapse_weights[s];
            if (std::abs(wn - wo) > 1e-9)
                out << "  ~ neuron[" << i << "] weight[" << s << "]  "
                    << wo << " → " << wn << "\n";
        }
    }

    // Actuators: count delta
    const int da = static_cast<int>(b.actuators.size()) - static_cast<int>(a.actuators.size());
    if (da > 0)      out << "  + " << da << " actuator(s) attached\n";
    else if (da < 0) out << "  - " << -da << " actuator(s) detached\n";

    // Actuators: detail new ones
    if (da > 0) {
        for (int i = static_cast<int>(a.actuators.size());
             i < static_cast<int>(b.actuators.size()); ++i) {
            out << "      actuator[" << i << "]: bar " << b.actuators[i].bar_idx
                << " ← neuron " << b.actuators[i].neuron_idx
                << "  range=" << b.actuators[i].bar_range << " m\n";
        }
    }

    // Actuators: rewiring of surviving actuators (bar_idx or neuron_idx changed)
    const int common_a = static_cast<int>(std::min(a.actuators.size(), b.actuators.size()));
    for (int i = 0; i < common_a; ++i) {
        const auto& ao = a.actuators[i];
        const auto& an = b.actuators[i];
        if (ao.bar_idx != an.bar_idx)
            out << "  ~ actuator[" << i << "] bar_idx  "
                << ao.bar_idx << " → " << an.bar_idx << "\n";
        if (ao.neuron_idx != an.neuron_idx)
            out << "  ~ actuator[" << i << "] neuron_idx  "
                << ao.neuron_idx << " → " << an.neuron_idx << "\n";
    }

    const std::string s = out.str();
    return s.empty() ? "  (no detectable change)\n" : s;
}

// ── short label describing what actually changed ──────────────────────────────

static std::string labelChange(const Robot& a, const Robot& b, const std::string& op)
{
    const int dv = static_cast<int>(b.vertices.size()) - static_cast<int>(a.vertices.size());
    const int db = static_cast<int>(b.bars.size())     - static_cast<int>(a.bars.size());
    const int dn = static_cast<int>(b.neurons.size())  - static_cast<int>(a.neurons.size());
    const int da = static_cast<int>(b.actuators.size())- static_cast<int>(a.actuators.size());

    // Split op: both paths give +1v +1b — distinguish by the new bar's length.
    // vertexSplit appends a tiny bar (offset in ±0.01 per axis → max ~0.017 m).
    // barSplit appends two half-bars (length = original/2, always ≥ 0.005 m).
    if (op == "split" && dv > 0 && db > 0) {
        const double new_bar_len = b.bars.back().rest_length;
        return (new_bar_len < 0.02) ? "vertex_split" : "bar_split";
    }

    // Structural graph count changes
    if (db > 0 && dn == 0 && da == 0) return "addbar";
    if (db < 0 && dn == 0)            return "removebar";
    if (dn > 0 && db == 0)            return "addneuron";
    if (dn < 0 && db == 0)            return "removeneuron";
    if (da > 0)                       return "added";    // attach op added coupling
    if (da < 0)                       return "removed";  // attach op removed coupling

    // Actuator rewiring (no count change)
    const int common_a = static_cast<int>(
        std::min(a.actuators.size(), b.actuators.size()));
    for (int i = 0; i < common_a; ++i) {
        if (a.actuators[i].bar_idx    != b.actuators[i].bar_idx)    return "bar";
        if (a.actuators[i].neuron_idx != b.actuators[i].neuron_idx) return "neuron";
    }

    // Perturb: may touch bar length, neuron threshold/weight, or both
    bool bar_changed = false, neuron_changed = false;

    const int common_b = static_cast<int>(std::min(a.bars.size(), b.bars.size()));
    for (int i = 0; i < common_b; ++i)
        if (std::abs(a.bars[i].rest_length - b.bars[i].rest_length) > 1e-9)
            { bar_changed = true; break; }

    const int common_n = static_cast<int>(std::min(a.neurons.size(), b.neurons.size()));
    for (int i = 0; i < common_n && !neuron_changed; ++i) {
        if (std::abs(a.neurons[i].threshold - b.neurons[i].threshold) > 1e-9)
            { neuron_changed = true; break; }
        const int common_w = static_cast<int>(std::min(
            a.neurons[i].synapse_weights.size(),
            b.neurons[i].synapse_weights.size()));
        for (int s = 0; s < common_w; ++s)
            if (std::abs(a.neurons[i].synapse_weights[s] -
                         b.neurons[i].synapse_weights[s]) > 1e-9)
                { neuron_changed = true; break; }
    }

    if (bar_changed && neuron_changed) return "barlen_neuron";
    if (bar_changed)                   return "barlen";
    if (neuron_changed)                return "neuron";

    return "nochange";
}

int main(int argc, char* argv[])
{
    if (argc != 6) {
        std::cerr << "Usage: demo_mutator <input.yaml> <op> <count> <seed> <outdir>\n"
                  << "  op:     perturb | add_remove | split | attach | rewire | all\n"
                  << "  count:  number of output robots\n"
                  << "  seed:   base RNG seed (robot i uses seed+i)\n"
                  << "  outdir: output directory\n";
        return 1;
    }

    const std::string input_path = argv[1];
    const std::string op         = argv[2];
    const int         count      = std::stoi(argv[3]);
    const unsigned    base_seed  = static_cast<unsigned>(std::stoul(argv[4]));
    const std::string outdir     = argv[5];

    fs::create_directories(outdir);

    // Validate seed robot
    Robot seed;
    try {
        seed = Robot::fromYAML(input_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load '" << input_path << "': " << e.what() << "\n";
        return 1;
    }

    std::cout << "Seed robot  —  "
              << seed.vertices.size()  << " vertices, "
              << seed.bars.size()      << " bars, "
              << seed.neurons.size()   << " neurons, "
              << seed.actuators.size() << " actuators\n"
              << "Operator: " << op << "  count: " << count
              << "  base seed: " << base_seed << "\n\n";

    std::map<std::string, int> label_count;
    for (int i = 0; i < count; ++i) {
        std::mt19937 rng(base_seed + static_cast<unsigned>(i));
        Robot r = Robot::fromYAML(input_path);
        apply_op(op, r, rng);

        const std::string label = labelChange(seed, r, op);
        const int idx = label_count[label]++;
        const std::string out_path =
            outdir + "/" + op + "_" + label + "_" + std::to_string(idx) + ".yaml";
        r.toYAML(out_path);

        std::cout << "  [" << i << "] " << out_path << "  —  "
                  << r.vertices.size()  << "v  "
                  << r.bars.size()      << "b  "
                  << r.neurons.size()   << "n  "
                  << r.actuators.size() << "a\n"
                  << describeDiff(seed, r);
    }

    return 0;
}
