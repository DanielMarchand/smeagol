#include "Robot.h"
#include "SnapshotRenderer.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

// ── Static member ─────────────────────────────────────────────────────────────

std::atomic<Robot::ID> Robot::s_next_id{0};

// ── Construction ──────────────────────────────────────────────────────────────

Robot::Robot()
    : id(s_next_id++)
{}

Robot::Robot(ID id)
    : id(id)
{}

Robot Robot::clone() const
{
    Robot copy;          // gets a fresh auto-ID
    copy.id       = id;  // preserve original ID for lineage tracking
    copy.vertices  = vertices;
    copy.bars      = bars;
    copy.neurons   = neurons;
    copy.actuators = actuators;
    return copy;
}

// ── Part mutators ─────────────────────────────────────────────────────────────

int Robot::addVertex(const Vertex& v)
{
    vertices.push_back(v);
    return static_cast<int>(vertices.size()) - 1;
}

int Robot::addBar(const Bar& b)
{
    bars.push_back(b);
    return static_cast<int>(bars.size()) - 1;
}

int Robot::addNeuron(const Neuron& n)
{
    const int new_idx = static_cast<int>(neurons.size());

    // Extend every existing neuron's weight vector by one zero entry
    for (auto& existing : neurons) {
        existing.synapse_weights.conservativeResize(new_idx + 1);
        existing.synapse_weights[new_idx] = 0.0;   // zero the new column
    }

    // The incoming neuron must have exactly new_idx+1 weights
    Neuron copy = n;
    copy.synapse_weights.conservativeResize(new_idx + 1);
    copy.synapse_weights[new_idx] = 0.0;   // zero self-weight
    neurons.push_back(std::move(copy));
    return new_idx;
}

int Robot::addActuator(const Actuator& a)
{
    actuators.push_back(a);
    return static_cast<int>(actuators.size()) - 1;
}

void Robot::removeVertex(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(vertices.size()))
        throw std::out_of_range("Robot::removeVertex: index out of range");

    vertices.erase(vertices.begin() + idx);

    // Remove bars that referenced this vertex; patch remaining indices
    bars.erase(std::remove_if(bars.begin(), bars.end(),
        [idx](const Bar& b) {
            return b.v1 == idx || b.v2 == idx;
        }), bars.end());

    for (auto& b : bars) {
        if (b.v1 > idx) --b.v1;
        if (b.v2 > idx) --b.v2;
    }

    // Re-patch actuator bar_idx in case a bar was removed above
    // (handled implicitly: removeBar keeps actuators consistent)
}

void Robot::removeBar(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(bars.size()))
        throw std::out_of_range("Robot::removeBar: index out of range");

    bars.erase(bars.begin() + idx);

    // Remove actuators that referenced this bar; patch remaining indices
    actuators.erase(std::remove_if(actuators.begin(), actuators.end(),
        [idx](const Actuator& a) { return a.bar_idx == idx; }), actuators.end());

    for (auto& a : actuators)
        if (a.bar_idx > idx) --a.bar_idx;
}

void Robot::removeNeuron(int idx)
{
    const int n = static_cast<int>(neurons.size());
    if (idx < 0 || idx >= n)
        throw std::out_of_range("Robot::removeNeuron: index out of range");

    neurons.erase(neurons.begin() + idx);

    // Erase column `idx` from every remaining neuron's weight vector
    const int new_size = n - 1;
    for (auto& neuron : neurons) {
        Eigen::VectorXd w(new_size);
        int col = 0;
        for (int j = 0; j < n; ++j) {
            if (j == idx) continue;
            w[col++] = (j < neuron.synapse_weights.size())
                       ? neuron.synapse_weights[j]
                       : 0.0;
        }
        neuron.synapse_weights = std::move(w);
    }

    // Remove actuators referencing this neuron; patch remaining
    actuators.erase(std::remove_if(actuators.begin(), actuators.end(),
        [idx](const Actuator& a) { return a.neuron_idx == idx; }), actuators.end());

    for (auto& a : actuators)
        if (a.neuron_idx > idx) --a.neuron_idx;
}

void Robot::removeActuator(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(actuators.size()))
        throw std::out_of_range("Robot::removeActuator: index out of range");

    actuators.erase(actuators.begin() + idx);
}

void Robot::clear()
{
    vertices.clear();
    bars.clear();
    neurons.clear();
    actuators.clear();
}

// ── Physics helpers ───────────────────────────────────────────────────────────

double Robot::vertexMass(int i) const
{
    double mass = 0.0;
    for (const auto& b : bars) {
        if (b.v1 != i && b.v2 != i) continue;
        const double L0 = b.rest_length;
        mass += 0.5 * Materials::rho * (b.stiffness / Materials::E) * L0 * L0;
    }
    return mass;
}

Eigen::Vector3d Robot::centerOfMass() const
{
    if (vertices.empty()) return Eigen::Vector3d::Zero();

    Eigen::Vector3d weighted = Eigen::Vector3d::Zero();
    double total_mass = 0.0;

    for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
        const double m = vertexMass(i);
        weighted    += m * vertices[i].pos;
        total_mass  += m;
    }

    // Fallback for massless robots (all bars zero-length or no bars)
    if (total_mass == 0.0) {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (const auto& v : vertices) sum += v.pos;
        return sum / static_cast<double>(vertices.size());
    }

    return weighted / total_mass;
}

// ── Validation ────────────────────────────────────────────────────────────────

bool Robot::isValid() const
{
    const int nv = static_cast<int>(vertices.size());
    const int nb = static_cast<int>(bars.size());
    const int nn = static_cast<int>(neurons.size());

    for (const auto& b : bars) {
        if (b.v1 < 0 || b.v1 >= nv) return false;
        if (b.v2 < 0 || b.v2 >= nv) return false;
        if (b.rest_length <= 0.0)    return false;
    }

    for (const auto& a : actuators) {
        if (a.bar_idx    < 0 || a.bar_idx    >= nb) return false;
        if (a.neuron_idx < 0 || a.neuron_idx >= nn) return false;
    }

    for (const auto& n : neurons) {
        if (n.synapse_weights.size() != nn) return false;
    }

    return true;
}

// ── Debug image ───────────────────────────────────────────────────────────────

void Robot::saveDebugImage(const std::string& path) const
{
    SnapshotRenderer snap;
    snap.render(*this, path);
}

// ── YAML I/O ──────────────────────────────────────────────────────────────────
//
// Format (mirrors the Lipson & Pollack paper section ordering):
//
//   id: <uint>
//   vertices:
//     - [x, y, z]
//   bars:
//     - {v1: int, v2: int, rest_length: float, stiffness: float}
//   neurons:
//     - {threshold: float, weights: [float, ...]}
//   actuators:
//     - {bar_idx: int, neuron_idx: int, bar_range: float}

void Robot::toYAML(const std::string& path) const
{
    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "id" << YAML::Value << static_cast<unsigned long long>(id);

    // ── vertices ──────────────────────────────────────────────────────────
    out << YAML::Key << "vertices" << YAML::Value;
    out << YAML::BeginSeq;
    for (const auto& v : vertices) {
        out << YAML::Flow << YAML::BeginSeq
            << v.pos.x() << v.pos.y() << v.pos.z()
            << YAML::EndSeq;
    }
    out << YAML::EndSeq;

    // ── bars ──────────────────────────────────────────────────────────────
    out << YAML::Key << "bars" << YAML::Value;
    out << YAML::BeginSeq;
    for (const auto& b : bars) {
        out << YAML::BeginMap
            << YAML::Key << "v1"          << YAML::Value << b.v1
            << YAML::Key << "v2"          << YAML::Value << b.v2
            << YAML::Key << "rest_length" << YAML::Value << b.rest_length
            << YAML::Key << "stiffness"   << YAML::Value << b.stiffness
            << YAML::EndMap;
    }
    out << YAML::EndSeq;

    // ── neurons ───────────────────────────────────────────────────────────
    out << YAML::Key << "neurons" << YAML::Value;
    out << YAML::BeginSeq;
    for (const auto& n : neurons) {
        out << YAML::BeginMap;
        out << YAML::Key << "threshold"  << YAML::Value << n.threshold;
        out << YAML::Key << "activation" << YAML::Value << n.activation;
        out << YAML::Key << "weights"    << YAML::Value;
        out << YAML::Flow << YAML::BeginSeq;
        for (int i = 0; i < n.synapse_weights.size(); ++i)
            out << n.synapse_weights[i];
        out << YAML::EndSeq;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    // ── actuators ─────────────────────────────────────────────────────────
    out << YAML::Key << "actuators" << YAML::Value;
    out << YAML::BeginSeq;
    for (const auto& a : actuators) {
        out << YAML::BeginMap
            << YAML::Key << "bar_idx"    << YAML::Value << a.bar_idx
            << YAML::Key << "neuron_idx" << YAML::Value << a.neuron_idx
            << YAML::Key << "bar_range"  << YAML::Value << a.bar_range
            << YAML::EndMap;
    }
    out << YAML::EndSeq;

    // ── debug_actuators ───────────────────────────────────────────────────
    out << YAML::Key << "debug_actuators" << YAML::Value;
    out << YAML::BeginSeq;
    for (const auto& da : debug_actuators) {
        out << YAML::BeginMap
            << YAML::Key << "bar_idx"   << YAML::Value << da.bar_idx
            << YAML::Key << "amplitude" << YAML::Value << da.amplitude
            << YAML::Key << "frequency" << YAML::Value << da.frequency
            << YAML::Key << "phase"     << YAML::Value << da.phase
            << YAML::Key << "bar_range" << YAML::Value << da.bar_range
            << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::EndMap;

    std::ofstream fout(path);
    if (!fout.is_open())
        throw std::runtime_error("Robot::toYAML: cannot open '" + path + "' for writing");
    fout << out.c_str();
}

Robot Robot::fromYAML(const std::string& path)
{
    YAML::Node doc;
    try {
        doc = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Robot::fromYAML: " + std::string(e.what()));
    }

    Robot r(doc["id"].as<unsigned long long>());

    // ── vertices ──────────────────────────────────────────────────────────
    for (const auto& node : doc["vertices"]) {
        r.vertices.emplace_back(
            node[0].as<double>(),
            node[1].as<double>(),
            node[2].as<double>()
        );
    }

    // ── bars ──────────────────────────────────────────────────────────────
    // rest_length is optional; if absent it is computed from the initial
    // vertex positions using Eigen so the bar starts at its natural length.
    for (const auto& node : doc["bars"]) {
        const int    v1     = node["v1"].as<int>();
        const int    v2     = node["v2"].as<int>();
        double rest_length = 0.0;
        if (node["rest_length"] && node["rest_length"].as<double>() != 0.0) {
            rest_length = node["rest_length"].as<double>();
        } else if (v1 >= 0 && v1 < static_cast<int>(r.vertices.size()) &&
                   v2 >= 0 && v2 < static_cast<int>(r.vertices.size())) {
            rest_length = (r.vertices[v2].pos - r.vertices[v1].pos).norm();
        }
        // Stiffness: read directly, or convert from legacy "radius" field
        double stiffness = Materials::k_default;
        if (node["stiffness"]) {
            stiffness = node["stiffness"].as<double>();
        } else if (node["radius"] && rest_length > 0.0) {
            const double rv = node["radius"].as<double>();
            stiffness = Materials::E * M_PI * rv * rv / rest_length;
        }
        r.bars.emplace_back(v1, v2, rest_length, stiffness);
    }

    // ── neurons ───────────────────────────────────────────────────────────
    for (const auto& node : doc["neurons"]) {
        Neuron n;
        n.threshold  = node["threshold"].as<double>();
        n.activation = node["activation"] ? node["activation"].as<double>() : 0.0;
        const auto& w_node = node["weights"];
        n.synapse_weights.resize(static_cast<int>(w_node.size()));
        for (std::size_t i = 0; i < w_node.size(); ++i)
            n.synapse_weights[static_cast<int>(i)] = w_node[i].as<double>();
        r.neurons.push_back(std::move(n));
    }

    // ── actuators ─────────────────────────────────────────────────────────
    for (const auto& node : doc["actuators"]) {
        r.actuators.emplace_back(
            node["bar_idx"].as<int>(),
            node["neuron_idx"].as<int>(),
            node["bar_range"].as<double>()
        );
    }

    // ── debug_actuators ───────────────────────────────────────────────────
    if (doc["debug_actuators"]) {
        for (const auto& node : doc["debug_actuators"]) {
            r.debug_actuators.emplace_back(
                node["bar_idx"].as<int>(),
                node["amplitude"].as<double>(),
                node["frequency"].as<double>(),
                node["phase"]     ? node["phase"].as<double>()     : 0.0,
                node["bar_range"] ? node["bar_range"].as<double>() : 0.01
            );
        }
    }

    return r;
}
