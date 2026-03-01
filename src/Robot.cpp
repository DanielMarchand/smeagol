#include "Robot.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

// ── Static member ─────────────────────────────────────────────────────────────

Robot::ID Robot::s_next_id = 0;

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
    for (auto& existing : neurons)
        existing.synapse_weights.conservativeResize(new_idx + 1);

    // The incoming neuron must have exactly new_idx+1 weights
    Neuron copy = n;
    copy.synapse_weights.conservativeResize(new_idx + 1);
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
        const double L = (vertices[b.v1].pos - vertices[b.v2].pos).norm();
        mass += 0.5 * Materials::rho * b.area() * L;
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

// ── YAML I/O stubs (phase 1.4) ────────────────────────────────────────────────

void Robot::toYAML(const std::string& /*path*/) const
{
    throw std::logic_error("Robot::toYAML not yet implemented (phase 1.4)");
}

Robot Robot::fromYAML(const std::string& /*path*/)
{
    throw std::logic_error("Robot::fromYAML not yet implemented (phase 1.4)");
}
