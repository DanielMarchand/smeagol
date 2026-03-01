#include "Simulator.h"

#include <stdexcept>

// ── Construction ──────────────────────────────────────────────────────────────

Simulator::Simulator(const Robot& robot)
    : robot_(robot)
{
    const int N = static_cast<int>(robot.vertices.size());

    // ── Copy vertex positions into N×3 Eigen matrix ───────────────────────
    positions.resize(N, 3);
    for (int i = 0; i < N; ++i)
    {
        positions(i, 0) = robot.vertices[i].pos.x();
        positions(i, 1) = robot.vertices[i].pos.y();
        positions(i, 2) = robot.vertices[i].pos.z();
    }

    // ── Precompute lumped vertex masses ───────────────────────────────────
    // Each bar's mass (ρ·A·L₀) is split equally between its two endpoints,
    // mirroring Robot::vertexMass().
    vertex_masses = Eigen::VectorXd::Zero(N);
    for (const Bar& bar : robot.bars)
    {
        const double bar_mass = Materials::rho * bar.area() * bar.rest_length;
        const double half     = 0.5 * bar_mass;
        vertex_masses(bar.v1) += half;
        vertex_masses(bar.v2) += half;
    }
}

// ── §3.1  Energy function ────────────────────────────────────────────────────

double Simulator::elasticEnergy() const
{
    double H = 0.0;

    for (const Bar& bar : robot_.bars)
    {
        // Displacement vector between the two connected vertices
        const Eigen::Vector3d dp =
            positions.row(bar.v2) - positions.row(bar.v1);

        // Current length vs. rest length → extension δ
        const double length = dp.norm();
        const double delta  = length - bar.rest_length;

        // k = E·A / L₀  (from Bar::stiffness(), but computed inline to
        // avoid recomputing area() twice when called in a tight loop)
        const double k = bar.stiffness();

        H += k * delta * delta;
    }

    return H;
}

double Simulator::gravitationalEnergy() const
{
    // H_gravity = Σ m_j · g · h_j
    // positions.col(2) is the Z (height) column — one Eigen expression.
    return Materials::g *
           (vertex_masses.array() * positions.col(2).array()).sum();
}

double Simulator::totalEnergy() const
{
    return elasticEnergy() + gravitationalEnergy();
}

// ── State helpers ─────────────────────────────────────────────────────────────

void Simulator::copyPositionsBack(Robot& robot) const
{
    const int N = static_cast<int>(robot.vertices.size());
    if (static_cast<int>(positions.rows()) != N)
        throw std::runtime_error(
            "Simulator::copyPositionsBack: position count mismatch");

    for (int i = 0; i < N; ++i)
    {
        robot.vertices[i].pos =
            Eigen::Vector3d(positions(i, 0), positions(i, 1), positions(i, 2));
    }
}
