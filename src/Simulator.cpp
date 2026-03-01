#include "Simulator.h"

#include <algorithm>
#include <random>
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

    // Seed PRNG from a fixed seed for reproducible relaxation.
    // The evolver can re-construct the Simulator to get a fresh seed.
    rng_.seed(42);

    // Initialise per-bar rest-length overrides from genotype.
    rest_lengths_.resize(robot_.bars.size());
    for (int i = 0; i < static_cast<int>(robot_.bars.size()); ++i)
        rest_lengths_[i] = robot_.bars[i].rest_length;

    // Initialise neuron activations from genotype's stored activation field.
    activations_.resize(robot_.neurons.size());
    for (int i = 0; i < static_cast<int>(robot_.neurons.size()); ++i)
        activations_[i] = robot_.neurons[i].activation;
}

// ── §3.1  Energy function ────────────────────────────────────────────────────

double Simulator::elasticEnergy() const
{
    double H = 0.0;

    for (int i = 0; i < static_cast<int>(robot_.bars.size()); ++i)
    {
        const Bar& bar = robot_.bars[i];
        const double L0 = rest_lengths_[i];   // may be overridden by actuator

        const Eigen::Vector3d dp =
            positions.row(bar.v2) - positions.row(bar.v1);

        const double length = dp.norm();
        const double delta  = length - L0;
        const double k      = Materials::E * bar.area() / L0;

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

double Simulator::collisionEnergy() const
{
    // H_collision = Σ_{z_j < 0}  k_floor · z_j²
    double H = 0.0;
    for (int j = 0; j < static_cast<int>(positions.rows()); ++j)
    {
        const double z = positions(j, 2);
        if (z < 0.0)
            H += Materials::k_floor * z * z;
    }
    return H;
}

double Simulator::totalEnergy() const
{
    return elasticEnergy() + gravitationalEnergy() + collisionEnergy();
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

// ── §3.2  Quasi-static relaxation ──────────────────────────────────────────

Eigen::MatrixX3d Simulator::computeGradient() const
{
    const int N = static_cast<int>(positions.rows());
    Eigen::MatrixX3d grad = Eigen::MatrixX3d::Zero(N, 3);

    // ── Elastic contribution ───────────────────────────────────────────────
    // For bar i connecting v1 and v2:
    //   ∂H_i/∂p_v1 = -2 k_i δ_i û_i
    //   ∂H_i/∂p_v2 = +2 k_i δ_i û_i
    // where û_i = (p_v2 - p_v1) / ||p_v2 - p_v1||
    for (int i = 0; i < static_cast<int>(robot_.bars.size()); ++i)
    {
        const Bar& bar = robot_.bars[i];
        const double L0 = rest_lengths_[i];   // may be overridden by actuator

        const Eigen::Vector3d dp =
            positions.row(bar.v2) - positions.row(bar.v1);
        const double length = dp.norm();

        // Guard against degenerate (zero-length) bar
        if (length < 1e-14) continue;

        const double delta   = length - L0;
        const double k       = Materials::E * bar.area() / L0;
        const Eigen::Vector3d g_contrib = (2.0 * k * delta / length) * dp;

        grad.row(bar.v1) -= g_contrib;
        grad.row(bar.v2) += g_contrib;
    }

    // ── Gravitational contribution ───────────────────────────────────────────
    // ∂H_gravity/∂p_j = [0, 0, m_j * g]  (only the z column)
    grad.col(2) += Materials::g * vertex_masses;

    // ── Wind (optional) ──────────────────────────────────────────────────────
    // Constant acceleration in +X, same form as gravity but horizontal.
    // ∂H_wind/∂x_j = -wind * m_j  →  subtract from X column.
    if (wind != 0.0)
        grad.col(0) -= wind * vertex_masses;

    // ── Floor collision penalty (§3.3) ─────────────────────────────────────
    // ∂H_collision/∂z_j = 2 * k_floor * z_j  (only when z_j < 0)
    for (int j = 0; j < static_cast<int>(positions.rows()); ++j)
    {
        const double z = positions(j, 2);
        if (z < 0.0)
            grad(j, 2) += 2.0 * Materials::k_floor * z;
    }

    return grad;
}

Simulator::RelaxResult Simulator::relax(int    max_iterations,
                                        double step_size,
                                        double noise_amplitude,
                                        double convergence_tol)
{
    std::uniform_real_distribution<double> noise_dist(
        -noise_amplitude, +noise_amplitude);

    const int N = static_cast<int>(positions.rows());

    int  iter      = 0;
    bool converged = false;

    for (; iter < max_iterations; ++iter)
    {
        Eigen::MatrixX3d grad = computeGradient();

        // §3.3 Friction: zero lateral grad for grounded vertices below threshold
        applyFriction(grad);

        // Convergence check (Frobenius norm of gradient)
        if (grad.norm() < convergence_tol)
        {
            converged = true;
            break;
        }

        // Gradient descent step
        positions -= step_size * grad;

        // Uniform noise to escape unstable equilibria
        for (int j = 0; j < N; ++j)
        {
            positions(j, 0) += noise_dist(rng_);
            positions(j, 1) += noise_dist(rng_);
            positions(j, 2) += noise_dist(rng_);
        }
    }

    return { iter, totalEnergy(), converged };
}

void Simulator::applyDebugActuators(double sim_time)
{
    for (const DebugActuator& da : robot_.debug_actuators)
    {
        if (da.bar_idx < 0 || da.bar_idx >= static_cast<int>(robot_.bars.size()))
            continue;
        const double base_L0  = robot_.bars[da.bar_idx].rest_length;
        rest_lengths_[da.bar_idx] = base_L0 + da.deltaLength(sim_time);
    }
}

// ── §3.4  Neural tick ─────────────────────────────────────────────────────────

void Simulator::tickNeural()
{
    const int N = static_cast<int>(robot_.neurons.size());
    if (N == 0) return;

    // Snapshot activations before the update (parallel evaluation).
    const std::vector<double> prev = activations_;

    for (int i = 0; i < N; ++i)
    {
        const Neuron& n  = robot_.neurons[i];
        const int     sw = static_cast<int>(n.synapse_weights.size());

        double weighted_sum = 0.0;
        for (int j = 0; j < N && j < sw; ++j)
            weighted_sum += n.synapse_weights(j) * prev[j];

        activations_[i] = (weighted_sum >= n.threshold) ? 1.0 : 0.0;
    }
}

// ── §3.5  Actuator coupling ────────────────────────────────────────────────

void Simulator::applyActuators()
{
    constexpr double MAX_DELTA = 0.01;  // 1 cm per cycle (paper spec)

    for (const Actuator& a : robot_.actuators)
    {
        if (a.bar_idx    < 0 || a.bar_idx    >= static_cast<int>(rest_lengths_.size())) continue;
        if (a.neuron_idx < 0 || a.neuron_idx >= static_cast<int>(activations_.size())) continue;

        const double delta = std::clamp(
            activations_[a.neuron_idx] * a.bar_range,
            -MAX_DELTA, +MAX_DELTA);

        rest_lengths_[a.bar_idx] += delta;
    }
}

void Simulator::applyFriction(Eigen::MatrixX3d& grad) const
{
    for (int j = 0; j < static_cast<int>(positions.rows()); ++j)
    {
        const double z = positions(j, 2);
        if (z > 0.0) continue;   // not in contact

        // Normal force magnitude from floor penalty gradient
        const double normal_force = 2.0 * Materials::k_floor * std::abs(z);

        // Lateral (x,y) net force magnitude
        const double lateral_force =
            std::sqrt(grad(j, 0) * grad(j, 0) + grad(j, 1) * grad(j, 1));

        // Static friction: lock x,y if lateral force is below threshold
        if (lateral_force <= Materials::mu_static * normal_force)
        {
            grad(j, 0) = 0.0;
            grad(j, 1) = 0.0;
        }
    }
}
