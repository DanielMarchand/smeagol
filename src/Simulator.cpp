#include "Simulator.h"

#include <algorithm>
#include <chrono>
#include <random>
#include <stdexcept>
#include <unordered_set>

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
        // m = rho * (k/E) * L0^2  (back-calculated from stiffness)
        const double bar_mass = Materials::rho * (bar.stiffness / Materials::E)
                                * bar.rest_length * bar.rest_length;
        const double half     = 0.5 * bar_mass;
        vertex_masses(bar.v1) += half;
        vertex_masses(bar.v2) += half;
    }

    // Seed PRNG from a fixed seed for reproducible relaxation.
    // The evolver can re-construct the Simulator to get a fresh seed.
    rng_.seed(42);

    // Initialise per-bar rest-length overrides from genotype.
    const int nb = static_cast<int>(robot_.bars.size());
    rest_lengths_.resize(nb);
    base_rest_lengths_.resize(nb);
    target_rest_lengths_.resize(nb);
    rest_length_step_delta_.resize(nb, 0.0);
    for (int i = 0; i < nb; ++i)
        rest_lengths_[i] = base_rest_lengths_[i] = target_rest_lengths_[i]
                         = robot_.bars[i].rest_length;

    // Initialise neuron activations from genotype's stored activation field.
    activations_.resize(robot_.neurons.size());
    for (int i = 0; i < static_cast<int>(robot_.neurons.size()); ++i)
        activations_[i] = robot_.neurons[i].activation;

    // ── Precompute topology skip-sets for repulsion ────────────────────────
    // These sets are based purely on robot topology and never change during
    // simulation, so building them once here avoids O(N²) or O(B²) work on
    // every gradient step when repulsion is enabled.
    const int NV = static_cast<int>(robot_.vertices.size());
    connected_vertex_pairs_.reserve(robot_.bars.size() * 2);
    for (const Bar& bar : robot_.bars) {
        const int a = std::min(bar.v1, bar.v2);
        const int b = std::max(bar.v1, bar.v2);
        connected_vertex_pairs_.insert(a * NV + b);
    }

    const int NB = static_cast<int>(robot_.bars.size());
    adjacent_bar_pairs_.reserve(NB * 4);
    for (int i = 0; i < NB; ++i)
        for (int j = i + 1; j < NB; ++j) {
            const Bar& bi = robot_.bars[i];
            const Bar& bj = robot_.bars[j];
            if (bi.v1 == bj.v1 || bi.v1 == bj.v2 ||
                bi.v2 == bj.v1 || bi.v2 == bj.v2)
                adjacent_bar_pairs_.insert(i * NB + j);
        }
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
        const double k      = bar.stiffness;

        H += k * delta * delta;
    }

    return H;
}

double Simulator::gravitationalEnergy() const
{
    // H_gravity = Σ m_j · g · h_j
    // positions.col(2) is the Z (height) column — one Eigen expression.
    return gravity *
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
            H += k_floor * z * z;
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

// ── Self-collision repulsion ────────────────────────────────────────────────────

void Simulator::addVertexRepulsion(Eigen::MatrixX3d& grad) const
{
    const int N = static_cast<int>(positions.rows());
    if (N < 2) return;

    const double r0 = repulse_vertex_min_dist;
    const double k2 = 2.0 * k_repulse_vertex;

    for (int i = 0; i < N - 1; ++i) {
        for (int j = i + 1; j < N; ++j) {
            if (connected_vertex_pairs_.count(i * N + j)) continue;

            const Eigen::Vector3d dp = positions.row(i) - positions.row(j);
            const double d = dp.norm();
            if (d >= r0 || d < 1e-14) continue;

            // Penalty: H = k*(r0-d)^2,  ∂H/∂p_i = -k2*(r0-d) * dp/d
            const double mag = k2 * (r0 - d) / d;
            const Eigen::Vector3d g = mag * dp;
            grad.row(i) -= g;  // pushes i away from j  (subtract → position update adds)
            grad.row(j) += g;  // pushes j away from i
        }
    }
}

// Returns the closest-point parameters t, u ∈ [0,1] for two line segments
// AB and CD, and the Euclidean distance between those closest points.
static double segSegDist(const Eigen::Vector3d& A, const Eigen::Vector3d& B,
                         const Eigen::Vector3d& C, const Eigen::Vector3d& D,
                         double& t, double& u)
{
    const Eigen::Vector3d d1 = B - A;
    const Eigen::Vector3d d2 = D - C;
    const Eigen::Vector3d r  = A - C;
    const double a  = d1.dot(d1);
    const double e  = d2.dot(d2);
    const double f  = d2.dot(r);

    if (a < 1e-14 && e < 1e-14) { t = u = 0.0; return r.norm(); }
    if (a < 1e-14) {
        t = 0.0;
        u = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = d1.dot(r);
        if (e < 1e-14) {
            u = 0.0;
            t = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b     = d1.dot(d2);
            const double denom = a * e - b * b;
            if (std::abs(denom) > 1e-14)
                t = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
            else
                t = 0.0;  // parallel — arbitrary

            u = (b * t + f) / e;
            if (u < 0.0) {
                u = 0.0;
                t = std::clamp(-c / a, 0.0, 1.0);
            } else if (u > 1.0) {
                u = 1.0;
                t = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    return ((A + t * d1) - (C + u * d2)).norm();
}

void Simulator::addBarRepulsion(Eigen::MatrixX3d& grad) const
{
    const int B = static_cast<int>(robot_.bars.size());
    if (B < 2) return;

    const double r0 = repulse_bar_min_dist;
    const double k2 = 2.0 * k_repulse_bar;

    for (int i = 0; i < B - 1; ++i) {
        for (int j = i + 1; j < B; ++j) {
            if (adjacent_bar_pairs_.count(i * B + j)) continue;

            const Bar& bi = robot_.bars[i];
            const Bar& bj = robot_.bars[j];
            const Eigen::Vector3d A = positions.row(bi.v1);
            const Eigen::Vector3d Bv = positions.row(bi.v2);
            const Eigen::Vector3d C = positions.row(bj.v1);
            const Eigen::Vector3d Dv = positions.row(bj.v2);

            double t, u;
            const double d = segSegDist(A, Bv, C, Dv, t, u);
            if (d >= r0 || d < 1e-14) continue;

            // Closest points: P = A + t*(B-A),  Q = C + u*(D-C)
            const Eigen::Vector3d P = A + t * (Bv - A);
            const Eigen::Vector3d Q = C + u * (Dv - C);
            const Eigen::Vector3d nhat = (P - Q) / d;  // unit vec from Q to P

            // Gradient of H = k*(r0-d)^2 w.r.t. each endpoint:
            //   dH/dA  = -k2*(r0-d)*(1-t)*nhat
            //   dH/dBv = -k2*(r0-d)*t*nhat
            //   dH/dC  = +k2*(r0-d)*(1-u)*nhat   (chain-rule sign flips)
            //   dH/dDv = +k2*(r0-d)*u*nhat
            const double factor = k2 * (r0 - d);
            grad.row(bi.v1) -= factor * (1.0 - t) * nhat;
            grad.row(bi.v2) -= factor * t          * nhat;
            grad.row(bj.v1) += factor * (1.0 - u) * nhat;
            grad.row(bj.v2) += factor * u          * nhat;
        }
    }
}

void Simulator::addVertexBarRepulsion(Eigen::MatrixX3d& grad) const
{
    const int NV = static_cast<int>(positions.rows());
    const int NB = static_cast<int>(robot_.bars.size());
    if (NV < 1 || NB < 1) return;

    const double r0 = repulse_bar_min_dist;
    const double k2 = 2.0 * k_repulse_bar;

    // Returns true if vertex indices a and b are endpoints of the same bar.
    auto are_connected = [&](int a, int bv) -> bool {
        const int lo = std::min(a, bv), hi = std::max(a, bv);
        return connected_vertex_pairs_.count(lo * NV + hi) != 0;
    };

    for (int i = 0; i < NV; ++i) {
        const Eigen::Vector3d P = positions.row(i);

        for (int j = 0; j < NB; ++j) {
            const Bar& b = robot_.bars[j];
            // Skip if vertex is an endpoint of this bar — always d=0 at joint.
            if (b.v1 == i || b.v2 == i) continue;

            const Eigen::Vector3d A  = positions.row(b.v1);
            const Eigen::Vector3d Bv = positions.row(b.v2);
            const Eigen::Vector3d AB = Bv - A;
            const double ab2 = AB.dot(AB);
            if (ab2 < 1e-14) continue;  // degenerate zero-length bar

            const double t = std::clamp(AB.dot(P - A) / ab2, 0.0, 1.0);

            // If the closest point clamped to an endpoint AND vertex i is
            // topologically connected to that endpoint (i.e. they share a
            // joint), the approach is due to angle closure, not genuine bar
            // penetration.  Skip to avoid spurious anti-hinge forces.
            // When the closest point falls in the bar's interior (t ∈ (0,1))
            // the vertex is truly penetrating the bar — repel in all cases.
            if (t < 1e-6       && are_connected(i, b.v1)) continue;
            if (t > 1.0 - 1e-6 && are_connected(i, b.v2)) continue;

            const Eigen::Vector3d Q  = A + t * AB;
            const Eigen::Vector3d PQ = P - Q;
            const double d = PQ.norm();
            if (d >= r0 || d < 1e-14) continue;

            // nhat points from bar toward vertex.
            const Eigen::Vector3d nhat = PQ / d;
            const double factor = k2 * (r0 - d);

            // Push vertex away from bar.
            grad.row(i)    -= factor *             nhat;
            // Push bar endpoints away from vertex (chain rule through Q).
            grad.row(b.v1) += factor * (1.0 - t) * nhat;
            grad.row(b.v2) += factor * t          * nhat;
        }
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
        const double k       = bar.stiffness;
        const Eigen::Vector3d g_contrib = (2.0 * k * delta / length) * dp;

        grad.row(bar.v1) -= g_contrib;
        grad.row(bar.v2) += g_contrib;
    }

    // ── Gravitational contribution ───────────────────────────────────────────
    // ∂H_gravity/∂p_j = [0, 0, m_j * gravity]  (only the z column)
    grad.col(2) += gravity * vertex_masses;

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
            grad(j, 2) += 2.0 * k_floor * z;
    }
    // ── Self-collision repulsion terms (optional) ─────────────────────────
    using Clock = std::chrono::high_resolution_clock;

    if (k_repulse_vertex > 0.0) {
        const auto t0 = Clock::now();
        addVertexRepulsion(grad);
        timing_vertex_repulse_ns_ +=
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count());
    }

    if (k_repulse_bar > 0.0) {
        const auto t0 = Clock::now();
        addBarRepulsion(grad);
        addVertexBarRepulsion(grad);
        timing_bar_repulse_ns_ +=
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count());
    }

    ++timing_calls_;
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
        // Advance actuator ramp: linearly move rest lengths toward their targets.
        // This spreads the elastic change from a neural tick evenly across the
        // full relaxation budget instead of applying it as a single step-change.
        for (int i = 0; i < static_cast<int>(rest_lengths_.size()); ++i)
            rest_lengths_[i] += rest_length_step_delta_[i];

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

void Simulator::applyActuators(int steps_per_cycle)
{
    const double inv_steps = (steps_per_cycle > 0)
                             ? 1.0 / static_cast<double>(steps_per_cycle)
                             : 1.0;

    // Reset all deltas to zero; only actuated bars will get a non-zero value.
    std::fill(rest_length_step_delta_.begin(), rest_length_step_delta_.end(), 0.0);

    for (const Actuator& a : robot_.actuators)
    {
        if (a.bar_idx    < 0 || a.bar_idx    >= static_cast<int>(rest_lengths_.size())) continue;
        if (a.neuron_idx < 0 || a.neuron_idx >= static_cast<int>(activations_.size())) continue;

        // Per-cycle extension: how much the bar can lengthen this tick.
        const double ext_this_cycle = std::clamp(
            activations_[a.neuron_idx] * a.bar_range,
            0.0, actuator_max_per_cycle);

        // Absolute cap: total deviation from rest length cannot exceed this.
        const double extension = std::min(ext_this_cycle, actuator_max_total);

        // Target is base length + extension (reverts to base when neuron quiesces)
        const double target = base_rest_lengths_[a.bar_idx] + extension;
        target_rest_lengths_[a.bar_idx] = target;

        // Per-step ramp: spread the length change evenly over the cycle
        rest_length_step_delta_[a.bar_idx] =
            (target - rest_lengths_[a.bar_idx]) * inv_steps;
    }
}

void Simulator::applyFriction(Eigen::MatrixX3d& grad) const
{
    for (int j = 0; j < static_cast<int>(positions.rows()); ++j)
    {
        const double z = positions(j, 2);
        if (z > 0.0) continue;   // not in contact

        // Normal force magnitude from floor penalty gradient
        const double normal_force = 2.0 * k_floor * std::abs(z);

        // Lateral (x,y) net force magnitude
        const double lateral_force =
            std::sqrt(grad(j, 0) * grad(j, 0) + grad(j, 1) * grad(j, 1));

        // Kinetic friction: reduce lateral gradient by mu*N, clamped to zero.
        // This always provides resistance but never reverses direction of motion.
        const double friction_limit = mu_static * normal_force;
        const double reduced = std::max(0.0, lateral_force - friction_limit);
        const double scale   = (lateral_force > 1e-14) ? (reduced / lateral_force) : 0.0;
        grad(j, 0) *= scale;
        grad(j, 1) *= scale;
    }
}
