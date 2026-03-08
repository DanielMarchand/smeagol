#include "FitnessEvaluator.h"
#include "Simulator.h"
#include "VideoRenderer.h"

#include <algorithm>
#include <cmath>
#include <iostream>

// ─────────────────────────────────────────────────────────────────────────────
// helpers
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Compute the mass-weighted centre-of-mass XY position from the current
 * Simulator state.
 */
static Eigen::Vector2d com_xy(const Simulator& sim)
{
    const double total_mass = sim.vertex_masses.sum();
    if (total_mass <= 0.0)
        return Eigen::Vector2d::Zero();

    // positions is (N, 3); columns 0 and 1 are X and Y
    const double cx = (sim.vertex_masses.array() * sim.positions.col(0).array()).sum() / total_mass;
    const double cy = (sim.vertex_masses.array() * sim.positions.col(1).array()).sum() / total_mass;
    return {cx, cy};
}

// ─────────────────────────────────────────────────────────────────────────────
// FitnessEvaluator
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// FitnessEvaluator::makeSimulator  — the single canonical simulator factory
//
// All physics runs (fitness eval, video recording, …) must go through here so
// that bar_stiffness_override and every other FitnessParams knob are applied
// identically everywhere.  The robot is modified in-place (stiffness override)
// so always pass a mutable copy.
// ─────────────────────────────────────────────────────────────────────────────

Simulator FitnessEvaluator::makeSimulator(Robot& robot, const FitnessParams& p)
{
    if (p.bar_stiffness_override > 0.0)
        for (auto& b : robot.bars)
            b.stiffness = p.bar_stiffness_override;

    Simulator sim(robot);
    sim.gravity                = p.gravity;
    sim.wind                   = p.wind;
    sim.mu_static              = p.mu_static;
    sim.k_floor                = p.k_floor;
    sim.actuator_max_per_cycle = p.actuator_max_extension_per_cycle;
    sim.actuator_max_total     = p.actuator_max_extension_total;
    sim.k_repulse_vertex       = p.k_repulse_vertex;
    sim.repulse_vertex_min_dist= p.repulse_vertex_min_dist;
    sim.k_repulse_bar          = p.k_repulse_bar;
    sim.repulse_bar_min_dist   = p.repulse_bar_min_dist;
    return sim;
}

FitnessEvaluator::FitnessEvaluator(Params params)
    : params_(params)
{}

double FitnessEvaluator::evaluate(const Robot&                  robot,
                                  std::vector<Eigen::Vector2d>* trajectory) const
{
    // Fast-path: no actuators → can never produce directed locomotion.
    if (robot.isNonMover())
        return 0.0;

    Robot robot_eval = robot;
    Simulator sim = makeSimulator(robot_eval, params_);
    sim.resetRepulseTiming();

    const int spc = (params_.steps_per_cycle > 0)
                    ? params_.steps_per_cycle
                    : params_.steps_per_frame;

    // ── Delay phase: run without recording fitness start ──────────────────
    int delay_done = 0;
    while (delay_done < params_.delay_steps) {
        sim.tickNeural();
        const int cycle_steps = std::min(spc, params_.delay_steps - delay_done);
        int remaining = cycle_steps;
        while (remaining > 0) {
            const int chunk = std::min(remaining, params_.steps_per_frame);
            sim.applyActuators(chunk);
            sim.relax(chunk, params_.step_size, params_.noise_amplitude, /*tol=*/0.0);
            remaining -= chunk;
        }
        delay_done += cycle_steps;
    }

    // ── Fitness phase: measure from here ─────────────────────────────────
    const Eigen::Vector2d start = com_xy(sim);

    if (trajectory) {
        trajectory->clear();
        const int fitness_steps = params_.num_steps - params_.delay_steps;
        trajectory->reserve(fitness_steps / spc + 2);
        trajectory->push_back(start);   // position at t=delay
    }

    int steps_done = params_.delay_steps;  // already consumed by delay phase
    while (steps_done < params_.num_steps) {
        sim.tickNeural();
        const int cycle_steps = std::min(spc, params_.num_steps - steps_done);
        int remaining = cycle_steps;
        while (remaining > 0) {
            const int chunk = std::min(remaining, params_.steps_per_frame);
            sim.applyActuators(chunk);
            sim.relax(chunk, params_.step_size, params_.noise_amplitude, /*tol=*/0.0);
            remaining -= chunk;
        }
        steps_done += cycle_steps;
        if (trajectory)
            trajectory->push_back(com_xy(sim));
    }

    last_vertex_repulse_us_ = sim.avgVertexRepulseUs();
    last_bar_repulse_us_    = sim.avgBarRepulseUs();
    return (com_xy(sim) - start).norm();
}

// ─────────────────────────────────────────────────────────────────────────────
// FitnessEvaluator::renderVideo  — single canonical video method
// ─────────────────────────────────────────────────────────────────────────────

void FitnessEvaluator::renderVideo(Robot robot, const FitnessParams& fp,
                                   const std::string& path,
                                   int fps, int width, int height, bool verbose,
                                   float render_vertex_radius, float render_bar_radius,
                                   float camera_distance, float camera_fov, float camera_elevation,
                                   bool camera_follow)
{
    Simulator sim = makeSimulator(robot, fp);

    const int spc = (fp.steps_per_cycle > 0)
                    ? fp.steps_per_cycle
                    : fp.steps_per_frame;
    const int frame_step      = std::min(spc, fp.steps_per_frame);
    const int expected_frames = fp.num_steps / frame_step;

    std::cout << "Rendering " << expected_frames << " frames"
              << "  (delay=" << fp.delay_steps / frame_step
              << " + fitness=" << (fp.num_steps - fp.delay_steps) / frame_step
              << ")  →  " << path << "\n";
    std::cout.flush();

    VideoRenderer vid(fps, width, height);
    vid.setVerbose(verbose);
    vid.render_vertex_radius = render_vertex_radius;
    vid.render_bar_radius    = render_bar_radius;
    vid.camera_distance      = camera_distance;
    vid.camera_fov           = camera_fov;
    vid.camera_elevation     = camera_elevation;
    vid.camera_follow        = camera_follow;
    int steps_done = 0;

    // ── Delay phase (rendered but not counted toward fitness) ─────────────
    if (fp.delay_steps > 0)
        vid.beginSettling();    // show "settling..." until delay ends

    int delay_done = 0;
    while (delay_done < fp.delay_steps) {
        sim.tickNeural();
        const int cycle_steps = std::min(spc, fp.delay_steps - delay_done);
        int remaining = cycle_steps;
        while (remaining > 0) {
            const int chunk = std::min(remaining, fp.steps_per_frame);
            sim.applyActuators(chunk);
            sim.relax(chunk, fp.step_size, fp.noise_amplitude, 0.0);
            remaining   -= chunk;
            steps_done  += chunk;
            sim.copyPositionsBack(robot);
            vid.addFrame(robot,
                         static_cast<double>(steps_done) * fp.step_size,
                         sim.activations_);
        }
        delay_done += cycle_steps;
    }

    // ── Fitness phase ─────────────────────────────────────────────────────
    vid.markFitnessOrigin();    // displacement banner resets to 0 from here
    while (steps_done < fp.num_steps) {
        sim.tickNeural();
        const int cycle_steps = std::min(spc, fp.num_steps - steps_done);
        int remaining = cycle_steps;
        while (remaining > 0) {
            const int chunk = std::min(remaining, fp.steps_per_frame);
            sim.applyActuators(chunk);
            sim.relax(chunk, fp.step_size, fp.noise_amplitude, 0.0);
            remaining   -= chunk;
            steps_done  += chunk;
            sim.copyPositionsBack(robot);
            vid.addFrame(robot,
                         static_cast<double>(steps_done) * fp.step_size,
                         sim.activations_);
        }
    }
    vid.finish(path);
}
