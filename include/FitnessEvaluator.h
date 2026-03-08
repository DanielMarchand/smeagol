#pragma once

#include "Robot.h"
#include "Simulator.h"

#include <Eigen/Dense>
#include <string>
#include <vector>

/**
 * @brief Parameters controlling a FitnessEvaluator run.
 *
 * Kept outside FitnessEvaluator so it can be used as a default argument
 * in the constructor without hitting GCC's incomplete-class restriction.
 */
struct FitnessParams
{
    /// Total number of physics steps to simulate.
    /// Neural ticks fire every steps_per_cycle steps (or steps_per_frame when
    /// steps_per_cycle == 0).  Video frames are captured every steps_per_frame
    /// steps.  So: num_neural_ticks = num_steps / spc,
    ///              num_video_frames = num_steps / steps_per_frame.
    int    num_steps       = 60000;  ///< default = 12 ticks × 5000 steps/tick

    /// Physics steps to run *before* recording the fitness start position.
    /// These are carved out of num_steps — total physics run = num_steps,
    /// fitness is measured over the last (num_steps - delay_steps) steps.
    /// Useful to filter out robots that merely fall over at the start.
    /// 0 = measure from the very first step (default).
    int    delay_steps     = 0;

    /// Physics steps per captured video frame (also the actuator sub-step).
    int    steps_per_frame = 5000;

    /// Physics steps per neural tick.  0 = same as steps_per_frame.
    int    steps_per_cycle = 0;


    /// Gravitational acceleration [m/s²].  Defaults to 9.81.
    double gravity         = 9.81;

    /// Gradient step size [m/N].  Must satisfy Ds < 1/(2·k_max).
    double step_size       = 1.0e-7;

    /// Optional wind acceleration [m/s²] in +X applied to all vertices.
    double wind            = 0.0;

    /// Kinetic friction coefficient.
    double mu_static       = 0.5;

    /// Floor penalty spring stiffness [N/m].
    /// Higher values push vertices above z=0 more aggressively.
    double k_floor         = 1.4e6;

    /// Maximum bar extension per neural cycle [m].
    /// Limits how far an actuator can lengthen in a single tick.
    double actuator_max_extension_per_cycle = 0.01;

    /// Maximum absolute bar extension from base rest length [m].
    /// Hard cap on total elongation regardless of per-cycle limit.
    double actuator_max_extension_total = 0.05;

    /// Optional stiffness override [N/m] applied to every bar before simulation.
    /// Mirrors the bar_stiffness field in run_simulation's simulation.yaml.
    /// Set <= 0 to leave each bar's stiffness unchanged (the default).
    double bar_stiffness_override = -1.0;

    // ── Self-collision prevention ─────────────────────────────────────────
    /// Vertex-vertex repulsion stiffness [N·m⁻²].  0 = disabled (default).
    double k_repulse_vertex        = 0.0;
    /// Minimum vertex-vertex separation before repulsion fires [m].
    double repulse_vertex_min_dist = 0.02;
    /// Bar-bar repulsion stiffness [N·m⁻²].  0 = disabled (default).
    double k_repulse_bar           = 0.0;
    /// Minimum bar clearance before bar repulsion fires [m].
    double repulse_bar_min_dist    = 0.01;
};

/**
 * @brief Evaluates the locomotion fitness of a Robot.
 *
 * Fitness is defined as the horizontal (XY-plane) displacement of the
 * centre of mass over a fixed number of neural cycles — matching the
 * evaluation criterion described in Lipson & Pollack (2000).
 *
 * One "cycle" consists of:
 *   1. tickNeural()      — synchronous parallel update of all neurons
 *   2. applyActuators()  — apply neuron outputs to bar rest-lengths
 *   3. relax()           — quasi-static gradient descent to the nearest
 *                          energy minimum (steps_per_cycle steps)
 *
 * The robot is evaluated statelessly: a fresh Simulator is constructed
 * from the Robot on every call to evaluate(), so repeated calls on the
 * same Robot are deterministic.
 *
 * Usage
 * -----
 * @code
 *   FitnessEvaluator eval;                   // default params
 *   double score = eval.evaluate(robot);      // metres of XY displacement
 *
 *   // record per-cycle CoM positions for analysis / rendering
 *   std::vector<Eigen::Vector2d> traj;
 *   double score = eval.evaluate(robot, &traj);
 * @endcode
 */
class FitnessEvaluator
{
public:
    // Expose the params type under a nested alias for convenience
    using Params = FitnessParams;

    explicit FitnessEvaluator(Params params = Params());

    /**
     * @brief Evaluate a robot's locomotion fitness.
     *
     * Constructs a fresh Simulator, runs @p params.num_frames neural–relax
     * frames, and returns the Euclidean distance in the XY plane between
     * the initial and final centre-of-mass positions.
     *
     * @param robot      Robot to evaluate.  Not modified.
     * @param trajectory Optional output: one (x, y) point per frame
     *                   (including the initial position at index 0).
     *                   Pass nullptr to skip recording.
     * @return           Horizontal displacement [m].  ≥ 0.
     */
    double evaluate(const Robot&                   robot,
                    std::vector<Eigen::Vector2d>*  trajectory = nullptr) const;

    /**
     * @brief Build a Simulator from a Robot copy, applying all FitnessParams
     * physics settings (bar_stiffness_override, gravity, k_floor, etc.).
     *
     * This is the single canonical way to construct a Simulator for any
     * physics run — fitness evaluation, video rendering, or otherwise.
     * The robot is modified in-place (stiffness override applied), so always
     * pass a mutable copy, never the original.
     */
    static Simulator makeSimulator(Robot& robot, const FitnessParams& p);

    /**
     * @brief Re-simulate a robot and write an MP4, using FitnessParams for
     * all physics and timing.  This is the single canonical video method.
     *
     * Neural ticks fire every steps_per_cycle steps (or every steps_per_frame
     * when steps_per_cycle == 0).  A video frame is captured every
     * steps_per_frame steps, potentially giving multiple frames per tick.
     * Actuators are applied once per steps_per_frame chunk.
     *
     * @param robot   Robot to simulate (passed by value — modified internally).
     * @param fp      Physics + timing parameters.
     * @param path    Output MP4 path.
     * @param fps     Video framerate (default 30).
     * @param verbose If true, suppress raylib / ffmpeg noise (default false).
     */
    static void renderVideo(Robot robot, const FitnessParams& fp,
                            const std::string& path,
                            int fps = 30,
                            int width = 1280, int height = 720,
                            bool verbose = false,
                            float render_vertex_radius = 0.010f,
                            float render_bar_radius    = 0.010f);

    const Params& params() const { return params_; }

    /** Average µs per gradient-step for vertex repulsion during the last evaluate() call. */
    double lastVertexRepulseUs() const { return last_vertex_repulse_us_; }
    /** Average µs per gradient-step for bar repulsion during the last evaluate() call. */
    double lastBarRepulseUs()    const { return last_bar_repulse_us_; }

private:
    Params params_;
    mutable double last_vertex_repulse_us_ = 0.0;
    mutable double last_bar_repulse_us_    = 0.0;
};

