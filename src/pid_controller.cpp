/**
 * @file pid_controller.cpp
 *
 * Implements the PIDController class: a resolved-rate task-space PID
 * controller with variable gain, integral anti-windup, adaptive damping,
 * null-space joint-limit repulsion, and velocity scaling.
 */

#include "g1_right_arm_reach/pid_controller.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <cmath>
#include <iostream>

namespace g1_right_arm_reach {

// ============================================================================
// MODULE: Constructor / Initialization
// ============================================================================

// Initialize the controller with gain parameters and damping/tolerance
// settings.  All internal state (error integrals, first-call flag) is
// zeroed implicitly by member initializers.
PIDController::PIDController(
    RobotModel& robot,
    Kinematics& kin,
    double kp,
    double ki,
    double kd,
    double damping,
    double tolerance,
    int max_iters)
    : robot_(robot)
    , kin_(kin)
    , kp_(kp)
    , ki_(ki)
    , kd_(kd)
    , damping_(damping)
    , tolerance_(tolerance)
    , max_iters_(max_iters)
    , prev_error_(Eigen::Vector3d::Zero())
    , error_integral_(Eigen::Vector3d::Zero())
    , first_call_(true)
{
}

// ============================================================================
// MODULE: PID Control Loop
// ============================================================================

// Run one iteration of the task-space resolved-rate PID controller.
// Returns a ControlResult with the new joint config, EE position, and
// position error.
//
// 1.  Update robot state from the caller's current_q, run FK
// 2.  Compute position error; return success if already within tolerance
// 3.  Compute numerical position Jacobian (3 x nv)
// 4.  Apply variable proportional gain: keep full kp far away (>5 cm),
//     smoothly reduce to 40 % near the target to prevent overshoot
// 5.  Accumulate integral error only when within 1 cm (anti-windup);
//     clamp accumulated integral per axis to 2 cm*s
// 6.  Compute adaptive damping: base damping + extra penalty when the
//     Jacobian is near-singular (det(JJ^T) is small)
// 7.  Form task-space velocity = P * eff_kp + I * ki + D * kd
// 8.  Resolve to joint velocity via damped pseudoinverse: dq = J^+ * v_task
// 9.  Add null-space joint-limit avoidance (repulsive potential)
// 10. Scale joint velocities to respect per-joint max velocity (6 rad/s cap)
// 11. Integrate via Lie-group step and clamp to hardware joint limits
// 12. Report final EE position and error
ControlResult PIDController::compute(
    const Eigen::Vector3d& target,
    const Eigen::VectorXd& current_q,
    const Eigen::VectorXd& current_dq,
    double dt)
{
    // Update robot state from caller's current_q
    if (current_q.size() == robot_.reduced_model_.nq) {
        robot_.q_ = current_q;
    }
    robot_.updateKinematics();

    const int nv = robot_.reduced_model_.nv;

    // Current end-effector position
    Eigen::Vector3d ee_pos = robot_.getEndEffectorPosition();

    // Position error
    Eigen::Vector3d error = target - ee_pos;
    double position_error = error.norm();

    // Already converged?
    if (position_error <= tolerance_) {
        return ControlResult::success_result(
            robot_.q_, ee_pos, position_error);
    }

    // --- Damped pseudoinverse IK ---
    // Jacobian (3 x nv)
    Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos(3, nv);
    kin_.computePositionJacobian(J_pos);

    // --- Variable gain: keep more authority near target ---
    // >5cm: full kp (fast approach)
    // 1–5cm: smooth reduction to 40% (avoid overshoot but keep pushing)
    // <1cm: hold at 40% + integral takes over for fine positioning
    double eff_kp = kp_;
    if (position_error < 0.05) {
        double ratio = position_error / 0.05;  // 0→1
        eff_kp = kp_ * (0.4 + 0.6 * ratio);   // floor at 40% (was 10%)
    }

    // --- Integral term with anti-windup ---
    // Only accumulate when close to target (< 1cm) to avoid large windup
    double integral_clamp_err = 0.01;
    if (position_error < integral_clamp_err) {
        error_integral_ += error * dt;
    } else {
        error_integral_.setZero();  // reset integral when far away
    }
    // Clamp integral magnitude per axis (anti-windup)
    double integral_max = 0.01;  // 1cm·s max accumulated
    for (int i = 0; i < 3; ++i) {
        error_integral_(i) = std::max(-integral_max,
                              std::min(integral_max, error_integral_(i)));
    }

    // --- Adaptive damping ---
    Eigen::Matrix3d JJt = J_pos * J_pos.transpose();
    double jjt_det = JJt.determinant();
    double adaptive_damp = std::max(damping_, 2e-3 / (1.0 + jjt_det * 1e5));
    JJt.diagonal().array() += adaptive_damp;

    // --- Task-space velocity: P + I + D ---
    Eigen::Vector3d task_vel = eff_kp * error;           // proportional
    task_vel += ki_ * error_integral_;                    // integral (active near target)
    if (!first_call_) {
        task_vel += kd_ * (error - prev_error_) / dt;   // derivative
    }

    // Resolve task velocity to joint velocity: dq = J^+ * task_vel
    Eigen::Matrix<double, Eigen::Dynamic, 3> J_pinv(nv, 3);
    kin_.dampedPseudoInverse(J_pos, adaptive_damp, J_pinv);
    Eigen::VectorXd dq = J_pinv * task_vel;

    // --- Null-space joint-limit avoidance ---
    Eigen::VectorXd dq_null = jointLimitAvoidance(0.1);
    Eigen::MatrixXd N(nv, nv);
    kin_.nullSpaceProjector(J_pos, adaptive_damp, N);
    dq += N * dq_null;

    // --- Velocity scaling ---
    double max_joint_vel = 6.0;  // rad/s (was 4.0) — faster approach
    for (int i = 0; i < nv; ++i) {
        max_joint_vel = std::min(max_joint_vel, robot_.dq_max_(i));
    }
    double scale = dq.lpNorm<Eigen::Infinity>() / max_joint_vel;
    if (scale > 1.0) {
        dq /= scale;
    }
    for (int i = 0; i < nv; ++i) {
        dq(i) = std::max(-max_joint_vel, std::min(max_joint_vel, dq(i)));
    }

    // --- DEBUG: print diagnostics every 40 cycles ---
    {
        static int dbg = 0;
        if (dbg++ % 40 == 0) {
            std::cout << "[PID_DBG] cycle=" << dbg
                      << " |J|=" << J_pos.norm()
                      << " det(JJt)=" << jjt_det
                      << " damp=" << adaptive_damp
                      << " |dq|=" << dq.norm()
                      << " dq=[" << dq.transpose() << "]"
                      << " q=[" << robot_.q_.transpose() << "]"
                      << " err=" << position_error
                      << std::endl;
        }
    }

    // --- Lie-group integration ---
    kin_.integrate(dq, dt);

    // --- Clamp to joint limits (hard safety) ---
    robot_.clampToLimits();

    // --- Update kinematics for result reporting ---
    robot_.updateKinematics();
    ee_pos = robot_.getEndEffectorPosition();
    position_error = (target - ee_pos).norm();

    // --- Update state ---
    prev_error_ = error;
    first_call_ = false;

    return ControlResult::success_result(robot_.q_, ee_pos, position_error);
}

// ============================================================================
// MODULE: Joint-Limit Avoidance
// ============================================================================

// Compute a repulsive joint-velocity vector that pushes joints away from
// their limits.  Uses a quadratic potential (strength^2 * 2.0) so the
// force grows smoothly as the joint approaches the limit.  Intended to
// be projected through the null-space projector so it doesn't disturb
// the end-effector task.
//
// 1. For each joint, check distance to lower and upper limits
// 2. If within the margin, compute a repulsive strength (0 at margin edge, 1 at limit)
// 3. Quadratic scaling: strength^2 * 2.0 — smooth and strong near the limit
Eigen::VectorXd PIDController::jointLimitAvoidance(double margin) const
{
    const int nv = robot_.reduced_model_.nv;
    Eigen::VectorXd dq_repulse = Eigen::VectorXd::Zero(nv);

    for (int i = 0; i < nv; ++i) {
        double q = robot_.q_(i);
        double q_min = robot_.q_min_(i);
        double q_max = robot_.q_max_(i);
        double range = q_max - q_min;

        if (range < 2.0 * margin) continue;  // joint has very small range, skip

        // Repulsive force near lower limit
        double dist_lower = q - q_min;
        if (dist_lower < margin) {
            double strength = (margin - dist_lower) / margin;  // 0→1
            dq_repulse(i) += strength * strength * 2.0;  // positive = push upward
        }

        // Repulsive force near upper limit
        double dist_upper = q_max - q;
        if (dist_upper < margin) {
            double strength = (margin - dist_upper) / margin;
            dq_repulse(i) -= strength * strength * 2.0;  // negative = push downward
        }
    }

    return dq_repulse;
}

// ============================================================================
// MODULE: Helpers
// ============================================================================

// Reset all internal PID state: error history, integral accumulator,
// and first-call flag.  Call when switching targets or restarting control.
void PIDController::reset()
{
    prev_error_.setZero();
    error_integral_.setZero();
    first_call_ = true;
}

}  // namespace g1_right_arm_reach
