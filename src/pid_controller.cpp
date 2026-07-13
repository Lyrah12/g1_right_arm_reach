/**
 * @file pid_controller.cpp
 * Task-space resolved-rate PID controller with damped pseudoinverse IK,
 * variable gain, integral anti-windup, adaptive damping, and null-space
 * joint-limit repulsion.
 */

#include "g1_right_arm_reach/pid_controller.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <cmath>
#include <iostream>

namespace g1_right_arm_reach {

// ============================================================================
// Constructor
// ============================================================================

PIDController::PIDController(
    RobotModel& robot, Kinematics& kin,
    double kp, double ki, double kd,
    double damping, double tolerance,
    double posture_gain)
    : robot_(robot), kin_(kin)
    , kp_(kp), ki_(ki), kd_(kd)
    , damping_(damping), tolerance_(tolerance)
    , posture_gain_(posture_gain)
{
    posture_target_ = Eigen::VectorXd::Zero(robot_.reduced_model_.nq);
}

// ============================================================================
// Main control cycle
// ============================================================================

ControlResult PIDController::compute(
    const Eigen::Vector3d& target,
    const Eigen::VectorXd& current_q,
    const Eigen::VectorXd& current_dq,
    double dt,
    const Eigen::Vector3d* external_error,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>* external_jacobian)
{
    // --- Sync robot state ---
    if (current_q.size() == robot_.reduced_model_.nq) {
        robot_.q_ = current_q;
    }

    const int nv = robot_.reduced_model_.nv;

    // --- Position error ---
    // Use external error (MuJoCo ground-truth) when available to eliminate
    // URDF/MJCF FK model mismatch.
    Eigen::Vector3d error;
    Eigen::Vector3d ee_pos;
    if (external_error) {
        error = *external_error;
        robot_.updateKinematics();
        ee_pos = robot_.getEndEffectorPosition();
    } else {
        robot_.updateKinematics();
        ee_pos = robot_.getEndEffectorPosition();
        error = target - ee_pos;
    }
    double position_error = error.norm();

    // Already converged — reset integral to prevent windup
    if (position_error <= tolerance_) {
        error_integral_.setZero();
        return ControlResult::success_result(robot_.q_, ee_pos, position_error);
    }

    // --- Divergence guard ---
    if (!first_call_ && position_error > prev_error_.norm() * 5.0) {
        std::cerr << "[PID] ERROR SPIKE: " << prev_error_.norm()
                  << " → " << position_error << " — resetting integral\n";
        error_integral_.setZero();
    }

    // --- Numerical Jacobian (3 × nv, reduced model) ---
    // Use MuJoCo external Jacobian when available to eliminate URDF/MJCF
    // kinematic model mismatch (pelvis→torso chain differs between models).
    Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos(3, nv);
    if (external_jacobian && external_jacobian->cols() == nv) {
        J_pos = *external_jacobian;
    } else {
        kin_.computePositionJacobian(J_pos);
    }

    // --- Variable proportional gain ---
    // >5 cm: full kp.  1–5 cm: smooth reduction to 60%.  <1 cm: 60%.
    double eff_kp = kp_;
    if (position_error < 0.05) {
        double ratio = position_error / 0.05;
        eff_kp = kp_ * (0.6 + 0.4 * ratio);
    }

    // --- Integral term with anti-windup ---
    // Accumulate when error < 20 cm (safety threshold, well above normal
    // operating range).  No leaky decay — the i_max clamp provides windup
    // protection, and back-calculation after velocity clamping (below)
    // handles actuator saturation.  Previously the error-based decay
    // created a catch-22: the steady-state error it was meant to correct
    // was also the condition that triggered the decay.
    if (position_error < 0.20) {
        error_integral_ += error * dt;
    }
    // else: error > 20 cm → freeze integral (don't accumulate, don't decay).
    //        This is a safety measure for wildly invalid targets.
    double i_max = 0.10;  // 10 cm·s per axis (was 0.05 — integral saturated
                           // at ~0.087 norm every cycle, unable to grow further
                           // to correct persistent ~10 cm steady-state error)
    for (int i = 0; i < 3; ++i)
        error_integral_(i) = std::max(-i_max, std::min(i_max, error_integral_(i)));

    // --- Adaptive damping ---
    Eigen::Matrix3d JJt = J_pos * J_pos.transpose();
    double jjt_det = JJt.determinant();
    double adaptive_damp = std::max(damping_, 1e-3 / (1.0 + jjt_det * 1e5));

    // --- Task-space velocity: P + I + D ---
    Eigen::Vector3d task_vel = eff_kp * error + ki_ * error_integral_;
    if (!first_call_)
        task_vel += kd_ * (error - prev_error_) / dt;

    double max_task_speed = 1.0;
    double task_speed = task_vel.norm();
    if (task_speed > max_task_speed)
        task_vel *= max_task_speed / task_speed;

    // --- Resolve to joint velocity: dq = J⁺ · v ---
    Eigen::Matrix<double, Eigen::Dynamic, 3> J_pinv(nv, 3);
    kin_.dampedPseudoInverse(J_pos, adaptive_damp, J_pinv);
    Eigen::VectorXd dq = J_pinv * task_vel;

    // --- Null-space: joint-limit avoidance + posture attraction ---
    // Pull joints toward the preferred posture (all zeros = arm at ready pose),
    // preventing the IK from taking "wrapping" solutions through extreme yaw.
    Eigen::VectorXd dq_null = jointLimitAvoidance(0.1);
    dq_null -= posture_gain_ * (robot_.q_ - posture_target_);  // attract toward preferred pose
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(nv, nv) - J_pinv * J_pos;
    dq += N * dq_null;

    // --- Velocity clamping ---
    double max_joint_vel = 6.0;
    for (int i = 0; i < nv; ++i)
        max_joint_vel = std::min(max_joint_vel, robot_.dq_max_(i));
    double dq_unclamped_norm = dq.lpNorm<Eigen::Infinity>();
    double scale = dq_unclamped_norm / max_joint_vel;
    if (scale > 1.0) dq /= scale;
    for (int i = 0; i < nv; ++i)
        dq(i) = std::max(-max_joint_vel, std::min(max_joint_vel, dq(i)));

    // --- Back-calculation anti-windup ---
    // If the velocity command was clamped (actuator saturation), reduce
    // the integral proportionally so it doesn't accumulate for motion
    // the system can't physically execute.
    double dq_clamped_norm = dq.lpNorm<Eigen::Infinity>();
    if (dq_unclamped_norm > 1e-6 && dq_clamped_norm < dq_unclamped_norm * 0.8) {
        double ratio = dq_clamped_norm / dq_unclamped_norm;
        error_integral_ *= ratio;
    }

    // --- Joint-limit direction clamp ---
    for (int i = 0; i < nv; ++i) {
        if (robot_.q_(i) <= robot_.q_min_(i) && dq(i) < 0.0) dq(i) = 0.0;
        if (robot_.q_(i) >= robot_.q_max_(i) && dq(i) > 0.0) dq(i) = 0.0;
    }

    // --- Integrate + hard clamp ---
    kin_.integrate(dq, dt);
    robot_.clampToLimits();

    // --- Per-cycle q change clamp (prevents Jacobian ill-conditioning jumps) ---
    for (int i = 0; i < nv; ++i) {
        double delta = robot_.q_(i) - current_q(i);
        double max_delta = 0.25;  // rad/cycle (5 rad/s effective at 20Hz)
        if (delta > max_delta)       robot_.q_(i) = current_q(i) + max_delta;
        else if (delta < -max_delta) robot_.q_(i) = current_q(i) - max_delta;
    }

    // --- Report ---
    // Preserve MuJoCo-based position_error before URDF FK overwrites it.
    // (When external_error is used, error is MuJoCo ground-truth, but
    // robot_.getEndEffectorPosition() returns URDF FK — they differ.)
    double mujoco_position_error = position_error;
    robot_.updateKinematics();
    ee_pos = robot_.getEndEffectorPosition();
    double urdf_position_error = (target - ee_pos).norm();

    prev_error_ = error;
    first_call_ = false;

    // Return MuJoCo-based error when external error was provided (more
    // accurate), otherwise URDF error.  Control uses external_error directly,
    // so this only affects the reported value in ControlResult.
    double reported_error = (external_error != nullptr)
        ? mujoco_position_error : urdf_position_error;

    auto result = ControlResult::success_result(robot_.q_, ee_pos, reported_error);
    result.effective_kp = eff_kp;
    result.integral_norm = error_integral_.norm();
    result.task_velocity = task_speed;
    result.adaptive_damping = adaptive_damp;
    result.nullspace_norm = dq_null.norm();
    result.max_joint_vel = dq.lpNorm<Eigen::Infinity>();
    result.dq_command = dq;
    return result;
}

// ============================================================================
// Joint-limit avoidance (null-space repulsion)
// ============================================================================

Eigen::VectorXd PIDController::jointLimitAvoidance(double margin) const
{
    const int nv = robot_.reduced_model_.nv;
    Eigen::VectorXd dq_repulse = Eigen::VectorXd::Zero(nv);

    for (int i = 0; i < nv; ++i) {
        double q = robot_.q_(i);
        double q_min = robot_.q_min_(i);
        double q_max = robot_.q_max_(i);
        double range = q_max - q_min;
        if (range < 2.0 * margin) continue;

        double dist_lower = q - q_min;
        if (dist_lower < margin) {
            double s = (margin - dist_lower) / margin;
            dq_repulse(i) += s * s * 2.0;
        }
        double dist_upper = q_max - q;
        if (dist_upper < margin) {
            double s = (margin - dist_upper) / margin;
            dq_repulse(i) -= s * s * 2.0;
        }
    }
    return dq_repulse;
}

// ============================================================================
// Reset
// ============================================================================

void PIDController::reset()
{
    prev_error_.setZero();
    error_integral_.setZero();
    first_call_ = true;
}

}  // namespace g1_right_arm_reach
