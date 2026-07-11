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
    double damping, double tolerance)
    : robot_(robot), kin_(kin)
    , kp_(kp), ki_(ki), kd_(kd)
    , damping_(damping), tolerance_(tolerance)
{}

// ============================================================================
// Main control cycle
// ============================================================================

ControlResult PIDController::compute(
    const Eigen::Vector3d& target,
    const Eigen::VectorXd& current_q,
    const Eigen::VectorXd& current_dq,
    double dt,
    const Eigen::Vector3d* external_error)
{
    // --- Sync robot state ---
    if (current_q.size() == robot_.reduced_model_.nq) {
        robot_.q_ = current_q;
        robot_.syncActiveToFull();
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
    Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos(3, nv);
    kin_.computePositionJacobian(J_pos);

    // --- Variable proportional gain ---
    // >5 cm: full kp.  1–5 cm: smooth reduction to 60%.  <1 cm: 60%.
    double eff_kp = kp_;
    if (position_error < 0.05) {
        double ratio = position_error / 0.05;
        eff_kp = kp_ * (0.6 + 0.4 * ratio);
    }

    // --- Integral term with anti-windup ---
    // Activate when error < 5 cm; leaky decay otherwise.
    if (position_error < 0.05) {
        error_integral_ += error * dt;
    } else {
        error_integral_ *= 0.78;  // τ ≈ 0.2 s
    }
    double i_max = 0.02;  // 2 cm·s per axis
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

    double max_task_speed = 0.5;
    double task_speed = task_vel.norm();
    if (task_speed > max_task_speed)
        task_vel *= max_task_speed / task_speed;

    // --- Resolve to joint velocity: dq = J⁺ · v ---
    Eigen::Matrix<double, Eigen::Dynamic, 3> J_pinv(nv, 3);
    kin_.dampedPseudoInverse(J_pos, adaptive_damp, J_pinv);
    Eigen::VectorXd dq = J_pinv * task_vel;

    // --- Null-space joint-limit avoidance ---
    Eigen::VectorXd dq_null = jointLimitAvoidance(0.1);
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(nv, nv) - J_pinv * J_pos;
    dq += N * dq_null;

    // --- Velocity clamping ---
    double max_joint_vel = 6.0;
    for (int i = 0; i < nv; ++i)
        max_joint_vel = std::min(max_joint_vel, robot_.dq_max_(i));
    double scale = dq.lpNorm<Eigen::Infinity>() / max_joint_vel;
    if (scale > 1.0) dq /= scale;
    for (int i = 0; i < nv; ++i)
        dq(i) = std::max(-max_joint_vel, std::min(max_joint_vel, dq(i)));

    // --- Joint-limit direction clamp ---
    for (int i = 0; i < nv; ++i) {
        if (robot_.q_(i) <= robot_.q_min_(i) && dq(i) < 0.0) dq(i) = 0.0;
        if (robot_.q_(i) >= robot_.q_max_(i) && dq(i) > 0.0) dq(i) = 0.0;
    }

    // --- Integrate + hard clamp ---
    kin_.integrate(dq, dt);
    robot_.clampToLimits();
    robot_.syncActiveToFull();

    // --- Report ---
    robot_.updateKinematics();
    ee_pos = robot_.getEndEffectorPosition();
    position_error = (target - ee_pos).norm();

    prev_error_ = error;
    first_call_ = false;
    return ControlResult::success_result(robot_.q_, ee_pos, position_error);
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
