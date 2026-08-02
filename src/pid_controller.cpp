/**
 * @file pid_controller.cpp
 * @brief Arm-Hand decoupled PID + IK: computeArm (4-DOF→3-DOF position).
 *
 * computeArm: 3×4 damped pseudoinverse + null-space posture attraction.
 * Wrist orientation control lives in HandController (hand_controller_node.cpp).
 */

#include "g1_right_arm_reach/pid_controller.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <cmath>

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
// Gravity resolution — external (MuJoCo, incl. hand mass) > Pinocchio (URDF only)
// ============================================================================

void PIDController::resolveGravityTorque(Eigen::VectorXd& tau_g) const
{
    const int nv = robot_.reduced_model_.nv;
    tau_g.resize(nv);
    // HARDWARE FIDELITY (2026-08-02): Pinocchio RNEA from the URDF is the ONLY
    // gravity source. On real hardware there is no MuJoCo, so relying on external
    // gravity here would tune the sim to a signal that does not exist on the robot.
    // The URDF is patched to match the MJCF mass distribution (config/
    // g1_29dof_inspire.urdf — 5010 wrist masses + Inspire hand lumped into the yaw
    // link, native right_rubber_hand zeroed), so Pinocchio == MuJoCo exactly
    // (verified: 7/7 joints diff = 0.000). external_gravity_ (MuJoCo) is kept only
    // for the [ARM] grav-diff diagnostic.
    kin_.computeGravityTorque(tau_g);
}

// ============================================================================
// Gravity diagnostic — max |Pinocchio − MuJoCo| over all joints
// ============================================================================

double PIDController::gravityDiff() const
{
    if (!hasExternalGravity()) return -1.0;
    Eigen::VectorXd tau_g;
    kin_.computeGravityTorque(tau_g);
    return (tau_g - external_gravity_).cwiseAbs().maxCoeff();
}

// ============================================================================
// Gravity hold — arm torque when there is no target yet (prevents free-fall)
// ============================================================================

ControlResult PIDController::computeGravityHold(
    const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
{
    const int n_arm = static_cast<int>(robot_.arm_joint_indices_.size());
    const int nv = robot_.reduced_model_.nv;

    if (q.size() == nv) robot_.q_ = q;
    robot_.updateKinematics();

    Eigen::VectorXd tau_g = Eigen::VectorXd::Zero(nv);
    if (gravity_comp_) resolveGravityTorque(tau_g);

    Eigen::VectorXd tau_cmd(n_arm);
    for (int k = 0; k < n_arm; ++k) {
        int i = robot_.arm_joint_indices_[k];
        double dq_i = (k < dq.size()) ? dq(i) : 0.0;
        // 姿态吸引：无目标时也把手臂拉回就绪姿态（posture_target，与桥接
        // ready pose 一致）。否则重力保持只是"停在当前位置"——启动瞬间
        // 若控制器接管前已下垂/被扰动，手臂会停在错误位置不回来。
        // 强度与 computeArm 扭矩模式的 posture 项一致（posture_gain×8 N·m/rad）。
        double q_err = posture_target_(i) - robot_.q_(i);
        tau_cmd(k) = tau_g(i) + posture_gain_ * 8.0 * q_err - inner_servo_kv_ * dq_i;
        tau_cmd(k) = std::max(-25.0, std::min(25.0, tau_cmd(k)));
    }

    auto result = ControlResult::success_result(robot_.q_,
        robot_.getWristPosition(), 0.0);
    result.torque_command = tau_cmd;
    return result;
}

// ============================================================================
// Arm IK — 4-DOF → 3-DOF wrist origin position
// ============================================================================

ControlResult PIDController::computeArm(
    const Eigen::Vector3d& target_pos,
    const Eigen::VectorXd& current_q,
    const Eigen::VectorXd& current_dq,
    double dt,
    const Eigen::Vector3d* external_error)
{
    // Sync robot state
    if (current_q.size() == robot_.reduced_model_.nq) {
        robot_.q_ = current_q;
    }

    const int n_arm = static_cast<int>(robot_.arm_joint_indices_.size());
    const int nv = robot_.reduced_model_.nv;

    // --- Position error ---
    Eigen::Vector3d error_pos;
    if (external_error) {
        error_pos = *external_error;
    } else {
        robot_.updateKinematics();
        Eigen::Vector3d ee_pos = robot_.getWristPosition();
        error_pos = target_pos - ee_pos;
    }
    double position_error = error_pos.norm();

    // Update FK for EE position reporting
    robot_.updateKinematics();
    Eigen::Vector3d ee_pos = robot_.getWristPosition();

    // Already converged
    if (position_error <= tolerance_) {
        error_integral_.setZero();
        if (torque_mode_) {
            // TORQUE mode: keep sending gravity compensation + damping (no early stop,
            // otherwise the arm collapses under gravity when the position converges).
            auto result = ControlResult::success_result(robot_.q_, ee_pos, position_error);
            Eigen::VectorXd tau_g_full = Eigen::VectorXd::Zero(nv);
            if (gravity_comp_) resolveGravityTorque(tau_g_full);
            Eigen::VectorXd tau_cmd(n_arm);
            for (int k = 0; k < n_arm; ++k) {
                int i = robot_.arm_joint_indices_[k];
                double dq_i = (k < current_dq.size()) ? current_dq(i) : 0.0;
                tau_cmd(k) = tau_g_full(i) - inner_servo_kv_ * dq_i;
                tau_cmd(k) = std::max(-25.0, std::min(25.0, tau_cmd(k)));
            }
            result.torque_command = tau_cmd;
            return result;
        }
        return ControlResult::success_result(robot_.q_, ee_pos, position_error);
    }

    // Divergence guard
    if (!first_call_ && position_error > prev_error_pos_.norm() * 5.0) {
        error_integral_.setZero();
    }

    // --- Compute arm position Jacobian (3×4) ---
    Eigen::Matrix<double, 3, Eigen::Dynamic> J_arm(3, n_arm);
    kin_.computeArmPositionJacobian(J_arm);

    // --- Variable Kp ---
    double eff_kp = kp_;
    if (position_error < 0.05) {
        double ratio = position_error / 0.05;
        eff_kp = kp_ * (0.6 + 0.4 * ratio);
    }

    // --- Integral with anti-windup ---
    // ROOT-CAUSE FIX (2026-08-02): the old gate (err<0.30) kept accumulating even
    // after the arm reached ~0.03 m. The integral saturated at i_max=0.5/axis, so
    // its force (ki·∫e ≈ 1.3 N) dwarfed the P term (kp·e ≈ 0.3 N at 4 cm) and,
    // leaking through inertia coupling into the null space, drove a slow steady
    // drift (ro/ya/elb → limits) that the 5%-per-cycle divergence guard never
    // caught. New rules: integrate ONLY in the near-target band AND while the
    // error is still shrinking; zero the instant the arm starts moving away.
    // ±2% jitter tolerance: a genuinely shrinking error still integrates (so the
    // I term can trim a steady-state gravity residual), while real growth resets.
    if (position_error < 0.08 && position_error <= prev_error_pos_.norm() * 1.02) {
        error_integral_ += error_pos * dt;
    } else {
        error_integral_.setZero();
    }
    double i_max = 0.15;   // max integral force = ki·0.15 — a trim, not a driver
    for (int i = 0; i < 3; ++i)
        error_integral_(i) = std::max(-i_max, std::min(i_max, error_integral_(i)));

    // --- Task velocity ---
    Eigen::Vector3d task_vel = eff_kp * error_pos + ki_ * error_integral_;
    if (!first_call_)
        task_vel += kd_ * (error_pos - prev_error_pos_) / dt;
    double task_speed = task_vel.norm();
    if (task_speed > 1.0) task_vel *= 1.0 / task_speed;

    // --- Adaptive damping ---
    Eigen::Matrix3d JJt = J_arm * J_arm.transpose();
    double adaptive_damp = std::max(damping_, 1e-3 / (1.0 + std::abs(JJt.determinant()) * 1e5));

    // --- Damped pseudoinverse (3×4) ---
    Eigen::Matrix<double, Eigen::Dynamic, 3> J_pinv(n_arm, 3);
    JJt.diagonal().array() += adaptive_damp;
    J_pinv = J_arm.transpose() * JJt.ldlt().solve(Eigen::Matrix3d::Identity());

    // --- Null-space (1 DOF: 4-3=1) ---
    Eigen::VectorXd dq_null = jointLimitAvoidanceArm(0.1);
    // Posture attraction: only for arm joints
    for (int k = 0; k < n_arm; ++k) {
        int i = robot_.arm_joint_indices_[k];
        dq_null(k) -= posture_gain_ * (robot_.q_(i) - posture_target_(i));
    }
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n_arm, n_arm) - J_pinv * J_arm;
    Eigen::VectorXd dq_arm = J_pinv * task_vel + N * dq_null;

    // --- Velocity clamping ---
    double max_joint_vel = 6.0;
    for (int k = 0; k < n_arm; ++k)
        max_joint_vel = std::min(max_joint_vel, robot_.dq_max_(robot_.arm_joint_indices_[k]));
    double dq_norm = dq_arm.lpNorm<Eigen::Infinity>();
    double scale = dq_norm / max_joint_vel;
    if (scale > 1.0) dq_arm /= scale;
    for (int k = 0; k < n_arm; ++k)
        dq_arm(k) = std::max(-max_joint_vel, std::min(max_joint_vel, dq_arm(k)));

    // --- Back-calculation anti-windup ---
    if (dq_norm > 1e-6) {
        double clamped_norm = dq_arm.lpNorm<Eigen::Infinity>();
        if (clamped_norm < dq_norm * 0.8)
            error_integral_ *= clamped_norm / dq_norm;
    }

    // --- Joint-limit direction clamp ---
    for (int k = 0; k < n_arm; ++k) {
        int i = robot_.arm_joint_indices_[k];
        if (robot_.q_(i) <= robot_.q_min_(i) && dq_arm(k) < 0.0) dq_arm(k) = 0.0;
        if (robot_.q_(i) >= robot_.q_max_(i) && dq_arm(k) > 0.0) dq_arm(k) = 0.0;
    }

    // --- Integrate arm joints (direct addition — 1-DOF revolute joints) ---
    for (int k = 0; k < n_arm; ++k) {
        int i = robot_.arm_joint_indices_[k];
        robot_.q_(i) = current_q(i) + dq_arm(k) * dt;
    }

    robot_.clampToLimits();

    // Per-cycle delta clamp
    for (int k = 0; k < n_arm; ++k) {
        int i = robot_.arm_joint_indices_[k];
        double delta = robot_.q_(i) - current_q(i);
        double max_delta = 0.25;
        if (delta > max_delta)       robot_.q_(i) = current_q(i) + max_delta;
        else if (delta < -max_delta) robot_.q_(i) = current_q(i) - max_delta;
    }

    // --- Torque command (TORQUE mode): τ = Jᵀ·F + τ_g − Kd·dq, per-joint clamped ---
    Eigen::VectorXd tau_cmd(n_arm);
    if (torque_mode_) {
        Eigen::Vector3d task_force = eff_kp * error_pos + ki_ * error_integral_;
        if (!first_call_)
            task_force += kd_ * (error_pos - prev_error_pos_) / dt;
        double f_norm = task_force.norm();
        if (f_norm > 50.0) task_force *= 50.0 / f_norm;  // max task force (N)
        Eigen::VectorXd tau_task = J_arm.transpose() * task_force;   // Jᵀ·F
        Eigen::VectorXd tau_g_full = Eigen::VectorXd::Zero(nv);
        if (gravity_comp_) resolveGravityTorque(tau_g_full);

        // --- Null-space posture attraction (TORQUE mode) ---
        // Without it, the arm reaches via whatever null-space solution the
        // pseudoinverse picks — often contorting the elbow to its negative limit
        // instead of extending forward naturally. Projected through the null space
        // so it never fights the task force.
        Eigen::MatrixXd N_null = Eigen::MatrixXd::Identity(n_arm, n_arm) - J_pinv * J_arm;
        Eigen::VectorXd tau_posture = Eigen::VectorXd::Zero(n_arm);
        for (int k = 0; k < n_arm; ++k) {
            int i = robot_.arm_joint_indices_[k];
            double q_err = posture_target_(i) - robot_.q_(i);
            tau_posture(k) = posture_gain_ * 8.0 * q_err;
        }
        tau_posture = N_null * tau_posture;

        for (int k = 0; k < n_arm; ++k) {
            int i = robot_.arm_joint_indices_[k];
            double dq_i = (k < current_dq.size()) ? current_dq(i) : 0.0;
            tau_cmd(k) = tau_task(k) + tau_g_full(i) + tau_posture(k) - inner_servo_kv_ * dq_i;
            tau_cmd(k) = std::max(-25.0, std::min(25.0, tau_cmd(k)));  // shoulder/elbow limit
        }
    }

    prev_error_pos_ = error_pos;
    first_call_ = false;

    auto result = ControlResult::success_result(robot_.q_, ee_pos, position_error);
    result.effective_kp = eff_kp;
    result.integral_norm = error_integral_.norm();
    result.task_velocity = task_speed;
    result.adaptive_damping = adaptive_damp;
    if (torque_mode_) result.torque_command = tau_cmd;
    result.nullspace_norm = dq_null.norm();
    result.max_joint_vel = max_joint_vel;
    result.dq_command = dq_arm;
    return result;
}

// ============================================================================
// Joint-limit avoidance — arm joints only
// ============================================================================

Eigen::VectorXd PIDController::jointLimitAvoidanceArm(double margin) const
{
    const int n_arm = static_cast<int>(robot_.arm_joint_indices_.size());
    Eigen::VectorXd dq_repulse = Eigen::VectorXd::Zero(n_arm);

    for (int k = 0; k < n_arm; ++k) {
        int i = robot_.arm_joint_indices_[k];
        double q = robot_.q_(i);
        double q_min = robot_.q_min_(i);
        double q_max = robot_.q_max_(i);
        double range = q_max - q_min;
        if (range < 2.0 * margin) continue;

        double dist_lower = q - q_min;
        if (dist_lower < margin) {
            double s = (margin - dist_lower) / margin;
            dq_repulse(k) += s * s * 2.0;
        }
        double dist_upper = q_max - q;
        if (dist_upper < margin) {
            double s = (margin - dist_upper) / margin;
            dq_repulse(k) -= s * s * 2.0;
        }
    }
    return dq_repulse;
}

// ============================================================================
// Reset
// ============================================================================

void PIDController::reset()
{
    prev_error_pos_.setZero();
    error_integral_.setZero();
    first_call_ = true;
}

}  // namespace g1_right_arm_reach
