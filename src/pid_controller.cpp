/**
 * @file pid_controller.cpp
 * @brief Arm-Hand decoupled PID + IK: computeArm (4-DOF→3-DOF) + computeWrist (3-DOF→3-DOF).
 *
 * computeArm:    3×4 damped pseudoinverse + null-space posture attraction
 * computeWrist:  3×N auto-adapting solver (N=1 damped pinv, N=3 direct solve)
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
    if (position_error < 0.20) {
        error_integral_ += error_pos * dt;
    }
    double i_max = 0.10;
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

    prev_error_pos_ = error_pos;
    first_call_ = false;

    auto result = ControlResult::success_result(robot_.q_, ee_pos, position_error);
    result.effective_kp = eff_kp;
    result.integral_norm = error_integral_.norm();
    result.task_velocity = task_speed;
    result.adaptive_damping = adaptive_damp;
    result.nullspace_norm = dq_null.norm();
    result.max_joint_vel = max_joint_vel;
    result.dq_command = dq_arm;
    return result;
}

// ============================================================================
// Wrist solver — 3-DOF → 3-DOF hand orientation
// ============================================================================

ControlResult PIDController::computeWrist(
    const Eigen::Quaterniond& target_ori,
    const Eigen::VectorXd& current_q,
    const Eigen::VectorXd& current_dq,
    double dt)
{
    // Sync robot state
    if (current_q.size() == robot_.reduced_model_.nq) {
        robot_.q_ = current_q;
    }

    const int n_wrist = static_cast<int>(robot_.wrist_joint_indices_.size());
    const int nv = robot_.reduced_model_.nv;

    // --- Orientation error ---
    robot_.updateKinematics();
    Eigen::Quaterniond hand_ori(robot_.getHandOrientation());
    Eigen::Vector3d error_ori = Kinematics::orientationError(target_ori, hand_ori);
    double orientation_error = error_ori.norm();

    // Already converged (0.05 rad ≈ 3°)
    if (orientation_error < 0.05) {
        ori_integral_.setZero();
        ControlResult r;
        r.success = true;
        r.command = robot_.q_;
        r.orientation_error = orientation_error;
        return r;
    }

    // --- Compute wrist orientation Jacobian (3 × N_wrist) ---
    Eigen::Matrix<double, 3, Eigen::Dynamic> J_wrist(3, n_wrist);
    kin_.computeWristOrientationJacobian(J_wrist);

    // --- Orientation task velocity ---
    Eigen::Vector3d task_vel_ori = kp_wrist_ * error_ori;
    if (!first_call_wrist_)
        task_vel_ori += kd_wrist_ * (error_ori - prev_error_ori_) / dt;
    double ori_speed = task_vel_ori.norm();
    if (ori_speed > 3.0) task_vel_ori *= 3.0 / ori_speed;

    // --- Solve for wrist joint velocities ---
    //   3×3 (full 3-DOF wrist): direct solve
    //   3×N (N=1,2; under-actuated): damped pseudoinverse
    Eigen::VectorXd dq_wrist(n_wrist);
    if (n_wrist == 3) {
        // Square Jacobian — direct solve
        double det = J_wrist.determinant();
        if (std::abs(det) > 1e-6) {
            dq_wrist = J_wrist.colPivHouseholderQr().solve(task_vel_ori);
        } else {
            // Singular fallback
            Eigen::Matrix3d JJt = J_wrist * J_wrist.transpose();
            JJt.diagonal().array() += 1e-3;
            dq_wrist = J_wrist.transpose() * JJt.ldlt().solve(task_vel_ori);
        }
    } else {
        // Non-square — damped pseudoinverse (N < 3, under-actuated)
        Eigen::Matrix3d JJt = J_wrist * J_wrist.transpose();
        JJt.diagonal().array() += 5e-4;
        dq_wrist = J_wrist.transpose() * JJt.ldlt().solve(task_vel_ori);
    }

    // --- Velocity clamping ---
    double max_joint_vel = 6.0;
    for (int k = 0; k < n_wrist; ++k)
        max_joint_vel = std::min(max_joint_vel, robot_.dq_max_(robot_.wrist_joint_indices_[k]));
    double dq_norm = dq_wrist.lpNorm<Eigen::Infinity>();
    double scale = dq_norm / max_joint_vel;
    if (scale > 1.0) dq_wrist /= scale;
    for (int k = 0; k < n_wrist; ++k)
        dq_wrist(k) = std::max(-max_joint_vel, std::min(max_joint_vel, dq_wrist(k)));

    // --- Joint-limit direction clamp ---
    for (int k = 0; k < n_wrist; ++k) {
        int i = robot_.wrist_joint_indices_[k];
        if (robot_.q_(i) <= robot_.q_min_(i) && dq_wrist(k) < 0.0) dq_wrist(k) = 0.0;
        if (robot_.q_(i) >= robot_.q_max_(i) && dq_wrist(k) > 0.0) dq_wrist(k) = 0.0;
    }

    // --- Integrate wrist joints only (direct addition for revolute joints) ---
    for (int k = 0; k < n_wrist; ++k) {
        int i = robot_.wrist_joint_indices_[k];
        robot_.q_(i) = current_q(i) + dq_wrist(k) * dt;
    }

    robot_.clampToLimits();

    // Per-cycle delta clamp
    for (int k = 0; k < n_wrist; ++k) {
        int i = robot_.wrist_joint_indices_[k];
        double delta = robot_.q_(i) - current_q(i);
        double max_delta = 0.25;
        if (delta > max_delta)       robot_.q_(i) = current_q(i) + max_delta;
        else if (delta < -max_delta) robot_.q_(i) = current_q(i) - max_delta;
    }

    prev_error_ori_ = error_ori;
    first_call_wrist_ = false;

    ControlResult r;
    r.success = true;
    r.command = robot_.q_;
    r.orientation_error = orientation_error;
    r.task_velocity = ori_speed;
    r.max_joint_vel = dq_wrist.lpNorm<Eigen::Infinity>();
    return r;
}

// ============================================================================
// Legacy: 3-DOF position-only control
// ============================================================================

ControlResult PIDController::compute(
    const Eigen::Vector3d& target,
    const Eigen::VectorXd& current_q,
    const Eigen::VectorXd& current_dq,
    double dt,
    const Eigen::Vector3d* external_error,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>* external_jacobian)
{
    return computeImpl(target, nullptr, current_q, current_dq, dt,
                       external_error, external_jacobian, nullptr);
}

// ============================================================================
// Legacy: 6-DOF pose control
// ============================================================================

ControlResult PIDController::computePose(
    const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond& target_ori,
    const Eigen::VectorXd& current_q,
    const Eigen::VectorXd& current_dq,
    double dt,
    const Eigen::Vector3d* external_error,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>* external_jacobian_6d)
{
    return computeImpl(target_pos, &target_ori, current_q, current_dq, dt,
                       external_error, nullptr, external_jacobian_6d);
}

// ============================================================================
// Legacy: Core implementation (unchanged, for backward compatibility)
// ============================================================================

ControlResult PIDController::computeImpl(
    const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond* target_ori,
    const Eigen::VectorXd& current_q,
    const Eigen::VectorXd& current_dq,
    double dt,
    const Eigen::Vector3d* external_error,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>* external_jacobian_3d,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>* external_jacobian_6d)
{
    if (current_q.size() == robot_.reduced_model_.nq) {
        robot_.q_ = current_q;
    }

    const int nv = robot_.reduced_model_.nv;
    bool has_orientation = (target_ori != nullptr);

    Eigen::Vector3d error_pos;
    Eigen::Vector3d ee_pos;
    if (external_error) {
        error_pos = *external_error;
        robot_.updateKinematics();
        ee_pos = robot_.getEndEffectorPosition();
    } else {
        robot_.updateKinematics();
        ee_pos = robot_.getEndEffectorPosition();
        error_pos = target_pos - ee_pos;
    }
    double position_error = error_pos.norm();

    Eigen::Vector3d error_ori = Eigen::Vector3d::Zero();
    double orientation_error = 0.0;
    if (has_orientation) {
        Eigen::Quaterniond ee_ori = kin_.getEEOrientation();
        error_ori = Kinematics::orientationError(*target_ori, ee_ori);
        orientation_error = error_ori.norm();
    }

    if (position_error <= tolerance_ && (!has_orientation || orientation_error < 0.05)) {
        error_integral_.setZero();
        auto result = ControlResult::success_result(robot_.q_, ee_pos, position_error);
        result.orientation_error = orientation_error;
        return result;
    }

    if (!first_call_ && position_error > prev_error_pos_.norm() * 5.0) {
        error_integral_.setZero();
    }

    Eigen::Matrix<double, 6, Eigen::Dynamic> J_full(6, nv);
    Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos(3, nv);

    if (has_orientation && external_jacobian_6d && external_jacobian_6d->cols() == nv) {
        J_full = *external_jacobian_6d;
        J_pos = J_full.topRows<3>();
    } else if (!has_orientation && external_jacobian_3d && external_jacobian_3d->cols() == nv) {
        J_pos = *external_jacobian_3d;
        if (has_orientation) {
            J_full.setZero();
            J_full.topRows<3>() = J_pos;
            kin_.computeFullJacobian(J_full);
            J_pos = J_full.topRows<3>();
        }
    } else if (has_orientation) {
        kin_.computeFullJacobian(J_full);
        J_pos = J_full.topRows<3>();
    } else {
        kin_.computePositionJacobian(J_pos);
    }

    double eff_kp = kp_;
    if (position_error < 0.05) {
        double ratio = position_error / 0.05;
        eff_kp = kp_ * (0.6 + 0.4 * ratio);
    }

    if (position_error < 0.20) {
        error_integral_ += error_pos * dt;
    }
    double i_max = 0.10;
    for (int i = 0; i < 3; ++i)
        error_integral_(i) = std::max(-i_max, std::min(i_max, error_integral_(i)));

    Eigen::Matrix<double, 6, 1> F_task = Eigen::Matrix<double, 6, 1>::Zero();
    F_task.head<3>() = eff_kp * error_pos + ki_ * error_integral_;
    if (!first_call_)
        F_task.head<3>() += kd_ * (error_pos - prev_error_pos_) / dt;

    if (has_orientation) {
        F_task.segment<3>(3) = kp_wrist_ * error_ori;
        if (!first_call_)
            F_task.segment<3>(3) += kd_wrist_ * (error_ori - prev_error_ori_) / dt;
    }

    double max_force = 50.0;
    double max_torque = 10.0;
    double force_norm = F_task.head<3>().norm();
    if (force_norm > max_force)
        F_task.head<3>() *= max_force / force_norm;
    if (has_orientation) {
        double ori_norm = F_task.segment<3>(3).norm();
        if (ori_norm > max_torque)
            F_task.segment<3>(3) *= max_torque / ori_norm;
    }

    Eigen::VectorXd tau_task(nv);
    if (has_orientation) {
        tau_task = J_full.transpose() * F_task;
    } else {
        tau_task = J_pos.transpose() * F_task.head<3>();
    }

    Eigen::VectorXd tau_gravity = Eigen::VectorXd::Zero(nv);
    if (gravity_comp_) {
        kin_.computeGravityTorque(tau_gravity);
    }

    Eigen::VectorXd tau_cmd = tau_task + tau_gravity;
    double max_joint_torque = 87.0;
    for (int i = 0; i < nv; ++i)
        tau_cmd(i) = std::max(-max_joint_torque, std::min(max_joint_torque, tau_cmd(i)));

    double adaptive_damp;
    if (has_orientation) {
        Eigen::Matrix<double, 6, 6> JJt = J_full * J_full.transpose();
        adaptive_damp = std::max(damping_, 1e-3 / (1.0 + std::abs(JJt.determinant()) * 1e5));
    } else {
        Eigen::Matrix3d JJt = J_pos * J_pos.transpose();
        adaptive_damp = std::max(damping_, 1e-3 / (1.0 + std::abs(JJt.determinant()) * 1e5));
    }

    Eigen::VectorXd dq_null = jointLimitAvoidance(0.1);
    dq_null -= posture_gain_ * (robot_.q_ - posture_target_);

    Eigen::VectorXd dq_ik(nv);

    if (has_orientation) {
        Eigen::Matrix<double, 6, 1> task_vel_6d;
        task_vel_6d.head<3>() = eff_kp * error_pos + ki_ * error_integral_;
        if (!first_call_)
            task_vel_6d.head<3>() += kd_ * (error_pos - prev_error_pos_) / dt;
        task_vel_6d.tail<3>() = kp_wrist_ * error_ori;
        if (!first_call_)
            task_vel_6d.tail<3>() += kd_wrist_ * (error_ori - prev_error_ori_) / dt;

        double pos_speed = task_vel_6d.head<3>().norm();
        if (pos_speed > 1.0) task_vel_6d.head<3>() *= 1.0 / pos_speed;
        double ori_speed = task_vel_6d.tail<3>().norm();
        if (ori_speed > 3.0) task_vel_6d.tail<3>() *= 3.0 / ori_speed;

        Eigen::Matrix<double, Eigen::Dynamic, 6> J_pinv_6d(nv, 6);
        kin_.dampedPseudoInverse6D(J_full, adaptive_damp, J_pinv_6d);
        dq_ik = J_pinv_6d * task_vel_6d;

        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(nv, nv) - J_pinv_6d * J_full;
        dq_ik += N * dq_null;
    } else {
        Eigen::Vector3d task_vel = eff_kp * error_pos + ki_ * error_integral_;
        if (!first_call_)
            task_vel += kd_ * (error_pos - prev_error_pos_) / dt;
        double task_speed = task_vel.norm();
        if (task_speed > 1.0) task_vel *= 1.0 / task_speed;

        Eigen::Matrix<double, Eigen::Dynamic, 3> J_pinv(nv, 3);
        kin_.dampedPseudoInverse(J_pos, adaptive_damp, J_pinv);
        dq_ik = J_pinv * task_vel;

        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(nv, nv) - J_pinv * J_pos;
        dq_ik += N * dq_null;
    }

    double max_joint_vel = 6.0;
    for (int i = 0; i < nv; ++i)
        max_joint_vel = std::min(max_joint_vel, robot_.dq_max_(i));
    double dq_unclamped_norm = dq_ik.lpNorm<Eigen::Infinity>();
    double scale_unclamped = dq_unclamped_norm / max_joint_vel;
    if (scale_unclamped > 1.0) dq_ik /= scale_unclamped;
    for (int i = 0; i < nv; ++i)
        dq_ik(i) = std::max(-max_joint_vel, std::min(max_joint_vel, dq_ik(i)));

    double dq_clamped_norm = dq_ik.lpNorm<Eigen::Infinity>();
    if (dq_unclamped_norm > 1e-6 && dq_clamped_norm < dq_unclamped_norm * 0.8) {
        double ratio = dq_clamped_norm / dq_unclamped_norm;
        error_integral_ *= ratio;
    }

    for (int i = 0; i < nv; ++i) {
        if (robot_.q_(i) <= robot_.q_min_(i) && dq_ik(i) < 0.0) dq_ik(i) = 0.0;
        if (robot_.q_(i) >= robot_.q_max_(i) && dq_ik(i) > 0.0) dq_ik(i) = 0.0;
    }

    if (torque_mode_) {
        kin_.integrate(dq_ik, dt);
        robot_.clampToLimits();
        Eigen::VectorXd q_ik = robot_.q_;
        for (int i = 0; i < nv; ++i) {
            double q_vel = (i < current_dq.size()) ? current_dq(i) : 0.0;
            double torque_offset = (tau_cmd(i) + inner_servo_kv_ * q_vel) / inner_servo_kp_;
            robot_.q_(i) = q_ik(i) + torque_offset;
        }
    } else {
        kin_.integrate(dq_ik, dt);
    }

    robot_.clampToLimits();

    for (int i = 0; i < nv; ++i) {
        double delta = robot_.q_(i) - current_q(i);
        double max_delta = 0.25;
        if (delta > max_delta)       robot_.q_(i) = current_q(i) + max_delta;
        else if (delta < -max_delta) robot_.q_(i) = current_q(i) - max_delta;
    }

    robot_.updateKinematics();
    ee_pos = robot_.getEndEffectorPosition();
    double urdf_position_error = (target_pos - ee_pos).norm();

    prev_error_pos_ = error_pos;
    prev_error_ori_ = error_ori;
    first_call_ = false;

    double reported_error = (external_error != nullptr)
        ? position_error : urdf_position_error;

    auto result = ControlResult::success_result(robot_.q_, ee_pos, reported_error);
    result.orientation_error = orientation_error;
    result.torque_command = tau_cmd;
    result.gravity_torque = tau_gravity;
    result.task_torque = tau_task;
    result.effective_kp = eff_kp;
    result.integral_norm = error_integral_.norm();
    result.task_velocity = (has_orientation ? F_task.norm() : force_norm);
    result.adaptive_damping = adaptive_damp;
    result.nullspace_norm = dq_null.norm();
    result.max_joint_vel = dq_ik.lpNorm<Eigen::Infinity>();
    result.dq_command = dq_ik;
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
// Joint-limit avoidance — all joints (legacy)
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
    prev_error_pos_.setZero();
    prev_error_ori_.setZero();
    error_integral_.setZero();
    ori_integral_.setZero();
    first_call_ = true;
    first_call_wrist_ = true;
}

}  // namespace g1_right_arm_reach
