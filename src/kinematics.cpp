/**
 * @file kinematics.cpp
 * @brief Numerical Jacobians (subset + full), damped pseudoinverse, gravity torque.
 *
 * Arm-Hand decoupling: computeArmPositionJacobian (3×4) and
 * computeWristOrientationJacobian (3×N) perturb only the relevant joint subsets,
 * enabling independent position and orientation control.
 */

#include "g1_right_arm_reach/kinematics.hpp"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <iostream>
#include <cmath>

namespace g1_right_arm_reach {

// ============================================================================
// Constructor
// ============================================================================

Kinematics::Kinematics(RobotModel& robot) : robot_(robot) {}

// ============================================================================
// Position Jacobian (numerical, central differences)
// ============================================================================

void Kinematics::computePositionJacobian(Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos)
{
    const int nv = robot_.reduced_model_.nv;
    const double eps = 1e-5;

    Eigen::VectorXd q_save = robot_.q_;
    robot_.updateKinematics();
    Eigen::Vector3d p0 = robot_.getEndEffectorPosition();

    for (int i = 0; i < nv; ++i) {
        robot_.q_ = q_save;  robot_.q_(i) += eps;
        robot_.updateKinematics();
        Eigen::Vector3d p_plus = robot_.getEndEffectorPosition();

        robot_.q_ = q_save;  robot_.q_(i) -= eps;
        robot_.updateKinematics();
        Eigen::Vector3d p_minus = robot_.getEndEffectorPosition();

        J_pos.col(i) = (p_plus - p_minus) / (2.0 * eps);
    }

    robot_.q_ = q_save;
    robot_.updateKinematics();
}

// ============================================================================
// Full 6×N Jacobian (position + angular velocity, world frame)
// ============================================================================

void Kinematics::computeFullJacobian(Eigen::Matrix<double, 6, Eigen::Dynamic>& J_full)
{
    const int nv = robot_.reduced_model_.nv;
    const double eps = 1e-5;

    Eigen::VectorXd q_save = robot_.q_;
    robot_.updateKinematics();

    Eigen::Vector3d p0 = robot_.getEndEffectorPosition();
    Eigen::Quaterniond q0 = getEEOrientation();

    for (int i = 0; i < nv; ++i) {
        robot_.q_ = q_save;  robot_.q_(i) += eps;
        robot_.updateKinematics();
        Eigen::Vector3d p_plus = robot_.getEndEffectorPosition();
        Eigen::Quaterniond q_plus = getEEOrientation();

        robot_.q_ = q_save;  robot_.q_(i) -= eps;
        robot_.updateKinematics();
        Eigen::Vector3d p_minus = robot_.getEndEffectorPosition();
        Eigen::Quaterniond q_minus = getEEOrientation();

        J_full.block<3, 1>(0, i) = (p_plus - p_minus) / (2.0 * eps);

        Eigen::Quaterniond dq_ori = q_plus * q_minus.inverse();
        dq_ori.normalize();
        Eigen::AngleAxisd aa(dq_ori);
        Eigen::Vector3d omega = aa.axis() * aa.angle() / (2.0 * eps);
        J_full.block<3, 1>(3, i) = omega;
    }

    robot_.q_ = q_save;
    robot_.updateKinematics();
}

// ============================================================================
// Arm position Jacobian (3×4): wrist origin position w.r.t arm joints only
// ============================================================================

void Kinematics::computeArmPositionJacobian(Eigen::Matrix<double, 3, Eigen::Dynamic>& J_arm)
{
    const int n_arm = static_cast<int>(robot_.arm_joint_indices_.size());
    const double eps = 1e-5;

    J_arm.resize(3, n_arm);

    Eigen::VectorXd q_save = robot_.q_;
    robot_.updateKinematics();

    for (int k = 0; k < n_arm; ++k) {
        int i = robot_.arm_joint_indices_[k];

        robot_.q_ = q_save;  robot_.q_(i) += eps;
        robot_.updateKinematics();
        Eigen::Vector3d p_plus = robot_.getWristPosition();

        robot_.q_ = q_save;  robot_.q_(i) -= eps;
        robot_.updateKinematics();
        Eigen::Vector3d p_minus = robot_.getWristPosition();

        J_arm.col(k) = (p_plus - p_minus) / (2.0 * eps);
    }

    robot_.q_ = q_save;
    robot_.updateKinematics();
}

// ============================================================================
// Wrist orientation Jacobian (3×3): hand orientation w.r.t. wrist joints
// ============================================================================

void Kinematics::computeWristOrientationJacobian(Eigen::Matrix<double, 3, Eigen::Dynamic>& J_wrist)
{
    const int n_wrist = static_cast<int>(robot_.wrist_joint_indices_.size());
    const double eps = 1e-4;  // slightly larger for orientation stability

    J_wrist.resize(3, n_wrist);

    Eigen::VectorXd q_save = robot_.q_;
    robot_.updateKinematics();

    for (int k = 0; k < n_wrist; ++k) {
        int i = robot_.wrist_joint_indices_[k];

        robot_.q_ = q_save;  robot_.q_(i) += eps;
        robot_.updateKinematics();
        Eigen::Quaterniond q_plus(robot_.getHandOrientation());

        robot_.q_ = q_save;  robot_.q_(i) -= eps;
        robot_.updateKinematics();
        Eigen::Quaterniond q_minus(robot_.getHandOrientation());

        Eigen::Quaterniond dq = q_plus * q_minus.inverse();
        dq.normalize();
        Eigen::AngleAxisd aa(dq);
        J_wrist.col(k) = aa.axis() * aa.angle() / (2.0 * eps);
    }

    robot_.q_ = q_save;
    robot_.updateKinematics();
}

// ============================================================================
// Gravity torque (Pinocchio RNEA: τ_g = g(q) for reduced model)
// ============================================================================

void Kinematics::computeGravityTorque(Eigen::VectorXd& tau_g)
{
    const int nv = robot_.reduced_model_.nv;
    tau_g.resize(nv);

    tau_g = pinocchio::computeGeneralizedGravity(
        robot_.reduced_model_, robot_.reduced_data_, robot_.q_);
}

// ============================================================================
// EE Orientation (quaternion, xyzw)
// ============================================================================

Eigen::Quaterniond Kinematics::getEEOrientation() const
{
    const auto& R = robot_.reduced_data_.oMf[robot_.ee_frame_id_].rotation();
    return Eigen::Quaterniond(R);
}

// ============================================================================
// Right shoulder position (world frame)
// ============================================================================

Eigen::Vector3d Kinematics::getRightShoulderPosition() const
{
    pinocchio::JointIndex shoulder_jid = 1;
    if (shoulder_jid < robot_.reduced_model_.joints.size()) {
        return robot_.reduced_data_.oMi[shoulder_jid].translation();
    }
    return Eigen::Vector3d::Zero();
}

// ============================================================================
// Orientation error (axis-angle, world frame)
// ============================================================================

Eigen::Vector3d Kinematics::orientationError(const Eigen::Quaterniond& q_des,
                                              const Eigen::Quaterniond& q_curr)
{
    Eigen::Quaterniond q_err = q_des * q_curr.inverse();
    q_err.normalize();
    Eigen::AngleAxisd aa(q_err);
    return aa.axis() * aa.angle();
}

// ============================================================================
// Configuration integration
// ============================================================================

void Kinematics::integrate(const Eigen::VectorXd& dq, double dt)
{
    robot_.q_ = pinocchio::integrate(robot_.reduced_model_, robot_.q_, dq * dt);
}

// ============================================================================
// Damped pseudoinverse (3×N)
// ============================================================================

void Kinematics::dampedPseudoInverse(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos,
    double damping,
    Eigen::Matrix<double, Eigen::Dynamic, 3>& J_pinv) const
{
    Eigen::Matrix3d JJt = J_pos * J_pos.transpose();
    JJt.diagonal().array() += damping;
    J_pinv = J_pos.transpose() * JJt.ldlt().solve(Eigen::Matrix3d::Identity());
}

// ============================================================================
// Damped pseudoinverse (6×N)
// ============================================================================

void Kinematics::dampedPseudoInverse6D(
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& J_full,
    double damping,
    Eigen::Matrix<double, Eigen::Dynamic, 6>& J_pinv) const
{
    Eigen::Matrix<double, 6, 6> JJt = J_full * J_full.transpose();
    JJt.diagonal().array() += damping;
    J_pinv = J_full.transpose() * JJt.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
}

}  // namespace g1_right_arm_reach
