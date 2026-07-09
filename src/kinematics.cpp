/**
 * @file kinematics.cpp
 *
 * Implements the Kinematics class: numerical position Jacobian (central
 * differences), Lie-group integration, damped pseudoinverse, and null-space
 * projector — the building blocks for resolved-rate IK and PID control.
 */

#include "g1_right_arm_reach/kinematics.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <iostream>

namespace g1_right_arm_reach {

// ============================================================================
// MODULE: Constructor / Initialization
// ============================================================================

// Store a reference to the RobotModel used by all kinematic computations.
Kinematics::Kinematics(RobotModel& robot)
    : robot_(robot)
{
}

// ============================================================================
// MODULE: Jacobian Computation
// ============================================================================

// Compute the 3xnv position Jacobian via central-difference numerical
// differentiation on the REDUCED model.
//
// Pinocchio's getFrameJacobian returns zeros on the G1 URDF with the
// ROS2 Jazzy Pinocchio package — likely a buildReducedModel bug.
// Numerical differentiation is reliable because FK works correctly.
//
// 1. Save current joint state
// 2. Run FK at current q to get baseline EE position p0
// 3. For each joint i: perturb +eps, FK, perturb -eps, FK
// 4. Compute central-difference column: (p_plus - p_minus) / (2*eps)
// 5. Restore original joint state
void Kinematics::computePositionJacobian(
    Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos)
{
    const int nv = robot_.reduced_model_.nv;   // 5
    const double eps = 1e-6;

    // Save current state
    Eigen::VectorXd q_save = robot_.q_;

    // FK at current q
    robot_.updateKinematics();
    Eigen::Vector3d p0 = robot_.getEndEffectorPosition();

    for (int i = 0; i < nv; ++i) {
        // Positive perturbation
        robot_.q_ = q_save;
        robot_.q_(i) += eps;
        robot_.updateKinematics();
        Eigen::Vector3d p_plus = robot_.getEndEffectorPosition();

        // Negative perturbation
        robot_.q_ = q_save;
        robot_.q_(i) -= eps;
        robot_.updateKinematics();
        Eigen::Vector3d p_minus = robot_.getEndEffectorPosition();

        // Central difference: dp/dq_i ≈ (p(q+ε) - p(q-ε)) / (2ε)
        J_pos.col(i) = (p_plus - p_minus) / (2.0 * eps);
    }

    // Restore
    robot_.q_ = q_save;
    robot_.updateKinematics();

    // One-time log
    static bool once = false;
    if (!once) {
        once = true;
        std::cout << "[Kinematics] Using NUMERICAL Jacobian. |J|=" << J_pos.norm()
                  << " J=[" << std::endl << J_pos << "]" << std::endl;
    }
}

// Compute the 6xnv full Jacobian (linear + angular) using Pinocchio's
// analytical getFrameJacobian.  Caller must zero-initialize J_full;
// this function will NOT zero it — otherwise results may be corrupted
// for models where getFrameJacobian only fills part of the matrix.
void Kinematics::computeFullJacobian(
    Eigen::Matrix<double, 6, Eigen::Dynamic>& J_full,
    pinocchio::ReferenceFrame ref_frame)
{
    // CRITICAL: Zero-initialize!
    J_full.setZero();

    pinocchio::getFrameJacobian(
        robot_.reduced_model_,
        robot_.reduced_data_,
        robot_.ee_frame_id_,
        ref_frame,
        J_full);
}

// ============================================================================
// MODULE: Kinematic Utilities
// ============================================================================

// Apply a joint-velocity step using Pinocchio's Lie-group integration.
// Direct Euler (q += dq * dt) is incorrect for angular joints; the
// accumulate operator handles SO(3)/SE(3) topology correctly.
void Kinematics::integrate(const Eigen::VectorXd& dq, double dt)
{
    // NEVER do: q += dq * dt  (this is wrong for angular joints)
    // Always use Pinocchio's Lie-group integration:
    robot_.q_ = pinocchio::integrate(
        robot_.reduced_model_,
        robot_.q_,
        dq * dt);
}

// Compute the damped pseudoinverse of the 3xnv position Jacobian:
//   J_pinv = J^T * (J*J^T + lambda*I)^{-1}
// Damping (lambda) prevents singularities when J*J^T is ill-conditioned.
// For the redundant arm (nv > 3), the 3x3 system is solved via LDLT.
void Kinematics::dampedPseudoInverse(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos,
    double damping,
    Eigen::Matrix<double, Eigen::Dynamic, 3>& J_pinv) const
{
    const int nv = static_cast<int>(J_pos.cols());

    // J * J^T + lambda * I
    Eigen::Matrix3d JJt = J_pos * J_pos.transpose();
    JJt.diagonal().array() += damping;

    // J_pinv = J^T * (J*J^T + lambda*I)^{-1}
    // For nv > 3 (redundant arm): use LDLT solver
    J_pinv = J_pos.transpose() * JJt.ldlt().solve(Eigen::Matrix3d::Identity());
}

// Compute the null-space projector: N = I - J_pinv * J
// Multiplying a joint-velocity vector by N projects it into the null space
// of the Jacobian, so it produces no end-effector motion (used for
// secondary tasks like joint-limit avoidance).
void Kinematics::nullSpaceProjector(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& J_pos,
    double damping,
    Eigen::MatrixXd& N) const
{
    const int nv = static_cast<int>(J_pos.cols());

    Eigen::Matrix<double, Eigen::Dynamic, 3> J_pinv(nv, 3);
    dampedPseudoInverse(J_pos, damping, J_pinv);

    // N = I - J^+ * J
    N = Eigen::MatrixXd::Identity(nv, nv) - J_pinv * J_pos;
}

}  // namespace g1_right_arm_reach
