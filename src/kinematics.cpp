/**
 * @file kinematics.cpp
 * Numerical position Jacobian, damped pseudoinverse, and Lie-group integration.
 */

#include "g1_right_arm_reach/kinematics.hpp"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <iostream>

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

    static bool once = false;
    if (!once) {
        once = true;
        std::cout << "[Kinematics] Numerical Jacobian, |J|=" << J_pos.norm() << "\n";
    }
}

// ============================================================================
// Configuration integration
// ============================================================================

void Kinematics::integrate(const Eigen::VectorXd& dq, double dt)
{
    robot_.q_ = pinocchio::integrate(robot_.reduced_model_, robot_.q_, dq * dt);
}

// ============================================================================
// Damped pseudoinverse
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

}  // namespace g1_right_arm_reach
