#include "g1_right_arm_reach/kinematics.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_right_arm_reach {

namespace {
Eigen::Vector3d rotationVector(Eigen::Quaterniond delta)
{
    delta.normalize();
    if (delta.w() < 0.0) delta.coeffs() *= -1.0;
    const Eigen::AngleAxisd angle_axis(delta);
    return angle_axis.axis() * angle_axis.angle();
}
}  // namespace

Kinematics::Kinematics(RobotModel& robot, const Eigen::Vector3d& tool_axis_local)
    : robot_(robot)
{
    if (!tool_axis_local.allFinite() || tool_axis_local.norm() < 1.0e-9) {
        throw std::invalid_argument("tool_axis_local must be a finite non-zero vector");
    }
    tool_axis_local_ = tool_axis_local.normalized();
}

void Kinematics::computeTaskJacobian(
    Eigen::Matrix<double, 4, Eigen::Dynamic>& jacobian)
{
    const int n = static_cast<int>(robot_.arm_joint_indices_.size());
    constexpr double epsilon = 1.0e-6;
    jacobian.resize(4, n);

    const Eigen::VectorXd saved_q = robot_.q_;
    robot_.updateKinematics();
    const Eigen::Vector3d axis_world =
        (robot_.getHandOrientation() * tool_axis_local_).normalized();

    for (int column = 0; column < n; ++column) {
        const int index = robot_.arm_joint_indices_[column];

        robot_.q_ = saved_q;
        robot_.q_(index) += epsilon;
        robot_.updateKinematics();
        const Eigen::Vector3d position_plus = robot_.getEndEffectorPosition();
        const Eigen::Quaterniond orientation_plus(robot_.getHandOrientation());

        robot_.q_ = saved_q;
        robot_.q_(index) -= epsilon;
        robot_.updateKinematics();
        const Eigen::Vector3d position_minus = robot_.getEndEffectorPosition();
        const Eigen::Quaterniond orientation_minus(robot_.getHandOrientation());

        jacobian.template block<3, 1>(0, column) =
            (position_plus - position_minus) / (2.0 * epsilon);
        const Eigen::Vector3d angular_velocity = rotationVector(
            orientation_plus * orientation_minus.inverse()) / (2.0 * epsilon);
        jacobian(3, column) = axis_world.dot(angular_velocity);
    }

    robot_.q_ = saved_q;
    robot_.updateKinematics();
}

void Kinematics::computePositionJacobian(
    Eigen::Matrix<double, 3, Eigen::Dynamic>& jacobian)
{
    Eigen::Matrix<double, 4, Eigen::Dynamic> task_jacobian;
    computeTaskJacobian(task_jacobian);
    jacobian = task_jacobian.topRows<3>();
}

void Kinematics::computeArmPositionJacobian(
    Eigen::Matrix<double, 3, Eigen::Dynamic>& jacobian)
{
    computePositionJacobian(jacobian);
}

void Kinematics::computeGravityTorque(Eigen::VectorXd& tau_g)
{
    tau_g = pinocchio::computeGeneralizedGravity(
        robot_.reduced_model_, robot_.reduced_data_, robot_.q_);
}

Eigen::Quaterniond Kinematics::getEEOrientation() const
{
    return Eigen::Quaterniond(robot_.getHandOrientation()).normalized();
}

Eigen::Vector3d Kinematics::getRightShoulderPosition() const
{
    return robot_.getRightShoulderPosition();
}

double Kinematics::wrapAngle(double angle)
{
    constexpr double pi = 3.14159265358979323846;
    return std::remainder(angle, 2.0 * pi);
}

double Kinematics::toolRoll(const Eigen::Quaterniond& zero_orientation,
                            const Eigen::Quaterniond& current_orientation,
                            const Eigen::Vector3d& tool_axis_local)
{
    Eigen::Quaterniond relative =
        zero_orientation.normalized().inverse() * current_orientation.normalized();
    relative.normalize();
    if (relative.w() < 0.0) relative.coeffs() *= -1.0;

    const Eigen::Vector3d axis = tool_axis_local.normalized();
    const double projected = relative.vec().dot(axis);
    const double norm = std::hypot(relative.w(), projected);
    if (norm < 1.0e-12) return 0.0;
    return wrapAngle(2.0 * std::atan2(projected / norm, relative.w() / norm));
}

Eigen::Vector3d Kinematics::orientationError(const Eigen::Quaterniond& desired,
                                             const Eigen::Quaterniond& current)
{
    return rotationVector(desired * current.inverse());
}

void Kinematics::integrate(const Eigen::VectorXd& dq, double dt)
{
    robot_.q_ = pinocchio::integrate(robot_.reduced_model_, robot_.q_, dq * dt);
}

void Kinematics::dampedPseudoInverse(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& jacobian, double damping,
    Eigen::Matrix<double, Eigen::Dynamic, 3>& pseudo_inverse) const
{
    Eigen::Matrix3d normal = jacobian * jacobian.transpose();
    normal.diagonal().array() += damping;
    pseudo_inverse = jacobian.transpose() * normal.ldlt().solve(Eigen::Matrix3d::Identity());
}

}  // namespace g1_right_arm_reach
