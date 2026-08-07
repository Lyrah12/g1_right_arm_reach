#include "g1_right_arm_reach/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace g1_right_arm_reach {

PIDController::PIDController(RobotModel& robot, Kinematics& kinematics,
                             double kp_position, double ki_position,
                             double kd_position, double damping,
                             double position_tolerance, double posture_gain)
    : robot_(robot), kinematics_(kinematics),
      kp_position_(kp_position), ki_position_(ki_position),
      kd_position_(kd_position), damping_(damping),
      position_tolerance_(position_tolerance), posture_gain_(posture_gain)
{
    posture_target_ = robot_.q_;
    last_torque_ = Eigen::VectorXd::Zero(robot_.arm_joint_indices_.size());
}

void PIDController::setPostureTarget(const Eigen::VectorXd& target)
{
    if (target.size() == robot_.nq()) posture_target_ = target;
}

void PIDController::relaxPostureTarget(const Eigen::VectorXd& current,
                                       double dt,
                                       double time_constant)
{
    if (current.size() != robot_.nq() || !current.allFinite()
        || !std::isfinite(dt) || dt <= 0.0) {
        return;
    }
    const double alpha = time_constant > 0.0
        ? -std::expm1(-dt / time_constant) : 1.0;
    posture_target_ += std::clamp(alpha, 0.0, 1.0)
        * (current - posture_target_);
}

void PIDController::setRollGains(double kp, double ki, double kd)
{
    kp_roll_ = kp;
    ki_roll_ = ki;
    kd_roll_ = kd;
}

void PIDController::setWrenchLimits(double max_force, double max_roll_moment)
{
    max_force_ = std::max(0.0, max_force);
    max_roll_moment_ = std::max(0.0, max_roll_moment);
}

void PIDController::setTorqueSafety(double effort_derate, double max_joint_torque,
                                    double max_torque_slew_rate)
{
    effort_derate_ = std::clamp(effort_derate, 0.0, 1.0);
    max_joint_torque_ = std::max(0.0, max_joint_torque);
    max_torque_slew_rate_ = std::max(0.0, max_torque_slew_rate);
}

void PIDController::setExternalGravity(const Eigen::VectorXd& gravity)
{
    external_gravity_ = gravity.size() == robot_.nv() ? gravity : Eigen::VectorXd();
}

bool PIDController::hasExternalGravity() const
{
    return external_gravity_.size() == robot_.nv();
}

void PIDController::resolveGravityTorque(Eigen::VectorXd& gravity) const
{
    if (!gravity_compensation_) {
        gravity = Eigen::VectorXd::Zero(robot_.nv());
        return;
    }
    if (use_external_gravity_ && hasExternalGravity()) {
        gravity = external_gravity_;
        return;
    }
    // Pinocchio remains the production gravity source. Simulation launch files
    // opt into MuJoCo's exact bias force so model-conversion differences cannot
    // destabilize the torque loop.
    const_cast<Kinematics&>(kinematics_).computeGravityTorque(gravity);
}

double PIDController::gravityDiff() const
{
    if (!hasExternalGravity()) return -1.0;
    Eigen::VectorXd gravity;
    const_cast<Kinematics&>(kinematics_).computeGravityTorque(gravity);
    return (gravity - external_gravity_).cwiseAbs().maxCoeff();
}

Eigen::VectorXd PIDController::selectArm(const Eigen::VectorXd& full) const
{
    Eigen::VectorXd selected(robot_.arm_joint_indices_.size());
    for (int k = 0; k < selected.size(); ++k) {
        selected(k) = full(robot_.arm_joint_indices_[k]);
    }
    return selected;
}

Eigen::VectorXd PIDController::jointLimitTorque(double margin) const
{
    Eigen::VectorXd torque = Eigen::VectorXd::Zero(robot_.arm_joint_indices_.size());
    for (int k = 0; k < torque.size(); ++k) {
        const int index = robot_.arm_joint_indices_[k];
        const double range = robot_.q_max_(index) - robot_.q_min_(index);
        const double zone = std::min(margin, 0.2 * range);
        if (zone <= 0.0) continue;
        const double lower_distance = robot_.q_(index) - robot_.q_min_(index);
        const double upper_distance = robot_.q_max_(index) - robot_.q_(index);
        if (lower_distance < zone) {
            const double ratio = (zone - lower_distance) / zone;
            torque(k) += posture_gain_ * ratio * ratio;
        }
        if (upper_distance < zone) {
            const double ratio = (zone - upper_distance) / zone;
            torque(k) -= posture_gain_ * ratio * ratio;
        }
    }
    return torque;
}

Eigen::VectorXd PIDController::applyTorqueLimitsAndSlew(
    const Eigen::VectorXd& requested, double dt,
    bool* joint_torque_limited, bool* slew_limited)
{
    Eigen::VectorXd output = requested;
    if (last_torque_.size() != output.size()) last_torque_.setZero(output.size());
    const double maximum_step = max_torque_slew_rate_ * std::max(0.0, dt);
    for (int k = 0; k < output.size(); ++k) {
        const int index = robot_.arm_joint_indices_[k];
        const double urdf_limit = robot_.tau_max_(index) * effort_derate_;
        const double limit = std::min(max_joint_torque_, urdf_limit);
        const double clamped = std::clamp(output(k), -limit, limit);
        if (joint_torque_limited && std::abs(clamped - output(k)) > 1.0e-12) {
            *joint_torque_limited = true;
        }
        output(k) = clamped;
        const double slewed = std::clamp(output(k), last_torque_(k) - maximum_step,
                                         last_torque_(k) + maximum_step);
        if (slew_limited && std::abs(slewed - output(k)) > 1.0e-12) {
            *slew_limited = true;
        }
        output(k) = slewed;
    }
    last_torque_ = output;
    return output;
}

ControlResult PIDController::computeTask(
    const Eigen::Vector3d& target_position, double target_roll,
    const Eigen::VectorXd& current_q, const Eigen::VectorXd& current_dq,
    double dt, const Eigen::Quaterniond& roll_zero_orientation,
    const Eigen::Matrix<double, 4, 1>& target_velocity)
{
    if (current_q.size() != robot_.nq() || current_dq.size() != robot_.nv() ||
        !std::isfinite(dt) || dt <= 0.0 || !target_velocity.allFinite()) {
        return {};
    }

    robot_.q_ = current_q;
    robot_.dq_ = current_dq;
    robot_.updateKinematics();
    const Eigen::Vector3d achieved_position = robot_.getEndEffectorPosition();
    const double achieved_roll = Kinematics::toolRoll(
        roll_zero_orientation, kinematics_.getEEOrientation(),
        kinematics_.toolAxisLocal());

    Eigen::Matrix<double, 4, 1> error;
    error.head<3>() = target_position - achieved_position;
    error(3) = Kinematics::wrapAngle(target_roll - achieved_roll);

    if (!first_call_ && (error.head<3>().norm() > previous_error_.head<3>().norm() * 4.0)) {
        error_integral_.setZero();
    }
    if (error.head<3>().norm() < 0.15 && std::abs(error(3)) < 0.5) {
        error_integral_ += error * dt;
    } else {
        error_integral_.setZero();
    }
    for (int i = 0; i < 3; ++i) {
        error_integral_(i) = std::clamp(error_integral_(i), -0.05, 0.05);
    }
    error_integral_(3) = std::clamp(error_integral_(3), -0.20, 0.20);

    Eigen::Matrix<double, 4, Eigen::Dynamic> jacobian;
    kinematics_.computeTaskJacobian(jacobian);
    const Eigen::VectorXd velocity = selectArm(current_dq);
    const Eigen::Matrix<double, 4, 1> task_velocity = jacobian * velocity;
    // Differentiate through measured velocity, not sampled pose error. The
    // moving trajectory reference contributes its velocity, otherwise the D
    // term incorrectly brakes every commanded motion and produces a long
    // asymptotic tail. For a stationary target this remains exactly -J*dq.
    const Eigen::Matrix<double, 4, 1> derivative =
        target_velocity - task_velocity;

    Eigen::Matrix<double, 4, 1> wrench;
    wrench.head<3>() = kp_position_ * error.head<3>()
        + ki_position_ * error_integral_.head<3>()
        + kd_position_ * derivative.head<3>();
    wrench(3) = kp_roll_ * error(3) + ki_roll_ * error_integral_(3)
        + kd_roll_ * derivative(3);
    const bool force_saturated = wrench.head<3>().norm() > max_force_;
    const double force_norm = wrench.head<3>().norm();
    if (force_norm > max_force_ && force_norm > 0.0) {
        wrench.head<3>() *= max_force_ / force_norm;
    }
    const bool roll_saturated = std::abs(wrench(3)) > max_roll_moment_;
    wrench(3) = std::clamp(wrench(3), -max_roll_moment_, max_roll_moment_);

    // Independent control-allocation Jacobian. The TCP sits ~0.09 m off the
    // wrist roll axis, so the raw transpose-Jacobian lets the position wrench
    // torque the wrist and the roll wrench torque the arm — the two loops fight
    // and enter a coupled limit cycle (roll error grows, dq_wrist spins). Drive
    // position ONLY through the arm joints (shoulder x3 + elbow) and roll ONLY
    // through the wrist: zero the wrist column of the position rows and the
    // roll row on the arm columns. Task-space velocity still uses the real
    // Jacobian, because a wrist roll does move the off-axis TCP and the
    // position loop must see it so the arm can absorb the swing.
    const int wrist_column = robot_.armJointColumn("right_wrist_roll_joint");
    Eigen::Matrix<double, 4, Eigen::Dynamic> allocation_jacobian = jacobian;
    double allocation_singular = 1.0;
    if (wrist_column >= 0) {
        allocation_jacobian.block<3, 1>(0, wrist_column).setZero();
        allocation_jacobian.row(3).setZero();
        allocation_jacobian(3, wrist_column) = jacobian(3, wrist_column);
    }
    {
        Eigen::JacobiSVD<Eigen::Matrix<double, 4, Eigen::Dynamic>> svd(
            allocation_jacobian);
        if (svd.singularValues().size() > 0) {
            allocation_singular = svd.singularValues().minCoeff();
        }
    }
    Eigen::VectorXd task_torque = allocation_jacobian.transpose() * wrench;

    Eigen::Matrix4d normal = allocation_jacobian * allocation_jacobian.transpose();
    normal.diagonal().array() += damping_;
    const Eigen::MatrixXd nullspace = Eigen::MatrixXd::Identity(
        allocation_jacobian.cols(), allocation_jacobian.cols())
        - allocation_jacobian.transpose() * normal.ldlt().solve(allocation_jacobian);

    Eigen::VectorXd posture_torque(allocation_jacobian.cols());
    for (int k = 0; k < posture_torque.size(); ++k) {
        const int index = robot_.arm_joint_indices_[k];
        posture_torque(k) = posture_gain_ * (posture_target_(index) - robot_.q_(index));
    }
    posture_torque += jointLimitTorque(0.20);
    posture_torque = nullspace * posture_torque;
    if (wrist_column >= 0) posture_torque(wrist_column) = 0.0;

    // The task-space derivative term already damps motion visible at the TCP.
    // Applying -D*dq directly to every joint would brake commanded shoulder
    // and elbow motion a second time. Keep joint damping only in the allocation
    // null space so it suppresses redundant motion without slowing the task.
    Eigen::VectorXd nullspace_damping =
        nullspace * (-joint_damping_ * velocity);
    if (wrist_column >= 0) nullspace_damping(wrist_column) = 0.0;

    Eigen::VectorXd gravity_full;
    resolveGravityTorque(gravity_full);
    const Eigen::VectorXd gravity = selectArm(gravity_full);
    const Eigen::VectorXd requested =
        task_torque + gravity + posture_torque + nullspace_damping;
    bool joint_torque_limited = false;
    bool slew_limited = false;
    const Eigen::VectorXd torque = applyTorqueLimitsAndSlew(
        requested, dt, &joint_torque_limited, &slew_limited);

    previous_error_ = error;
    first_call_ = false;

    ControlResult result;
    result.success = error.head<3>().norm() <= position_tolerance_
        && std::abs(error(3)) <= roll_tolerance_;
    result.torque_limited = !torque.isApprox(requested, 1.0e-8);
    result.command = current_q;
    result.torque_command = torque;
    result.gravity_torque = gravity;
    result.task_torque = task_torque;
    result.dq_command = Eigen::VectorXd::Zero(torque.size());
    result.achieved_position = achieved_position;
    result.achieved_roll = achieved_roll;
    result.position_error = error.head<3>().norm();
    result.orientation_error = std::abs(error(3));
    result.effective_kp = kp_position_;
    result.integral_norm = error_integral_.norm();
    result.task_velocity = task_velocity.norm();
    result.adaptive_damping = damping_;
    result.nullspace_norm = posture_torque.norm();
    result.linear_vel = task_velocity.head<3>().norm();
    result.roll_vel = task_velocity(3);
    result.force_saturated = force_saturated;
    result.roll_saturated = roll_saturated;
    result.joint_torque_limited = joint_torque_limited;
    result.slew_limited = slew_limited;
    result.allocation_min_singular = allocation_singular;
    return result;
}

ControlResult PIDController::computeGravityHold(const Eigen::VectorXd& q,
                                                const Eigen::VectorXd& dq)
{
    if (q.size() != robot_.nq() || dq.size() != robot_.nv()) return {};
    robot_.q_ = q;
    robot_.dq_ = dq;
    robot_.updateKinematics();
    Eigen::VectorXd gravity_full;
    resolveGravityTorque(gravity_full);
    const Eigen::VectorXd gravity = selectArm(gravity_full);
    Eigen::VectorXd requested = gravity - joint_damping_ * selectArm(dq);
    for (int k = 0; k < requested.size(); ++k) {
        const int index = robot_.arm_joint_indices_[k];
        requested(k) += posture_gain_ * (posture_target_(index) - robot_.q_(index));
    }

    ControlResult result;
    result.success = true;
    result.command = q;
    result.torque_command = applyTorqueLimitsAndSlew(requested, 0.02);
    result.torque_limited = !result.torque_command.isApprox(requested, 1.0e-8);
    result.gravity_torque = gravity;
    result.achieved_position = robot_.getEndEffectorPosition();
    return result;
}

void PIDController::reset()
{
    previous_error_.setZero();
    error_integral_.setZero();
    last_torque_.setZero();
    first_call_ = true;
}

}  // namespace g1_right_arm_reach
