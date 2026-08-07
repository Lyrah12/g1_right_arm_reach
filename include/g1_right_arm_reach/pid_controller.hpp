#ifndef G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_
#define G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_

#include "g1_right_arm_reach/kinematics.hpp"
#include "g1_right_arm_reach/robot_model.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace g1_right_arm_reach {

struct ControlResult {
    bool success{false};
    bool torque_limited{false};
    Eigen::VectorXd command;
    Eigen::VectorXd torque_command;
    Eigen::VectorXd gravity_torque;
    Eigen::VectorXd task_torque;
    Eigen::VectorXd dq_command;
    Eigen::Vector3d achieved_position{Eigen::Vector3d::Zero()};
    double achieved_roll{0.0};
    double position_error{0.0};
    double orientation_error{0.0};
    double effective_kp{0.0};
    double integral_norm{0.0};
    double task_velocity{0.0};
    double adaptive_damping{0.0};
    double nullspace_norm{0.0};
    double max_joint_vel{0.0};
    // Split diagnostics: velocity components and per-limit saturation source.
    double linear_vel{0.0};          // |J_lin * dq|, m/s
    double roll_vel{0.0};            // J_roll * dq, rad/s
    bool force_saturated{false};     // position wrench hit max_task_force
    bool roll_saturated{false};      // roll moment hit max_roll_moment
    bool joint_torque_limited{false}; // a joint hit the derated URDF / max joint clamp
    bool slew_limited{false};        // torque change hit max_torque_slew_rate
    double allocation_min_singular{0.0};  // min singular value of allocation Jacobian
};

/**
 * Four-dimensional operational-space PID.
 *
 * The PID output is a task wrench [Fx,Fy,Fz,Mroll]. Joint torque is
 * tau = J_task^T*wrench + gravity + N*(tau_posture - D*dq).
 */
class PIDController {
public:
    PIDController(RobotModel& robot, Kinematics& kinematics,
                  double kp_position = 220.0,
                  double ki_position = 0.5,
                  double kd_position = 22.0,
                  double damping = 1.0e-3,
                  double position_tolerance = 0.005,
                  double posture_gain = 2.0);

    ControlResult computeTask(const Eigen::Vector3d& target_position,
                              double target_roll,
                              const Eigen::VectorXd& current_q,
                              const Eigen::VectorXd& current_dq,
                              double dt,
                              const Eigen::Quaterniond& roll_zero_orientation,
                              const Eigen::Matrix<double, 4, 1>& target_velocity =
                                  Eigen::Matrix<double, 4, 1>::Zero());

    ControlResult computeGravityHold(const Eigen::VectorXd& q,
                                     const Eigen::VectorXd& dq);

    void reset();
    void setPostureTarget(const Eigen::VectorXd& target);
    void relaxPostureTarget(const Eigen::VectorXd& current,
                            double dt,
                            double time_constant);
    void setKp(double value) { kp_position_ = value; }
    void setKi(double value) { ki_position_ = value; }
    void setKd(double value) { kd_position_ = value; }
    void setRollGains(double kp, double ki, double kd);
    void setDamping(double value) { damping_ = value; }
    void setTolerance(double value) { position_tolerance_ = value; }
    void setRollTolerance(double value) { roll_tolerance_ = value; }
    void setPostureGain(double value) { posture_gain_ = value; }
    void setGravityCompensation(bool enabled) { gravity_compensation_ = enabled; }
    void setUseExternalGravity(bool enabled) { use_external_gravity_ = enabled; }
    void setJointDamping(double value) { joint_damping_ = value; }
    void setWrenchLimits(double max_force, double max_roll_moment);
    void setTorqueSafety(double effort_derate, double max_joint_torque,
                         double max_torque_slew_rate);
    double getTolerance() const { return position_tolerance_; }

    // Kept for source compatibility: this controller always computes torque.
    void setTorqueMode(bool) {}
    bool isTorqueMode() const { return true; }
    void setInnerServoKp(double) {}
    void setInnerServoKv(double value) { setJointDamping(value); }

    void setExternalGravity(const Eigen::VectorXd& gravity);
    bool hasExternalGravity() const;
    double gravityDiff() const;
    void resetIntegral() { error_integral_.setZero(); }

private:
    void resolveGravityTorque(Eigen::VectorXd& gravity) const;
    Eigen::VectorXd jointLimitTorque(double margin) const;
    Eigen::VectorXd selectArm(const Eigen::VectorXd& full) const;
    Eigen::VectorXd applyTorqueLimitsAndSlew(const Eigen::VectorXd& requested,
                                             double dt,
                                             bool* joint_torque_limited = nullptr,
                                             bool* slew_limited = nullptr);

    RobotModel& robot_;
    Kinematics& kinematics_;
    double kp_position_, ki_position_, kd_position_;
    double kp_roll_{10.0}, ki_roll_{0.0}, kd_roll_{1.2};
    double damping_, position_tolerance_, roll_tolerance_{0.035};
    double posture_gain_, joint_damping_{2.0};
    double max_force_{40.0}, max_roll_moment_{6.0};
    double effort_derate_{0.50}, max_joint_torque_{20.0};
    double max_torque_slew_rate_{80.0};
    bool gravity_compensation_{true};
    bool use_external_gravity_{false};

    Eigen::VectorXd posture_target_, external_gravity_, last_torque_;
    Eigen::Matrix<double, 4, 1> previous_error_{Eigen::Matrix<double, 4, 1>::Zero()};
    Eigen::Matrix<double, 4, 1> error_integral_{Eigen::Matrix<double, 4, 1>::Zero()};
    bool first_call_{true};
};

}  // namespace g1_right_arm_reach

#endif
