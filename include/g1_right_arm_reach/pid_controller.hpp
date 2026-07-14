/** @file pid_controller.hpp
 *  @brief Task-space PID with damped pseudoinverse IK.
 *
 *  Arm-Hand decoupled control:
 *    computeArm()   — 4-DOF arm → 3-DOF wrist-origin position (3×4 damped pinv + null-space)
 *    computeWrist() — variable-DOF wrist → 3-DOF hand orientation (3×N auto-adapting solver)
 *
 *  computeWrist supports 1~3 wrist DOF without code changes.
 *  23-DOF model: wrist_roll only (1-DOF, under-actuated → damped pseudoinverse).
 *  29-DOF model: wrist_roll+pitch+yaw (3-DOF, square Jacobian → direct solve).
 */

#ifndef G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_
#define G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_

#include "g1_right_arm_reach/robot_model.hpp"
#include "g1_right_arm_reach/kinematics.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace g1_right_arm_reach {

// ============================================================================
// ControlResult
// ============================================================================

struct ControlResult {
    bool success = false;
    Eigen::VectorXd command;             ///< Joint position targets (rad)
    Eigen::Vector3d achieved_position;   ///< Current EE position (m)
    double position_error = 0.0;         ///< ||target_pos - achieved|| (m)
    double orientation_error = 0.0;      ///< ||orientation_error_vector|| (rad)

    // --- Torque-mode fields ---
    Eigen::VectorXd torque_command;      ///< Joint torque τ_cmd (N·m)
    Eigen::VectorXd gravity_torque;      ///< Gravity feedforward τ_g (N·m)
    Eigen::VectorXd task_torque;         ///< Task-space control torque Jᵀ×F (N·m)

    // --- Debug / telemetry fields ---
    double effective_kp = 0.0;
    double integral_norm = 0.0;
    double task_velocity = 0.0;
    double adaptive_damping = 0.0;
    double nullspace_norm = 0.0;
    double max_joint_vel = 0.0;
    Eigen::VectorXd dq_command;

    static ControlResult success_result(const Eigen::VectorXd& cmd,
                                        const Eigen::Vector3d& pos, double error) {
        ControlResult r;
        r.success = true;
        r.command = cmd;
        r.achieved_position = pos;
        r.position_error = error;
        return r;
    }
};

// ============================================================================
// PIDController
// ============================================================================

class PIDController {
public:
    PIDController(RobotModel& robot, Kinematics& kin,
                  double kp = 2.0, double ki = 0.4, double kd = 0.05,
                  double damping = 2e-4, double tolerance = 0.01,
                  double posture_gain = 0.2);

    /// Set the preferred joint posture for null-space attraction (full 5-DOF).
    void setPostureTarget(const Eigen::VectorXd& q_pref) {
        if (q_pref.size() == posture_target_.size())
            posture_target_ = q_pref;
    }

    // ======================================================================
    // Arm IK — 4-DOF position control (primary interface)
    // ======================================================================

    /// Run one arm control cycle: 4-DOF → 3-DOF wrist origin position.
    /// Modifies only arm joints (indices 0-3) in robot_.q_.
    /// @param target_pos  Desired wrist origin position (shoulder-relative)
    /// @param current_q   Current full 5-DOF joint positions
    /// @param current_dq  Current full 5-DOF joint velocities
    /// @param dt          Control period (s)
    /// @param external_error Override internal FK error
    ControlResult computeArm(const Eigen::Vector3d& target_pos,
                             const Eigen::VectorXd& current_q,
                             const Eigen::VectorXd& current_dq,
                             double dt,
                             const Eigen::Vector3d* external_error = nullptr);

    // ======================================================================
    // Wrist solver — orientation control (primary interface, 1~3 DOF)
    // ======================================================================

    /// Run one wrist control cycle: 3-DOF → 3-DOF hand orientation.
    /// Modifies only wrist joints (indices 4-6) in robot_.q_.
    /// @param target_ori  Desired hand orientation quaternion (world frame)
    /// @param current_q   Current full 5-DOF joint positions
    /// @param current_dq  Current full 5-DOF joint velocities
    /// @param dt          Control period (s)
    ControlResult computeWrist(const Eigen::Quaterniond& target_ori,
                               const Eigen::VectorXd& current_q,
                               const Eigen::VectorXd& current_dq,
                               double dt);

    // ======================================================================
    // Legacy (kept for backward compatibility, not used in normal operation)
    // ======================================================================

    /// @deprecated Use computeArm() + computeWrist() instead.
    ControlResult compute(const Eigen::Vector3d& target,
                          const Eigen::VectorXd& current_q,
                          const Eigen::VectorXd& current_dq,
                          double dt,
                          const Eigen::Vector3d* external_error = nullptr,
                          const Eigen::Matrix<double, 3, Eigen::Dynamic>* external_jacobian = nullptr);

    /// @deprecated Use computeArm() + computeWrist() instead.
    ControlResult computePose(const Eigen::Vector3d& target_pos,
                              const Eigen::Quaterniond& target_ori,
                              const Eigen::VectorXd& current_q,
                              const Eigen::VectorXd& current_dq,
                              double dt,
                              const Eigen::Vector3d* external_error = nullptr,
                              const Eigen::Matrix<double, 6, Eigen::Dynamic>* external_jacobian_6d = nullptr);

    /// Reset all integral/derivative state.
    void reset();

    // --- Setters ---
    void setKp(double kp) { kp_ = kp; }
    void setKi(double ki) { ki_ = ki; }
    void setKd(double kd) { kd_ = kd; }
    void setDamping(double d) { damping_ = d; }
    void setTolerance(double tol) { tolerance_ = tol; }
    void setPostureGain(double g) { posture_gain_ = g; }
    void setKpWrist(double kp_w) { kp_wrist_ = kp_w; }
    void setKdWrist(double kd_w) { kd_wrist_ = kd_w; }
    double getTolerance() const { return tolerance_; }

    void setTorqueMode(bool enable) { torque_mode_ = enable; }
    void setGravityCompensation(bool enable) { gravity_comp_ = enable; }
    bool isTorqueMode() const { return torque_mode_; }

    void setInnerServoKp(double kp) { inner_servo_kp_ = kp; }
    void setInnerServoKv(double kv) { inner_servo_kv_ = kv; }

private:
    /// Legacy core implementation shared by compute() and computePose().
    ControlResult computeImpl(const Eigen::Vector3d& target_pos,
                              const Eigen::Quaterniond* target_ori,
                              const Eigen::VectorXd& current_q,
                              const Eigen::VectorXd& current_dq,
                              double dt,
                              const Eigen::Vector3d* external_error,
                              const Eigen::Matrix<double, 3, Eigen::Dynamic>* external_jacobian_3d,
                              const Eigen::Matrix<double, 6, Eigen::Dynamic>* external_jacobian_6d);

    /// Null-space joint-limit avoidance (arm-only for computeArm, all-joints for legacy).
    Eigen::VectorXd jointLimitAvoidanceArm(double margin = 0.1) const;
    Eigen::VectorXd jointLimitAvoidance(double margin = 0.1) const;

    RobotModel& robot_;
    Kinematics& kin_;

    // Arm position gains
    double kp_, ki_, kd_;
    double damping_, tolerance_, posture_gain_;
    Eigen::VectorXd posture_target_;

    // Wrist orientation gains
    double kp_wrist_ = 1.0, kd_wrist_ = 0.02;

    // Torque mode (disabled — kept for future re-evaluation)
    bool torque_mode_ = false, gravity_comp_ = true;
    double inner_servo_kp_ = 300.0, inner_servo_kv_ = 5.0;

    // Arm PID state
    Eigen::Vector3d prev_error_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d error_integral_{Eigen::Vector3d::Zero()};
    bool first_call_{true};

    // Wrist PID state
    Eigen::Vector3d prev_error_ori_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d ori_integral_{Eigen::Vector3d::Zero()};
    bool first_call_wrist_{true};
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_
