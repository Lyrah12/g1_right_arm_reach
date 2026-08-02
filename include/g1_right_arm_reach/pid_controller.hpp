/** @file pid_controller.hpp
 *  @brief Task-space PID with damped pseudoinverse IK (arm position control).
 *
 *  Arm-Hand decoupled control:
 *    computeArm()       — 4-DOF arm → 3-DOF wrist-origin position (3×4 damped pinv + null-space)
 *    computeGravityHold — arm torque when no target yet (prevents free-fall)
 *
 *  Wrist orientation is handled entirely by HandController (see hand_controller.hpp),
 *  so this class no longer exposes a wrist solver.
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
                  double damping = 2e-4, double tolerance = 0.005,
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

    /// Reset all integral/derivative state.
    void reset();

    // --- Setters ---
    void setKp(double kp) { kp_ = kp; }
    void setKi(double ki) { ki_ = ki; }
    void setKd(double kd) { kd_ = kd; }
    void setDamping(double d) { damping_ = d; }
    void setTolerance(double tol) { tolerance_ = tol; }
    void setPostureGain(double g) { posture_gain_ = g; }
    double getTolerance() const { return tolerance_; }

    void setTorqueMode(bool enable) { torque_mode_ = enable; }
    void setGravityCompensation(bool enable) { gravity_comp_ = enable; }
    bool isTorqueMode() const { return torque_mode_; }

    void setInnerServoKp(double kp) { inner_servo_kp_ = kp; }
    void setInnerServoKv(double kv) { inner_servo_kv_ = kv; }

    /// External gravity feedforward (e.g. MuJoCo pure gravity incl. hand mass).
    /// Takes priority over Pinocchio computeGeneralizedGravity when size == nv.
    void setExternalGravity(const Eigen::VectorXd& tau_g) {
        external_gravity_ = (tau_g.size() == robot_.reduced_model_.nv) ? tau_g : Eigen::VectorXd();
    }
    bool hasExternalGravity() const { return external_gravity_.size() == robot_.reduced_model_.nv; }
    /// Max |Pinocchio − MuJoCo| gravity over all joints (diagnostic; −1 if no external).
    double gravityDiff() const;

    /// Reset only the integral (keep derivative state) — safe on divergence.
    void resetIntegral() { error_integral_.setZero(); }

    /// Pure gravity-hold torque for the arm joints: τ = g − kv·dq.
    /// Used when there is no target yet, so the arm does not free-fall.
    ControlResult computeGravityHold(const Eigen::VectorXd& q,
                                     const Eigen::VectorXd& dq);

private:
    /// Null-space joint-limit avoidance (arm joints only).
    Eigen::VectorXd jointLimitAvoidanceArm(double margin = 0.1) const;

    /// Fill tau_g with external (MuJoCo) gravity if set, else Pinocchio RNEA.
    void resolveGravityTorque(Eigen::VectorXd& tau_g) const;

    RobotModel& robot_;
    Kinematics& kin_;

    // Arm position gains
    double kp_, ki_, kd_;
    double damping_, tolerance_, posture_gain_;
    Eigen::VectorXd posture_target_;

    // Torque mode
    bool torque_mode_ = false, gravity_comp_ = true;
    double inner_servo_kp_ = 5.0, inner_servo_kv_ = 5.0;

    // External gravity feedforward (MuJoCo pure gravity, includes hand mass).
    Eigen::VectorXd external_gravity_;

    // Arm PID state
    Eigen::Vector3d prev_error_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d error_integral_{Eigen::Vector3d::Zero()};
    bool first_call_{true};
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_
