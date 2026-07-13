/** @file pid_controller.hpp
 *  Resolved-rate task-space PID controller with damped pseudoinverse IK.
 */

#ifndef G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_
#define G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_

#include "g1_right_arm_reach/robot_model.hpp"
#include "g1_right_arm_reach/kinematics.hpp"
#include <Eigen/Dense>

namespace g1_right_arm_reach {

// ============================================================================
// ControlResult
// ============================================================================

struct ControlResult {
    bool success = false;
    Eigen::VectorXd command;             ///< Joint position targets
    Eigen::Vector3d achieved_position;   ///< Current EE position
    double position_error = 0.0;         ///< ||target - achieved|| (m)

    // --- Debug / telemetry fields (populated by compute()) ---
    double effective_kp = 0.0;           ///< Variable proportional gain used
    double integral_norm = 0.0;          ///< ||error_integral|| (m·s)
    double task_velocity = 0.0;          ///< ||task_vel|| before null-space (m/s)
    double adaptive_damping = 0.0;       ///< Damping used in damped pseudoinverse
    double nullspace_norm = 0.0;         ///< ||dq_null|| — null-space contribution (rad/s)
    double max_joint_vel = 0.0;          ///< Max |dq_i| this cycle (rad/s)
    Eigen::VectorXd dq_command;          ///< Raw dq before integration (rad/s)

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
                  double kp = 1.5, double ki = 0.5, double kd = 0.05,
                  double damping = 5e-4, double tolerance = 0.01,
                  double posture_gain = 0.5);

    /// Set the preferred joint posture for null-space attraction.
    /// Must match the MuJoCo ready pose (currently [-0.2, 0, 0, 0.3, 0]:
    /// shoulder slightly forward, elbow slightly bent) to avoid the
    /// controller pulling the arm toward a kinematic singularity.
    void setPostureTarget(const Eigen::VectorXd& q_pref) {
        if (q_pref.size() == posture_target_.size())
            posture_target_ = q_pref;
    }

    /// Run one control cycle. If external_error is provided, it overrides
    /// the internal FK-based error (used with MuJoCo ground-truth EE position).
    /// If external_jacobian is provided, it replaces the URDF numerical
    /// Jacobian (eliminating URDF/MJCF kinematic model mismatch).
    ControlResult compute(const Eigen::Vector3d& target,
                          const Eigen::VectorXd& current_q,
                          const Eigen::VectorXd& current_dq,
                          double dt,
                          const Eigen::Vector3d* external_error = nullptr,
                          const Eigen::Matrix<double, 3, Eigen::Dynamic>* external_jacobian = nullptr);

    /// Reset integral accumulator and derivative state.
    void reset();

    // --- Setters (for runtime parameter tuning) ---
    void setKp(double kp) { kp_ = kp; }
    void setKi(double ki) { ki_ = ki; }
    void setKd(double kd) { kd_ = kd; }
    void setDamping(double d) { damping_ = d; }
    void setTolerance(double tol) { tolerance_ = tol; }
    void setPostureGain(double g) { posture_gain_ = g; }
    double getTolerance() const { return tolerance_; }

private:
    /// Null-space repulsive velocity for joint-limit avoidance.
    Eigen::VectorXd jointLimitAvoidance(double margin = 0.1) const;

    RobotModel& robot_;
    Kinematics& kin_;

    double kp_, ki_, kd_;
    double damping_;
    double tolerance_;
    double posture_gain_;
    Eigen::VectorXd posture_target_;   ///< Preferred joint posture (nq × 1)

    Eigen::Vector3d prev_error_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d error_integral_{Eigen::Vector3d::Zero()};
    bool first_call_{true};
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_
