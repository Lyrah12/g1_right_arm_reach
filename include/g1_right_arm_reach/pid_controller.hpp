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
                  double damping = 5e-4, double tolerance = 0.01);

    /// Run one control cycle. If external_error is provided, it overrides
    /// the internal FK-based error (used with MuJoCo ground-truth EE position).
    ControlResult compute(const Eigen::Vector3d& target,
                          const Eigen::VectorXd& current_q,
                          const Eigen::VectorXd& current_dq,
                          double dt,
                          const Eigen::Vector3d* external_error = nullptr);

    /// Reset integral accumulator and derivative state.
    void reset();

    // --- Setters (for runtime parameter tuning) ---
    void setKp(double kp) { kp_ = kp; }
    void setKi(double ki) { ki_ = ki; }
    void setKd(double kd) { kd_ = kd; }
    void setDamping(double d) { damping_ = d; }
    void setTolerance(double tol) { tolerance_ = tol; }
    double getTolerance() const { return tolerance_; }

private:
    /// Null-space repulsive velocity for joint-limit avoidance.
    Eigen::VectorXd jointLimitAvoidance(double margin = 0.1) const;

    RobotModel& robot_;
    Kinematics& kin_;

    double kp_, ki_, kd_;
    double damping_;
    double tolerance_;

    Eigen::Vector3d prev_error_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d error_integral_{Eigen::Vector3d::Zero()};
    bool first_call_{true};
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_
