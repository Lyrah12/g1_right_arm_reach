/** @file pid_controller.hpp
 *  @brief Damped-least-squares PID IK controller with joint-limit avoidance.
 *
 *  PIDController implements resolved motion rate control (RMRC).
 *  It is the sole controller for the G1 right arm reaching task.
 */

#ifndef G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_
#define G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_

#include "g1_right_arm_reach/robot_model.hpp"
#include "g1_right_arm_reach/kinematics.hpp"

#include <Eigen/Dense>
#include <string>

// ============================================================================

namespace g1_right_arm_reach {

// ============================================================================
// MODULE: Data structures — ControlResult
// ============================================================================

/**
 * @brief Result produced by one PID control cycle.
 */
struct ControlResult {
    bool success = false;                ///< Whether the control computation succeeded
    Eigen::VectorXd command;             ///< Joint position targets q_des
    Eigen::Vector3d achieved_position;   ///< Current end-effector position (world frame)
    double position_error = 0.0;         ///< ||target - achieved_position|| (meters)
    int solver_iterations = 0;           ///< Always 0 for PID (kept for API compat)
    std::string status_msg;              ///< Human-readable status message

    /// Factory for a successful result.
    static ControlResult success_result(
        const Eigen::VectorXd& cmd,
        const Eigen::Vector3d& pos,
        double error) {
        ControlResult r;
        r.success = true;
        r.command = cmd;
        r.achieved_position = pos;
        r.position_error = error;
        r.status_msg = "OK";
        return r;
    }

    /// Factory for a failed result.
    static ControlResult failure_result(const std::string& reason) {
        ControlResult r;
        r.success = false;
        r.status_msg = reason;
        return r;
    }
};

// ============================================================================
// MODULE: PIDController — damped pseudoinverse IK with null-space repulsion
// ============================================================================

/**
 * @brief PID controller using damped least-squares pseudoinverse IK.
 *
 * This is the primary (and only) controller for reaching.  It
 * implements resolved motion rate control (RMRC):
 *
 *   1. Compute task-space position error: e = p_target - p_ee
 *   2. Compute damped pseudoinverse: J^+ = J^T (J J^T + λI)^(-1)
 *   3. Compute task-space velocity via PID : 
 *          v = Kp·e + Kd·(e - e_prev)/dt + Ki·∫e dt
 *   4. Resolve to joint velocity: dq = J^+ · v
 *   5. Add null-space joint-limit avoidance torque
 *   6. Integrate: q_next = integrate(q, dq · dt)
 *
 * Reference: Whitney, "Resolved motion rate control of manipulators and
 * human prostheses," IEEE Trans. Man-Machine Systems, 1969.
 */
class PIDController {
public:
    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /**
     * @brief Construct a PID IK controller.
     * @param robot     Reference to the RobotModel (reduced 5-DOF model).
     * @param kin       Reference to the Kinematics helper.
     * @param kp        Proportional gain (default 2.5).
     * @param ki        Integral gain (default 3.0).
     * @param kd        Derivative gain (default 0.05).
     * @param damping   Jacobian damping factor λ (default 1e-4).
     * @param tolerance Convergence threshold in meters (default 0.003 = 3 mm).
     * @param max_iters Max internal iterations per control cycle (default 5000).
     */
    PIDController(RobotModel& robot,
                  Kinematics& kin,
                  double kp = 2.5,
                  double ki = 3.0,
                  double kd = 0.05,
                  double damping = 1e-4,
                  double tolerance = 0.003,
                  int max_iters = 5000);

    // ------------------------------------------------------------------
    // Public interface
    // ------------------------------------------------------------------

    /**
     * @brief Compute one control cycle of damped pseudoinverse PID IK.
     * @param target     Desired end-effector position (world frame, xyz).
     * @param current_q  Current joint positions (size nq).
     * @param current_dq Current joint velocities (size nv), may be zero.
     * @param dt         Control timestep in seconds.
     * @return ControlResult with joint position targets and status.
     */
    ControlResult compute(
        const Eigen::Vector3d& target,
        const Eigen::VectorXd& current_q,
        const Eigen::VectorXd& current_dq,
        double dt);

    /// @brief Return controller name: "PID".
    std::string name() const { return "PID"; }

    /**
     * @brief Reset internal state (integral term, previous error).
     */
    void reset();

    // ------------------------------------------------------------------
    // Setters
    // ------------------------------------------------------------------

    /// Set all three PID gains at once.
    void setGains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }

    /// Set the Jacobian damping factor λ.
    void setDamping(double d) { damping_ = d; }

    /// Set the convergence tolerance (meters).
    void setTolerance(double tol) { tolerance_ = tol; }

private:
    // ------------------------------------------------------------------
    // Private helpers
    // ------------------------------------------------------------------

    /**
     * @brief Compute joint-limit repulsive potential gradient for null-space.
     *
     * Pushes joints away from their limits without affecting end-effector
     * motion.  The repulsion is zero when the joint is farther than `margin`
     * from either limit.
     *
     * @param margin  Angular distance from limit at which repulsion starts
     *                (radians, default 0.1).
     * @return Joint velocity in null-space that pushes away from limits.
     */
    Eigen::VectorXd jointLimitAvoidance(double margin = 0.1) const;

    // ------------------------------------------------------------------
    // Data members
    // ------------------------------------------------------------------

    RobotModel& robot_;      ///< Reduced 5-DOF robot model
    Kinematics& kin_;        ///< Kinematics helper (Jacobian, integration, etc.)

    // Gains
    double kp_;              ///< Proportional gain
    double ki_;              ///< Integral gain
    double kd_;              ///< Derivative gain
    double damping_;         ///< Jacobian damping factor λ
    double tolerance_;       ///< Convergence threshold (meters)
    int max_iters_;          ///< Max internal iterations per control cycle

    // State
    Eigen::Vector3d prev_error_;       ///< Previous position error (for derivative term)
    Eigen::Vector3d error_integral_;   ///< Accumulated error (for integral term, with anti-windup)
    bool first_call_;                  ///< True until the first call to compute()
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__PID_CONTROLLER_HPP_
