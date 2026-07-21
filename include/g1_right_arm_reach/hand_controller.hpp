/**
 * @file hand_controller.hpp
 * @brief Dexterous hand module — wrist orientation PID + finger impedance (Plan C).
 *
 * Plan C: Arm-Hand fully decoupled.
 *   - Arm (reach_node): 4-DOF → wrist position
 *   - Hand (this node): 23-DOF → orientation + grasp
 *
 * Control:
 *   Wrist:   orientation PID + MuJoCo numerical Jacobian (3×3) → joint positions
 *   Fingers: impedance τ = Kp·e + Kd·ė → position-encoded torque, smoothstep trajectory
 *
 * @subscribed /hand_command             HandCommand (quat + grasp)
 * @subscribed /joint_states             actual wrist + finger positions
 * @subscribed /actual_hand_orientation  QuaternionStamped (world-frame palm orientation)
 * @subscribed /mujoco_wrist_jacobian    Float64MultiArray 3×3 (row-major)
 *
 * @published  /hand_commands            JointState, 23-DOF (wrist×3 + fingers×20)
 * @published  /debug/orientation_error  Float64 (rad)
 */

#ifndef G1_RIGHT_ARM_REACH__HAND_CONTROLLER_HPP_
#define G1_RIGHT_ARM_REACH__HAND_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include "g1_right_arm_reach/msg/hand_command.hpp"
#include "g1_right_arm_reach/grasp_library.hpp"

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace g1_right_arm_reach {

// ============================================================================
// HandController — wrist orientation PID + finger impedance control
// ============================================================================

class HandController : public rclcpp::Node {
public:
    HandController();

private:
    // --- ROS2 communication ---
    rclcpp::Subscription<msg::HandCommand>::SharedPtr hand_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ee_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr hand_ori_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wrist_jac_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hand_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ori_error_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void handCommandCallback(const msg::HandCommand::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void handOrientationCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
    void wristJacobianCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void eePoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void controlLoop();

    // ========================================================================
    // Wrist orientation control
    // ========================================================================

    /// Orientation PID: error → task angular velocity → J⁻¹ → wrist_offset_ increment.
    void stepWristControl(double dt);

    /// Quaternion orientation error as axis-angle vector (3-DOF).
    static Eigen::Vector3d orientationError(const Eigen::Quaterniond& desired,
                                            const Eigen::Quaterniond& current);

    // ========================================================================
    // Finger impedance control
    // ========================================================================

    /// Impedance: τ = Kp·(des − act) + Kd·(0 − dq), encoded as position offset.
    void stepFingerControl();

    // ========================================================================
    // State
    // ========================================================================

    GraspLibrary grasp_library_;

    // --- Joint names ---
    std::vector<std::string> wrist_names_;     // 3 joints
    std::vector<std::string> finger_names_;    // 20 joints
    std::vector<std::string> all_names_;       // 23 joints (wrist + fingers)

    int n_wrist_{0};    // 3
    int n_fingers_{0};  // 20
    int n_total_{0};    // 23

    // --- Actual state (from /joint_states) ---
    Eigen::VectorXd actual_q_;     // 23-DOF: wrist(3) + fingers(20)
    Eigen::VectorXd actual_dq_;    // 23-DOF

    // --- Wrist state ---
    Eigen::Vector3d target_ori_quat_vec_;  // [x, y, z, w] from HandCommand (unused w/ default)

    /// Default target = injection quat [0.5, 0.5, 0.5, 0.5] (fingers +X, palm −Y at wrist=0).
    /// Overridden on first HandCommand or orientation callback.
    Eigen::Quaterniond target_ori_{0.5, 0.5, 0.5, 0.5};
    bool has_ori_target_{false};  // true after first HandCommand or orientation capture

    /// Wrist command = fixed reference + clamped PID offset (anti-windup).
    ///   wrist_cmd = wrist_ref_ + wrist_offset_
    ///   wrist_ref_ = [0,0,0] (ready pose, NEVER changes)
    ///   wrist_offset_ += dq * dt, clamped to ±wrist_offset_limit_
    /// Fixed reference ensures servo always pulls toward ready pose,
    /// preventing drift-following (act + offset → follows drift).
    Eigen::Vector3d wrist_ref_{0.0, 0.0, 0.0};
    Eigen::Vector3d wrist_offset_{0.0, 0.0, 0.0};
    double wrist_offset_limit_{2.0};  // ±2.0 rad max offset (set by params.yaml)

    Eigen::Quaterniond current_hand_ori_{1.0, 0.0, 0.0, 0.0};  // from /actual_hand_orientation
    bool hand_ori_received_{false};
    bool hand_ori_fresh_{false};    // true when new ori data arrived this cycle

    double actual_ee_x_{0.0}, actual_ee_y_{0.0}, actual_ee_z_{0.0};  // from /actual_ee_pose
    bool ee_pose_received_{false};

    Eigen::Matrix3d wrist_jacobian_{Eigen::Matrix3d::Identity()};  // 3×3 from /mujoco_wrist_jacobian
    bool wrist_jac_received_{false};
    bool wrist_jac_fresh_{false};   // true when new Jacobian arrived this cycle

    Eigen::Vector3d wrist_error_integral_{0.0, 0.0, 0.0};
    Eigen::Vector3d wrist_prev_error_{0.0, 0.0, 0.0};
    bool wrist_first_{true};
    bool wrist_frozen_{false};  ///< hysteresis: true once error drops below 0.03, resets at >0.07

    /// Cache of last user quat (before composing with current palm).
    /// Used to detect repeated publishes from sequencer (avoid recomposition).
    Eigen::Quaterniond last_user_ori_{0.0, 0.0, 0.0, 0.0};

    int wrist_cycles_since_step_{0};  ///< cycles since last wrist step → effective dt = dt_ * count

    // --- Finger state ---
    Eigen::VectorXd finger_target_q_;   // 20-DOF target (from grasp pattern)
    Eigen::VectorXd finger_des_q_;      // 20-DOF desired (set to target on grasp cmd, held for impedance)
    Eigen::VectorXd finger_cmd_q_;      // 20-DOF actual command (des_q + impedance offset)
    Eigen::VectorXd finger_effort_;     // 20-DOF joint torque from /joint_states effort (N·m)

    std::string current_grasp_type_;

    bool joint_state_received_{false};
    double dt_{0.02};

    // --- Control gains (overridden by params.yaml) ---
    double kp_wrist_{2.0};
    double ki_wrist_{0.05};
    double kd_wrist_{0.12};
    double wrist_vel_limit_{3.0};
    double wrist_delta_limit_{0.12};

    double kp_finger_{50.0};
    double kd_finger_{0.5};
    double tau_max_finger_{200.0};
    double finger_delta_limit_{1.5};
    double inner_servo_kp_{5.0};
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__HAND_CONTROLLER_HPP_
