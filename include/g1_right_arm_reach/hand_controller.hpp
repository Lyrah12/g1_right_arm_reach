/**
 * @file hand_controller.hpp
 * @brief Dexterous hand module — wrist orientation PID + finger impedance (Plan C).
 *
 * Plan C: Arm-Hand fully decoupled.
 *   - Arm (reach_node): 4-DOF → wrist position
 *   - Hand (this node): 9-DOF → orientation + grasp
 *
 * Control:
 *   Wrist:   orientation PID + 3×3 Jacobian → joint torques
 *   Fingers: impedance τ = Kp·e + Kd·ė (direct torque output)
 *
 * Wrist feedback sources (hardware-fidelity, Pinocchio-primary):
 *   /actual_hand_orientation  QuaternionStamped (world-frame palm ori)  — sim only
 *   /mujoco_wrist_jacobian    Float64MultiArray 3×3 (row-major)         — sim only
 *   /mujoco_gravity           Float64MultiArray 7 (wrist indices 4-6)   — sim only
 *   On real hardware these topics do not exist; the node falls back to
 *   Pinocchio FK / numerical Jacobian / RNEA gravity (see controlLoop).
 *
 * @subscribed /hand_command             HandCommand (quat + grasp)
 * @subscribed /joint_states             actual wrist + finger positions
 *
 * @published  /hand_commands            JointState, 9-DOF (wrist×3 + fingers×6)
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
#include "g1_right_arm_reach/robot_model.hpp"
#include "g1_right_arm_reach/kinematics.hpp"

#include <string>
#include <vector>
#include <map>
#include <memory>
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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hand_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ori_error_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void handCommandCallback(const msg::HandCommand::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void handOrientationCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
    void wristJacobianCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void gravityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void eePoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void controlLoop();

    // ========================================================================
    // Wrist orientation control
    // ========================================================================

    /// Orientation PID: error → task angular velocity → τ = Jᵀ·(Kp·e + Ki·∫e + Kd·ė)
    /// + g_wrist − kd_joint·dq (direct torque output, no offset integration).
    void stepWristControl(double dt);

    /// Gravity-hold wrist torque (every cycle): τ = g_wrist − kd_joint·dq.
    /// Runs even when the orientation PID is frozen/absent so the wrist never free-falls.
    void applyWristHold();

    /// Quaternion orientation error as axis-angle vector (3-DOF).
    static Eigen::Vector3d orientationError(const Eigen::Quaterniond& desired,
                                            const Eigen::Quaterniond& current);

    // ========================================================================
    // Pinocchio fallback (hardware path)
    // ========================================================================

    /// Compute wrist ori / 3×3 Jacobian / gravity from Pinocchio, using the
    /// latest /joint_states. Used when the MuJoCo-only topics
    /// (/actual_hand_orientation, /mujoco_wrist_jacobian, /mujoco_gravity)
    /// are absent — i.e. on real hardware. Pinocchio is the PRIMARY source:
    /// the MuJoCo topics are treated as an exact-sim override when present.
    void refreshPinocchio();

    // ========================================================================
    // Finger impedance control
    // ========================================================================

    /// Impedance: τ = Kp·(des − act) + Kd·(0 − dq), encoded as position offset.
    void stepFingerControl();

    // ========================================================================
    // State
    // ========================================================================

    GraspLibrary grasp_library_;

    // --- Pinocchio model (hardware path) ---
    std::unique_ptr<RobotModel> robot_;
    std::unique_ptr<Kinematics> kin_;
    bool pinocchio_ready_{false};    ///< URDF loaded (model available)
    bool pin_fb_ready_{false};       ///< first Pinocchio FK done (feedback available)
    int pin_arm_count_{0};           ///< number of arm joints in the Pinocchio model
    std::vector<std::string> pin_arm_names_;  ///< arm joint names (for /joint_states sync)

    // --- Joint names ---
    std::vector<std::string> wrist_names_;     // 3 joints
    std::vector<std::string> finger_names_;    // 6 active joints
    std::vector<std::string> all_names_;       // 9 joints (wrist + fingers)

    int n_wrist_{0};    // 3
    int n_fingers_{0};  // 6
    int n_total_{0};    // 9

    // --- Actual state (from /joint_states) ---
    Eigen::VectorXd actual_q_;     // 9-DOF: wrist(3) + fingers(6)
    Eigen::VectorXd actual_dq_;    // 9-DOF

    // --- Wrist state ---
    /// Default target = injection quat [0.5, 0.5, 0.5, 0.5] (fingers +X, palm −Y at wrist=0).
    /// Overridden on first HandCommand or orientation callback.
    Eigen::Quaterniond target_ori_{0.5, 0.5, 0.5, 0.5};
    bool has_ori_target_{false};  // true after first HandCommand or orientation capture

    /// Max wrist offset from ready pose the orientation target may demand (rad).
    /// Used to warn when a composed target is beyond wrist authority.
    double wrist_offset_limit_{2.0};

    Eigen::Quaterniond current_hand_ori_{1.0, 0.0, 0.0, 0.0};  // from /actual_hand_orientation
    bool hand_ori_received_{false};
    bool hand_ori_fresh_{false};    // true when new ori data arrived this cycle

    double actual_ee_x_{0.0}, actual_ee_y_{0.0}, actual_ee_z_{0.0};  // from /actual_ee_pose (diagnostic)

    Eigen::Matrix3d wrist_jacobian_{Eigen::Matrix3d::Identity()};  // 3×3 from /mujoco_wrist_jacobian
    bool wrist_jac_received_{false};
    bool wrist_jac_fresh_{false};   // true when new Jacobian arrived this cycle

    Eigen::Vector3d wrist_error_integral_{0.0, 0.0, 0.0};
    Eigen::Vector3d wrist_prev_error_{0.0, 0.0, 0.0};
    bool wrist_first_{true};
    bool wrist_frozen_{false};  ///< hysteresis: true once error drops below 0.03, resets at >0.07

    /// MuJoCo pure-gravity torque for the 3 wrist joints (incl. hand mass).
    Eigen::Vector3d wrist_gravity_{0.0, 0.0, 0.0};
    bool gravity_received_{false};

    /// Cache of last user quat (before composing with current palm).
    /// Used to detect repeated publishes from sequencer (avoid recomposition).
    Eigen::Quaterniond last_user_ori_{0.0, 0.0, 0.0, 0.0};

    int wrist_cycles_since_step_{0};  ///< cycles since last wrist step → effective dt = dt_ * count

    // --- Finger state ---
    Eigen::VectorXd finger_des_q_;      // n_fingers desired (set to target on grasp cmd, held for impedance)
    bool finger_cmd_received_{false};   ///< a grasp command has set finger_des_q_ (guard first joint_state init)

    // --- Torque output (torque mode) ---
    Eigen::Vector3d wrist_cmd_tau_{0.0, 0.0, 0.0};   // wrist joint torque command (N·m)
    Eigen::VectorXd finger_cmd_tau_;                 // finger joint torque command (N·m)
    double tau_max_wrist_{13.4};                     // ±max wrist torque (5010: 13.4 N·m)

    bool joint_state_received_{false};
    double dt_{0.02};

    // --- Control gains (overridden by params.yaml) ---
    double kp_wrist_{3.5};
    double ki_wrist_{0.05};
    double kd_wrist_{0.8};
    double wrist_joint_damping_{0.6};   // joint-space velocity damping (N·m·s/rad)

    double kp_finger_{5.0};
    double kd_finger_{0.3};
    double tau_max_finger_{1.5};
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__HAND_CONTROLLER_HPP_
