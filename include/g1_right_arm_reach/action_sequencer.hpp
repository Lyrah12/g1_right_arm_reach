/** @file action_sequencer.hpp
 *  @brief Coordinated action sequencer — arm movement + hand grasp (Plan C).
 *
 *  Plan C Architecture:
 *    Position (XYZ) → /target_point → reach_node (4-DOF arm IK)
 *    Orientation + grasp → /hand_command → hand_controller (9-DOF)
 *
 *  State machine:
 *    IDLE → REACHING → SETTLING → GRASPING → IDLE
 *
 *  Subscribes to /action_command (ActionCommand) for high-level commands.
 *  Monitors /debug/position_error for convergence decisions.
 */

#ifndef G1_RIGHT_ARM_REACH__ACTION_SEQUENCER_HPP_
#define G1_RIGHT_ARM_REACH__ACTION_SEQUENCER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>

#include "g1_right_arm_reach/msg/action_command.hpp"
#include "g1_right_arm_reach/msg/hand_command.hpp"

#include <string>

namespace g1_right_arm_reach {

// ============================================================================
// ActionSequencerNode — high-level action coordinator (Plan C)
// ============================================================================

class ActionSequencerNode : public rclcpp::Node {
public:
    ActionSequencerNode();

private:
    // --- State machine ---
    enum class State { IDLE, REACHING, SETTLING, GRASPING };

    void actionCallback(const msg::ActionCommand::SharedPtr msg);
    void errorCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void oriErrorCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void controlLoop();

    void startReaching();
    void startGrasping();
    void finishAction(bool success);

    // --- ROS2 communication ---
    rclcpp::Subscription<msg::ActionCommand>::SharedPtr action_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ori_error_sub_;

    // Plan C: position → /target_point, orientation + grasp → /hand_command
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
    rclcpp::Publisher<msg::HandCommand>::SharedPtr hand_cmd_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    // --- Current action state ---
    State state_{State::IDLE};

    // Arm target (3-DOF position only)
    double target_x_{0.18}, target_y_{-0.05}, target_z_{-0.10};

    // Hand target (orientation + grasp)
    double ori_x_{0.0}, ori_y_{0.0}, ori_z_{0.0}, ori_w_{1.0};
    std::string grasp_type_;
    double grasp_duration_{0.5};

    // Timing
    double convergence_timeout_{5.0};
    double settle_time_{0.15};
    double state_elapsed_{0.0};
    double position_error_{999.0};

    // Control
    double dt_{0.02};
    bool error_received_{false};
    bool ori_error_received_{false};
    bool has_orientation_{false};  ///< True when user specifies non-identity orientation
    double orientation_error_{999.0};
    double ori_convergence_tol_{0.08};  ///< ~4.6° for orientation convergence

    /// Saturation detection: when wrist offset hits limit, ori_err stops improving.
    /// Track best error and time since last improvement → skip wait if stuck.
    double best_ori_error_{999.0};
    double ori_stuck_time_{0.0};
    static constexpr double ori_stuck_threshold_{4.0};  ///< 4s no improvement → skip to grasp
};

}  // namespace g1_right_arm_reach

#endif  // G1_RIGHT_ARM_REACH__ACTION_SEQUENCER_HPP_
