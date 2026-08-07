#ifndef G1_RIGHT_ARM_REACH__ACTION_SEQUENCER_HPP_
#define G1_RIGHT_ARM_REACH__ACTION_SEQUENCER_HPP_

#include "g1_right_arm_reach/msg/action_command.hpp"
#include "g1_right_arm_reach/msg/action_result.hpp"
#include "g1_right_arm_reach/msg/end_effector_command.hpp"
#include "g1_right_arm_reach/msg/hand_command.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <string>

namespace g1_right_arm_reach {

class ActionSequencerNode final : public rclcpp::Node {
public:
    ActionSequencerNode();

private:
    enum class State { IDLE, REACHING, SETTLING, GRASPING };
    void actionCallback(const msg::ActionCommand::SharedPtr command);
    void positionErrorCallback(const std_msgs::msg::Float64::SharedPtr value);
    void rollErrorCallback(const std_msgs::msg::Float64::SharedPtr value);
    void controlLoop();
    void publishReach();
    void publishGrasp();
    void finish(bool success, const std::string& reason);
    bool feedbackFresh() const;

    State state_{State::IDLE};
    msg::ActionCommand action_;
    int32_t last_action_id_{0};
    double position_error_{999.0}, roll_error_{999.0};
    double elapsed_{0.0}, total_elapsed_{0.0}, dt_{0.02};
    // Steady-clock anchors for real-time timeouts (not accumulated dt ticks).
    std::chrono::steady_clock::time_point action_start_{};
    std::chrono::steady_clock::time_point state_start_{};
    bool have_position_error_{false}, have_roll_error_{false};
    rclcpp::Time last_position_error_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_roll_error_{0, 0, RCL_ROS_TIME};
    double feedback_timeout_{0.5};

    rclcpp::Subscription<msg::ActionCommand>::SharedPtr action_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr roll_error_sub_;
    rclcpp::Publisher<msg::EndEffectorCommand>::SharedPtr reach_pub_;
    rclcpp::Publisher<msg::HandCommand>::SharedPtr hand_pub_;
    rclcpp::Publisher<msg::ActionResult>::SharedPtr result_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace g1_right_arm_reach

#endif
