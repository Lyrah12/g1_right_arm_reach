#include "g1_right_arm_reach/action_sequencer.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <cmath>
#include <memory>

namespace g1_right_arm_reach {

ActionSequencerNode::ActionSequencerNode() : Node("action_sequencer")
{
    declare_parameter("control_rate", 50.0);
    declare_parameter("position_tolerance", 0.015);
    declare_parameter("roll_tolerance", 0.06);
    declare_parameter("feedback_timeout", 0.5);
    const double rate = get_parameter("control_rate").as_double();
    dt_ = 1.0 / rate;
    feedback_timeout_ = get_parameter("feedback_timeout").as_double();

    action_sub_ = create_subscription<msg::ActionCommand>(
        "/action_command", 10,
        std::bind(&ActionSequencerNode::actionCallback, this, std::placeholders::_1));
    position_error_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/debug/position_error", 10,
        std::bind(&ActionSequencerNode::positionErrorCallback, this, std::placeholders::_1));
    roll_error_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/debug/roll_error", 10,
        std::bind(&ActionSequencerNode::rollErrorCallback, this, std::placeholders::_1));
    reach_pub_ = create_publisher<msg::EndEffectorCommand>("/end_effector_command", 10);
    hand_pub_ = create_publisher<msg::HandCommand>("/hand_command", 10);
    result_pub_ = create_publisher<msg::ActionResult>("/action_result", 10);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(dt_)),
        std::bind(&ActionSequencerNode::controlLoop, this));
}

void ActionSequencerNode::actionCallback(const msg::ActionCommand::SharedPtr command)
{
    // The CLI republishes the command a few times to defeat the DDS discovery
    // race. Ignore repeat delivery of the same action id (either during the
    // action or after it has already finished).
    if (command->action_id != 0 && command->action_id == last_action_id_) return;
    const bool reset = command->action_type == "home" || command->grasp_type == "open";
    if (state_ != State::IDLE && !reset) {
        RCLCPP_WARN(get_logger(), "Action sequencer busy");
        return;
    }
    const double values[]{command->target_x, command->target_y, command->target_z,
                          command->target_roll, command->grasp_duration,
                          command->convergence_timeout, command->settle_time};
    for (double value : values) {
        if (!std::isfinite(value)) {
            RCLCPP_ERROR(get_logger(), "Rejected action with non-finite field");
            return;
        }
    }
    if (std::abs(command->target_roll) > 3.14159265358979323846
        || command->grasp_duration < 0.0 || command->settle_time < 0.0
        || command->convergence_timeout <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Rejected action outside limits");
        return;
    }

    action_ = *command;
    last_action_id_ = action_.action_id;
    if (action_.action_type == "home") {
        action_.target_x = 0.30;
        action_.target_y = -0.10;
        action_.target_z = 0.0;
        action_.target_roll = 0.0;
        action_.grasp_type = "home";
        action_.grasp_duration = 0.5;
        action_.convergence_timeout = 8.0;
        action_.settle_time = 0.2;
    }
    elapsed_ = total_elapsed_ = 0.0;
    action_start_ = state_start_ = std::chrono::steady_clock::now();
    have_position_error_ = have_roll_error_ = false;

    RCLCPP_INFO(get_logger(),
                "Accepted action type=%s grasp=%s target=[%.3f %.3f %.3f] roll=%.3f",
                action_.action_type.c_str(), action_.grasp_type.c_str(),
                action_.target_x, action_.target_y, action_.target_z, action_.target_roll);

    if (action_.action_type == "grasp") {
        state_ = State::GRASPING;
        publishGrasp();
    } else if (action_.action_type == "reach"
               || action_.action_type == "reach_and_grasp"
               || action_.action_type == "home") {
        state_ = State::REACHING;
        publishReach();
    } else {
        finish(false, "UNKNOWN_ACTION");
    }
}

void ActionSequencerNode::positionErrorCallback(
    const std_msgs::msg::Float64::SharedPtr value)
{
    position_error_ = value->data;
    have_position_error_ = std::isfinite(value->data);
    last_position_error_ = now();
}

void ActionSequencerNode::rollErrorCallback(const std_msgs::msg::Float64::SharedPtr value)
{
    roll_error_ = value->data;
    have_roll_error_ = std::isfinite(value->data);
    last_roll_error_ = now();
}

bool ActionSequencerNode::feedbackFresh() const
{
    return have_position_error_ && have_roll_error_
        && (now() - last_position_error_).seconds() <= feedback_timeout_
        && (now() - last_roll_error_).seconds() <= feedback_timeout_;
}

void ActionSequencerNode::publishReach()
{
    msg::EndEffectorCommand command;
    command.target_x = action_.target_x;
    command.target_y = action_.target_y;
    command.target_z = action_.target_z;
    command.target_roll = action_.target_roll;
    command.command_id = action_.action_id;
    reach_pub_->publish(command);
}

void ActionSequencerNode::publishGrasp()
{
    msg::HandCommand command;
    command.grasp_type = action_.grasp_type;
    command.grasp_duration = action_.grasp_duration;
    command.use_joint_positions = false;
    command.joint_positions.fill(0.0);
    command.command_id = action_.action_id;
    hand_pub_->publish(command);
}

void ActionSequencerNode::finish(bool success, const std::string& reason)
{
    msg::ActionResult result;
    result.action_id = action_.action_id;
    result.success = success;
    result.reason = reason;
    result.position_error_m = have_position_error_ ? position_error_ : -1.0;
    result.orientation_error_rad = have_roll_error_ ? roll_error_ : -1.0;
    result.duration_s = total_elapsed_;
    result_pub_->publish(result);
    state_ = State::IDLE;
    RCLCPP_INFO(get_logger(), "Action finished %s (reason=%s) pos=%.4f roll=%.4f dur=%.2fs",
                success ? "SUCCESS" : "FAILURE", reason.c_str(),
                position_error_, roll_error_, total_elapsed_);
}

void ActionSequencerNode::controlLoop()
{
    if (state_ == State::IDLE) return;
    // Timeouts are real wall-clock budgets measured from steady anchors, not
    // accumulated timer ticks, so a jittery or delayed timer cannot stretch or
    // shorten an action's convergence / settle / grasp windows.
    const auto now_steady = std::chrono::steady_clock::now();
    elapsed_ = std::chrono::duration<double>(now_steady - state_start_).count();
    total_elapsed_ = std::chrono::duration<double>(now_steady - action_start_).count();

    if (state_ == State::REACHING) {
        publishReach();
        const double position_tolerance =
            get_parameter("position_tolerance").as_double();
        const double roll_tolerance =
            get_parameter("roll_tolerance").as_double();
        const bool converged = feedbackFresh()
            && position_error_ <= position_tolerance
            && roll_error_ <= roll_tolerance;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ACTION_DIAG state=REACHING pos=%.4f/%.4f roll=%.4f/%.4f "
            "elapsed=%.1f/%.1fs feedback=%d",
            position_error_, position_tolerance, roll_error_, roll_tolerance,
            total_elapsed_, action_.convergence_timeout, feedbackFresh());
        if (converged) {
            // A single zero-crossing is not convergence. Require every action,
            // including reach-only, to remain inside the hysteresis window for
            // its requested settling time before reporting success.
            state_ = State::SETTLING;
            state_start_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(get_logger(),
                        "Reach converged (pos=%.4f roll=%.4f), settling",
                        position_error_, roll_error_);
        } else if (total_elapsed_ > action_.convergence_timeout) {
            finish(false, "REACH_TIMEOUT");
        }
    } else if (state_ == State::SETTLING) {
        publishReach();
        const double position_exit =
            1.5 * get_parameter("position_tolerance").as_double();
        const double roll_exit =
            1.5 * get_parameter("roll_tolerance").as_double();
        // Hysteresis prevents boundary chatter, but both position and roll
        // remain mandatory for every reach-and-grasp action.
        if (!feedbackFresh() || position_error_ > position_exit
            || roll_error_ > roll_exit) {
            state_ = State::REACHING;
            state_start_ = std::chrono::steady_clock::now();
            RCLCPP_WARN(get_logger(),
                        "Settling bounced (pos=%.4f roll=%.4f), back to reaching",
                        position_error_, roll_error_);
        } else if (elapsed_ >= action_.settle_time) {
            if (action_.action_type == "reach") {
                finish(true, "CONVERGED_AND_SETTLED");
            } else {
                state_ = State::GRASPING;
                state_start_ = std::chrono::steady_clock::now();
                RCLCPP_INFO(get_logger(), "Settled; starting grasp '%s' (%.2fs)",
                            action_.grasp_type.c_str(), action_.grasp_duration);
                publishGrasp();
            }
        }
    } else if (state_ == State::GRASPING) {
        // Repeat the grasp each cycle: a single publish can be dropped by the
        // DDS discovery race before the hand_controller subscribes, and the
        // hand_controller de-duplicates identical targets, so this is safe.
        publishGrasp();
        if (elapsed_ >= std::max(0.05, action_.grasp_duration)) finish(true, "DONE");
    }
}

}  // namespace g1_right_arm_reach

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_right_arm_reach::ActionSequencerNode>());
    rclcpp::shutdown();
    return 0;
}
