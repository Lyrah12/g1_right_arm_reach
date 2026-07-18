/**
 * @file action_sequencer_node.cpp
 * @brief Coordinated action sequencer — arm reach + hand grasp (50 Hz, Plan C).
 *
 * Plan C Architecture:
 *   Position (XYZ)   → /target_point  → reach_node (4-DOF arm IK)
 *   Orientation+grasp → /hand_command  → hand_controller (23-DOF)
 *
 * State machine:
 *   IDLE → (command) → REACHING → (converged) → SETTLING → (settled) → GRASPING → (done) → IDLE
 *   Any state → (timeout) → TIMEOUT → IDLE
 *
 * Subscribes:  /action_command (ActionCommand)
 *              /debug/position_error (Float64)
 * Publishes:   /target_point (Point)
 *              /hand_command (HandCommand)
 */

#include "g1_right_arm_reach/action_sequencer.hpp"

namespace g1_right_arm_reach {

using namespace std::literals::chrono_literals;

// ============================================================================
// Constructor
// ============================================================================

ActionSequencerNode::ActionSequencerNode()
    : Node("action_sequencer")
{
    // --- Subscribers ---
    action_sub_ = this->create_subscription<msg::ActionCommand>(
        "/action_command", 10,
        std::bind(&ActionSequencerNode::actionCallback, this, std::placeholders::_1));

    error_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/debug/position_error", 10,
        std::bind(&ActionSequencerNode::errorCallback, this, std::placeholders::_1));

    ori_error_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/debug/orientation_error", 10,
        std::bind(&ActionSequencerNode::oriErrorCallback, this, std::placeholders::_1));

    // --- Publishers (Plan C) ---
    // Position → /target_point (reach_node)
    point_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
        "/target_point", 10);

    // Orientation + grasp → /hand_command (hand_controller)
    hand_cmd_pub_ = this->create_publisher<msg::HandCommand>(
        "/hand_command", 10);

    // Legacy: /grasp_command for hand-only actions (backward compat)
    grasp_pub_ = this->create_publisher<msg::GraspCommand>(
        "/grasp_command", 10);

    // --- Control timer (50 Hz) ---
    dt_ = 0.02;
    control_timer_ = this->create_wall_timer(
        20ms, std::bind(&ActionSequencerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
        "ActionSequencer ready (Plan C). "
        "Position → /target_point | Orientation+Grasp → /hand_command");
}

// ============================================================================
// Callbacks
// ============================================================================

void ActionSequencerNode::actionCallback(const msg::ActionCommand::SharedPtr msg)
{
    if (state_ != State::IDLE) {
        RCLCPP_WARN(get_logger(),
            "Busy (state=%d). Ignoring action '%s'. Wait for completion.",
            static_cast<int>(state_), msg->action_type.c_str());
        return;
    }

    const std::string& type = msg->action_type;

    // Detect non-identity orientation
    auto hasUserOri = [](const auto& m) {
        return std::abs(m->ori_x) > 1e-9 || std::abs(m->ori_y) > 1e-9
            || std::abs(m->ori_z) > 1e-9 || std::abs(m->ori_w - 1.0) > 1e-9;
    };

    if (type == "home") {
        // Go to ready pose + home grasp
        target_x_ = 0.18; target_y_ = -0.05; target_z_ = -0.10;
        ori_x_ = 0.0; ori_y_ = 0.0; ori_z_ = 0.0; ori_w_ = 1.0;
        has_orientation_ = false;
        grasp_type_ = "home";
        grasp_duration_ = 0.5;
        convergence_timeout_ = 10.0;
        settle_time_ = 0.15;
        startReaching();
        RCLCPP_INFO(get_logger(), "Action: HOME → ready pose + home grasp");

    } else if (type == "reach") {
        // Arm only (3-DOF position)
        target_x_ = msg->target_x;
        target_y_ = msg->target_y;
        target_z_ = msg->target_z;
        ori_x_ = msg->ori_x; ori_y_ = msg->ori_y; ori_z_ = msg->ori_z; ori_w_ = msg->ori_w;
        has_orientation_ = hasUserOri(msg);
        grasp_type_ = "";
        convergence_timeout_ = msg->convergence_timeout;
        settle_time_ = msg->settle_time;
        startReaching();
        RCLCPP_INFO(get_logger(),
            "Action: REACH → [%.3f, %.3f, %.3f]%s",
            target_x_, target_y_, target_z_,
            has_orientation_ ? " +ori→hand" : "");

    } else if (type == "grasp") {
        // Hand only (orientation + grasp)
        ori_x_ = msg->ori_x; ori_y_ = msg->ori_y; ori_z_ = msg->ori_z; ori_w_ = msg->ori_w;
        has_orientation_ = hasUserOri(msg);
        grasp_type_ = msg->grasp_type;
        grasp_duration_ = msg->grasp_duration;
        startGrasping();
        RCLCPP_INFO(get_logger(),
            "Action: GRASP → %s (%.2fs)%s",
            grasp_type_.c_str(), grasp_duration_,
            has_orientation_ ? " +ori" : "");

    } else if (type == "reach_and_grasp") {
        // Full pipeline: arm reach → settle → hand orientation + grasp
        target_x_ = msg->target_x;
        target_y_ = msg->target_y;
        target_z_ = msg->target_z;
        ori_x_ = msg->ori_x; ori_y_ = msg->ori_y; ori_z_ = msg->ori_z; ori_w_ = msg->ori_w;
        has_orientation_ = hasUserOri(msg);
        grasp_type_ = msg->grasp_type;
        grasp_duration_ = msg->grasp_duration;
        convergence_timeout_ = msg->convergence_timeout;
        settle_time_ = msg->settle_time;
        startReaching();
        RCLCPP_INFO(get_logger(),
            "Action: REACH_AND_GRASP → [%.3f, %.3f, %.3f]%s → %s (%.2fs)",
            target_x_, target_y_, target_z_,
            has_orientation_ ? " +ori" : "",
            grasp_type_.c_str(), grasp_duration_);

    } else {
        RCLCPP_WARN(get_logger(),
            "Unknown action type '%s'. Valid: reach, grasp, reach_and_grasp, home.",
            type.c_str());
    }
}

void ActionSequencerNode::errorCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    position_error_ = msg->data;
    error_received_ = true;
}

void ActionSequencerNode::oriErrorCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    orientation_error_ = msg->data;
    ori_error_received_ = true;
}

// ============================================================================
// State transitions
// ============================================================================

void ActionSequencerNode::startReaching()
{
    state_ = State::REACHING;
    state_elapsed_ = 0.0;
    best_ori_error_ = 999.0;
    ori_stuck_time_ = 0.0;

    // Plan C: always publish position to /target_point (Point, 3-DOF)
    auto pt = geometry_msgs::msg::Point();
    pt.x = target_x_;
    pt.y = target_y_;
    pt.z = target_z_;
    point_pub_->publish(pt);

    // Phase 1: send orientation to hand_controller concurrently with arm reaching.
    // Both converge before Phase 2 (grasp).
    if (has_orientation_) {
        auto cmd = msg::HandCommand();
        cmd.ori_x = ori_x_; cmd.ori_y = ori_y_; cmd.ori_z = ori_z_; cmd.ori_w = ori_w_;
        cmd.grasp_type = "";        // orientation only, no finger movement
        cmd.grasp_duration = 0.0;
        hand_cmd_pub_->publish(cmd);
        RCLCPP_INFO(get_logger(), "Phase 1: Arm reach + wrist orient → converging both");
    } else {
        // No orientation → RESET to injection quat [0.5,0.5,0.5,0.5].
        // This prevents stale target_ori_ from a previous 6-DOF command (e.g. RPY(0,0,0))
        // from causing 100°+ wrist rotation → saturation → oscillation.
        auto cmd = msg::HandCommand();
        cmd.ori_x = 0.5; cmd.ori_y = 0.5; cmd.ori_z = 0.5; cmd.ori_w = 0.5;
        cmd.grasp_type = "";
        cmd.grasp_duration = 0.0;
        hand_cmd_pub_->publish(cmd);
        RCLCPP_INFO(get_logger(), "Phase 1: Arm reach + wrist hold (injection quat → default palm ori)");
    }
}

void ActionSequencerNode::startGrasping()
{
    state_ = State::GRASPING;
    state_elapsed_ = 0.0;

    // Phase 2: grasp only. Identity quat → hold wrist orientation. grasp_type → close fingers.
    auto cmd = msg::HandCommand();
    cmd.ori_x = 0.0; cmd.ori_y = 0.0; cmd.ori_z = 0.0; cmd.ori_w = 1.0;  // identity → hold
    cmd.grasp_type = grasp_type_;
    cmd.grasp_duration = grasp_duration_;
    cmd.force_limit = 0.0;
    hand_cmd_pub_->publish(cmd);

    // Also publish legacy /grasp_command for backward compatibility
    auto legacy = msg::GraspCommand();
    legacy.grasp_type = grasp_type_;
    legacy.duration = grasp_duration_;
    legacy.force_limit = 0.0;
    grasp_pub_->publish(legacy);
}

void ActionSequencerNode::finishAction(bool success)
{
    const char* status = success ? "OK" : "TIMEOUT";
    RCLCPP_INFO(get_logger(),
        "[%s] Action complete. pos=%.1f mm  ori=%.3f rad  elapsed=%.2f s",
        status, position_error_ * 1000.0, orientation_error_, state_elapsed_);
    state_ = State::IDLE;
}

// ============================================================================
// Control loop (50 Hz) — state machine
// ============================================================================

void ActionSequencerNode::controlLoop()
{
    switch (state_) {
    case State::IDLE:
        break;

    case State::REACHING:
        state_elapsed_ += dt_;

        // Re-publish position target every cycle
        {
            auto pt = geometry_msgs::msg::Point();
            pt.x = target_x_;
            pt.y = target_y_;
            pt.z = target_z_;
            point_pub_->publish(pt);
        }

        // Re-publish orientation target every cycle for robustness (hand_controller
        // may miss the first message at startReaching).
        {
            auto cmd = msg::HandCommand();
            if (has_orientation_) {
                cmd.ori_x = ori_x_;
                cmd.ori_y = ori_y_;
                cmd.ori_z = ori_z_;
                cmd.ori_w = ori_w_;
            } else {
                // No orientation specified → reset to injection quat
                cmd.ori_x = 0.5; cmd.ori_y = 0.5; cmd.ori_z = 0.5; cmd.ori_w = 0.5;
            }
            cmd.grasp_type = "";       // orientation only, no finger movement
            cmd.grasp_duration = 0.0;
            hand_cmd_pub_->publish(cmd);
        }

        // Phase 1 convergence: BOTH position AND orientation
        {
            bool pos_ok = error_received_ && position_error_ < 0.015;

            // Saturation detection: if wrist offset hits limit, ori_err stops improving.
            // Track best error and time since last improvement → skip wait if stuck.
            bool ori_stuck = false;
            if (has_orientation_ && ori_error_received_) {
                if (orientation_error_ < best_ori_error_ - 0.005) {
                    best_ori_error_ = orientation_error_;
                    ori_stuck_time_ = 0.0;
                } else {
                    ori_stuck_time_ += dt_;
                }
                ori_stuck = (ori_stuck_time_ > ori_stuck_threshold_)
                         && (orientation_error_ > ori_convergence_tol_);
            }

            bool ori_ok = !has_orientation_
                       || (ori_error_received_ && orientation_error_ < ori_convergence_tol_);

            if (pos_ok && (ori_ok || ori_stuck)) {
                if (ori_stuck) {
                    RCLCPP_WARN(get_logger(),
                        "Wrist orientation stuck at %.3f rad (%.1fs no improvement, "
                        "offset likely saturated). Skipping wait → grasp.",
                        orientation_error_, ori_stuck_threshold_);
                }
                if (grasp_type_.empty()) {
                    finishAction(true);  // reach only
                } else {
                    state_ = State::SETTLING;
                    state_elapsed_ = 0.0;
                    RCLCPP_INFO(get_logger(),
                        "Phase 1 done: pos=%.1f mm, ori=%.3f rad → settling",
                        position_error_ * 1000.0,
                        has_orientation_ ? orientation_error_ : -1.0);
                }
            }
        }

        if (state_elapsed_ > convergence_timeout_) {
            RCLCPP_WARN(get_logger(),
                "REACHING timeout (%.1fs, pos=%.3f m, ori=%.3f rad). %s",
                state_elapsed_, position_error_, orientation_error_,
                grasp_type_.empty() ? "Aborting." : "Proceeding to grasp.");
            if (grasp_type_.empty()) {
                finishAction(false);
            } else {
                startGrasping();
            }
        }
        break;

    case State::SETTLING:
        state_elapsed_ += dt_;

        // Keep publishing position target (arm stays locked)
        {
            auto pt = geometry_msgs::msg::Point();
            pt.x = target_x_;
            pt.y = target_y_;
            pt.z = target_z_;
            point_pub_->publish(pt);
        }

        if (state_elapsed_ >= settle_time_) {
            RCLCPP_INFO(get_logger(),
                "Phase 2: Grasping → %s (%.2fs)",
                grasp_type_.c_str(), grasp_duration_);
            startGrasping();
        }

        // Overshoot check
        if (error_received_ && position_error_ > 0.03) {
            RCLCPP_INFO(get_logger(),
                "SETTLING overshoot (err=%.3f m), back to REACHING", position_error_);
            state_ = State::REACHING;
            state_elapsed_ = 0.0;
        }
        break;

    case State::GRASPING:
        state_elapsed_ += dt_;

        if (state_elapsed_ >= grasp_duration_) {
            finishAction(true);
        }
        break;
    }
}

}  // namespace g1_right_arm_reach

// ============================================================================
// Main — ROS2 entry point
// ============================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<g1_right_arm_reach::ActionSequencerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
