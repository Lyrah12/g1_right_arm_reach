/**
 * @file action_sequencer_node.cpp
 * @brief Coordinated action sequencer — arm reach + hand grasp (50 Hz, Plan C).
 *
 * Plan C Architecture:
 *   Position (XYZ)   → /target_point  → reach_node (4-DOF arm IK)
 *   Orientation+grasp → /hand_command  → hand_controller (9-DOF)
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

#include <cstdio>
#include <cmath>
#include <string>
#include <Eigen/Geometry>

namespace g1_right_arm_reach {

using namespace std::literals::chrono_literals;

// ============================================================================
// Input validation (P0-5)
// ============================================================================

namespace {

/// Reject malformed / out-of-range action commands at the entry point.
/// A target may only reach the controller if every field is finite, the
/// quaternion (when a non-identity orientation is requested) is unit-norm,
/// durations are positive, and the position is inside a generous workspace
/// sanity box (shoulder-relative). Returns false + a reason on failure.
bool validateActionCommand(const msg::ActionCommand& m, std::string& why)
{
    const double fields[] = {
        m.target_x, m.target_y, m.target_z,
        m.ori_x, m.ori_y, m.ori_z, m.ori_w,
        m.convergence_timeout, m.settle_time, m.grasp_duration,
    };
    for (double v : fields) {
        if (!std::isfinite(v)) { why = "non-finite field"; return false; }
    }

    if (m.convergence_timeout <= 0.0) { why = "convergence_timeout must be > 0"; return false; }
    if (m.settle_time < 0.0)          { why = "settle_time must be >= 0"; return false; }
    if (m.grasp_duration < 0.0)       { why = "grasp_duration must be >= 0"; return false; }

    // Quaternion norm — only when a non-identity orientation is requested.
    const bool has_ori = std::abs(m.ori_x) > 1e-9 || std::abs(m.ori_y) > 1e-9
                      || std::abs(m.ori_z) > 1e-9 || std::abs(m.ori_w - 1.0) > 1e-9;
    if (has_ori) {
        const double n2 = m.ori_x*m.ori_x + m.ori_y*m.ori_y
                        + m.ori_z*m.ori_z + m.ori_w*m.ori_w;
        if (std::abs(n2 - 1.0) > 1e-3) { why = "quaternion norm != 1"; return false; }
    }

    // Workspace sanity box (shoulder-relative, generous). Precise reachability is
    // checked downstream by the IK; this only gates clearly-out-of-range targets.
    if (m.target_x < 0.0 || m.target_x > 0.55 ||
        std::abs(m.target_y) > 0.45 ||
        m.target_z < -0.35 || m.target_z > 0.45) {
        why = "target outside workspace box";
        return false;
    }

    return true;
}

}  // namespace

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

    // One-shot action outcome (P0-2) — lets callers/tests verify that a timeout
    // or rejection was acted on, and lets logs correlate failures to action_id.
    result_pub_ = this->create_publisher<msg::ActionResult>(
        "/action_result", 10);

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
    const std::string& type = msg->action_type;

    // P0-2: remember the caller's id for /action_result correlation.
    current_action_id_ = msg->action_id;

    // open / home are EMERGENCY RESET commands — always accepted even when busy
    bool is_reset = (type == "home" || msg->grasp_type == "open");
    if (!is_reset && state_ != State::IDLE) {
        RCLCPP_WARN(get_logger(),
            "Busy (state=%d). Ignoring action '%s'. "
            "Use 'open' or 'home' to reset.",
            static_cast<int>(state_), type.c_str());
        return;
    }

    // P0-5: entry validation — reject malformed / out-of-range commands. 'home'
    // uses internal constants (not msg fields), so it skips validation.
    if (type != "home") {
        std::string why;
        if (!validateActionCommand(*msg, why)) {
            RCLCPP_WARN(get_logger(),
                "Rejected action '%s': %s (tgt=[%.3f %.3f %.3f] "
                "q=(%.3f,%.3f,%.3f,%.3f) conv=%.2f settle=%.2f grasp=%.2f)",
                type.c_str(), why.c_str(),
                msg->target_x, msg->target_y, msg->target_z,
                msg->ori_x, msg->ori_y, msg->ori_z, msg->ori_w,
                msg->convergence_timeout, msg->settle_time, msg->grasp_duration);
            position_error_ = 0.0;
            orientation_error_ = 0.0;
            total_elapsed_ = 0.0;
            publishResult(false, "VALIDATION_REJECT");
            return;
        }
    }

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
        printf("[CMD] HOME → x=%.3f y=%.3f z=%.3f grasp=%s\n",
            target_x_, target_y_, target_z_, grasp_type_.c_str());

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
        if (has_orientation_) {
            Eigen::Quaterniond q(ori_w_, ori_x_, ori_y_, ori_z_);
            Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
            printf("[CMD] REACH → x=%.3f y=%.3f z=%.3f R=%.2f P=%.2f Y=%.2f\n",
                target_x_, target_y_, target_z_, rpy(0), rpy(1), rpy(2));
        } else {
            printf("[CMD] REACH → x=%.3f y=%.3f z=%.3f\n",
                target_x_, target_y_, target_z_);
        }

    } else if (type == "grasp") {
        // Hand only (orientation + grasp)
        ori_x_ = msg->ori_x; ori_y_ = msg->ori_y; ori_z_ = msg->ori_z; ori_w_ = msg->ori_w;
        has_orientation_ = hasUserOri(msg);
        grasp_type_ = msg->grasp_type;
        grasp_duration_ = msg->grasp_duration;
        startGrasping();
        if (has_orientation_) {
            Eigen::Quaterniond q(ori_w_, ori_x_, ori_y_, ori_z_);
            Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
            printf("[CMD] GRASP → %s (%.2fs) R=%.2f P=%.2f Y=%.2f\n",
                grasp_type_.c_str(), grasp_duration_, rpy(0), rpy(1), rpy(2));
        } else {
            printf("[CMD] GRASP → %s (%.2fs)\n",
                grasp_type_.c_str(), grasp_duration_);
        }

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
        if (has_orientation_) {
            Eigen::Quaterniond q(ori_w_, ori_x_, ori_y_, ori_z_);
            Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
            printf("[CMD] REACH_AND_GRASP → x=%.3f y=%.3f z=%.3f R=%.2f P=%.2f Y=%.2f → %s (%.2fs)\n",
                target_x_, target_y_, target_z_, rpy(0), rpy(1), rpy(2),
                grasp_type_.c_str(), grasp_duration_);
        } else {
            printf("[CMD] REACH_AND_GRASP → x=%.3f y=%.3f z=%.3f → %s (%.2fs)\n",
                target_x_, target_y_, target_z_, grasp_type_.c_str(), grasp_duration_);
        }

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
    // P0-2: stamp every feedback so convergence requires FRESH data. Stale
    // feedback (bridge/controller dead) must not be treated as evidence of
    // arrival — otherwise the sequencer "converges" on an old error.
    last_pos_err_time_ = this->now();
    pos_err_time_valid_ = true;
}

void ActionSequencerNode::oriErrorCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    orientation_error_ = msg->data;
    ori_error_received_ = true;
    last_ori_err_time_ = this->now();
    ori_err_time_valid_ = true;
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

    // Hard per-action budget across REACHING + SETTLING. The phase-level
    // convergence_timeout_ resets every time the arm bounces back from SETTLING
    // (overshoot), so a persistently-oscillating arm could loop forever; the
    // total budget does not reset, and aborts the whole action if exceeded.
    total_elapsed_ = 0.0;
    total_timeout_ = convergence_timeout_ + settle_time_ + 2.0;

    // Reset convergence telemetry. Without this, stale errors from the previous
    // action (e.g. a converged 5 mm residual) would make the FIRST REACHING cycle
    // think position/orientation already converged and jump straight to grasp
    // before the arm starts moving.
    position_error_ = 999.0;
    error_received_ = false;
    orientation_error_ = 999.0;
    ori_error_received_ = false;
    // P0-2: feedback freshness must also reset — a timestamp from the previous
    // action must not make the new action's feedback look "fresh".
    pos_err_time_valid_ = false;
    ori_err_time_valid_ = false;

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
    } else {
        // No orientation → HOLD the current wrist orientation (identity = no change).
        // Previously this sent the injection quat [0.5,0.5,0.5,0.5] as an ABSOLUTE
        // world palm orientation. That was designed for the Ruka mount; on the Inspire
        // mount it demands a large wrist swing during arm reaching → arm+wrist fight
        // → oscillation. Holding avoids the disturbance entirely.
        auto cmd = msg::HandCommand();
        cmd.ori_x = 0.0; cmd.ori_y = 0.0; cmd.ori_z = 0.0; cmd.ori_w = 1.0;  // identity = hold
        cmd.grasp_type = "";
        cmd.grasp_duration = 0.0;
        hand_cmd_pub_->publish(cmd);
    }
}

void ActionSequencerNode::startGrasping()
{
    state_ = State::GRASPING;
    state_elapsed_ = 0.0;

    // Phase 2: grasp only. Identity quat → hold wrist orientation. grasp_type → close fingers.
    // P1-7: forward grasp_duration_ as the finger interpolation ramp — the hand
    // now rates-limits the finger target over that time (<=0 = immediate).
    auto cmd = msg::HandCommand();
    cmd.ori_x = 0.0; cmd.ori_y = 0.0; cmd.ori_z = 0.0; cmd.ori_w = 1.0;  // identity → hold
    cmd.grasp_type = grasp_type_;
    cmd.grasp_duration = grasp_duration_;
    hand_cmd_pub_->publish(cmd);
}

void ActionSequencerNode::publishResult(bool success, const std::string& reason)
{
    auto res = msg::ActionResult();
    res.action_id = current_action_id_;
    res.success = success;
    res.reason = reason;
    res.position_error_m = position_error_;
    // No orientation requested → no meaningful orientation error (999 reset
    // sentinel must not leak into a result).
    res.orientation_error_rad = has_orientation_ ? orientation_error_ : 0.0;
    res.duration_s = total_elapsed_;
    result_pub_->publish(res);
}

void ActionSequencerNode::finishAction(bool success, const std::string& reason)
{
    printf("[SEQ] %s (action_id=%d) pos=%.1f mm ori=%.3f rad elapsed=%.2f s\n",
        reason.c_str(), current_action_id_,
        position_error_ * 1000.0,
        has_orientation_ ? orientation_error_ : 0.0,  // no ori requested → 0, not the 999 sentinel
        state_elapsed_);
    fflush(stdout);
    publishResult(success, reason);
    state_ = State::IDLE;
}

bool ActionSequencerNode::posErrorFresh() const
{
    return pos_err_time_valid_
        && (this->now() - last_pos_err_time_).seconds() <= feedback_timeout_;
}

bool ActionSequencerNode::oriErrorFresh() const
{
    return ori_err_time_valid_
        && (this->now() - last_ori_err_time_).seconds() <= feedback_timeout_;
}

void ActionSequencerNode::publishHoldCommand()
{
    auto cmd = msg::HandCommand();
    cmd.ori_x = 0.0; cmd.ori_y = 0.0; cmd.ori_z = 0.0; cmd.ori_w = 1.0;  // identity → hold
    cmd.grasp_type = "";
    cmd.grasp_duration = 0.0;
    hand_cmd_pub_->publish(cmd);
}

// ============================================================================
// Control loop (50 Hz) — state machine
// ============================================================================

void ActionSequencerNode::controlLoop()
{
    switch (state_) {
    case State::IDLE:
        // P0-2: liveness heartbeat — keeps the hand's command watchdog armed
        // while idle, so a held grasp persists but a dead sequencer unloads.
        publishHoldCommand();
        break;

    case State::REACHING:
        state_elapsed_ += dt_;
        total_elapsed_ += dt_;

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
                // No orientation specified → hold current (identity), no swing demand
                cmd.ori_x = 0.0; cmd.ori_y = 0.0; cmd.ori_z = 0.0; cmd.ori_w = 1.0;
            }
            cmd.grasp_type = "";       // orientation only, no finger movement
            cmd.grasp_duration = 0.0;
            hand_cmd_pub_->publish(cmd);
        }

        // Phase 1 convergence: BOTH position AND orientation. Convergence
        // requires FRESH feedback (P0-2) — an old error from a dead feedback
        // stream is not evidence of arrival.
        {
            bool pos_ok = posErrorFresh() && position_error_ <= 0.015;

            // Saturation detection: if wrist offset hits limit, ori_err stops improving.
            // Track best error and time since last improvement → skip wait if stuck.
            bool ori_stuck = false;
            if (has_orientation_ && oriErrorFresh()) {
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
                       || (oriErrorFresh() && orientation_error_ < ori_convergence_tol_);

            if (pos_ok && (ori_ok || ori_stuck)) {
                if (ori_stuck) {
                    printf("[SEQ] Wrist orientation stuck at %.3f rad, proceeding anyway\n",
                        orientation_error_);
                    fflush(stdout);
                }
                if (grasp_type_.empty()) {
                    finishAction(true, "CONVERGED");  // reach only
                } else {
                    state_ = State::SETTLING;
                    state_elapsed_ = 0.0;
                }
            }
        }

        // Both guards are gated on still being REACHING: the convergence block
        // above may already have finished the action (reach-only) or moved to
        // SETTLING, and a double-finish on the boundary cycle would emit a false
        // abort after a success.
        if (state_ == State::REACHING && state_elapsed_ > convergence_timeout_) {
            // P0: timeout MUST abort — never proceed to grasp without a converged
            // (fresh) position. Closing the fingers at an unreached target is unsafe.
            printf("[SEQ] REACHING timeout (%.1fs pos=%.3f m ori=%.3f rad) → abort, no grasp\n",
                state_elapsed_, position_error_, orientation_error_);
            fflush(stdout);
            finishAction(false, "REACH_TIMEOUT");
            break;
        }

        // Hard per-action budget (across REACHING+SETTLING, see startReaching)
        if (state_ == State::REACHING && total_elapsed_ > total_timeout_) {
            printf("[SEQ] ACTION budget exceeded (%.1fs > %.1fs) → abort, no grasp\n",
                total_elapsed_, total_timeout_);
            fflush(stdout);
            finishAction(false, "ACTION_BUDGET");
            break;
        }
        break;

    case State::SETTLING:
        state_elapsed_ += dt_;
        total_elapsed_ += dt_;

        // Keep publishing position target (arm stays locked)
        {
            auto pt = geometry_msgs::msg::Point();
            pt.x = target_x_;
            pt.y = target_y_;
            pt.z = target_z_;
            point_pub_->publish(pt);
        }

        // P0-2: keep the hand command stream alive (liveness heartbeat).
        publishHoldCommand();

        // Hard per-action budget: if settle (incl. REACHING↔SETTLING bounces)
        // drags on, abort the whole action instead of grasping late.
        if (total_elapsed_ > total_timeout_) {
            printf("[SEQ] ACTION budget exceeded in SETTLING (%.1fs > %.1fs) → abort\n",
                total_elapsed_, total_timeout_);
            fflush(stdout);
            finishAction(false, "ACTION_BUDGET");
            break;
        }

        if (state_elapsed_ >= settle_time_) {
            startGrasping();
        }

        // Overshoot check — only while still settling AND feedback is fresh.
        // Once grasp has started (same-cycle startGrasping above), do NOT revert
        // to REACHING mid-grasp.
        if (state_ == State::SETTLING && posErrorFresh() && position_error_ > 0.03) {
            state_ = State::REACHING;
            state_elapsed_ = 0.0;
        }
        break;

    case State::GRASPING:
        state_elapsed_ += dt_;

        // P0-2: re-publish the grasp command every cycle so the hand's command
        // watchdog sees a live stream. Idempotent: the hand re-arms interpolation
        // only when the pattern target changes, so re-publishing with the same
        // grasp_duration_ does not restart the ramp.
        {
            auto cmd = msg::HandCommand();
            cmd.ori_x = 0.0; cmd.ori_y = 0.0; cmd.ori_z = 0.0; cmd.ori_w = 1.0;
            cmd.grasp_type = grasp_type_;
            cmd.grasp_duration = grasp_duration_;
            hand_cmd_pub_->publish(cmd);
        }

        if (state_elapsed_ >= grasp_duration_) {
            finishAction(true, "DONE");
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
