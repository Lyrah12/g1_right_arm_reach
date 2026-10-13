#include "g1_right_arm_reach/hand_controller.hpp"

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

namespace g1_right_arm_reach {

HandController::HandController() : Node("hand_controller")
{
    const std::vector<std::string> default_names{
        "right_thumb_proximal_joint", "right_thumb_metacarpal_joint",
        "right_index_proximal_joint", "right_middle_proximal_joint",
        "right_ring_proximal_joint", "right_pinky_proximal_joint"};
    declare_parameter("joint_names", default_names);
    declare_parameter("lower_limits", std::vector<double>(6, 0.0));
    declare_parameter("upper_limits", std::vector<double>{
        1.0297443, 1.5707963, 1.4137167, 1.4137167, 1.4137167, 1.4137167});
    declare_parameter("control_rate", 50.0);
    declare_parameter("motion.speed_scale", 2.0);
    declare_parameter("command_timeout", 0.20);
    declare_parameter("safety.enable_output", false);
    declare_parameter("safety.deadman_required", false);
    declare_parameter("safety.deadman_timeout", 0.10);
    declare_parameter("safety.joint_state_timeout", 0.10);
    declare_parameter("tactile.enable", false);
    declare_parameter("tactile.timeout", 0.20);
    declare_parameter("tactile.target_force", 0.35);
    declare_parameter("tactile.max_force", 0.75);
    declare_parameter("tactile.position_gain", 0.20);
    declare_parameter("topics.command", "/hand_command");
    declare_parameter("topics.joint_states", "/joint_states");
    declare_parameter("topics.sim_command", "/hand_commands");
    declare_parameter("topics.trajectory_command", "/right_revo2_hand_controller/joint_trajectory");
    declare_parameter("topics.tactile_summary", "/revo2/tactile_force_summary");
    declare_parameter("topics.deadman", "/hand_deadman");

    joint_names_ = get_parameter("joint_names").as_string_array();
    const auto lower = get_parameter("lower_limits").as_double_array();
    const auto upper = get_parameter("upper_limits").as_double_array();
    if (joint_names_.size() != 6 || lower.size() != 6 || upper.size() != 6) {
        throw std::runtime_error("Revo2 controller requires exactly six joints and limits");
    }
    lower_limits_ = Eigen::Map<const Eigen::VectorXd>(lower.data(), lower.size());
    upper_limits_ = Eigen::Map<const Eigen::VectorXd>(upper.data(), upper.size());
    actual_position_ = lower_limits_;
    desired_position_ = lower_limits_;
    final_position_ = lower_limits_;
    tactile_force_ = Eigen::VectorXd::Zero(5);  // thumb, index, middle, ring, pinky

    const double rate = get_parameter("control_rate").as_double();
    if (!std::isfinite(rate) || rate <= 0.0) {
        throw std::runtime_error("control_rate must be finite and positive");
    }
    dt_ = 1.0 / rate;
    speed_scale_ = get_parameter("motion.speed_scale").as_double();
    if (!std::isfinite(speed_scale_) || speed_scale_ <= 0.0) {
        throw std::runtime_error("motion.speed_scale must be finite and positive");
    }
    command_timeout_ = get_parameter("command_timeout").as_double();
    output_enabled_ = get_parameter("safety.enable_output").as_bool();
    deadman_required_ = get_parameter("safety.deadman_required").as_bool();
    deadman_timeout_ = get_parameter("safety.deadman_timeout").as_double();
    state_timeout_ = get_parameter("safety.joint_state_timeout").as_double();
    if (command_timeout_ < 0.0 || state_timeout_ <= 0.0 || deadman_timeout_ <= 0.0) {
        throw std::runtime_error("hand safety timeouts are invalid");
    }
    tactile_enabled_ = get_parameter("tactile.enable").as_bool();
    tactile_timeout_ = get_parameter("tactile.timeout").as_double();
    tactile_target_ = get_parameter("tactile.target_force").as_double();
    tactile_max_ = get_parameter("tactile.max_force").as_double();
    tactile_gain_ = get_parameter("tactile.position_gain").as_double();

    command_sub_ = create_subscription<msg::HandCommand>(
        get_parameter("topics.command").as_string(), 10,
        std::bind(&HandController::handCommandCallback, this, std::placeholders::_1));
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("topics.joint_states").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&HandController::jointStateCallback, this, std::placeholders::_1));
    tactile_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        get_parameter("topics.tactile_summary").as_string(), 10,
        std::bind(&HandController::tactileCallback, this, std::placeholders::_1));
    deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
        get_parameter("topics.deadman").as_string(), 10,
        [this](const std_msgs::msg::Bool::SharedPtr value) {
            deadman_ = value->data;
            last_deadman_time_ = now();
        });
    sim_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        get_parameter("topics.sim_command").as_string(), 10);
    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        get_parameter("topics.trajectory_command").as_string(), 10);

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(dt_)),
        std::bind(&HandController::controlLoop, this));
    RCLCPP_WARN(get_logger(),
                "Revo2 output is %s; tactile stabilization is %s; deadman is %s",
                output_enabled_ ? "ENABLED" : "LOCKED",
                tactile_enabled_ ? "ENABLED" : "disabled",
                deadman_required_ ? "required" : "not required");
}

void HandController::handCommandCallback(const msg::HandCommand::SharedPtr command)
{
    // The action sequencer repeats messages to survive DDS discovery. Accept
    // the first copy only, otherwise every copy restarts grasp interpolation.
    if (command->command_id != 0 && have_command_id_
        && command->command_id == last_command_id_) {
        // The action sequencer repeats the same ID as a keepalive. Refresh the
        // timeout while the target is still valid, but never revive a target
        // after deadman, feedback, or command-timeout invalidation.
        if (have_command_) last_command_time_ = now();
        return;
    }

    Eigen::VectorXd target(6);
    if (command->use_joint_positions) {
        for (size_t i = 0; i < 6; ++i) target(i) = command->joint_positions[i];
    } else {
        const auto pattern = grasp_library_.getPattern(command->grasp_type);
        if (pattern.name.empty()) {
            RCLCPP_ERROR(get_logger(), "Unknown Revo2 grasp '%s'", command->grasp_type.c_str());
            return;
        }
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            target(i) = pattern.joints.at(joint_names_[i]);
        }
    }
    if (setTarget(target, command->grasp_duration, command->grasp_type)) {
        have_command_ = true;
        if (command->command_id != 0) {
            last_command_id_ = command->command_id;
            have_command_id_ = true;
        }
        last_command_time_ = now();
        RCLCPP_INFO(get_logger(),
                    "Accepted Revo2 command id=%d grasp=%s duration=%.2fs interpolation=%.2fs",
                    command->command_id, command->grasp_type.c_str(),
                    command->grasp_duration, command->grasp_duration / speed_scale_);
    }
}

void HandController::invalidateCommand(const char* reason)
{
    if (have_command_) {
        RCLCPP_WARN(get_logger(), "Revo2 command invalidated: %s", reason);
    }
    have_command_ = false;
    interpolation_remaining_ = 0.0;
    tactile_valid_ = false;
    active_grasp_.clear();
}

bool HandController::setTarget(const Eigen::VectorXd& target, double duration,
                               const std::string& grasp_name)
{
    if (target.size() != 6 || !target.allFinite()
        || !std::isfinite(duration) || duration < 0.0) return false;
    for (int i = 0; i < target.size(); ++i) {
        if (target(i) < lower_limits_(i) || target(i) > upper_limits_(i)) {
            RCLCPP_ERROR(get_logger(), "Rejected Revo2 target: joint %s out of range",
                         joint_names_[i].c_str());
            return false;
        }
    }
    if (!have_command_ && have_state_) desired_position_ = actual_position_;
    // De-duplicate identical retries (the sequencer repeats the grasp while it
    // is active so a single DDS race cannot drop it): restarting the
    // interpolation on every duplicate would keep resetting it and the fingers
    // would never finish closing.
    if (!have_command_ || grasp_name != active_grasp_
        || !target.isApprox(active_target_, 1.0e-6)) {
        final_position_ = target;
        interpolation_remaining_ = duration / speed_scale_;
    }
    active_target_ = target;
    active_grasp_ = grasp_name;
    return true;
}

void HandController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state)
{
    bool complete = true;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const auto iterator = std::find(state->name.begin(), state->name.end(), joint_names_[i]);
        if (iterator == state->name.end()) {
            complete = false;
            continue;
        }
        const size_t index = std::distance(state->name.begin(), iterator);
        if (index >= state->position.size() || !std::isfinite(state->position[index])) {
            complete = false;
            continue;
        }
        actual_position_(i) = state->position[index];
    }
    if (complete) {
        if (!have_state_ && !have_command_) {
            desired_position_ = actual_position_;
            final_position_ = actual_position_;
        }
        have_state_ = true;
        last_state_time_ = now();
    }
}

void HandController::tactileCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr tactile)
{
    if (tactile->data.size() < 5) {
        tactile_valid_ = false;
        return;
    }
    for (int i = 0; i < 5; ++i) tactile_force_(i) = tactile->data[i];
    tactile_valid_ = tactile_force_.allFinite();
    if (tactile_valid_) last_tactile_time_ = now();
}

void HandController::applyTactileStabilization(Eigen::VectorXd& target)
{
    if (!tactile_enabled_ || !tactile_valid_
        || (active_grasp_ != "pinch" && active_grasp_ != "tripod"
            && active_grasp_ != "cable_pinch")) return;

    // Input is a normalized 0..1 per-fingertip normal-force summary produced by
    // the hardware adapter. Simulators may omit it. Thumb controls indices 0/1;
    // index and optional middle control indices 2/3. Thumb opposition is not
    // adjusted after contact; only thumb flexion is.
    const auto adjust = [this, &target](int joint, double measured) {
        // Finish the approach before seeking contact. Excess force can stop
        // and release the affected finger even while the others are closing.
        if (interpolation_remaining_ > 0.0 && measured <= tactile_max_) return;
        const double delta = measured > tactile_max_
            ? -tactile_gain_ * (measured - tactile_max_) * dt_
            : tactile_gain_ * (tactile_target_ - measured) * dt_;
        target(joint) = std::clamp(target(joint) + delta,
                                   lower_limits_(joint), upper_limits_(joint));
        final_position_(joint) = target(joint);
    };
    adjust(0, tactile_force_(0));
    adjust(2, tactile_force_(1));
    if (active_grasp_ != "pinch") adjust(3, tactile_force_(2));
}

void HandController::publish(const Eigen::VectorXd& target)
{
    sensor_msgs::msg::JointState simulation_command;
    simulation_command.header.stamp = now();
    simulation_command.name = joint_names_;
    simulation_command.position.resize(6);
    simulation_command.velocity.assign(6, 0.0);
    simulation_command.effort.assign(6, 0.0);
    for (int i = 0; i < 6; ++i) simulation_command.position[i] = target(i);
    sim_command_pub_->publish(simulation_command);

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = simulation_command.position;
    point.time_from_start =
        rclcpp::Duration::from_seconds(std::max(0.04, dt_ * 2.0));
    trajectory.points.push_back(point);
    trajectory_pub_->publish(trajectory);
}

void HandController::controlLoop()
{
    if (!have_state_ || !have_command_) return;

    const rclcpp::Time current_time = now();
    const bool state_fresh =
        (current_time - last_state_time_).seconds() <= state_timeout_;
    if (output_enabled_ && !state_fresh) {
        invalidateCommand("joint state is stale");
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Revo2 output inhibited: joint state is stale");
        return;
    }
    const bool deadman_fresh = !deadman_required_
        || (deadman_ && last_deadman_time_.nanoseconds() > 0
            && (current_time - last_deadman_time_).seconds() <= deadman_timeout_);
    if (output_enabled_ && !deadman_fresh) {
        invalidateCommand("hand deadman is false or stale");
        // Do not advance interpolation while inhibited: re-enabling the hand
        // must receive a new command instead of exposing a stale target.
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Revo2 output inhibited: /hand_deadman is false or stale");
        return;
    }
    if (command_timeout_ > 0.0
        && (current_time - last_command_time_).seconds() > command_timeout_) {
        invalidateCommand("hand command is stale");
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Revo2 output inhibited: hand command is stale");
        return;
    }
    if (last_tactile_time_.nanoseconds() == 0
        || (current_time - last_tactile_time_).seconds() > tactile_timeout_) {
        tactile_valid_ = false;
    }

    if (interpolation_remaining_ > 0.0) {
        const double fraction = std::min(1.0, dt_ / interpolation_remaining_);
        desired_position_ += fraction * (final_position_ - desired_position_);
        interpolation_remaining_ = std::max(0.0, interpolation_remaining_ - dt_);
    } else {
        desired_position_ = final_position_;
    }
    Eigen::VectorXd output = desired_position_;
    applyTactileStabilization(output);
    desired_position_ = output;
    if (output_enabled_) publish(output);
}

}  // namespace g1_right_arm_reach

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<g1_right_arm_reach::HandController>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("hand_controller"), "%s", error.what());
    }
    rclcpp::shutdown();
    return 0;
}
