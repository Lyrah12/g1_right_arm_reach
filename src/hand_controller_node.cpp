/**
 * @file hand_controller_node.cpp
 * @brief Hand module: wrist orientation PID + finger impedance control (50 Hz, Plan C).
 *
 * Plan C: Arm-Hand fully decoupled.
 *   - reach_node: 4-DOF arm → wrist position
 *   - This node:  23-DOF hand → orientation + grasp
 *
 * Control:
 *   Wrist:   orientation PID → J⁻¹(3×3) → offset-based position targets
 *   Fingers: impedance τ = Kp·e + Kd·ė → position-encoded torque + smoothstep trajectory
 *
 * @subscribed /hand_command             HandCommand (quat + grasp)
 * @subscribed /joint_states             actual wrist + finger positions
 * @subscribed /actual_hand_orientation  QuaternionStamped (world-frame palm ori)
 * @subscribed /mujoco_wrist_jacobian    Float64MultiArray 3×3 (row-major)
 *
 * @published  /hand_commands            JointState, 23-DOF (wrist×3 + fingers×20)
 * @published  /debug/orientation_error  Float64 (rad)
 */

#include "g1_right_arm_reach/hand_controller.hpp"

#include <algorithm>
#include <cmath>

namespace g1_right_arm_reach {

using namespace std::literals::chrono_literals;

// ============================================================================
// Constructor
// ============================================================================

HandController::HandController()
    : Node("hand_controller")
{
    // --- Parameters: joint names ---
    this->declare_parameter("wrist_joint_names",
        std::vector<std::string>{
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        });

    this->declare_parameter("finger_joint_names",
        std::vector<std::string>{
            "right_finger1_joint1", "right_finger1_joint2",
            "right_finger1_joint3", "right_finger1_joint4",
            "right_finger2_joint1", "right_finger2_joint2",
            "right_finger2_joint3", "right_finger2_joint4",
            "right_finger3_joint1", "right_finger3_joint2",
            "right_finger3_joint3", "right_finger3_joint4",
            "right_finger4_joint1", "right_finger4_joint2",
            "right_finger4_joint3", "right_finger4_joint4",
            "right_finger5_joint1", "right_finger5_joint2",
            "right_finger5_joint3", "right_finger5_joint4",
        });

    wrist_names_ = this->get_parameter("wrist_joint_names").as_string_array();
    finger_names_ = this->get_parameter("finger_joint_names").as_string_array();
    n_wrist_ = static_cast<int>(wrist_names_.size());
    n_fingers_ = static_cast<int>(finger_names_.size());
    n_total_ = n_wrist_ + n_fingers_;

    // Build combined name list
    all_names_.reserve(n_total_);
    for (const auto& n : wrist_names_) all_names_.push_back(n);
    for (const auto& n : finger_names_) all_names_.push_back(n);

    // --- Parameters: control gains ---
    this->declare_parameter("kp_wrist", 2.0);
    this->declare_parameter("ki_wrist", 0.05);
    this->declare_parameter("kd_wrist", 0.12);
    this->declare_parameter("wrist_vel_limit", 3.0);
    this->declare_parameter("wrist_delta_limit", 0.12);
    this->declare_parameter("wrist_offset_limit", 2.0);

    this->declare_parameter("kp_finger", 200.0);
    this->declare_parameter("kd_finger", 0.3);
    this->declare_parameter("tau_max_finger", 50.0);
    this->declare_parameter("finger_delta_limit", 0.60);
    this->declare_parameter("inner_servo_kp", 300.0);
    this->declare_parameter("interp_duration", 0.05);

    kp_wrist_ = this->get_parameter("kp_wrist").as_double();
    ki_wrist_ = this->get_parameter("ki_wrist").as_double();
    kd_wrist_ = this->get_parameter("kd_wrist").as_double();
    wrist_vel_limit_ = this->get_parameter("wrist_vel_limit").as_double();
    wrist_delta_limit_ = this->get_parameter("wrist_delta_limit").as_double();
    wrist_offset_limit_ = this->get_parameter("wrist_offset_limit").as_double();

    kp_finger_ = this->get_parameter("kp_finger").as_double();
    kd_finger_ = this->get_parameter("kd_finger").as_double();
    tau_max_finger_ = this->get_parameter("tau_max_finger").as_double();
    finger_delta_limit_ = this->get_parameter("finger_delta_limit").as_double();
    inner_servo_kp_ = this->get_parameter("inner_servo_kp").as_double();

    // --- State vectors ---
    actual_q_ = Eigen::VectorXd::Zero(n_total_);
    actual_dq_ = Eigen::VectorXd::Zero(n_total_);
    finger_target_q_ = Eigen::VectorXd::Zero(n_fingers_);
    finger_start_q_ = Eigen::VectorXd::Zero(n_fingers_);
    finger_des_q_ = Eigen::VectorXd::Zero(n_fingers_);
    finger_cmd_q_ = Eigen::VectorXd::Zero(n_fingers_);

    // --- ROS2 communication ---
    hand_cmd_sub_ = this->create_subscription<msg::HandCommand>(
        this->declare_parameter("hand_command_topic", "/hand_command"),
        10,
        std::bind(&HandController::handCommandCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        this->declare_parameter("joint_state_topic", "/joint_states"),
        10,
        std::bind(&HandController::jointStateCallback, this, std::placeholders::_1));

    hand_ori_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        this->declare_parameter("hand_orientation_topic", "/actual_hand_orientation"),
        10,
        std::bind(&HandController::handOrientationCallback, this, std::placeholders::_1));

    wrist_jac_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        this->declare_parameter("wrist_jacobian_topic", "/mujoco_wrist_jacobian"),
        10,
        std::bind(&HandController::wristJacobianCallback, this, std::placeholders::_1));

    hand_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/hand_commands", 10);

    ori_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/debug/orientation_error", 10);

    // --- Control timer (50 Hz) ---
    dt_ = 0.02;
    control_timer_ = this->create_wall_timer(
        20ms, std::bind(&HandController::controlLoop, this));

    // --- Parameter callback for runtime tuning ---
    [[maybe_unused]] auto _param_cb = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& p : params) {
                if (p.get_name() == "kp_wrist") kp_wrist_ = p.as_double();
                else if (p.get_name() == "ki_wrist") ki_wrist_ = p.as_double();
                else if (p.get_name() == "kd_wrist") kd_wrist_ = p.as_double();
                else if (p.get_name() == "wrist_offset_limit" && p.as_double() > 0.0)
                    wrist_offset_limit_ = p.as_double();
                else if (p.get_name() == "kp_finger") kp_finger_ = p.as_double();
                else if (p.get_name() == "kd_finger") kd_finger_ = p.as_double();
                else if (p.get_name() == "tau_max_finger") tau_max_finger_ = p.as_double();
                else if (p.get_name() == "inner_servo_kp" && p.as_double() > 0.0)
                    inner_servo_kp_ = p.as_double();
            }
            return result;
        });

    RCLCPP_INFO(get_logger(),
        "HandController ready (Plan C). %d joints: wrist=%d + fingers=%d. "
        "GraspLibrary: %zu patterns. "
        "Wrist: PID + MuJoCo J(3×3). Fingers: impedance (τ=Kp·e+Kd·ė→pos encoding).",
        n_total_, n_wrist_, n_fingers_, grasp_library_.size());
}

// ============================================================================
// Callbacks
// ============================================================================

void HandController::handCommandCallback(const msg::HandCommand::SharedPtr msg)
{
    // --- Wrist orientation target ---
    // Identity quaternion (0,0,0,1) → no orientation change (hold current).
    // Non-identity quaternions are interpreted as rotations RELATIVE to current palm orientation:
    //   target_abs = current_palm * q_user
    // Injection quat [0.5, 0.5, 0.5, 0.5] is treated as an ABSOLUTE world-frame target
    // (reset to default palm pose, fingers +X, palm −Y).
    bool is_identity = std::abs(msg->ori_x) < 1e-9 && std::abs(msg->ori_y) < 1e-9
                    && std::abs(msg->ori_z) < 1e-9 && std::abs(msg->ori_w - 1.0) < 1e-9;

    if (!is_identity) {
        Eigen::Quaterniond new_target(msg->ori_w, msg->ori_x, msg->ori_y, msg->ori_z);
        new_target.normalize();

        // Detect injection quat reset [0.5, 0.5, 0.5, 0.5]
        bool is_injection = std::abs(msg->ori_x - 0.5) < 1e-6
                         && std::abs(msg->ori_y - 0.5) < 1e-6
                         && std::abs(msg->ori_z - 0.5) < 1e-6
                         && std::abs(msg->ori_w - 0.5) < 1e-6;

        if (is_injection) {
            // Reset to default palm pose (injection quat = absolute world-frame target)
            target_ori_ = new_target;
            has_ori_target_ = true;
            wrist_first_ = true;
            wrist_error_integral_.setZero();
            wrist_cycles_since_step_ = 0;

            RCLCPP_INFO(get_logger(),
                "Hand command: ori=RESET (injection quat [0.5,0.5,0.5,0.5]) grasp=%s dur=%.2fs",
                msg->grasp_type.c_str(), msg->grasp_duration);
        } else {
            // ── Relative orientation: user quat is relative to CURRENT palm ──
            // Compose: target_abs = current_hand_ori * user_quat
            // Cache user quat to avoid recomposition on repeated sequencer publishes.
            bool user_changed = (new_target.coeffs() - last_user_ori_.coeffs()).norm() > 1e-8;

            if (user_changed) {
                last_user_ori_ = new_target;

                if (hand_ori_received_) {
                    // Compose user rotation with current palm orientation
                    Eigen::Quaterniond abs_target = current_hand_ori_ * new_target;
                    abs_target.normalize();
                    target_ori_ = abs_target;

                    // Validate: if the composed target is beyond wrist authority, warn
                    double jump = orientationError(target_ori_, current_hand_ori_).norm();
                    if (jump > wrist_offset_limit_) {
                        RCLCPP_WARN(get_logger(),
                            "Orientation target requires %.2f rad (%.0f°) rotation. "
                            "Wrist offset limit is ±%.1f rad → may not reach. "
                            "Smaller RPY values or pre-positioning the arm may help.",
                            jump, jump * 57.3, wrist_offset_limit_);
                    }
                } else {
                    // Palm orientation not yet received → defer composition.
                    // handOrientationCallback will recompose when first ori arrives.
                    target_ori_ = new_target;  // temporary, will be replaced
                }

                has_ori_target_ = true;
                wrist_first_ = true;
                wrist_error_integral_.setZero();
                wrist_cycles_since_step_ = 0;

                RCLCPP_INFO(get_logger(),
                    "Hand command: ori=REL [%.2f,%.2f,%.2f,%.2f] → abs=[%.2f,%.2f,%.2f,%.2f] grasp=%s dur=%.2fs",
                    msg->ori_x, msg->ori_y, msg->ori_z, msg->ori_w,
                    target_ori_.w(), target_ori_.x(), target_ori_.y(), target_ori_.z(),
                    msg->grasp_type.c_str(), msg->grasp_duration);
            }
            // else: same user quat re-published (sequencer loop) → target unchanged
        }
    } else {
        RCLCPP_INFO(get_logger(),
            "Hand command: ori=HOLD grasp=%s dur=%.2fs",
            msg->grasp_type.c_str(), msg->grasp_duration);
    }
    // else: has_ori_target_ stays as-is (no change requested)

    // --- Finger grasp target ---
    const std::string& gtype = msg->grasp_type;
    if (!gtype.empty()) {
        // Use ACTUAL positions as interpolation start
        for (int j = 0; j < n_fingers_; ++j) {
            finger_start_q_(j) = actual_q_(n_wrist_ + j);
        }

        if (gtype == "custom") {
            // custom mode not supported in HandCommand yet; fallback to home
            RCLCPP_WARN(get_logger(), "Custom grasp not yet supported in HandCommand. Using 'home'.");
            auto pattern = grasp_library_.getPattern("home");
            for (int j = 0; j < n_fingers_; ++j) {
                auto it = pattern.joints.find(finger_names_[j]);
                finger_target_q_(j) = (it != pattern.joints.end()) ? it->second : actual_q_(n_wrist_ + j);
            }
        } else {
            auto pattern = grasp_library_.getPattern(gtype);
            if (pattern.name.empty()) {
                RCLCPP_WARN(get_logger(),
                    "Unknown grasp type '%s'. Defaulting to 'home'.", gtype.c_str());
                pattern = grasp_library_.getPattern("home");
            }
            for (int j = 0; j < n_fingers_; ++j) {
                auto it = pattern.joints.find(finger_names_[j]);
                finger_target_q_(j) = (it != pattern.joints.end()) ? it->second : actual_q_(n_wrist_ + j);
            }
        }

        finger_interp_elapsed_ = 0.0;
        finger_interp_duration_ = (msg->grasp_duration > 0.0) ? msg->grasp_duration
            : this->get_parameter("interp_duration").as_double();
        finger_active_ = true;
        current_grasp_type_ = gtype;
    }
}

void HandController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.empty()) return;

    bool first_time = !joint_state_received_;

    // Read wrist joints (indices 0..2)
    for (int j = 0; j < n_wrist_; ++j) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == wrist_names_[j]) {
                actual_q_(j) = msg->position[i];
                if (i < msg->velocity.size())
                    actual_dq_(j) = msg->velocity[i];
                break;
            }
        }
    }

    // Read finger joints (indices 3..22)
    for (int j = 0; j < n_fingers_; ++j) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == finger_names_[j]) {
                int idx = n_wrist_ + j;
                actual_q_(idx) = msg->position[i];
                if (i < msg->velocity.size())
                    actual_dq_(idx) = msg->velocity[i];

                if (first_time) {
                    // Initialize finger feedforward target = actual position
                    finger_des_q_(j) = actual_q_(idx);
                }
                break;
            }
        }
    }

    if (first_time) {
        joint_state_received_ = true;
        // Initialize wrist offset = 0 (no correction). wrist_cmd = act + 0 = act.
        // The bridge's ready pose holds wrist at 0 via position servo (kp=300).
        // Offset will be modified by stepWristControl when orientation PID activates.
        wrist_offset_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        // Initialize finger command = actual position
        finger_cmd_q_ = actual_q_.tail(n_fingers_);
        RCLCPP_INFO(get_logger(),
            "First joint_state received. Wrist act=[%.3f,%.3f,%.3f]. "
            "Offset=0. PID activates on first orientation+Jacobian.",
            actual_q_(0), actual_q_(1), actual_q_(2));
    }
}

void HandController::handOrientationCallback(
    const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
    current_hand_ori_ = Eigen::Quaterniond(
        msg->quaternion.w, msg->quaternion.x,
        msg->quaternion.y, msg->quaternion.z);
    current_hand_ori_.normalize();
    hand_ori_fresh_ = true;

    if (!hand_ori_received_) {
        hand_ori_received_ = true;

        // If a relative HandCommand arrived before the first palm orientation,
        // target_ori_ was set to the raw user quat (deferred). Re-compose now.
        bool is_injection = std::abs(target_ori_.w() - 0.5) < 1e-6
                         && std::abs(target_ori_.x() - 0.5) < 1e-6
                         && std::abs(target_ori_.y() - 0.5) < 1e-6
                         && std::abs(target_ori_.z() - 0.5) < 1e-6;

        if (has_ori_target_ && !is_injection && last_user_ori_.w() > 1e-9) {
            // Re-compose user quat with the now-available palm orientation
            Eigen::Quaterniond abs_target = current_hand_ori_ * last_user_ori_;
            abs_target.normalize();
            target_ori_ = abs_target;
            RCLCPP_INFO(get_logger(),
                "First hand ori received. Deferred compose: user→abs Target=[%.3f,%.3f,%.3f,%.3f] "
                "Current=[%.3f,%.3f,%.3f,%.3f].",
                target_ori_.w(), target_ori_.x(), target_ori_.y(), target_ori_.z(),
                current_hand_ori_.w(), current_hand_ori_.x(), current_hand_ori_.y(), current_hand_ori_.z());
        } else {
            // No prior HandCommand → default target = injection quat (already set in ctor).
            // We do NOT overwrite target_ori_ with current_ori — this would lock in
            // any gravity-induced droop that happened before the first callback.
            has_ori_target_ = true;
            RCLCPP_INFO(get_logger(),
                "First hand orientation received. Target=[%.3f,%.3f,%.3f,%.3f] "
                "Current=[%.3f,%.3f,%.3f,%.3f]. PID activated.",
                target_ori_.w(), target_ori_.x(), target_ori_.y(), target_ori_.z(),
                current_hand_ori_.w(), current_hand_ori_.x(), current_hand_ori_.y(), current_hand_ori_.z());
        }
    }
}

void HandController::wristJacobianCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Expect 9 elements (3×3, row-major)
    if (msg->data.size() < 9) return;

    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            wrist_jacobian_(r, c) = msg->data[r * 3 + c];

    wrist_jac_fresh_ = true;   // gate wrist control to Jacobian publish rate (~20 Hz)

    if (!wrist_jac_received_) {
        wrist_jac_received_ = true;
        RCLCPP_INFO(get_logger(),
            "Wrist Jacobian received (3×3), |det|=%.6f",
            wrist_jacobian_.determinant());
    }
}

// ============================================================================
// Control loop (50 Hz)
// ============================================================================

void HandController::controlLoop()
{
    if (!joint_state_received_) return;

    // Update feedforward finger trajectory (always runs if active)
    if (finger_active_) {
        stepFingerTrajectory(dt_);
    }

    // Capture actual wrist positions
    Eigen::Vector3d wrist_act = actual_q_.head<3>();

    // Snapshot fresh flags for diagnostic (before consumption)
    bool j_fresh = wrist_jac_fresh_;
    bool o_fresh = hand_ori_fresh_;

    // Wrist orientation control — gated on FRESH feedback (~20 Hz).
    // Without this gate, 50 Hz control with 20 Hz feedback causes stale-Jacobian divergence.
    // Effective dt (dt_ × cycles) corrects integral starvation and derivative over-amplification.
    wrist_cycles_since_step_++;
    if (hand_ori_received_ && wrist_jac_received_ && hand_ori_fresh_ && wrist_jac_fresh_) {
        double effective_dt = dt_ * wrist_cycles_since_step_;
        stepWristControl(effective_dt);
        wrist_cycles_since_step_ = 0;
        hand_ori_fresh_ = false;
        wrist_jac_fresh_ = false;

        // Publish orientation error for sequencer convergence check
        auto err = orientationError(target_ori_, current_hand_ori_);
        auto err_msg = std_msgs::msg::Float64();
        err_msg.data = err.norm();
        ori_error_pub_->publish(err_msg);
    }

    // Finger impedance control (runs every cycle at 50 Hz)
    stepFingerControl(dt_);

    // Publish 23-DOF command
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();

    // Wrist joints (3) — publish fixed_ref + clamped_PID_offset
    //   cmd = wrist_ref_(0,0,0) + wrist_offset_  (NOT wrist_act + offset!)
    //   If we use act + offset, a drifted act drags the command with it.
    //   Fixed reference ensures the servo always pulls toward the ready pose.
    for (int j = 0; j < n_wrist_; ++j) {
        msg.name.push_back(wrist_names_[j]);
        msg.position.push_back(wrist_ref_(j) + wrist_offset_(j));
        msg.velocity.push_back(0.0);
        msg.effort.push_back(0.0);
    }

    // Finger joints (20) — publish impedance-encoded position command
    for (int j = 0; j < n_fingers_; ++j) {
        msg.name.push_back(finger_names_[j]);
        msg.position.push_back(finger_cmd_q_(j));
        msg.velocity.push_back(0.0);
        msg.effort.push_back(0.0);
    }

    hand_cmd_pub_->publish(msg);

    // --- Wrist state diagnostic (every 2s) ---
    static int state_diag = 0;
    if (++state_diag % 100 == 0 && joint_state_received_) {
        // cmd = wrist_ref_ + wrist_offset_ (the value actually sent to servo)
        Eigen::Vector3d wrist_cmd = wrist_ref_ + wrist_offset_;
        RCLCPP_INFO(get_logger(),
            "Wrist diag: cmd=[%+.3f %+.3f %+.3f] | "
            "act=[%+.3f %+.3f %+.3f] | "
            "offset=[%+.2f %+.2f %+.2f] | "
            "PID=%d J_f=%d O_f=%d",
            wrist_cmd(0), wrist_cmd(1), wrist_cmd(2),
            wrist_act(0), wrist_act(1), wrist_act(2),
            wrist_offset_(0), wrist_offset_(1), wrist_offset_(2),
            hand_ori_received_ && wrist_jac_received_,
            j_fresh, o_fresh);
    }

    // --- Completion check ---
    if (finger_active_ && finger_interp_elapsed_ >= finger_interp_duration_) {
        finger_active_ = false;
        // Print finger convergence: actual vs target for first 3 fingers
        double max_err = 0.0;
        for (int j = 0; j < n_fingers_; ++j)
            max_err = std::max(max_err, std::abs(finger_target_q_(j) - actual_q_(n_wrist_ + j)));
        RCLCPP_INFO(get_logger(),
            "Grasp '%s' DONE | elapsed=%.2fs | "
            "F1: des=%.2f→tgt=%.2f act=%.2f | "
            "F2: des=%.2f→tgt=%.2f act=%.2f | "
            "F3: des=%.2f→tgt=%.2f act=%.2f | "
            "max_err=%.3f rad",
            current_grasp_type_.c_str(), finger_interp_elapsed_,
            finger_des_q_(0), finger_target_q_(0), actual_q_(n_wrist_ + 0),
            finger_des_q_(4), finger_target_q_(4), actual_q_(n_wrist_ + 4),
            finger_des_q_(8), finger_target_q_(8), actual_q_(n_wrist_ + 8),
            max_err);
    }

    // Finger progress every 0.5s during active grasp
    static int finger_progress_counter = 0;
    if (finger_active_ && ++finger_progress_counter % 25 == 0) {
        RCLCPP_INFO(get_logger(),
            "FINGER PROGRESS t=%.2f/%.2fs | "
            "F1: des=%.3f act=%.3f cmd=%.3f | "
            "F2: des=%.3f act=%.3f cmd=%.3f",
            finger_interp_elapsed_, finger_interp_duration_,
            finger_des_q_(0), actual_q_(n_wrist_ + 0), finger_cmd_q_(0),
            finger_des_q_(4), actual_q_(n_wrist_ + 4), finger_cmd_q_(4));
    }
}

// ============================================================================
// Wrist orientation control
// ============================================================================

Eigen::Vector3d HandController::orientationError(
    const Eigen::Quaterniond& desired, const Eigen::Quaterniond& current)
{
    // q_error = q_des * q_curr^{-1}
    Eigen::Quaterniond q_err = desired * current.inverse();
    q_err.normalize();

    // Axis-angle: error = 2 * [x, y, z] (small-angle approximation within [-π, π])
    // Use sign of w to ensure shortest path
    double w = q_err.w();
    double scale = (w >= 0.0) ? 2.0 : -2.0;
    return Eigen::Vector3d(scale * q_err.x(), scale * q_err.y(), scale * q_err.z());
}

void HandController::stepWristControl(double dt)
{
    // Capture actual joint positions BEFORE PID modifies actual_q_
    Eigen::Vector3d wrist_act = actual_q_.head<3>();

    // 1. Compute orientation error
    Eigen::Vector3d error_ori = orientationError(target_ori_, current_hand_ori_);
    double ori_err_norm = error_ori.norm();

    // Deadband: freeze at ori_err < 0.05 rad (~2.9°) to prevent post-convergence
    // tremor near singular wrist configurations. Freezing integral + offset stops
    // the limit cycle without affecting convergence speed.
    if (ori_err_norm < 0.05) {
        wrist_error_integral_.setZero();
        wrist_prev_error_ = error_ori;
        return;
    }

    // 2. PID → task angular velocity
    Eigen::Vector3d task_vel = kp_wrist_ * error_ori;

    // Integral (only in the small-error linear regime, using effective dt)
    if (ori_err_norm < 0.5) {
        wrist_error_integral_ += error_ori * dt;
        // Anti-windup: clamp integral
        double i_max = 1.0 / (ki_wrist_ + 1e-6);  // scale clamp with integral gain
        for (int i = 0; i < 3; ++i)
            wrist_error_integral_(i) = std::max(-i_max, std::min(i_max, wrist_error_integral_(i)));
        task_vel += ki_wrist_ * wrist_error_integral_;
    }

    // Derivative (with effective dt)
    if (!wrist_first_) {
        Eigen::Vector3d error_dot = (error_ori - wrist_prev_error_) / dt;
        task_vel += kd_wrist_ * error_dot;
    }
    wrist_prev_error_ = error_ori;
    wrist_first_ = false;

    // Clamp task velocity
    double speed = task_vel.norm();
    if (speed > wrist_vel_limit_) task_vel *= wrist_vel_limit_ / speed;

    // 3. Solve: dq_wrist = J⁻¹ * task_vel
    Eigen::Vector3d dq_wrist;
    double det = wrist_jacobian_.determinant();
    if (std::abs(det) > 1e-4) {
        dq_wrist = wrist_jacobian_.colPivHouseholderQr().solve(task_vel);
    } else {
        // Singular fallback: damped pseudoinverse
        Eigen::Matrix3d JJt = wrist_jacobian_ * wrist_jacobian_.transpose();
        JJt.diagonal().array() += 1e-2;
        dq_wrist = wrist_jacobian_.transpose() * JJt.ldlt().solve(task_vel);
    }

    // 4. Velocity and delta clamping
    double dq_norm = dq_wrist.lpNorm<Eigen::Infinity>();
    if (dq_norm > wrist_vel_limit_) dq_wrist *= wrist_vel_limit_ / dq_norm;
    for (int j = 0; j < n_wrist_; ++j) {
        dq_wrist(j) = std::max(-wrist_vel_limit_, std::min(wrist_vel_limit_, dq_wrist(j)));
        // Per-cycle delta limit
        double delta = dq_wrist(j) * dt;
        if (delta > wrist_delta_limit_)       dq_wrist(j) = wrist_delta_limit_ / dt;
        else if (delta < -wrist_delta_limit_)  dq_wrist(j) = -wrist_delta_limit_ / dt;
    }

    // 5. Integrate into wrist_offset_ (clamped, anti-windup).
    //    Offset-based: wrist_cmd = wrist_ref_ + wrist_offset_ (NOT wrist_act + offset).
    for (int j = 0; j < n_wrist_; ++j) {
        wrist_offset_(j) += dq_wrist(j) * dt;
        wrist_offset_(j) = std::max(-wrist_offset_limit_,
                            std::min(wrist_offset_limit_, wrist_offset_(j)));
    }

    // --- Diagnostic log (every 2s) ---
    static int wrist_diag_count = 0;
    if (++wrist_diag_count % 100 == 0) {
        Eigen::Vector3d wrist_cmd = wrist_ref_ + wrist_offset_;
        RCLCPP_INFO(get_logger(),
            "Wrist: ori_err=%.3f rad | dt_eff=%.0f ms | "
            "cmd=[%+.3f %+.3f %+.3f] | "
            "act=[%+.3f %+.3f %+.3f] | "
            "dq=[%+.3f %+.3f %+.3f] | "
            "tgt_ori=[%.2f %.2f %.2f %.2f] | "
            "cur_ori=[%.2f %.2f %.2f %.2f]",
            ori_err_norm, dt * 1000.0,
            wrist_cmd(0), wrist_cmd(1), wrist_cmd(2),
            wrist_act(0), wrist_act(1), wrist_act(2),
            dq_wrist.x(), dq_wrist.y(), dq_wrist.z(),
            target_ori_.w(), target_ori_.x(), target_ori_.y(), target_ori_.z(),
            current_hand_ori_.w(), current_hand_ori_.x(), current_hand_ori_.y(), current_hand_ori_.z());
    }
}

// ============================================================================
// Finger impedance control
// ============================================================================

void HandController::stepFingerTrajectory(double dt)
{
    finger_interp_elapsed_ += dt;
    double alpha = (finger_interp_duration_ > 1e-6)
        ? std::min(1.0, finger_interp_elapsed_ / finger_interp_duration_)
        : 1.0;

    // Smoothstep: alpha = 3t² − 2t³
    double t = alpha;
    double eased = 3.0 * t * t - 2.0 * t * t * t;

    for (int j = 0; j < n_fingers_; ++j) {
        finger_des_q_(j) = finger_start_q_(j)
            + eased * (finger_target_q_(j) - finger_start_q_(j));
    }
}

void HandController::stepFingerControl(double dt)
{
    (void)dt;  // dt not needed for per-cycle control (impedance is instantaneous)

    for (int j = 0; j < n_fingers_; ++j) {
        int idx = n_wrist_ + j;  // index into actual_q_ / actual_dq_

        // Position error against FEEDFORWARD target (des_q, NOT self-referencing)
        //   During trajectory: des_q = smoothstep(t) → track planned motion
        //   After trajectory:   des_q = target_q (fixed) → P controller toward target
        double e = finger_des_q_(j) - actual_q_(idx);

        // Impedance torque: τ = Kp * e + Kd * (0 − dq_act)
        double tau = kp_finger_ * e + kd_finger_ * (-actual_dq_(idx));

        // Clamp torque
        if (tau > tau_max_finger_)       tau = tau_max_finger_;
        else if (tau < -tau_max_finger_)  tau = -tau_max_finger_;

        // Encode as position offset: q_cmd = q_des + τ / Kp_inner
        // (MuJoCo position servo will produce force ≈ Kp_inner × (q_cmd − q_act) ≈ τ)
        double q_cmd = finger_des_q_(j) + tau / inner_servo_kp_;

        // Per-cycle delta limit (safety — relative to actual position)
        double delta = q_cmd - actual_q_(idx);
        if (delta > finger_delta_limit_)        q_cmd = actual_q_(idx) + finger_delta_limit_;
        else if (delta < -finger_delta_limit_)   q_cmd = actual_q_(idx) - finger_delta_limit_;

        // Store in separate command variable (do NOT overwrite des_q!)
        finger_cmd_q_(j) = q_cmd;
    }
}

}  // namespace g1_right_arm_reach

// ============================================================================
// Main — ROS2 entry point
// ============================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<g1_right_arm_reach::HandController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
