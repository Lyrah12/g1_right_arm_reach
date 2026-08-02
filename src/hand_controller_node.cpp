/**
 * @file hand_controller_node.cpp
 * @brief Hand module: wrist orientation PID + finger impedance control (50 Hz, Plan C).
 *
 * Plan C: Arm-Hand fully decoupled.
 *   - reach_node: 4-DOF arm → wrist position
 *   - This node:  9-DOF hand → orientation + grasp
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
 * @published  /hand_commands            JointState, 9-DOF (wrist×3 + fingers×6)
 * @published  /debug/orientation_error  Float64 (rad)
 */

#include "g1_right_arm_reach/hand_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace g1_right_arm_reach {

using namespace std::literals::chrono_literals;

// ============================================================================
// Constructor
// ============================================================================

HandController::HandController()
    : Node("hand_controller")
{
    // --- Parameters: joint names ---
    this->declare_parameter("robot_model.urdf_path", "");
    this->declare_parameter("robot_model.arm_joints",
        std::vector<std::string>{"right_shoulder_pitch_joint",
            "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint"});
    this->declare_parameter("wrist_joint_names",
        std::vector<std::string>{
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        });

    this->declare_parameter("finger_joint_names",
        std::vector<std::string>{
            // Inspire RH56 — 6 active joints (6 slaves equality-coupled, not commanded)
            "R_thumb_proximal_yaw_joint",
            "R_thumb_proximal_pitch_joint",
            "R_index_proximal_joint",
            "R_middle_proximal_joint",
            "R_ring_proximal_joint",
            "R_pinky_proximal_joint",
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

    // --- Pinocchio model (hardware path) ---
    // Loads the right-arm URDF so wrist orientation / Jacobian / gravity can be
    // computed without the MuJoCo bridge. On real hardware the MuJoCo topics
    // (/actual_hand_orientation, /mujoco_wrist_jacobian, /mujoco_gravity) do not
    // exist; Pinocchio is the primary source and the MuJoCo topics override it
    // when they arrive (exact-sim enhancement in simulation).
    {
        std::string urdf_path = this->get_parameter("robot_model.urdf_path").as_string();
        if (urdf_path.empty()) {
            RCLCPP_WARN(get_logger(),
                "robot_model.urdf_path not set — wrist feedback from MuJoCo topics only. "
                "On real hardware set urdf_path to the G1 URDF.");
        } else {
            auto arm_names = this->get_parameter("robot_model.arm_joints").as_string_array();
            pin_arm_names_.assign(arm_names.begin(), arm_names.end());
            std::vector<std::string> active_names;
            active_names.insert(active_names.end(), arm_names.begin(), arm_names.end());
            active_names.insert(active_names.end(), wrist_names_.begin(), wrist_names_.end());

            robot_ = std::make_unique<RobotModel>(urdf_path, active_names);
            robot_->buildSubsetIndices(
                std::vector<std::string>(arm_names.begin(), arm_names.end()),
                wrist_names_);
            pin_arm_count_ = static_cast<int>(arm_names.size());
            kin_ = std::make_unique<Kinematics>(*robot_);
            pinocchio_ready_ = true;
            RCLCPP_INFO(get_logger(),
                "Pinocchio model ready: %d arm + %d wrist joints. "
                "Wrist ori/Jacobian/gravity computed locally (hardware path).",
                pin_arm_count_, n_wrist_);
        }
    }

    // --- Parameters: control gains ---
    this->declare_parameter("kp_wrist", 3.5);
    this->declare_parameter("ki_wrist", 0.05);
    this->declare_parameter("kd_wrist", 0.8);
    this->declare_parameter("wrist_joint_damping", 0.6);
    this->declare_parameter("wrist_vel_limit", 3.0);
    this->declare_parameter("wrist_delta_limit", 0.12);
    this->declare_parameter("wrist_offset_limit", 2.0);

    this->declare_parameter("kp_finger", 50.0);
    this->declare_parameter("kd_finger", 0.5);
    this->declare_parameter("tau_max_finger", 1.5);   // hand motor torque limit (N·m)
    this->declare_parameter("tau_max_wrist", 13.4);   // 5010 wrist torque limit (N·m)
    this->declare_parameter("finger_delta_limit", 1.5);
    this->declare_parameter("inner_servo_kp", 5.0);

    kp_wrist_ = this->get_parameter("kp_wrist").as_double();
    ki_wrist_ = this->get_parameter("ki_wrist").as_double();
    kd_wrist_ = this->get_parameter("kd_wrist").as_double();
    wrist_joint_damping_ = this->get_parameter("wrist_joint_damping").as_double();
    wrist_vel_limit_ = this->get_parameter("wrist_vel_limit").as_double();
    wrist_delta_limit_ = this->get_parameter("wrist_delta_limit").as_double();
    wrist_offset_limit_ = this->get_parameter("wrist_offset_limit").as_double();

    kp_finger_ = this->get_parameter("kp_finger").as_double();
    kd_finger_ = this->get_parameter("kd_finger").as_double();
    tau_max_finger_ = this->get_parameter("tau_max_finger").as_double();
    tau_max_wrist_ = this->get_parameter("tau_max_wrist").as_double();
    finger_delta_limit_ = this->get_parameter("finger_delta_limit").as_double();
    inner_servo_kp_ = this->get_parameter("inner_servo_kp").as_double();

    // --- State vectors ---
    actual_q_ = Eigen::VectorXd::Zero(n_total_);
    actual_dq_ = Eigen::VectorXd::Zero(n_total_);
    finger_target_q_ = Eigen::VectorXd::Zero(n_fingers_);
    finger_des_q_ = Eigen::VectorXd::Zero(n_fingers_);
    finger_cmd_q_ = Eigen::VectorXd::Zero(n_fingers_);
    finger_effort_ = Eigen::VectorXd::Zero(n_fingers_);
    finger_cmd_tau_ = Eigen::VectorXd::Zero(n_fingers_);

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

    // MuJoCo pure-gravity (7 arm+wrist joints). Wrist part = indices 4..6.
    gravity_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        this->declare_parameter("gravity_topic", "/mujoco_gravity"),
        10,
        std::bind(&HandController::gravityCallback, this, std::placeholders::_1));

    ee_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        this->declare_parameter("ee_pose_topic", "/actual_ee_pose"),
        10,
        std::bind(&HandController::eePoseCallback, this, std::placeholders::_1));

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
                else if (p.get_name() == "wrist_joint_damping") wrist_joint_damping_ = p.as_double();
                else if (p.get_name() == "wrist_offset_limit" && p.as_double() > 0.0)
                    wrist_offset_limit_ = p.as_double();
                else if (p.get_name() == "kp_finger") kp_finger_ = p.as_double();
                else if (p.get_name() == "kd_finger") kd_finger_ = p.as_double();
                else if (p.get_name() == "tau_max_finger") tau_max_finger_ = p.as_double();
                else if (p.get_name() == "tau_max_wrist" && p.as_double() > 0.0)
                    tau_max_wrist_ = p.as_double();
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
            // Reset to default palm pose (injection quat = absolute world-frame target).
            // Guard against repeated sequencer re-publishes.
            if (has_ori_target_ && target_ori_.isApprox(new_target, 1e-6))
                return;
            target_ori_ = new_target;
            has_ori_target_ = true;
            wrist_first_ = true;
            wrist_error_integral_.setZero();
            wrist_cycles_since_step_ = 0;

            RCLCPP_INFO(get_logger(),
                "Hand command: ori=RESET (injection quat) grasp=%s dur=%.2fs",
                msg->grasp_type.c_str(), msg->grasp_duration);
        } else {
            // ── Relative orientation: user quat is relative to CURRENT palm ──
            // Compose: target_abs = current_hand_ori * user_quat
            // Cache user quat to avoid recomposition on repeated sequencer publishes.
            bool user_changed = (new_target.coeffs() - last_user_ori_.coeffs()).norm() > 1e-8;

            if (user_changed) {
                last_user_ori_ = new_target;

                // Compose against the palm orientation source actually available:
                //   - sim:  hand_ori_received_ (MuJoCo /actual_hand_orientation)
                //   - hardware: pin_fb_ready_  (Pinocchio FK)
                // Otherwise defer — the first ori callback / refreshPinocchio()
                // will recompose when the source becomes available.
                bool palm_known = hand_ori_received_ || pin_fb_ready_;
                if (palm_known) {
                    // Compose user rotation with current palm orientation
                    Eigen::Quaterniond abs_target = current_hand_ori_ * new_target;
                    abs_target.normalize();
                    target_ori_ = abs_target;

                    // Validate: if the composed target is beyond wrist authority, warn
                    double jump = orientationError(target_ori_, current_hand_ori_).norm();
                    if (jump > wrist_offset_limit_) {
                        RCLCPP_WARN(get_logger(),
                            "Orientation target requires %.2f rad (%.0f deg) rotation. "
                            "Wrist offset limit is +-%.1f rad -> may not reach. "
                            "Smaller RPY values or pre-positioning the arm may help.",
                            jump, jump * 57.3, wrist_offset_limit_);
                    }
                } else {
                    // Palm orientation not yet available -> defer composition.
                    // First ori callback / refreshPinocchio() will recompose.
                    target_ori_ = new_target;  // temporary, will be replaced
                }

                has_ori_target_ = true;
                wrist_first_ = true;
                wrist_error_integral_.setZero();
                wrist_cycles_since_step_ = 0;

                // Show user's RELATIVE RPY (not absolute composed target)
                Eigen::Vector3d rpy = new_target.toRotationMatrix().eulerAngles(0, 1, 2);
                printf("[ORI] target R=%.2f P=%.2f Y=%.2f (%s)\n",
                    rpy(0), rpy(1), rpy(2),
                    msg->grasp_type.empty() ? "hold" : msg->grasp_type.c_str());
                fflush(stdout);
            }
            // else: same user quat re-published (sequencer loop) -> target unchanged
        }
    } else {
        // ori=HOLD: log only if there's a new grasp command
        if (!msg->grasp_type.empty()) {
            printf("[ORI] hold + grasp=%s (%.2fs)\n",
                msg->grasp_type.c_str(), msg->grasp_duration);
            fflush(stdout);
        }
    }
    // else: has_ori_target_ stays as-is (no change requested)

    // --- Finger grasp target ---
    const std::string& gtype = msg->grasp_type;
    if (!gtype.empty()) {
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

        // Snap des_q directly to target — impedance controller handles transition.
        finger_des_q_ = finger_target_q_;
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

    // Sync right-arm joints (arm + wrist) into the Pinocchio model so
    // refreshPinocchio() can compute FK/Jacobian/gravity from the real state.
    if (pinocchio_ready_) {
        for (size_t k = 0; k < pin_arm_names_.size() && k < robot_->q_.size(); ++k) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == pin_arm_names_[k]) {
                    robot_->q_(static_cast<int>(k)) = msg->position[i];
                    break;
                }
            }
        }
        for (int j = 0; j < n_wrist_; ++j) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == wrist_names_[j]) {
                    robot_->q_(pin_arm_count_ + j) = msg->position[i];
                    break;
                }
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
                if (i < msg->effort.size())
                    finger_effort_(j) = msg->effort[i];

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

void HandController::eePoseCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    actual_ee_x_ = msg->point.x;
    actual_ee_y_ = msg->point.y;
    actual_ee_z_ = msg->point.z;
    ee_pose_received_ = true;
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
            bool was_target_set = has_ori_target_;
            if (!was_target_set) {
                // No prior HandCommand → do NOT activate the orientation PID and do
                // NOT chase an absolute world-orientation target.
                // A world-orientation target captured here goes stale the moment the
                // arm moves (the palm rotates with the arm), leaving the wrist with a
                // 2-rad unreachable error → it slams against its joint limits and
                // saturates. Instead the wrist stays in gravity-hold (joint angles
                // held). Only an EXPLICIT user quat activates the orientation PID.
                RCLCPP_INFO(get_logger(),
                    "First hand ori, no command → wrist joint-angle hold "
                    "(no orientation target). Gravity hold active.");
            } else {
                // A prior HandCommand set target_ori_ — keep it and activate.
                RCLCPP_INFO(get_logger(),
                    "First hand orientation received. Target=[%.3f,%.3f,%.3f,%.3f] "
                    "Current=[%.3f,%.3f,%.3f,%.3f]. PID activated.",
                    target_ori_.w(), target_ori_.x(), target_ori_.y(), target_ori_.z(),
                    current_hand_ori_.w(), current_hand_ori_.x(), current_hand_ori_.y(), current_hand_ori_.z());
            }
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

void HandController::gravityCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // 7 values: [sp, sr, sy, elbow, wrist_roll, wrist_pitch, wrist_yaw].
    // Wrist joints are indices 4,5,6.
    if (msg->data.size() < 7) return;
    for (int j = 0; j < 3; ++j) {
        double g = msg->data[4 + j];
        if (std::isnan(g) || std::abs(g) > 20.0) return;  // sanity guard
        wrist_gravity_(j) = g;
    }
    if (!gravity_received_) {
        gravity_received_ = true;
        RCLCPP_INFO(get_logger(),
            "Wrist gravity FF (incl. hand): roll=%.2f pitch=%.2f yaw=%.2f N·m",
            wrist_gravity_(0), wrist_gravity_(1), wrist_gravity_(2));
    }
}

// ============================================================================
// Control loop (50 Hz)
// ============================================================================

void HandController::controlLoop()
{
    if (!joint_state_received_) return;

    // Hardware-fidelity (2026-08-02): Pinocchio is the PRIMARY wrist feedback
    // source — real hardware has no MuJoCo, so ori/Jacobian/gravity are computed
    // locally from the URDF. The MuJoCo topics (/actual_hand_orientation,
    // /mujoco_wrist_jacobian, /mujoco_gravity) act as an exact-sim override when
    // they arrive, preserving the previously-verified simulation behavior.
    //
    // Feedback path selection (fully isolated):
    //   - mujoco_available: all three MuJoCo topics received → use them, 20 Hz-
    //     gated as before (verified sim behavior).
    //   - otherwise (real hardware): Pinocchio, refreshed every cycle at 50 Hz.
    const bool mujoco_available = hand_ori_received_ && wrist_jac_received_
                               && gravity_received_;
    if (pinocchio_ready_ && !mujoco_available)
        refreshPinocchio();

    // Wrist orientation control. In sim this is gated on FRESH MuJoCo feedback
    // (~20 Hz) — without the gate, 50 Hz control with 20 Hz feedback causes
    // stale-Jacobian divergence. On hardware (no MuJoCo topics) Pinocchio runs
    // every cycle at 50 Hz. Effective dt (dt_ × cycles) corrects integral
    // starvation and derivative over-amplification.

    // Gravity-hold wrist torque FIRST, then stepWristControl overrides with the full PID
    // torque when feedback is available. Prevents the wrist from free-falling under
    // the hand mass when the orientation PID is frozen/absent — root cause of the
    // ~1 Hz wrist droop/oscillation.
    applyWristHold();

    wrist_cycles_since_step_++;

    bool fb_ready;
    if (mujoco_available) {
        fb_ready = hand_ori_fresh_ && wrist_jac_fresh_;
    } else {
        fb_ready = pin_fb_ready_;
    }

    if (has_ori_target_ && fb_ready) {
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
    stepFingerControl();

    // Publish torque command (TORQUE MODE): effort = τ, position = current (diagnostic)
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();

    // Wrist joints (3) — torque τ_wrist = Jᵀ·(Kp·e + Ki·∫e + Kd·ė)
    for (int j = 0; j < n_wrist_; ++j) {
        msg.name.push_back(wrist_names_[j]);
        msg.position.push_back(actual_q_(j));
        msg.velocity.push_back(0.0);
        msg.effort.push_back(wrist_cmd_tau_(j));
    }

    // Finger joints (6 active) — impedance torque τ = Kp·e + Kd·(0 − dq)
    for (int j = 0; j < n_fingers_; ++j) {
        msg.name.push_back(finger_names_[j]);
        msg.position.push_back(actual_q_(n_wrist_ + j));
        msg.velocity.push_back(0.0);
        msg.effort.push_back(finger_cmd_tau_(j));
    }

    hand_cmd_pub_->publish(msg);

    // --- Unified periodic status (every 1s, 50 cycles at 50 Hz) ---
    static int hand_status_counter = 0;
    if (++hand_status_counter % 50 == 0 && joint_state_received_) {
        // RPY from TARGET-RELATIVE error quaternion (stable, no gimbal-lock jumps).
        // (0,0,0) = perfectly converged. World-frame eulerAngles() is unstable
        // near singularities — a tiny quat change can flip R/P/Y by π.
        Eigen::Vector3d rpy_rel(0, 0, 0);
        double ori_err = -1.0;
        bool ori_source_ready = hand_ori_received_ || pin_fb_ready_;
        if (ori_source_ready && has_ori_target_) {
            ori_err = orientationError(target_ori_, current_hand_ori_).norm();
            // Error vector is axis-angle; approximate small-error RPY = error vector
            rpy_rel = orientationError(target_ori_, current_hand_ori_);
        }
        const char* ori_mark = (ori_err >= 0 && ori_err < 0.04) ? "+" : "";

        // Finger positions (base joint per finger)
        // Inspire 6 active joints: thumb_yaw, thumb_pitch, index, middle, ring, pinky
        double ty = actual_q_(n_wrist_ + 0);
        double tp = actual_q_(n_wrist_ + 1);
        double ix = actual_q_(n_wrist_ + 2);
        double md = actual_q_(n_wrist_ + 3);
        double rg = actual_q_(n_wrist_ + 4);
        double pk = actual_q_(n_wrist_ + 5);
        // DIAGNOSTIC: finger des targets (finger_des_q_)
        double dty = finger_des_q_(0);
        double dtp = finger_des_q_(1);
        double dix = finger_des_q_(2);

        printf("[Wrist] x=%.3f y=%.3f z=%.3f dR=%.2f dP=%.2f dY=%.2f err=%.3f%s "
               "tauW=[%.2f %.2f %.2f] grav=[%.2f %.2f %.2f] "
               "| Ty:%.2f(d%.2f) Tp:%.2f(d%.2f) Ix:%.2f(d%.2f) Md:%.2f Rg:%.2f Pk:%.2f\n",
            actual_ee_x_, actual_ee_y_, actual_ee_z_,
            rpy_rel(0), rpy_rel(1), rpy_rel(2), ori_err, ori_mark,
            wrist_cmd_tau_(0), wrist_cmd_tau_(1), wrist_cmd_tau_(2),
            wrist_gravity_(0), wrist_gravity_(1), wrist_gravity_(2),
            ty, dty, tp, dtp, ix, dix, md, rg, pk);
        fflush(stdout);
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

// ============================================================================
// Pinocchio fallback (hardware path)
// ============================================================================

void HandController::refreshPinocchio()
{
    if (!pinocchio_ready_ || !robot_ || !kin_) return;

    // Update FK from the synced /joint_states (arm + wrist positions).
    robot_->updateKinematics();

    // 1. Palm orientation = hand frame rotation (world frame).
    //    Note: the patched URDF lumps the Inspire hand into right_wrist_yaw_link,
    //    so getHandOrientation() resolves to that frame's rotation — equivalent
    //    to MuJoCo's /actual_hand_orientation for the wrist PID.
    current_hand_ori_ = Eigen::Quaterniond(robot_->getHandOrientation());
    current_hand_ori_.normalize();

    // 1b. First-orientation handling (mirrors handOrientationCallback). On real
    //     hardware /actual_hand_orientation never arrives, so Pinocchio must
    //     trigger the deferred composition of a relative target received earlier.
    if (!pin_fb_ready_) {
        pin_fb_ready_ = true;
        if (has_ori_target_ && last_user_ori_.w() > 1e-9) {
            // A relative HandCommand arrived before Pinocchio was ready → compose now.
            Eigen::Quaterniond abs_target = current_hand_ori_ * last_user_ori_;
            abs_target.normalize();
            target_ori_ = abs_target;
            RCLCPP_INFO(get_logger(),
                "First Pinocchio ori. Deferred compose: user→abs Target=[%.3f,%.3f,%.3f,%.3f]",
                target_ori_.w(), target_ori_.x(), target_ori_.y(), target_ori_.z());
        } else if (!has_ori_target_) {
            // No prior command → wrist stays in gravity-hold (no orientation chase).
            RCLCPP_INFO(get_logger(),
                "First Pinocchio ori, no command → wrist joint-angle hold.");
        }
    }

    // 2. Wrist 3×3 orientation Jacobian (numerical, Pinocchio).
    //    Fixed-size assignment: Eigen permits Dynamic→Fixed only when the runtime
    //    column count equals the fixed size — here n_wrist_ is always 3.
    Eigen::Matrix<double, 3, Eigen::Dynamic> J(3, n_wrist_);
    kin_->computeWristOrientationJacobian(J);
    if (J.cols() == 3) wrist_jacobian_ = J;  // compile-time 3×3 = runtime 3×3

    // 3. Wrist gravity torque (7-DOF model; wrist = last 3 indices).
    //    Includes the Inspire hand mass (lumped into the yaw link), matching
    //    MuJoCo's /mujoco_gravity on the 4..6 slice.
    Eigen::VectorXd tau_g;
    kin_->computeGravityTorque(tau_g);
    if (tau_g.size() >= pin_arm_count_ + n_wrist_) {
        for (int j = 0; j < n_wrist_; ++j)
            wrist_gravity_(j) = tau_g(pin_arm_count_ + j);
    }
}

void HandController::applyWristHold()
{
    // τ = g_wrist − kd_joint·dq — holds the wrist against gravity every cycle.
    // stepWristControl (when it runs) overwrites this with the full PID torque.
    Eigen::Vector3d tau = wrist_gravity_ - wrist_joint_damping_ * actual_dq_.head<3>();
    for (int j = 0; j < n_wrist_; ++j) {
        tau(j) = std::max(-tau_max_wrist_, std::min(tau_max_wrist_, tau(j)));
    }
    wrist_cmd_tau_ = tau;
}

void HandController::stepWristControl(double dt)
{
    // Capture actual joint positions BEFORE PID modifies actual_q_
    Eigen::Vector3d wrist_act = actual_q_.head<3>();

    // 1. Compute orientation error
    Eigen::Vector3d error_ori = orientationError(target_ori_, current_hand_ori_);
    double ori_err_norm = error_ori.norm();

    // Deadband with hysteresis: once frozen (error < 0.03), stay frozen until
    // error exceeds 0.07 rad. Prevents PID from oscillating on/off near the
    // 0.05 rad threshold due to MuJoCo numerical noise (~±0.01 rad).
    // Without hysteresis: every noise spike above 0.05 triggers one-cycle PID →
    // wrist twitches at ~20 Hz. With hysteresis: once settled, stays settled.
    if (wrist_frozen_) {
        if (ori_err_norm > 0.07) {
            wrist_frozen_ = false;  // re-activate
        } else {
            wrist_error_integral_.setZero();
            return;
        }
    } else {
        if (ori_err_norm < 0.03) {
            wrist_frozen_ = true;   // enter frozen state
            wrist_error_integral_.setZero();
            return;
        }
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

    // 3. TORQUE OUTPUT:
    //    tau_wrist = J^T·(task torque) + g_wrist − kd_joint·dq
    //    The gravity term holds the hand mass (otherwise the wrist droops ~1.4 rad);
    //    the joint-space damping kills the ~1 Hz underdamped oscillation.
    Eigen::Vector3d tau_wrist =
        wrist_jacobian_.transpose() * task_vel
        + wrist_gravity_
        - wrist_joint_damping_ * actual_dq_.head<3>();
    for (int j = 0; j < n_wrist_; ++j) {
        tau_wrist(j) = std::max(-tau_max_wrist_, std::min(tau_max_wrist_, tau_wrist(j)));
    }
    wrist_cmd_tau_ = tau_wrist;

}

// ============================================================================
// Finger impedance control
// ============================================================================

void HandController::stepFingerControl()
{
    for (int j = 0; j < n_fingers_; ++j) {
        int idx = n_wrist_ + j;
        double e = finger_des_q_(j) - actual_q_(idx);

        // Impedance torque: τ = Kp·e + Kd·(0 − dq) — direct torque output
        double tau = kp_finger_ * e + kd_finger_ * (-actual_dq_(idx));
        if (tau > tau_max_finger_)       tau = tau_max_finger_;
        else if (tau < -tau_max_finger_)  tau = -tau_max_finger_;

        finger_cmd_tau_(j) = tau;
        finger_cmd_q_(j) = actual_q_(idx);  // diagnostic: current position
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
