/**
 * @file reach_node.cpp
 * @brief G1 right-arm end-effector control node (50 Hz).
 *
 * Arm-Hand decoupled architecture:
 *   computeArm()   — 4-DOF shoulder×3+elbow → wrist origin position (3×4 damped IK)
 *   computeWrist() — variable-DOF wrist → hand orientation (3×N pseudoinverse)
 *
 * Subscribes:  /target_pose (PoseStamped, 6-DOF, shoulder-relative)
 *              /joint_states, /actual_ee_pose, /mujoco_jacobian
 * Publishes:   /joint_commands (JointState, 5-DOF)
 *
 * Current limitation (23-DOF): only wrist_roll is active.
 * Switching to 29-DOF model auto-activates wrist_pitch/yaw.
 */

#include "g1_right_arm_reach/robot_model.hpp"
#include "g1_right_arm_reach/kinematics.hpp"
#include "g1_right_arm_reach/pid_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>
#include <vector>

namespace g1_right_arm_reach {

class ReachNode : public rclcpp::Node {
public:
    // ========================================================================
    // Constructor — loads model, creates Arm-Hand decoupled controller
    // ========================================================================

    ReachNode() : Node("reach_node")
    {
        // --- Parameters: robot model ---
        this->declare_parameter("robot_model.urdf_path", "");
        this->declare_parameter("robot_model.active_joints",
            std::vector<std::string>{"right_shoulder_pitch_joint",
                "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                "right_elbow_joint", "right_wrist_roll_joint"});
        this->declare_parameter("robot_model.arm_joints",
            std::vector<std::string>{"right_shoulder_pitch_joint",
                "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                "right_elbow_joint"});
        this->declare_parameter("robot_model.wrist_joints",
            std::vector<std::string>{"right_wrist_roll_joint",
                "right_wrist_pitch_joint", "right_wrist_yaw_joint"});

        // --- Parameters: PID ---
        this->declare_parameter("pid.kp", 2.0);
        this->declare_parameter("pid.ki", 0.4);
        this->declare_parameter("pid.kd", 0.05);
        this->declare_parameter("pid.damping", 2.0e-4);
        this->declare_parameter("pid.tolerance", 0.01);
        this->declare_parameter("pid.posture_gain", 0.2);
        this->declare_parameter("pid.posture_target",
            std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});  // arm(4) + wrist_roll(1)
        this->declare_parameter("pid.kp_wrist", 1.0);
        this->declare_parameter("pid.kd_wrist", 0.02);
        this->declare_parameter("pid.torque_mode", false);
        this->declare_parameter("pid.gravity_compensation", false);
        this->declare_parameter("pid.inner_servo_kp", 300.0);
        this->declare_parameter("pid.inner_servo_kv", 5.0);

        // --- Parameters: reach_node ---
        this->declare_parameter("manager.control_rate", 50.0);
        this->declare_parameter("reach_node.target_topic", "/target_pose");
        this->declare_parameter("reach_node.target_point_topic", "/target_point");
        this->declare_parameter("reach_node.joint_state_topic", "/joint_states");
        this->declare_parameter("reach_node.origin_frame", "right_shoulder");
        this->declare_parameter("reach_node.default_target",
            std::vector<double>{0.18, -0.05, -0.10});
        this->declare_parameter("reach_node.default_orientation",
            std::vector<double>{0.0, 0.0, 0.0, 1.0});
        this->declare_parameter("reach_node.pelvis_offset",
            std::vector<double>{0.0, 0.0, 0.823});

        // --- Pelvis offset ---
        auto po = this->get_parameter("reach_node.pelvis_offset").as_double_array();
        if (po.size() >= 3) pelvis_offset_ << po[0], po[1], po[2];
        RCLCPP_INFO(get_logger(),
            "Pelvis offset (URDF→MuJoCo): [%.3f, %.3f, %.3f]",
            pelvis_offset_.x(), pelvis_offset_.y(), pelvis_offset_.z());

        // --- Default target ---
        auto dt_vec = this->get_parameter("reach_node.default_target").as_double_array();
        if (dt_vec.size() >= 3) target_pos_ << dt_vec[0], dt_vec[1], dt_vec[2];
        auto do_vec = this->get_parameter("reach_node.default_orientation").as_double_array();
        if (do_vec.size() >= 4) {
            target_ori_ = Eigen::Quaterniond(do_vec[3], do_vec[0], do_vec[1], do_vec[2]);
        }

        // --- Load robot model ---
        std::string urdf_path = this->get_parameter("robot_model.urdf_path").as_string();
        if (urdf_path.empty()) {
            RCLCPP_ERROR(get_logger(), "URDF path not set!");
            rclcpp::shutdown(); return;
        }
        robot_ = std::make_unique<RobotModel>(urdf_path,
            this->get_parameter("robot_model.active_joints").as_string_array());

        // Build arm/wrist subset indices
        robot_->buildSubsetIndices(
            this->get_parameter("robot_model.arm_joints").as_string_array(),
            this->get_parameter("robot_model.wrist_joints").as_string_array());
        robot_->printSummary();

        // --- Kinematics & PID ---
        kin_ = std::make_unique<Kinematics>(*robot_);
        pid_ = std::make_unique<PIDController>(*robot_, *kin_,
            this->get_parameter("pid.kp").as_double(),
            this->get_parameter("pid.ki").as_double(),
            this->get_parameter("pid.kd").as_double(),
            this->get_parameter("pid.damping").as_double(),
            this->get_parameter("pid.tolerance").as_double(),
            this->get_parameter("pid.posture_gain").as_double());

        pid_->setTorqueMode(this->get_parameter("pid.torque_mode").as_bool());
        pid_->setGravityCompensation(this->get_parameter("pid.gravity_compensation").as_bool());
        pid_->setInnerServoKp(this->get_parameter("pid.inner_servo_kp").as_double());
        pid_->setInnerServoKv(this->get_parameter("pid.inner_servo_kv").as_double());
        pid_->setKpWrist(this->get_parameter("pid.kp_wrist").as_double());
        pid_->setKdWrist(this->get_parameter("pid.kd_wrist").as_double());

        RCLCPP_INFO(get_logger(),
            "PID mode: %s | Gravity comp: %s | Inner servo: kp=%.0f kv=%.0f",
            pid_->isTorqueMode() ? "TORQUE" : "POSITION",
            this->get_parameter("pid.gravity_compensation").as_bool() ? "ON" : "OFF",
            this->get_parameter("pid.inner_servo_kp").as_double(),
            this->get_parameter("pid.inner_servo_kv").as_double());

        // Set posture target
        {
            auto pt = this->get_parameter("pid.posture_target").as_double_array();
            Eigen::VectorXd q_pref(static_cast<int>(pt.size()));
            for (size_t i = 0; i < pt.size(); ++i) q_pref(static_cast<int>(i)) = pt[i];
            pid_->setPostureTarget(q_pref);
            RCLCPP_INFO(get_logger(),
                "Posture target: arm=[%.2f,%.2f,%.2f,%.2f] wrist=[%.2f]",
                q_pref(0), q_pref(1), q_pref(2), q_pref(3), q_pref(4));
        }

        // --- ROS2 communication ---
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("reach_node.target_topic").as_string(), 10,
            std::bind(&ReachNode::poseCallback, this, std::placeholders::_1));

        point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            this->get_parameter("reach_node.target_point_topic").as_string(), 10,
            std::bind(&ReachNode::pointCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            this->get_parameter("reach_node.joint_state_topic").as_string(), 10,
            std::bind(&ReachNode::jointStateCallback, this, std::placeholders::_1));

        ee_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/actual_ee_pose", 10,
            std::bind(&ReachNode::eePoseCallback, this, std::placeholders::_1));

        jacobian_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/mujoco_jacobian", 10,
            std::bind(&ReachNode::jacobianCallback, this, std::placeholders::_1));

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_commands", 10);

        error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/debug/position_error", 10);

        target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/debug/target_marker", rclcpp::QoS(10).transient_local());

        // --- Control loop timer ---
        double control_rate = this->get_parameter("manager.control_rate").as_double();
        dt_ = 1.0 / control_rate;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate)),
            std::bind(&ReachNode::controlLoop, this));

        // --- Parameter callback ---
        [[maybe_unused]] auto _param_cb = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto& p : params) {
                    if (p.get_name() == "pid.kp") pid_->setKp(p.as_double());
                    else if (p.get_name() == "pid.ki") pid_->setKi(p.as_double());
                    else if (p.get_name() == "pid.kd") pid_->setKd(p.as_double());
                    else if (p.get_name() == "pid.kp_wrist") pid_->setKpWrist(p.as_double());
                    else if (p.get_name() == "pid.kd_wrist") pid_->setKdWrist(p.as_double());
                    else if (p.get_name() == "pid.damping") {
                        if (p.as_double() <= 0.0) { result.successful = false; result.reason = "pid.damping > 0 required"; }
                        else pid_->setDamping(p.as_double());
                    } else if (p.get_name() == "pid.tolerance") {
                        if (p.as_double() <= 0.0) { result.successful = false; result.reason = "pid.tolerance > 0 required"; }
                        else pid_->setTolerance(p.as_double());
                    } else if (p.get_name() == "pid.posture_gain") {
                        pid_->setPostureGain(p.as_double());
                    } else if (p.get_name() == "pid.torque_mode") {
                        pid_->setTorqueMode(p.as_bool());
                    } else if (p.get_name() == "pid.gravity_compensation") {
                        pid_->setGravityCompensation(p.as_bool());
                    } else if (p.get_name() == "pid.inner_servo_kp") {
                        pid_->setInnerServoKp(p.as_double());
                    } else if (p.get_name() == "pid.inner_servo_kv") {
                        pid_->setInnerServoKv(p.as_double());
                    }
                }
                return result;
            });

        RCLCPP_INFO(get_logger(),
            "ReachNode ready. Rate: %.1f Hz | Origin: %s | Arm: %d-DOF | Wrist: %d-DOF",
            control_rate,
            this->get_parameter("reach_node.origin_frame").as_string().c_str(),
            static_cast<int>(robot_->arm_joint_indices_.size()),
            static_cast<int>(robot_->wrist_joint_indices_.size()));
    }

private:
    // ========================================================================
    // Callbacks
    // ========================================================================

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        Eigen::Vector3d new_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        Eigen::Quaterniond new_ori(msg->pose.orientation.w,
                                    msg->pose.orientation.x,
                                    msg->pose.orientation.y,
                                    msg->pose.orientation.z);
        new_ori.normalize();

        if (has_target_ && new_pos.isApprox(target_pos_, 1e-3) &&
            (!has_orientation_ || new_ori.isApprox(target_ori_, 1e-3)))
            return;

        bool significant_change = !has_target_
            || (new_pos - target_pos_).norm() > 0.01
            || (has_orientation_ && Kinematics::orientationError(new_ori, target_ori_).norm() > 0.05);

        target_pos_ = new_pos;
        target_ori_ = new_ori;
        has_orientation_ = true;
        has_target_ = true;

        if (significant_change) {
            locked_ = false;
            pid_->reset();
            RCLCPP_INFO(get_logger(),
                "New target [6-DOF]: pos=[%.3f, %.3f, %.3f] ori=[%.3f, %.3f, %.3f, %.3f] (shoulder-relative)",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                msg->pose.orientation.x, msg->pose.orientation.y,
                msg->pose.orientation.z, msg->pose.orientation.w);
        }
    }

    void pointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (has_orientation_) return;
        Eigen::Vector3d new_pos(msg->x, msg->y, msg->z);
        if (has_target_ && new_pos.isApprox(target_pos_, 1e-6)) return;
        target_pos_ = new_pos;
        has_orientation_ = false;
        has_target_ = true;
        locked_ = false;
        pid_->reset();
        RCLCPP_INFO(get_logger(),
            "New target [3-DOF]: [%.3f, %.3f, %.3f] (shoulder-relative)",
            msg->x, msg->y, msg->z);
    }

    void eePoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        actual_ee_world_ << msg->point.x, msg->point.y, msg->point.z;
        ee_pose_received_ = true;
    }

    void jacobianCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Expect 3×5 = 15 elements (position Jacobian from MuJoCo bridge)
        if (msg->data.size() < 15) return;
        mujoco_jacobian_3d_.resize(3, 5);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 5; ++c)
                mujoco_jacobian_3d_(r, c) = msg->data[r * 5 + c];
        if (!jacobian_received_) {
            jacobian_received_ = true;
            RCLCPP_INFO(get_logger(), "MuJoCo Jacobian received (3×5), |J[:3,:3]|=%.6f",
                mujoco_jacobian_3d_.leftCols<3>().determinant());
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < static_cast<size_t>(robot_->reduced_model_.nq)) return;
        for (size_t i = 0; i < robot_->active_joint_names_.size(); ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), robot_->active_joint_names_[i]);
            if (it != msg->name.end())
                robot_->q_(static_cast<int>(i)) = msg->position[std::distance(msg->name.begin(), it)];
        }
        if (msg->velocity.size() >= static_cast<size_t>(robot_->reduced_model_.nq)) {
            for (size_t i = 0; i < robot_->active_joint_names_.size(); ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), robot_->active_joint_names_[i]);
                if (it != msg->name.end())
                    robot_->dq_(static_cast<int>(i)) = msg->velocity[std::distance(msg->name.begin(), it)];
            }
        }
        joint_state_received_ = true;
    }

    // ========================================================================
    // Control loop (50 Hz) — Arm IK + Wrist solver
    // ========================================================================

    void controlLoop()
    {
        if (!joint_state_received_ || !ee_pose_received_ || !has_target_) return;

        Eigen::VectorXd q = robot_->q_;
        Eigen::VectorXd dq = robot_->dq_;

        // --- Compute shoulder position ---
        robot_->updateKinematics();
        Eigen::Vector3d shoulder_urdf = robot_->getRightShoulderPosition();
        Eigen::Vector3d shoulder_world = shoulder_urdf + pelvis_offset_;

        // --- Target in world frame ---
        Eigen::Vector3d target_world = shoulder_world + target_pos_;

        // --- Position error from MuJoCo ground-truth ---
        double position_error = (target_world - actual_ee_world_).norm();

        // --- Convergence lock (position + orientation) ---
        double ori_err_norm = 0.0;
        if (has_orientation_) {
            robot_->updateKinematics();
            Eigen::Matrix3d R_curr = robot_->getHandOrientation();
            Eigen::Quaterniond q_curr(R_curr);
            ori_err_norm = Kinematics::orientationError(target_ori_, q_curr).norm();
        }
        if (locked_) {
            if (position_error > 0.05 || ori_err_norm > 0.10) {
                locked_ = false; pid_->reset();
            } else {
                robot_->q_ = converged_q_; publishJointStates(); publishTargetMarker(); return;
            }
        }
        if (position_error <= pid_->getTolerance() && ori_err_norm <= 0.05) {
            locked_ = true; converged_q_ = robot_->q_;
            RCLCPP_INFO(get_logger(), "[CONVERGED] pos_err=%.3f m  ori_err=%.3f rad",
                        position_error, ori_err_norm);
            publishJointStates(); publishTargetMarker(); return;
        }

        // --- Compute shoulder-relative error ---
        Eigen::Vector3d error_shoulder = target_pos_ - (actual_ee_world_ - shoulder_world);

        // --- Arm IK (4-DOF → 3-DOF position) ---
        ControlResult arm_result = pid_->computeArm(target_pos_, q, dq, dt_, &error_shoulder);
        total_cycles_++;

        // --- Wrist solver (3-DOF → 3-DOF orientation) ---
        ControlResult wrist_result;
        if (has_orientation_) {
            wrist_result = pid_->computeWrist(target_ori_, robot_->q_, dq, dt_);
        }

        // Apply arm joint commands (preserve wrist changes from computeWrist)
        if (arm_result.command.size() == robot_->reduced_model_.nq) {
            for (int k = 0; k < static_cast<int>(robot_->arm_joint_indices_.size()); ++k) {
                int i = robot_->arm_joint_indices_[k];
                robot_->q_(i) = arm_result.command(i);
            }
        }
        robot_->updateKinematics();

        // --- Divergence guard ---
        static double prev_position_error = 0.0;
        static int divergence_count = 0;
        if (has_target_ && !locked_ && total_cycles_ > 0) {
            if (position_error > prev_position_error * 1.05) {
                divergence_count++;
                if (divergence_count >= 5) {
                    RCLCPP_WARN(get_logger(),
                        "[DIVERGE] error rising %.3f→%.3f over %d cycles — resetting PID",
                        prev_position_error, position_error, divergence_count);
                    pid_->reset();
                    divergence_count = 0;
                }
            } else {
                divergence_count = 0;
            }
        }
        prev_position_error = position_error;

        publishJointStates();
        publishTargetMarker();

        auto err_msg = std_msgs::msg::Float64();
        err_msg.data = position_error;
        error_pub_->publish(err_msg);

        // --- Diagnostic log (every 2s) ---
        static int cycle_count = 0;
        if (++cycle_count % 100 == 0) {
            RCLCPP_INFO(get_logger(),
                "┌─ [PID #%d] mode=POS (decoupled)", total_cycles_);
            RCLCPP_INFO(get_logger(),
                "├ TARGET (shoulder-relative): [%.3f, %.3f, %.3f] %s",
                target_pos_.x(), target_pos_.y(), target_pos_.z(),
                has_orientation_ ? "+ori" : "");
            RCLCPP_INFO(get_logger(),
                "├ TARGET (world): [%.3f, %.3f, %.3f]  SHOULDER:[%.3f, %.3f, %.3f]",
                target_world.x(), target_world.y(), target_world.z(),
                shoulder_world.x(), shoulder_world.y(), shoulder_world.z());
            RCLCPP_INFO(get_logger(),
                "├ ERR  dX=%+.4f  dY=%+.4f  dZ=%+.4f  |e|=%.4f m  %s  ori=%.3f rad",
                error_shoulder.x(), error_shoulder.y(), error_shoulder.z(),
                position_error,
                position_error <= pid_->getTolerance() ? "✓" : "",
                arm_result.orientation_error);
            RCLCPP_INFO(get_logger(),
                "├ Arm  kp=%.2f ∫e=%.4f |v|=%.3f damp=%.1e |dq_null|=%.3f",
                arm_result.effective_kp, arm_result.integral_norm,
                arm_result.task_velocity, arm_result.adaptive_damping,
                arm_result.nullspace_norm);
            RCLCPP_INFO(get_logger(),
                "├ Wrist kp=%.2f |v_ori|=%.3f ori_err=%.3f rad",
                this->get_parameter("pid.kp_wrist").as_double(),
                wrist_result.task_velocity, wrist_result.orientation_error);
            RCLCPP_INFO(get_logger(),
                "├ CMD arm:    [%+.3f, %+.3f, %+.3f, %+.3f] rad",
                arm_result.command.size() >= 4 ? arm_result.command(0) : 0.0,
                arm_result.command.size() >= 4 ? arm_result.command(1) : 0.0,
                arm_result.command.size() >= 4 ? arm_result.command(2) : 0.0,
                arm_result.command.size() >= 4 ? arm_result.command(3) : 0.0);
            RCLCPP_INFO(get_logger(),
                "├ CMD wrist:  [%+.3f] rad",
                robot_->q_.size() > 4 ? robot_->q_(4) : 0.0);
            RCLCPP_INFO(get_logger(),
                "└ ACT arm:    [%+.3f, %+.3f, %+.3f, %+.3f] rad",
                q.size() >= 4 ? q(0) : 0.0,
                q.size() >= 4 ? q(1) : 0.0,
                q.size() >= 4 ? q(2) : 0.0,
                q.size() >= 4 ? q(3) : 0.0);
        }
    }

    // ========================================================================
    // Publishers
    // ========================================================================

    void publishJointStates()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        for (int i = 0; i < robot_->reduced_model_.nq; ++i) {
            msg.name.push_back(robot_->active_joint_names_[i]);
            msg.position.push_back(robot_->q_(i));
            msg.velocity.push_back(0.0);
            msg.effort.push_back(0.0);
        }
        joint_state_pub_->publish(msg);
    }

    void publishTargetMarker()
    {
        if (!has_target_) return;
        robot_->updateKinematics();
        Eigen::Vector3d shoulder_urdf = robot_->getRightShoulderPosition();
        Eigen::Vector3d shoulder_world = shoulder_urdf + pelvis_offset_;
        Eigen::Vector3d target_world = shoulder_world + target_pos_;

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "pelvis";
        marker.header.stamp = this->now();
        marker.ns = "target"; marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = target_world.x();
        marker.pose.position.y = target_world.y();
        marker.pose.position.z = target_world.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.03; marker.scale.y = 0.03; marker.scale.z = 0.03;
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.8;
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        target_marker_pub_->publish(marker);
    }

    // ========================================================================
    // State
    // ========================================================================

    std::unique_ptr<RobotModel> robot_;
    std::unique_ptr<Kinematics> kin_;
    std::unique_ptr<PIDController> pid_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ee_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    Eigen::Vector3d pelvis_offset_{0.0, 0.0, 0.823};
    Eigen::Vector3d target_pos_{0.18, -0.05, -0.10};
    Eigen::Quaterniond target_ori_{1.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d actual_ee_world_{0.0, 0.0, 0.0};
    Eigen::Matrix<double, 3, Eigen::Dynamic> mujoco_jacobian_3d_{3, 5};

    bool has_target_{false};
    bool has_orientation_{false};
    bool joint_state_received_{false};
    bool ee_pose_received_{false};
    bool jacobian_received_{false};
    bool locked_{false};
    int total_cycles_{0};

    Eigen::VectorXd converged_q_;
    double dt_{0.02};
};

}  // namespace g1_right_arm_reach

// ============================================================================
// MODULE: Main — ROS2 entry point
// ============================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<g1_right_arm_reach::ReachNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
