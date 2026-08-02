/**
 * @file reach_node.cpp
 * @brief G1 right-arm end-effector position control node (50 Hz).
 *
 * Plan C Architecture (Arm-Hand fully decoupled):
 *   Arm module (this node): shoulder×3 + elbow = 4-DOF → wrist origin position (3×4 IK)
 *   Hand module (hand_controller): wrist×3 + fingers×20 → orientation + grasp
 *
 * Subscribes:  /target_point (Point, 3-DOF, shoulder-relative)
 *              /joint_states, /actual_ee_pose
 * Publishes:   /joint_commands (JointState, 4-DOF)
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
#include <cstdio>

namespace g1_right_arm_reach {

class ReachNode : public rclcpp::Node {
public:
    // ========================================================================
    // Constructor — loads model, creates Arm-only position controller
    // ========================================================================

    ReachNode() : Node("reach_node")
    {
        // --- Parameters: robot model ---
        this->declare_parameter("robot_model.urdf_path", "");
        this->declare_parameter("robot_model.active_joints",
            std::vector<std::string>{"right_shoulder_pitch_joint",
                "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                "right_elbow_joint", "right_wrist_roll_joint",
                "right_wrist_pitch_joint", "right_wrist_yaw_joint"});
        this->declare_parameter("robot_model.arm_joints",
            std::vector<std::string>{"right_shoulder_pitch_joint",
                "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                "right_elbow_joint"});
        this->declare_parameter("robot_model.wrist_joints",
            std::vector<std::string>{});  // Plan C: wrist → hand_controller

        // --- Parameters: PID ---
        this->declare_parameter("pid.kp", 2.0);
        this->declare_parameter("pid.ki", 0.4);
        this->declare_parameter("pid.kd", 0.05);
        this->declare_parameter("pid.damping", 2.0e-4);
        this->declare_parameter("pid.tolerance", 0.005);
        this->declare_parameter("pid.posture_gain", 0.2);
        this->declare_parameter("pid.posture_target",
            std::vector<double>{0.0, 0.0, 0.0, 0.0});  // arm(4) — natural hang + forearm forward
        this->declare_parameter("pid.kp_wrist", 1.0);     // unused, kept for compat
        this->declare_parameter("pid.kd_wrist", 0.02);
        this->declare_parameter("pid.torque_mode", false);
        this->declare_parameter("pid.gravity_compensation", false);
        this->declare_parameter("pid.inner_servo_kp", 5.0);
        this->declare_parameter("pid.inner_servo_kv", 5.0);

        // --- Parameters: reach_node ---
        this->declare_parameter("manager.control_rate", 50.0);
        this->declare_parameter("reach_node.target_point_topic", "/target_point");
        this->declare_parameter("reach_node.joint_state_topic", "/joint_states");
        this->declare_parameter("reach_node.gravity_topic", "/mujoco_gravity");
        this->declare_parameter("reach_node.origin_frame", "right_shoulder");
        this->declare_parameter("reach_node.default_target",
            std::vector<double>{0.18, -0.05, -0.10});
        this->declare_parameter("reach_node.pelvis_offset",
            std::vector<double>{0.0, 0.0, 0.793});

        // --- Pelvis offset ---
        auto po = this->get_parameter("reach_node.pelvis_offset").as_double_array();
        if (po.size() >= 3) pelvis_offset_ << po[0], po[1], po[2];
        RCLCPP_INFO(get_logger(),
            "Pelvis offset (URDF→MuJoCo): [%.3f, %.3f, %.3f]",
            pelvis_offset_.x(), pelvis_offset_.y(), pelvis_offset_.z());

        // --- Default target ---
        auto dt_vec = this->get_parameter("reach_node.default_target").as_double_array();
        if (dt_vec.size() >= 3) target_pos_ << dt_vec[0], dt_vec[1], dt_vec[2];

        // --- Load robot model ---
        std::string urdf_path = this->get_parameter("robot_model.urdf_path").as_string();
        if (urdf_path.empty()) {
            RCLCPP_ERROR(get_logger(), "URDF path not set!");
            rclcpp::shutdown(); return;
        }
        robot_ = std::make_unique<RobotModel>(urdf_path,
            this->get_parameter("robot_model.active_joints").as_string_array());

        // Build arm-only subset indices (Plan C: empty wrist → no wrist solver)
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

        // Set posture target (arm only)
        {
            auto pt = this->get_parameter("pid.posture_target").as_double_array();
            int n_arm = static_cast<int>(robot_->arm_joint_indices_.size());
            Eigen::VectorXd q_pref(robot_->reduced_model_.nq);
            q_pref.setZero();
            for (int k = 0; k < n_arm && k < static_cast<int>(pt.size()); ++k) {
                int i = robot_->arm_joint_indices_[k];
                q_pref(i) = pt[k];
            }
            pid_->setPostureTarget(q_pref);
            RCLCPP_INFO(get_logger(),
                "Posture target: [%.2f, %.2f, %.2f, %.2f] (arm 4-DOF)",
                q_pref(0), q_pref(1), q_pref(2), q_pref(3));
        }

        RCLCPP_INFO(get_logger(),
            "PID mode: POSITION | Arm: %d-DOF → 3-DOF position | Plan C",
            static_cast<int>(robot_->arm_joint_indices_.size()));

        // --- ROS2 communication ---
        // Primary: /target_point (3-DOF position only)
        point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            this->get_parameter("reach_node.target_point_topic").as_string(), 10,
            std::bind(&ReachNode::pointCallback, this, std::placeholders::_1));

        // Legacy: /target_pose (extract position only, ignore orientation)
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10,
            std::bind(&ReachNode::poseCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            this->get_parameter("reach_node.joint_state_topic").as_string(), 10,
            std::bind(&ReachNode::jointStateCallback, this, std::placeholders::_1));

        ee_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/actual_ee_pose", 10,
            std::bind(&ReachNode::eePoseCallback, this, std::placeholders::_1));

        // MuJoCo pure-gravity (7 arm+wrist joints, incl. hand mass) → gravity feedforward
        gravity_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            this->get_parameter("reach_node.gravity_topic").as_string(), 10,
            std::bind(&ReachNode::gravityCallback, this, std::placeholders::_1));

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
                    else if (p.get_name() == "pid.damping") {
                        if (p.as_double() <= 0.0) { result.successful = false; result.reason = "pid.damping > 0 required"; }
                        else pid_->setDamping(p.as_double());
                    } else if (p.get_name() == "pid.tolerance") {
                        if (p.as_double() <= 0.0) { result.successful = false; result.reason = "pid.tolerance > 0 required"; }
                        else pid_->setTolerance(p.as_double());
                    } else if (p.get_name() == "pid.posture_gain") {
                        pid_->setPostureGain(p.as_double());
                    }
                }
                return result;
            });

        RCLCPP_INFO(get_logger(),
            "ReachNode ready (Plan C). Rate: %.1f Hz | Arm: %d-DOF → 3-DOF position",
            control_rate, static_cast<int>(robot_->arm_joint_indices_.size()));
    }

private:
    // ========================================================================
    // Callbacks
    // ========================================================================

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Legacy: extract position only (orientation → hand_controller in Plan C)
        Eigen::Vector3d new_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        if (has_target_ && new_pos.isApprox(target_pos_, 1e-3))
            return;

        bool significant_change = !has_target_
            || (new_pos - target_pos_).norm() > 0.01;

        target_pos_ = new_pos;
        has_target_ = true;

        if (significant_change) {
            locked_ = false;
            pid_->reset();
            RCLCPP_INFO(get_logger(),
                "New target [3-DOF]: [%.3f, %.3f, %.3f] (shoulder-relative)",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        }
    }

    void pointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        Eigen::Vector3d new_pos(msg->x, msg->y, msg->z);
        if (has_target_ && new_pos.isApprox(target_pos_, 1e-6)) return;

        target_pos_ = new_pos;
        has_target_ = true;
        locked_ = false;
        pid_->reset();
        printf("[TARGET] x=%.3f y=%.3f z=%.3f\n", msg->x, msg->y, msg->z);
        fflush(stdout);
    }

    void eePoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        actual_ee_world_ << msg->point.x, msg->point.y, msg->point.z;
        ee_pose_received_ = true;
    }

    void gravityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // 7 values in arm_joint_order = active_joint_names_ order.
        // Map directly onto the reduced-model nv vector for the PID gravity FF.
        if (msg->data.size() < static_cast<size_t>(robot_->reduced_model_.nv)) return;
        Eigen::VectorXd g(robot_->reduced_model_.nv);
        for (int i = 0; i < robot_->reduced_model_.nv; ++i)
            g(i) = msg->data[i];
        // Sanity guard: reject NaN / absurd values (should be ~ ±60 N·m for G1 arm)
        if (g.array().isNaN().any() || g.array().abs().maxCoeff() > 200.0) return;
        pid_->setExternalGravity(g);
        gravity_received_ = true;
        if (!gravity_logged_) {
            gravity_logged_ = true;
            RCLCPP_INFO(get_logger(),
                "MuJoCo gravity FF (incl. hand): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                g(0), g(1), g(2), g(3), g(4), g(5), g(6));
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
    // Control loop (50 Hz) — Arm IK (4-DOF → 3-DOF position)
    // ========================================================================

    void controlLoop()
    {
        if (!joint_state_received_) return;

        // --- No target yet → gravity hold (arm must not free-fall under gravity) ---
        // Publishes τ = g − kv·dq for the arm joints, keeping the arm where it is.
        if (!has_target_ || !ee_pose_received_) {
            ControlResult hold = pid_->computeGravityHold(robot_->q_, robot_->dq_);
            if (hold.torque_command.size() == static_cast<int>(robot_->arm_joint_indices_.size())) {
                last_arm_tau_ = hold.torque_command;
                publishJointStates();
            }
            return;
        }

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

        // --- Convergence lock (position only) ---
        // TORQUE mode: never early-return on convergence — must keep sending
        // gravity compensation + damping torque or the arm collapses.
        bool torque_mode = pid_->isTorqueMode();
        if (locked_) {
            if (position_error > 0.05) {
                locked_ = false; pid_->reset();
            } else if (!torque_mode) {
                robot_->q_ = converged_q_; publishJointStates(); publishTargetMarker(); return;
            }
        }
        if (position_error <= pid_->getTolerance()) {
            locked_ = true; converged_q_ = robot_->q_;
            if (!torque_mode) {
                printf("[WRIST] Position converged: x=%.3f y=%.3f z=%.3f (err=%.3f m)\n",
                    actual_ee_world_.x(), actual_ee_world_.y(), actual_ee_world_.z(),
                    position_error);
                fflush(stdout);
                publishJointStates(); publishTargetMarker(); return;
            }
        }

        // --- Compute shoulder-relative error ---
        Eigen::Vector3d error_shoulder = target_pos_ - (actual_ee_world_ - shoulder_world);

        // --- Arm IK (4-DOF → 3-DOF position) ---
        ControlResult arm_result = pid_->computeArm(target_pos_, q, dq, dt_, &error_shoulder);
        total_cycles_++;

        // Apply arm joint commands
        if (arm_result.command.size() == robot_->reduced_model_.nq) {
            for (int k = 0; k < static_cast<int>(robot_->arm_joint_indices_.size()); ++k) {
                int i = robot_->arm_joint_indices_[k];
                robot_->q_(i) = arm_result.command(i);
            }
        }
        // Torque mode: cache arm torque command for publishing
        if (arm_result.torque_command.size() == static_cast<int>(robot_->arm_joint_indices_.size())) {
            last_arm_tau_ = arm_result.torque_command;
        }
        last_integral_norm_ = arm_result.integral_norm;
        last_nullspace_norm_ = arm_result.nullspace_norm;
        robot_->updateKinematics();

        // --- Divergence guard ---
        // Only clear the INTEGRAL — a full pid_->reset() also drops the derivative
        // state and forces first_call_, momentarily removing the damping term.
        // During an oscillation that makes the guard fire repeatedly, removing
        // damping each time adds energy and makes it worse.
        static double prev_position_error = 0.0;
        static int divergence_count = 0;
        if (has_target_ && !locked_ && total_cycles_ > 0) {
            if (position_error > prev_position_error * 1.05) {
                divergence_count++;
                if (divergence_count >= 5) {
                    RCLCPP_WARN(get_logger(),
                        "[DIVERGE] error rising %.3f→%.3f over %d cycles — clearing integral",
                        prev_position_error, position_error, divergence_count);
                    pid_->resetIntegral();
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

        // --- Periodic status (1 Hz, 50 cycles at 50 Hz) ---
        static int status_counter = 0;
        if (++status_counter % 50 == 0 && gravity_received_) {
            double q0 = robot_->q_(robot_->arm_joint_indices_[0]);
            double q1 = robot_->q_(robot_->arm_joint_indices_[1]);
            double q2 = robot_->q_(robot_->arm_joint_indices_[2]);
            double q3 = robot_->q_(robot_->arm_joint_indices_[3]);
            double dq0 = robot_->dq_(robot_->arm_joint_indices_[0]);
            double dq1 = robot_->dq_(robot_->arm_joint_indices_[1]);
            double dq2 = robot_->dq_(robot_->arm_joint_indices_[2]);
            double dq3 = robot_->dq_(robot_->arm_joint_indices_[3]);
            double t0 = last_arm_tau_.size() >= 4 ? last_arm_tau_(0) : 0.0;
            double t1 = last_arm_tau_.size() >= 4 ? last_arm_tau_(1) : 0.0;
            double t2 = last_arm_tau_.size() >= 4 ? last_arm_tau_(2) : 0.0;
            double t3 = last_arm_tau_.size() >= 4 ? last_arm_tau_(3) : 0.0;
            printf("[ARM] tgt=[%.3f %.3f %.3f] ee=[%.3f %.3f %.3f] err=%.3f m "
                   "q[sp=%.2f ro=%.2f ya=%.2f elb=%.2f] "
                   "tau[%.1f %.1f %.1f %.1f] int=%.3f ns=%.2f "
                   "dq[%.2f %.2f %.2f %.2f] grav=PINOCCHIO gd=%.2f\n",
                target_pos_.x(), target_pos_.y(), target_pos_.z(),
                actual_ee_world_.x(), actual_ee_world_.y(), actual_ee_world_.z(),
                position_error, q0, q1, q2, q3,
                t0, t1, t2, t3,
                last_integral_norm_, last_nullspace_norm_,
                dq0, dq1, dq2, dq3,
                pid_->hasExternalGravity() ? pid_->gravityDiff() : -1.0);
            fflush(stdout);
        }

    }

    // ========================================================================
    // Publishers
    // ========================================================================

    void publishJointStates()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        // Plan C: publish only arm joints (4-DOF). Wrist joints come from hand_controller.
        int n_arm = static_cast<int>(robot_->arm_joint_indices_.size());
        for (int k = 0; k < n_arm; ++k) {
            int i = robot_->arm_joint_indices_[k];
            msg.name.push_back(robot_->active_joint_names_[i]);
            msg.position.push_back(robot_->q_(i));
            msg.velocity.push_back(0.0);
            // Torque mode: effort = τ_cmd (from computeArm). Position mode: 0.
            double tau = (last_arm_tau_.size() == n_arm) ? last_arm_tau_(k) : 0.0;
            msg.effort.push_back(tau);
        }
        joint_state_pub_->publish(msg);
    }

    void publishTargetMarker()
    {
        if (!has_target_) return;
        robot_->updateKinematics();
        // Marker frame_id is "pelvis" (matches RViz fixed frame), so compute
        // target position in pelvis frame — do NOT add pelvis_offset_ here.
        // pelvis_offset_ is only needed when converting to MuJoCo world coords.
        Eigen::Vector3d shoulder_urdf = robot_->getRightShoulderPosition();
        Eigen::Vector3d target_in_pelvis = shoulder_urdf + target_pos_;

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "pelvis";
        marker.header.stamp = this->now();
        marker.ns = "target"; marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = target_in_pelvis.x();
        marker.pose.position.y = target_in_pelvis.y();
        marker.pose.position.z = target_in_pelvis.z();
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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    Eigen::Vector3d pelvis_offset_{0.0, 0.0, 0.793};
    Eigen::Vector3d target_pos_{0.18, -0.05, -0.10};
    Eigen::Vector3d actual_ee_world_{0.0, 0.0, 0.0};

    bool has_target_{false};
    bool joint_state_received_{false};
    bool ee_pose_received_{false};
    bool gravity_received_{false};
    bool gravity_logged_{false};
    bool locked_{false};
    int total_cycles_{0};

    Eigen::VectorXd converged_q_;
    Eigen::VectorXd last_arm_tau_;   // torque-mode: τ_cmd for the 4 arm joints (N·m)
    double last_integral_norm_{0.0};   // task-space integral |∫e| (diagnostic)
    double last_nullspace_norm_{0.0};  // null-space dq magnitude (diagnostic)
    double dt_{0.02};
};

}  // namespace g1_right_arm_reach

// ============================================================================
// Main — ROS2 entry point
// ============================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<g1_right_arm_reach::ReachNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
