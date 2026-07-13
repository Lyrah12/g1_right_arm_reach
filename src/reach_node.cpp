/**
 * @file reach_node.cpp
 * ROS2 node: subscribes /target_point, runs PID IK, publishes /joint_commands.
 *
 * TARGET SEMANTICS: /target_point is a 3D world-coordinate target for the
 * rubber hand WRIST JOINT (right_wrist_roll_rubber_hand body origin), NOT the
 * palm center.  The rubber hand mesh extends ~10-20 cm forward from the wrist.
 * For grasping, offset the target by ~10 cm in the hand's local forward direction
 * (roughly world +X when arm is extended forward).
 */

#include "g1_right_arm_reach/robot_model.hpp"
#include "g1_right_arm_reach/kinematics.hpp"
#include "g1_right_arm_reach/pid_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
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
    // Constructor
    // ========================================================================

    ReachNode() : Node("reach_node")
    {
        // --- Parameters ---
        this->declare_parameter("robot_model.urdf_path", "");
        this->declare_parameter("robot_model.active_joints",
            std::vector<std::string>{"right_shoulder_pitch_joint",
                "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                "right_elbow_joint", "right_wrist_roll_joint"});

        this->declare_parameter("pid.kp", 1.5);
        this->declare_parameter("pid.ki", 0.5);
        this->declare_parameter("pid.kd", 0.05);
        this->declare_parameter("pid.damping", 5.0e-4);
        this->declare_parameter("pid.tolerance", 0.01);
        this->declare_parameter("pid.posture_gain", 0.5);
        this->declare_parameter("pid.posture_target",
            std::vector<double>{-0.2, 0.0, 0.0, 0.3, 0.0});  // slightly forward + bent elbow

        this->declare_parameter("manager.control_rate", 20.0);
        this->declare_parameter("reach_node.target_topic", "/target_point");
        this->declare_parameter("reach_node.joint_state_topic", "/joint_states");
        this->declare_parameter("reach_node.base_offset",
            std::vector<double>{0.0, 0.0, 0.793});
        this->declare_parameter("reach_node.default_target",
            std::vector<double>{0.20, -0.25, 0.893});

        // --- Base offset (world → pelvis) ---
        auto bo = this->get_parameter("reach_node.base_offset").as_double_array();
        if (bo.size() >= 3) base_offset_ << bo[0], bo[1], bo[2];

        // --- Default target ---
        auto dt_vec = this->get_parameter("reach_node.default_target").as_double_array();
        if (dt_vec.size() >= 3) target_ << dt_vec[0], dt_vec[1], dt_vec[2];

        // --- Load robot model ---
        std::string urdf_path = this->get_parameter("robot_model.urdf_path").as_string();
        if (urdf_path.empty()) {
            RCLCPP_ERROR(get_logger(), "URDF path not set!");
            rclcpp::shutdown(); return;
        }
        robot_ = std::make_unique<RobotModel>(urdf_path,
            this->get_parameter("robot_model.active_joints").as_string_array());
        robot_->printSummary();

        // --- PID controller (kin_ kept alive for PID's internal reference) ---
        kin_ = std::make_unique<Kinematics>(*robot_);
        pid_ = std::make_unique<PIDController>(*robot_, *kin_,
            this->get_parameter("pid.kp").as_double(),
            this->get_parameter("pid.ki").as_double(),
            this->get_parameter("pid.kd").as_double(),
            this->get_parameter("pid.damping").as_double(),
            this->get_parameter("pid.tolerance").as_double(),
            this->get_parameter("pid.posture_gain").as_double());

        // Set posture target from parameters (must match MuJoCo ready pose)
        {
            auto pt = this->get_parameter("pid.posture_target").as_double_array();
            Eigen::VectorXd q_pref(static_cast<int>(pt.size()));
            for (size_t i = 0; i < pt.size(); ++i) q_pref(static_cast<int>(i)) = pt[i];
            pid_->setPostureTarget(q_pref);
            RCLCPP_INFO(get_logger(), "Posture target: [%.2f, %.2f, %.2f, %.2f, %.2f]",
                q_pref(0), q_pref(1), q_pref(2), q_pref(3), q_pref(4));
        }

        // --- ROS2 communication ---
        target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            this->get_parameter("reach_node.target_topic").as_string(), 10,
            std::bind(&ReachNode::targetCallback, this, std::placeholders::_1));

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

        // --- Parameter callback for runtime PID tuning ---
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

        RCLCPP_INFO(get_logger(), "ReachNode ready. Rate: %.1f Hz", control_rate);
    }

private:
    // ========================================================================
    // Callbacks
    // ========================================================================

    void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        Eigen::Vector3d new_target(msg->x, msg->y, msg->z);
        // Deduplicate: skip if target unchanged (prevents PID reset from
        // repeated identical publishes, e.g. target_publisher_node at 10 Hz).
        if (has_target_ && new_target.isApprox(target_, 1e-6)) return;
        target_ = new_target;
        has_target_ = true;
        locked_ = false;
        pid_->reset();
        RCLCPP_INFO(get_logger(), "New target: [%.3f, %.3f, %.3f]", msg->x, msg->y, msg->z);
    }

    void eePoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        actual_ee_world_ << msg->point.x, msg->point.y, msg->point.z;
        ee_pose_received_ = true;
    }

    void jacobianCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Expect 15 elements: 3×5 row-major [J00..J04, J10..J14, J20..J24]
        if (msg->data.size() < 15) return;
        mujoco_jacobian_.resize(3, 5);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 5; ++c)
                mujoco_jacobian_(r, c) = msg->data[r * 5 + c];
        if (!jacobian_received_) {
            jacobian_received_ = true;
            RCLCPP_INFO(get_logger(), "MuJoCo Jacobian received (3x5), |J[:3,:3]|=%.6f",
                mujoco_jacobian_.leftCols<3>().determinant());
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
        joint_state_received_ = true;
    }

    // ========================================================================
    // Control loop (20 Hz)
    // ========================================================================

    void controlLoop()
    {
        if (!joint_state_received_ || !ee_pose_received_ || !has_target_) return;

        Eigen::VectorXd q = robot_->q_;
        Eigen::VectorXd dq = robot_->dq_;

        // Error from MuJoCo ground-truth EE (world frame)
        double position_error = (target_ - actual_ee_world_).norm();

        // Convergence lock with 5cm hysteresis
        if (locked_) {
            if (position_error > 0.05) { locked_ = false; pid_->reset(); }
            else { robot_->q_ = converged_q_; robot_->updateKinematics(); publishJointStates(); publishTargetMarker(); return; }
        }
        if (position_error <= pid_->getTolerance()) {
            locked_ = true; converged_q_ = robot_->q_;
            RCLCPP_INFO(get_logger(), "[CONVERGED] error=%.3f m", position_error);
            publishJointStates(); publishTargetMarker(); return;
        }

        // PID step (external error = MuJoCo world error = pelvis error since offset cancels)
        Eigen::Vector3d pelvis_target = target_ - base_offset_;
        Eigen::Vector3d pelvis_error = pelvis_target - (actual_ee_world_ - base_offset_);
        auto result = pid_->compute(pelvis_target, q, dq, dt_, &pelvis_error,
                                    jacobian_received_ ? &mujoco_jacobian_ : nullptr);
        total_cycles_++;

        if (result.command.size() == robot_->reduced_model_.nq)
            robot_->q_ = result.command;
        robot_->updateKinematics();

        // --- Divergence guard: auto-reset if error grows for 5+ consecutive cycles ---
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

        static int cycle_count = 0;
        if (++cycle_count % 40 == 0) {
            // Rich debug output: target, EE, PID internal state, joint commands
            RCLCPP_INFO(get_logger(),
                "┌─ [PID #%d] J=%s",
                total_cycles_,
                jacobian_received_ ? "MuJoCo" : "URDF");
            RCLCPP_INFO(get_logger(),
                "├ TARGET world:[%.3f, %.3f, %.3f]  pelvis:[%.3f, %.3f, %.3f]",
                target_.x(), target_.y(), target_.z(),
                target_.x() - base_offset_.x(),
                target_.y() - base_offset_.y(),
                target_.z() - base_offset_.z());
            RCLCPP_INFO(get_logger(),
                "├ ERR    dX=%+.4f  dY=%+.4f  dZ=%+.4f  |e|=%.4f m  %s",
                target_.x() - actual_ee_world_.x(),
                target_.y() - actual_ee_world_.y(),
                target_.z() - actual_ee_world_.z(),
                position_error,
                position_error <= pid_->getTolerance() ? "✓ CONVERGED" : "");
            RCLCPP_INFO(get_logger(),
                "├ PID   eff_kp=%.2f  Ki∫e=%.4f  |v_task|=%.3f m/s  damp=%.1e  |dq_null|=%.3f  max|dq|=%.2f rad/s",
                result.effective_kp,
                result.integral_norm,
                result.task_velocity,
                result.adaptive_damping,
                result.nullspace_norm,
                result.max_joint_vel);
            RCLCPP_INFO(get_logger(),
                "├ CMD q: [%+.3f, %+.3f, %+.3f, %+.3f, %+.3f] rad",
                result.command.size() >= 5 ? result.command(0) : 0.0,
                result.command.size() >= 5 ? result.command(1) : 0.0,
                result.command.size() >= 5 ? result.command(2) : 0.0,
                result.command.size() >= 5 ? result.command(3) : 0.0,
                result.command.size() >= 5 ? result.command(4) : 0.0);
            RCLCPP_INFO(get_logger(),
                "├ ACT q: [%+.3f, %+.3f, %+.3f, %+.3f, %+.3f] rad",
                q.size() >= 5 ? q(0) : 0.0,
                q.size() >= 5 ? q(1) : 0.0,
                q.size() >= 5 ? q(2) : 0.0,
                q.size() >= 5 ? q(3) : 0.0,
                q.size() >= 5 ? q(4) : 0.0);
            RCLCPP_INFO(get_logger(),
                "├ DQ cmd: [%+.3f, %+.3f, %+.3f, %+.3f, %+.3f] rad/s",
                result.dq_command.size() >= 5 ? result.dq_command(0) : 0.0,
                result.dq_command.size() >= 5 ? result.dq_command(1) : 0.0,
                result.dq_command.size() >= 5 ? result.dq_command(2) : 0.0,
                result.dq_command.size() >= 5 ? result.dq_command(3) : 0.0,
                result.dq_command.size() >= 5 ? result.dq_command(4) : 0.0);
            RCLCPP_INFO(get_logger(),
                "└ q_err: [%+.3f, %+.3f, %+.3f, %+.3f, %+.3f] rad",
                result.command.size() >= 5 && q.size() >= 5 ? result.command(0) - q(0) : 0.0,
                result.command.size() >= 5 && q.size() >= 5 ? result.command(1) - q(1) : 0.0,
                result.command.size() >= 5 && q.size() >= 5 ? result.command(2) - q(2) : 0.0,
                result.command.size() >= 5 && q.size() >= 5 ? result.command(3) - q(3) : 0.0,
                result.command.size() >= 5 && q.size() >= 5 ? result.command(4) - q(4) : 0.0);
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
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = base_frame_;
        marker.header.stamp = this->now();
        marker.ns = "target"; marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = target_.x() - base_offset_.x();
        marker.pose.position.y = target_.y() - base_offset_.y();
        marker.pose.position.z = target_.z() - base_offset_.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.03; marker.scale.y = 0.03; marker.scale.z = 0.03;
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.8;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        target_marker_pub_->publish(marker);
    }

    // ========================================================================
    // State
    // ========================================================================

    std::unique_ptr<RobotModel> robot_;
    std::unique_ptr<Kinematics> kin_;          // kept alive for PID's reference
    std::unique_ptr<PIDController> pid_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ee_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    Eigen::Vector3d base_offset_{0.0, 0.0, 0.793};
    Eigen::Vector3d target_{0.20, -0.25, 0.893};
    Eigen::Vector3d actual_ee_world_{0.0, 0.0, 0.0};
    Eigen::Matrix<double, 3, Eigen::Dynamic> mujoco_jacobian_{3, 5};

    bool has_target_{false};
    bool joint_state_received_{false};
    bool ee_pose_received_{false};
    bool jacobian_received_{false};
    bool locked_{false};
    int total_cycles_{0};

    Eigen::VectorXd converged_q_;
    double dt_{0.05};
    std::string base_frame_{"pelvis"};
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
