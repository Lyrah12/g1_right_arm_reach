/** @file reach_node.cpp
 *  ROS2 node that implements closed-loop Cartesian reaching for the G1 right arm.
 *  Subscribes to a 3D target point, runs a PID joint-space controller to drive
 *  the end-effector there, and publishes joint commands plus debug topics.
 */

#include "g1_right_arm_reach/robot_model.hpp"
#include "g1_right_arm_reach/kinematics.hpp"
#include "g1_right_arm_reach/pid_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

namespace g1_right_arm_reach {

class ReachNode : public rclcpp::Node {
public:
    // ========================================================================
    // MODULE: Constructor — parameter loading, robot init, controller init,
    //          ROS2 communication setup, control-loop timer
    // ========================================================================

    ReachNode()
        : Node("reach_node")
    {
        // --------------------------------------------------------------------
        // SECTION: Declare ROS2 parameters
        // --------------------------------------------------------------------
        this->declare_parameter("robot_model.urdf_path", "");
        this->declare_parameter("robot_model.active_joints",
            std::vector<std::string>{
                "right_shoulder_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_joint",
                "right_wrist_roll_joint"
            });
        this->declare_parameter("robot_model.ee_frame_hint", "right_rubber_hand");

        this->declare_parameter("pid.kp", 2.5);
        this->declare_parameter("pid.ki", 3.0);
        this->declare_parameter("pid.kd", 0.05);
        this->declare_parameter("pid.damping", 1.0e-4);
        this->declare_parameter("pid.tolerance", 0.003);

        this->declare_parameter("manager.control_rate", 20.0);
        this->declare_parameter("reach_node.base_frame", "pelvis");

        this->declare_parameter("reach_node.target_topic", "/target_point");
        this->declare_parameter("reach_node.joint_state_topic", "/joint_states");
        this->declare_parameter("reach_node.debug_error_topic", "/debug/position_error");
        this->declare_parameter("reach_node.debug_status_topic", "/debug/controller_status");
        this->declare_parameter("reach_node.default_target",
            std::vector<double>{0.10, -0.15, 0.10});

        // --------------------------------------------------------------------
        // SECTION: Read default target from parameters
        // --------------------------------------------------------------------
        auto default_target_vec = this->get_parameter("reach_node.default_target").as_double_array();
        if (default_target_vec.size() >= 3) {
            target_ << default_target_vec[0], default_target_vec[1], default_target_vec[2];
            RCLCPP_INFO(this->get_logger(),
                "Default target: [%.3f, %.3f, %.3f]", target_.x(), target_.y(), target_.z());
        }

        // --------------------------------------------------------------------
        // SECTION: Load robot model from URDF
        // --------------------------------------------------------------------
        std::string urdf_path = this->get_parameter("robot_model.urdf_path").as_string();
        auto active_joints = this->get_parameter("robot_model.active_joints").as_string_array();

        if (urdf_path.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                "URDF path not set! Provide via --ros-args -p robot_model.urdf_path:=<path>");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loading URDF from: %s", urdf_path.c_str());
        robot_ = std::make_unique<RobotModel>(urdf_path, active_joints);
        robot_->printSummary();

        // --------------------------------------------------------------------
        // SECTION: Create kinematics helper
        // --------------------------------------------------------------------
        kin_ = std::make_unique<Kinematics>(*robot_);

        // --------------------------------------------------------------------
        // SECTION: Create PID controller
        // --------------------------------------------------------------------
        pid_ = std::make_unique<PIDController>(
            *robot_, *kin_,
            this->get_parameter("pid.kp").as_double(),
            this->get_parameter("pid.ki").as_double(),
            this->get_parameter("pid.kd").as_double(),
            this->get_parameter("pid.damping").as_double(),
            this->get_parameter("pid.tolerance").as_double()
        );

        // --------------------------------------------------------------------
        // SECTION: ROS2 communication — subscribers, publishers
        // --------------------------------------------------------------------
        std::string target_topic = this->get_parameter("reach_node.target_topic").as_string();
        std::string js_topic = this->get_parameter("reach_node.joint_state_topic").as_string();
        std::string err_topic = this->get_parameter("reach_node.debug_error_topic").as_string();
        std::string status_topic = this->get_parameter("reach_node.debug_status_topic").as_string();

        target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            target_topic, 10,
            std::bind(&ReachNode::targetCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            js_topic, 10,
            std::bind(&ReachNode::jointStateCallback, this, std::placeholders::_1));

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_commands", 10);

        error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            err_topic, 10);

        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            status_topic, 10);

        target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/debug/target_marker", 10);

        // --------------------------------------------------------------------
        // SECTION: Control loop timer — fires at the configured control rate
        // --------------------------------------------------------------------
        double control_rate = this->get_parameter("manager.control_rate").as_double();
        dt_ = 1.0 / control_rate;
        int period_ms = static_cast<int>(1000.0 / control_rate);
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&ReachNode::controlLoop, this));

        base_frame_ = this->get_parameter("reach_node.base_frame").as_string();

        // --------------------------------------------------------------------
        // SECTION: Dynamic parameter callback — runtime tuning of PID gains
        // --------------------------------------------------------------------
        [[maybe_unused]] auto _param_cb = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto& p : params) {
                    if (p.get_name() == "pid.kp") {
                        pid_->setKp(p.as_double());
                        RCLCPP_INFO(this->get_logger(), "PID kp = %.2f", p.as_double());
                    } else if (p.get_name() == "pid.ki") {
                        pid_->setKi(p.as_double());
                        RCLCPP_INFO(this->get_logger(), "PID ki = %.2f", p.as_double());
                    } else if (p.get_name() == "pid.kd") {
                        pid_->setKd(p.as_double());
                        RCLCPP_INFO(this->get_logger(), "PID kd = %.3f", p.as_double());
                    } else if (p.get_name() == "pid.damping") {
                        if (p.as_double() <= 0.0) {
                            result.successful = false;
                            result.reason = "pid.damping must be > 0";
                        } else {
                            pid_->setDamping(p.as_double());
                            RCLCPP_INFO(this->get_logger(), "PID damping = %.1e", p.as_double());
                        }
                    } else if (p.get_name() == "pid.tolerance") {
                        if (p.as_double() <= 0.0) {
                            result.successful = false;
                            result.reason = "pid.tolerance must be > 0";
                        } else {
                            pid_->setTolerance(p.as_double());
                            RCLCPP_INFO(this->get_logger(), "PID tolerance = %.3f", p.as_double());
                        }
                    }
                }
                return result;
            });

        // Publish an initial joint-state message so RViz shows the arm right away.
        publishJointStates();

        RCLCPP_INFO(this->get_logger(),
            "ReachNode ready. Control rate: %.1f Hz. Controller: PID",
            control_rate);
    }

private:
    // ========================================================================
    // MODULE: Subscription callbacks — receive target points and joint feedback
    // ========================================================================

    // Callback for /target_point (geometry_msgs/Point).
    // Updates the internal Cartesian target and flags that we have one.
    void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        target_ << msg->x, msg->y, msg->z;
        has_target_ = true;
        RCLCPP_DEBUG(this->get_logger(),
            "New target: [%.3f, %.3f, %.3f]", msg->x, msg->y, msg->z);
    }

    // Callback for joint state feedback from the simulator or real robot.
    // Maps the full joint-state array down to our 5-DOF active-joint subset.
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() >= static_cast<size_t>(robot_->reduced_model_.nq)) {
            // For each active joint, find its index in the incoming message.
            for (size_t i = 0; i < robot_->active_joint_names_.size(); ++i) {
                const auto& name = robot_->active_joint_names_[i];
                auto it = std::find(msg->name.begin(), msg->name.end(), name);
                if (it != msg->name.end()) {
                    size_t idx = std::distance(msg->name.begin(), it);
                    if (idx < msg->position.size()) {
                        robot_->q_(static_cast<int>(i)) = msg->position[idx];
                    }
                }
            }
        }
    }

    // ========================================================================
    // MODULE: Control loop — the main tick: compute, apply, publish, log
    // ========================================================================

    // Periodic control-loop callback driven by the wall timer.
    // Steps:
    //   1. If no target has been received, publish a neutral pose and return.
    //   2. Snapshot current joint state (q, dq) from the robot model.
    //   3. Run one PID compute cycle.
    //   4. Apply the resulting joint commands to the robot model and update FK.
    //   5. Publish joint commands, target marker, position error, and status.
    //   6. Periodically log status at INFO level (~every 2 s at 20 Hz).
    //   7. Print a one-shot message when the target is reached (error < 1 cm).
    void controlLoop()
    {
        if (!has_target_) {
            // No target yet — publish current (neutral) joint positions.
            publishJointStates();
            return;
        }

        double dt = dt_;

        // Snapshot the robot's internal joint state (may be updated by feedback).
        Eigen::VectorXd q = robot_->q_;
        Eigen::VectorXd dq = robot_->dq_;

        // Time the computation for diagnostic output.
        auto t_start = std::chrono::steady_clock::now();
        auto result = pid_->compute(target_, q, dq, dt);
        auto t_end = std::chrono::steady_clock::now();
        double compute_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        total_cycles_++;

        // Write the controller's joint-space command back into the robot model.
        if (result.command.size() == robot_->reduced_model_.nq) {
            robot_->q_ = result.command;
        }
        robot_->updateKinematics();

        // --- Publish all outputs ---
        publishJointStates();
        publishTargetMarker();

        // Debug: position error (scalar float).
        auto err_msg = std_msgs::msg::Float64();
        err_msg.data = result.position_error;
        error_pub_->publish(err_msg);

        // Debug: controller status (human-readable string).
        auto status_msg = std_msgs::msg::String();
        if (result.success) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                "[PID] |error|=%.4f m (%.1f cm) | compute=%.2f ms | cycles=%d",
                result.position_error,
                result.position_error * 100.0,
                compute_ms,
                total_cycles_);
            status_msg.data = buf;
        } else {
            status_msg.data = "[FAIL] " + result.status_msg;
        }
        status_pub_->publish(status_msg);

        // Log at info level every ~2 seconds (40 cycles at 20 Hz).
        static int cycle_count = 0;
        if (++cycle_count % 40 == 0) {
            RCLCPP_INFO(this->get_logger(), "%s", status_msg.data.c_str());
        }

        // Print a one-shot message when the arm is within 1 cm of the target.
        if (result.position_error < 0.01) {
            RCLCPP_INFO_ONCE(this->get_logger(),
                "Target reached! Position error: %.3f m (%.1f cm)",
                result.position_error, result.position_error * 100.0);
        }
    }

    // ========================================================================
    // MODULE: Publishers — joint commands and visualization markers
    // ========================================================================

    // Publish the current robot joint state as a JointState message on
    // /joint_commands.  Used both for the initial pose and each control step.
    void publishJointStates()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();

        for (int i = 0; i < robot_->reduced_model_.nq; ++i) {
            msg.name.push_back(
                i < static_cast<int>(robot_->active_joint_names_.size())
                ? robot_->active_joint_names_[i]
                : "joint_" + std::to_string(i));
            msg.position.push_back(robot_->q_(i));
            msg.velocity.push_back(0.0);
            msg.effort.push_back(0.0);
        }

        joint_state_pub_->publish(msg);
    }

    // Publish an RViz Marker (red sphere) at the current target location
    // so the user can visually verify where the arm is aiming.
    void publishTargetMarker()
    {
        if (!has_target_) return;

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = base_frame_;
        marker.header.stamp = this->now();
        marker.ns = "target";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = target_.x();
        marker.pose.position.y = target_.y();
        marker.pose.position.z = target_.z();
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;

        // Bright red, 80% opaque.
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;

        // Short lifetime so the marker disappears if publishing stops.
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        target_marker_pub_->publish(marker);
    }

    // ========================================================================
    // MODULE: State — owned sub-objects, ROS2 handles, target flags
    // ========================================================================

    // Core robotics objects.
    std::unique_ptr<RobotModel> robot_;
    std::unique_ptr<Kinematics> kin_;
    std::unique_ptr<PIDController> pid_;

    // ROS2 subscribers.
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // ROS2 publishers.
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;

    // Control-loop timer.
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Current Cartesian target (default: a test point in front of the robot).
    Eigen::Vector3d target_{0.10, -0.15, 0.10};
    bool has_target_{false};
    int total_cycles_{0};  ///< Cycle counter for diagnostics

    double dt_{0.05};       ///< Control timestep derived from control_rate
    std::string base_frame_{"pelvis"};  ///< TF base frame for markers
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
