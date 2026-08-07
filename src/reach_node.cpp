#include "g1_right_arm_reach/kinematics.hpp"
#include "g1_right_arm_reach/pid_controller.hpp"
#include "g1_right_arm_reach/robot_model.hpp"
#include "g1_right_arm_reach/msg/end_effector_command.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace g1_right_arm_reach {

class ReachNode final : public rclcpp::Node {
public:
    ReachNode() : Node("reach_node")
    {
        declareParameters();
        const auto active_joints = get_parameter("robot_model.active_joints").as_string_array();
        const std::string urdf_path = get_parameter("robot_model.urdf_path").as_string();
        if (urdf_path.empty()) throw std::runtime_error("robot_model.urdf_path is required");

        robot_ = std::make_unique<RobotModel>(
            urdf_path, active_joints, Eigen::VectorXd(),
            get_parameter("robot_model.ee_frame").as_string());
        robot_->buildSubsetIndices(active_joints);
        const auto axis_values =
            get_parameter("robot_model.tool_axis_local").as_double_array();
        if (axis_values.size() != 3) {
            throw std::runtime_error("robot_model.tool_axis_local requires three values");
        }
        kinematics_ = std::make_unique<Kinematics>(
            *robot_, Eigen::Vector3d(axis_values[0], axis_values[1], axis_values[2]));

        controller_ = std::make_unique<PIDController>(
            *robot_, *kinematics_,
            get_parameter("task_pid.kp_position").as_double(),
            get_parameter("task_pid.ki_position").as_double(),
            get_parameter("task_pid.kd_position").as_double(),
            get_parameter("task_pid.damping").as_double(),
            get_parameter("task_pid.position_tolerance").as_double(),
            get_parameter("task_pid.posture_gain").as_double());
        controller_->setRollGains(
            get_parameter("task_pid.kp_roll").as_double(),
            get_parameter("task_pid.ki_roll").as_double(),
            get_parameter("task_pid.kd_roll").as_double());
        controller_->setRollTolerance(get_parameter("task_pid.roll_tolerance").as_double());
        controller_->setJointDamping(get_parameter("task_pid.joint_damping").as_double());
        controller_->setGravityCompensation(
            get_parameter("task_pid.gravity_compensation").as_bool());
        controller_->setUseExternalGravity(
            get_parameter("task_pid.use_external_gravity").as_bool());
        controller_->setWrenchLimits(
            get_parameter("safety.max_task_force").as_double(),
            get_parameter("safety.max_roll_moment").as_double());
        controller_->setTorqueSafety(
            get_parameter("safety.effort_derate").as_double(),
            get_parameter("safety.max_joint_torque").as_double(),
            get_parameter("safety.max_torque_slew_rate").as_double());

        Eigen::VectorXd posture = robot_->q_;
        const auto posture_values = get_parameter("task_pid.posture_target").as_double_array();
        for (size_t k = 0; k < active_joints.size() && k < posture_values.size(); ++k) {
            posture(robot_->coordinateIndex(active_joints[k])) = posture_values[k];
        }
        controller_->setPostureTarget(posture);

        // Neutral-model zero is deterministic. The calibration service replaces
        // it with the actual mounted TCP orientation before physical acceptance.
        robot_->setNeutralConfig();
        roll_zero_orientation_ = kinematics_->getEEOrientation();

        command_sub_ = create_subscription<msg::EndEffectorCommand>(
            get_parameter("topics.command").as_string(), 10,
            std::bind(&ReachNode::commandCallback, this, std::placeholders::_1));
        legacy_point_sub_ = create_subscription<geometry_msgs::msg::Point>(
            get_parameter("topics.legacy_target_point").as_string(), 10,
            std::bind(&ReachNode::legacyPointCallback, this, std::placeholders::_1));
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("topics.joint_states").as_string(), rclcpp::SensorDataQoS(),
            std::bind(&ReachNode::jointStateCallback, this, std::placeholders::_1));
        deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
            get_parameter("topics.deadman").as_string(), 10,
            [this](const std_msgs::msg::Bool::SharedPtr value) {
                deadman_ = value->data;
                last_deadman_time_ = now();
            });
        gravity_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            get_parameter("topics.sim_gravity").as_string(), 10,
            std::bind(&ReachNode::gravityCallback, this, std::placeholders::_1));

        command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            get_parameter("topics.joint_commands").as_string(), 10);
        position_error_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/debug/position_error", 10);
        roll_error_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/debug/roll_error", 10);
        ctrl_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/debug/ctrl_state", 10);

        calibration_service_ = create_service<std_srvs::srv::Trigger>(
            "/calibrate_roll_zero",
            std::bind(&ReachNode::calibrateRollZero, this,
                      std::placeholders::_1, std::placeholders::_2));

        const double rate = get_parameter("control_rate").as_double();
        dt_ = 1.0 / rate;
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(dt_)),
            std::bind(&ReachNode::controlLoop, this));

        robot_->printSummary();
        RCLCPP_WARN(get_logger(),
            "Torque output is %s. Physical use also requires fresh joint states%s.",
            torque_output_enabled_ ? "ENABLED" : "LOCKED",
            deadman_required_ ? " and /arm_deadman=true" : "");
    }

private:
    void declareParameters()
    {
        const std::vector<std::string> joints{
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint"};
        declare_parameter("robot_model.urdf_path", "");
        declare_parameter("robot_model.ee_frame", "right_revo2_tcp");
        declare_parameter("robot_model.tool_axis_local",
                          std::vector<double>{0.0, 0.0, 1.0});
        declare_parameter("robot_model.active_joints", joints);
        declare_parameter("control_rate", 50.0);

        declare_parameter("task_pid.kp_position", 220.0);
        declare_parameter("task_pid.ki_position", 0.5);
        declare_parameter("task_pid.kd_position", 22.0);
        declare_parameter("task_pid.kp_roll", 10.0);
        declare_parameter("task_pid.ki_roll", 0.0);
        declare_parameter("task_pid.kd_roll", 1.2);
        declare_parameter("task_pid.damping", 1.0e-3);
        declare_parameter("task_pid.position_tolerance", 0.005);
        declare_parameter("task_pid.roll_tolerance", 0.035);
        declare_parameter("task_pid.posture_gain", 2.0);
        declare_parameter("task_pid.joint_damping", 2.0);
        declare_parameter("task_pid.gravity_compensation", true);
        declare_parameter("task_pid.use_external_gravity", false);
        declare_parameter("task_pid.posture_target",
                          std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

        declare_parameter("safety.enable_torque_output", false);
        declare_parameter("safety.deadman_required", false);
        declare_parameter("safety.deadman_timeout", 0.10);
        declare_parameter("safety.joint_state_timeout", 0.10);
        declare_parameter("safety.effort_derate", 0.50);
        declare_parameter("safety.max_joint_torque", 20.0);
        declare_parameter("safety.max_torque_slew_rate", 80.0);
        declare_parameter("safety.max_task_force", 15.0);
        declare_parameter("safety.max_roll_moment", 1.5);
        declare_parameter("trajectory.max_linear_velocity", 0.15);
        declare_parameter("trajectory.max_linear_acceleration", 0.60);
        declare_parameter("trajectory.max_roll_velocity", 0.50);
        declare_parameter("trajectory.max_roll_acceleration", 2.00);
        declare_parameter("diagnostics.log_period", 1.0);

        declare_parameter("topics.command", "/end_effector_command");
        declare_parameter("topics.legacy_target_point", "/target_point");
        declare_parameter("topics.joint_states", "/joint_states");
        declare_parameter("topics.joint_commands", "/joint_commands");
        declare_parameter("topics.deadman", "/arm_deadman");
        declare_parameter("topics.sim_gravity", "/mujoco_gravity");

        torque_output_enabled_ = get_parameter("safety.enable_torque_output").as_bool();
        deadman_required_ = get_parameter("safety.deadman_required").as_bool();
        deadman_timeout_ = get_parameter("safety.deadman_timeout").as_double();
        state_timeout_ = get_parameter("safety.joint_state_timeout").as_double();
        diagnostic_period_ = get_parameter("diagnostics.log_period").as_double();
        max_linear_velocity_ = std::max(
            0.001, get_parameter("trajectory.max_linear_velocity").as_double());
        max_linear_acceleration_ = std::max(
            0.001, get_parameter("trajectory.max_linear_acceleration").as_double());
        max_roll_velocity_ = std::max(
            0.001, get_parameter("trajectory.max_roll_velocity").as_double());
        max_roll_acceleration_ = std::max(
            0.001, get_parameter("trajectory.max_roll_acceleration").as_double());
    }

    void stopReferenceTrajectory()
    {
        reference_linear_velocity_.setZero();
        reference_roll_velocity_ = 0.0;
    }

    void updateReferenceTrajectory(double dt)
    {
        const Eigen::Vector3d delta = target_relative_ - reference_relative_;
        const double distance = delta.norm();
        if (distance > 1.0e-9) {
            const double desired_speed = std::min(
                max_linear_velocity_,
                std::sqrt(2.0 * max_linear_acceleration_ * distance));
            const Eigen::Vector3d desired_velocity = delta * (desired_speed / distance);
            Eigen::Vector3d velocity_delta =
                desired_velocity - reference_linear_velocity_;
            const double maximum_change = max_linear_acceleration_ * dt;
            if (velocity_delta.norm() > maximum_change) {
                velocity_delta *= maximum_change / velocity_delta.norm();
            }
            reference_linear_velocity_ += velocity_delta;
            const Eigen::Vector3d step = reference_linear_velocity_ * dt;
            if (step.dot(delta) > 0.0 && step.norm() >= distance) {
                reference_relative_ = target_relative_;
                reference_linear_velocity_.setZero();
            } else {
                reference_relative_ += step;
            }
        } else {
            reference_linear_velocity_.setZero();
        }

        const double roll_delta = Kinematics::wrapAngle(
            target_roll_ - reference_roll_);
        if (std::abs(roll_delta) > 1.0e-9) {
            const double desired_speed = std::min(
                max_roll_velocity_,
                std::sqrt(2.0 * max_roll_acceleration_ * std::abs(roll_delta)));
            const double desired_velocity = std::copysign(desired_speed, roll_delta);
            const double maximum_change = max_roll_acceleration_ * dt;
            reference_roll_velocity_ += std::clamp(
                desired_velocity - reference_roll_velocity_,
                -maximum_change, maximum_change);
            const double step = reference_roll_velocity_ * dt;
            if (step * roll_delta > 0.0 && std::abs(step) >= std::abs(roll_delta)) {
                reference_roll_ = target_roll_;
                reference_roll_velocity_ = 0.0;
            } else {
                reference_roll_ = Kinematics::wrapAngle(reference_roll_ + step);
            }
        } else {
            reference_roll_velocity_ = 0.0;
        }
    }

    bool validTarget(const Eigen::Vector3d& target, double roll) const
    {
        if (!target.allFinite() || !std::isfinite(roll)) return false;
        return target.x() >= -0.10 && target.x() <= 0.65
            && std::abs(target.y()) <= 0.50
            && target.z() >= -0.55 && target.z() <= 0.55
            && std::abs(roll) <= 3.14159265358979323846;
    }

    void commandCallback(const msg::EndEffectorCommand::SharedPtr command)
    {
        const Eigen::Vector3d target(
            command->target_x, command->target_y, command->target_z);
        if (!validTarget(target, command->target_roll)) {
            RCLCPP_ERROR(get_logger(), "Rejected invalid TCP command id=%d", command->command_id);
            return;
        }
        if (has_target_ && (target - target_relative_).norm() < 1.0e-10
            && std::abs(Kinematics::wrapAngle(
                command->target_roll - target_roll_)) < 1.0e-10) {
            return;
        }
        target_relative_ = target;
        target_roll_ = command->target_roll;
        has_target_ = true;
        controller_->resetIntegral();
        // Hold the configuration current when the command arrives as the
        // null-space posture preference. Otherwise the posture term keeps
        // pulling toward the startup pose, which fights the roll task once the
        // wrist has to move to a large angle (e.g. roll +0.5).
        if (have_joint_state_) {
            robot_->updateKinematics();
            controller_->setPostureTarget(robot_->q_);
        }
        RCLCPP_INFO(get_logger(), "TCP command id=%d: p=[%.3f %.3f %.3f], roll=%.3f",
                    command->command_id, target.x(), target.y(), target.z(), target_roll_);
    }

    void legacyPointCallback(const geometry_msgs::msg::Point::SharedPtr point)
    {
        const Eigen::Vector3d target(point->x, point->y, point->z);
        if (!validTarget(target, target_roll_)) return;
        if (has_target_ && (target - target_relative_).norm() < 1.0e-10) return;
        target_relative_ = target;
        has_target_ = true;
        controller_->resetIntegral();
        if (have_joint_state_) {
            robot_->updateKinematics();
            controller_->setPostureTarget(robot_->q_);
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state)
    {
        bool complete = true;
        for (const auto& name : robot_->active_joint_names_) {
            const auto iterator = std::find(state->name.begin(), state->name.end(), name);
            if (iterator == state->name.end()) {
                complete = false;
                continue;
            }
            const size_t message_index = std::distance(state->name.begin(), iterator);
            const int model_index = robot_->coordinateIndex(name);
            if (message_index >= state->position.size()
                || !std::isfinite(state->position[message_index])) {
                complete = false;
                continue;
            }
            robot_->q_(model_index) = state->position[message_index];
            robot_->dq_(model_index) =
                message_index < state->velocity.size()
                && std::isfinite(state->velocity[message_index])
                ? state->velocity[message_index] : 0.0;
        }
        if (complete) {
            have_joint_state_ = true;
            last_joint_state_time_ = now();
        }
    }

    void gravityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr message)
    {
        if (message->data.size() != static_cast<size_t>(robot_->nv())) return;
        Eigen::VectorXd gravity(robot_->nv());
        for (int i = 0; i < gravity.size(); ++i) gravity(i) = message->data[i];
        if (gravity.allFinite()) controller_->setExternalGravity(gravity);
    }

    void calibrateRollZero(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        if (!have_joint_state_) {
            response->success = false;
            response->message = "No complete joint state received";
            return;
        }
        robot_->updateKinematics();
        roll_zero_orientation_ = kinematics_->getEEOrientation();
        target_roll_ = 0.0;
        reference_initialized_ = false;
        stopReferenceTrajectory();
        controller_->reset();
        response->success = true;
        response->message = "TCP tool-axis roll zero captured from current pose";
    }

    void publishTorque(const Eigen::VectorXd& torque, bool enabled)
    {
        sensor_msgs::msg::JointState command;
        command.header.stamp = now();
        command.name = robot_->active_joint_names_;
        command.position.resize(command.name.size());
        command.velocity.assign(command.name.size(), 0.0);
        command.effort.assign(command.name.size(), 0.0);
        for (size_t k = 0; k < command.name.size(); ++k) {
            const int model_index = robot_->coordinateIndex(command.name[k]);
            command.position[k] = robot_->q_(model_index);
            if (enabled && k < static_cast<size_t>(torque.size())) command.effort[k] = torque(k);
        }
        command_pub_->publish(command);
    }

    void controlLoop()
    {
        if (!have_joint_state_) return;
        const rclcpp::Time current_time = now();
        // Use the real wall-clock control period, not the nominal dt, so the
        // reference governor advances proportionally to elapsed time even when
        // the wall timer jitters. Measured with steady_clock so ROS time /
        // /clock adjustments cannot skew the reference rate.
        const auto steady_now = std::chrono::steady_clock::now();
        double actual_dt = dt_;
        if (last_control_time_.time_since_epoch().count() > 0) {
            actual_dt = std::chrono::duration<double>(
                steady_now - last_control_time_).count();
        }
        actual_dt = std::clamp(actual_dt, 0.005, 0.05);
        last_control_time_ = steady_now;
        const bool state_fresh =
            (current_time - last_joint_state_time_).seconds() <= state_timeout_;
        const bool deadman_fresh = !deadman_required_
            || (deadman_ && last_deadman_time_.nanoseconds() > 0
                && (current_time - last_deadman_time_).seconds() <= deadman_timeout_);
        const bool safety_enabled = torque_output_enabled_ && state_fresh
            && deadman_fresh;
        if (safety_enabled && !was_safety_enabled_ && !has_target_) {
            // First takeover without an explicit task must hold the measured
            // pose, never move toward a compile-time posture default.
            controller_->setPostureTarget(robot_->q_);
            controller_->reset();
        }

        ControlResult result;
        double reported_position_error = 0.0;
        double reported_roll_error = 0.0;
        if (has_target_) {
            robot_->updateKinematics();
            const Eigen::Vector3d shoulder = robot_->getRightShoulderPosition();
            const Eigen::Vector3d achieved = robot_->getEndEffectorPosition();
            const double achieved_roll = Kinematics::toolRoll(
                roll_zero_orientation_, kinematics_->getEEOrientation(),
                kinematics_->toolAxisLocal());

            if (!reference_initialized_) {
                reference_relative_ = achieved - shoulder;
                reference_roll_ = achieved_roll;
                stopReferenceTrajectory();
                reference_initialized_ = true;
                controller_->reset();
            }

            if (safety_enabled) {
                updateReferenceTrajectory(actual_dt);
            } else {
                stopReferenceTrajectory();
            }

            const Eigen::Vector3d target_world = shoulder + reference_relative_;
            Eigen::Matrix<double, 4, 1> target_velocity;
            target_velocity.head<3>() = reference_linear_velocity_;
            target_velocity(3) = reference_roll_velocity_;
            result = controller_->computeTask(
                target_world, reference_roll_, robot_->q_, robot_->dq_, actual_dt,
                roll_zero_orientation_, target_velocity);
            // Completion feedback is always measured against the final goal,
            // not the moving setpoint governor reference.
            reported_position_error = (shoulder + target_relative_ - achieved).norm();
            reported_roll_error = std::abs(
                Kinematics::wrapAngle(target_roll_ - achieved_roll));
            // Relax the old null-space preference continuously near the goal.
            // The previous hard reset at 20 mm removed posture torque in one
            // control tick and could produce a visible late-motion hesitation.
            // This fade starts at 40 mm and reaches full rate at 10 mm.
            const double posture_relax_weight = std::clamp(
                (0.04 - reported_position_error) / 0.03, 0.0, 1.0);
            controller_->relaxPostureTarget(
                robot_->q_, actual_dt * posture_relax_weight, 0.25);
        } else {
            result = controller_->computeGravityHold(robot_->q_, robot_->dq_);
        }
        if (result.torque_command.size() !=
            static_cast<int>(robot_->active_joint_names_.size())) return;
        if (!result.torque_command.allFinite()) {
            RCLCPP_ERROR(get_logger(), "Non-finite torque command; output forced to zero");
            publishTorque(Eigen::VectorXd::Zero(robot_->nv()), false);
            controller_->reset();
            was_safety_enabled_ = false;
            return;
        }
        publishTorque(result.torque_command, safety_enabled);
        // Keep the internal slew-rate state anchored at zero while output is
        // inhibited. Re-enabling deadman therefore cannot expose a torque that
        // accumulated invisibly during the locked interval.
        if (!safety_enabled) controller_->reset();
        was_safety_enabled_ = safety_enabled;

        std_msgs::msg::Float64 value;
        value.data = reported_position_error;
        position_error_pub_->publish(value);
        value.data = reported_roll_error;
        roll_error_pub_->publish(value);

        std_msgs::msg::Float64MultiArray ctrl_state;
        ctrl_state.data = {
            result.linear_vel, result.roll_vel,
            reference_linear_velocity_.norm(), reference_roll_velocity_,
            result.force_saturated ? 1.0 : 0.0,
            result.roll_saturated ? 1.0 : 0.0,
            result.joint_torque_limited ? 1.0 : 0.0,
            result.slew_limited ? 1.0 : 0.0,
            result.allocation_min_singular};
        ctrl_state_pub_->publish(ctrl_state);

        if (diagnostic_period_ > 0.0
            && (current_time - last_diagnostic_time_).seconds() >= diagnostic_period_) {
            const auto format = [](const Eigen::VectorXd& vector) {
                std::ostringstream stream;
                stream.setf(std::ios::fixed);
                stream.precision(3);
                stream << '[';
                for (int i = 0; i < vector.size(); ++i) {
                    if (i) stream << ", ";
                    stream << vector(i);
                }
                stream << ']';
                return stream.str();
            };
            const double gravity_difference = controller_->gravityDiff();
            RCLCPP_INFO(get_logger(),
                "CTRL_DIAG mode=%s enabled=%d pos_err=%.5f roll_err=%.5f "
                "ref_pos_err=%.5f ref_roll_err=%.5f lin_vel=%.4f roll_vel=%.4f "
                "ref_vel=%.4f ref_roll_vel=%.4f limited=%d "
                "fsat=%d rsa=%d jlim=%d slew=%d alloc_sing=%.4f "
                "q=%s dq=%s tau=%s gravity=%s mujoco_gravity_diff=%.4f",
                has_target_ ? "TASK" : "HOLD", safety_enabled,
                reported_position_error, reported_roll_error,
                result.position_error, result.orientation_error,
                result.linear_vel, result.roll_vel,
                reference_linear_velocity_.norm(), reference_roll_velocity_,
                result.torque_limited,
                result.force_saturated, result.roll_saturated,
                result.joint_torque_limited, result.slew_limited,
                result.allocation_min_singular,
                format(robot_->q_).c_str(), format(robot_->dq_).c_str(),
                format(result.torque_command).c_str(),
                format(result.gravity_torque).c_str(), gravity_difference);
            if (result.allocation_min_singular < 0.05) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Low allocation Jacobian singular value %.4f; "
                    "arm pose near position-task singularity",
                    result.allocation_min_singular);
            }
            last_diagnostic_time_ = current_time;
        }

        if (!safety_enabled && torque_output_enabled_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Torque inhibited: joint_state_fresh=%d deadman=%d",
                state_fresh, deadman_fresh);
        }
    }

    std::unique_ptr<RobotModel> robot_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<PIDController> controller_;
    Eigen::Vector3d target_relative_{0.30, -0.10, 0.0};
    Eigen::Vector3d reference_relative_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d reference_linear_velocity_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond roll_zero_orientation_{Eigen::Quaterniond::Identity()};
    double target_roll_{0.0}, reference_roll_{0.0}, reference_roll_velocity_{0.0};
    double dt_{0.005}, state_timeout_{0.10};
    double deadman_timeout_{0.10}, diagnostic_period_{1.0};
    double max_linear_velocity_{0.15}, max_roll_velocity_{0.50};
    double max_linear_acceleration_{0.60}, max_roll_acceleration_{2.00};
    bool has_target_{false}, have_joint_state_{false};
    bool reference_initialized_{false};
    bool torque_output_enabled_{false}, deadman_required_{false}, deadman_{false};
    bool was_safety_enabled_{false};
    rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_deadman_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_diagnostic_time_{0, 0, RCL_ROS_TIME};
    std::chrono::steady_clock::time_point last_control_time_{};

    rclcpp::Subscription<msg::EndEffectorCommand>::SharedPtr command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr legacy_point_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ctrl_state_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace g1_right_arm_reach

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<g1_right_arm_reach::ReachNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("reach_node"), "%s", error.what());
    }
    rclcpp::shutdown();
    return 0;
}
