#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace g1_right_arm_reach {

class G1ArmSdkAdapter final : public rclcpp::Node {
public:
    G1ArmSdkAdapter() : Node("g1_arm_sdk_adapter")
    {
        declare_parameter("network_interface", "");
        declare_parameter("control_rate", 200.0);
        declare_parameter("enable_hardware_output", false);
        declare_parameter("deadman_required", true);
        declare_parameter("deadman_timeout", 0.10);
        // The task controller runs at 50 Hz. Five missed control periods is
        // enough scheduling margin while still failing closed quickly.
        declare_parameter("command_timeout", 0.10);
        declare_parameter("state_timeout", 0.10);
        declare_parameter("weight_ramp_rate", 0.50);
        declare_parameter("hold_kp", 40.0);
        declare_parameter("hold_kd", 1.5);
        declare_parameter("max_abs_torque", 20.0);

        const std::string interface = get_parameter("network_interface").as_string();
        if (interface.empty()) throw std::runtime_error("network_interface is required");
        output_enabled_ = get_parameter("enable_hardware_output").as_bool();
        deadman_required_ = get_parameter("deadman_required").as_bool();
        deadman_timeout_ = get_parameter("deadman_timeout").as_double();
        command_timeout_ = get_parameter("command_timeout").as_double();
        state_timeout_ = get_parameter("state_timeout").as_double();
        weight_rate_ = get_parameter("weight_ramp_rate").as_double();
        hold_kp_ = get_parameter("hold_kp").as_double();
        hold_kd_ = get_parameter("hold_kd").as_double();
        max_abs_torque_ = get_parameter("max_abs_torque").as_double();

        unitree::robot::ChannelFactory::Instance()->Init(0, interface);
        publisher_.reset(new unitree::robot::ChannelPublisher<LowCmd>("rt/arm_sdk"));
        publisher_->InitChannel();
        subscriber_.reset(new unitree::robot::ChannelSubscriber<LowState>("rt/lowstate"));
        subscriber_->InitChannel([this](const void* message) {
            std::lock_guard<std::mutex> guard(mutex_);
            std::memcpy(&state_, message, sizeof(LowState));
            have_state_ = true;
            last_state_ = std::chrono::steady_clock::now();
        }, 1);

        command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_commands", 10,
            std::bind(&G1ArmSdkAdapter::commandCallback, this, std::placeholders::_1));
        deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/arm_deadman", 10,
            [this](const std_msgs::msg::Bool::SharedPtr value) {
                deadman_ = value->data;
                last_deadman_ = std::chrono::steady_clock::now();
            });
        state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        const double rate = get_parameter("control_rate").as_double();
        dt_ = 1.0 / rate;
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(dt_)),
            std::bind(&G1ArmSdkAdapter::step, this));

        RCLCPP_WARN(get_logger(),
            "Unitree rt/arm_sdk output is %s; interface=%s; deadman=%s",
            output_enabled_ ? "ENABLED" : "LOCKED", interface.c_str(),
            deadman_required_ ? "required" : "not required");
    }

private:
    using LowCmd = unitree_hg::msg::dds_::LowCmd_;
    using LowState = unitree_hg::msg::dds_::LowState_;
    static constexpr int kWeightIndex = 29;
    static constexpr std::array<int, 5> kControlledIndices{22, 23, 24, 25, 26};
    static constexpr std::array<int, 6> kHeldIndices{12, 15, 16, 17, 18, 19};
    const std::array<std::string, 5> names_{
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"};

    void commandCallback(const sensor_msgs::msg::JointState::SharedPtr command)
    {
        std::array<double, 5> torque{};
        bool complete = true;
        for (size_t i = 0; i < names_.size(); ++i) {
            const auto iterator = std::find(command->name.begin(), command->name.end(), names_[i]);
            if (iterator == command->name.end()) { complete = false; break; }
            const size_t index = std::distance(command->name.begin(), iterator);
            if (index >= command->effort.size() || !std::isfinite(command->effort[index])) {
                complete = false;
                break;
            }
            torque[i] = std::clamp(command->effort[index], -max_abs_torque_, max_abs_torque_);
        }
        if (!complete) return;
        std::lock_guard<std::mutex> guard(mutex_);
        torque_ = torque;
        have_command_ = true;
        last_command_ = std::chrono::steady_clock::now();
    }

    void setMotor(LowCmd& command, int index, float q, float kp, float kd, float tau)
    {
        auto& motor = command.motor_cmd().at(index);
        motor.q() = q;
        motor.dq() = 0.0F;
        motor.kp() = kp;
        motor.kd() = kd;
        motor.tau() = tau;
    }

    void publishState(const LowState& state)
    {
        sensor_msgs::msg::JointState message;
        message.header.stamp = now();
        for (size_t i = 0; i < names_.size(); ++i) {
            const auto& motor = state.motor_state().at(kControlledIndices[i]);
            message.name.push_back(names_[i]);
            message.position.push_back(motor.q());
            message.velocity.push_back(motor.dq());
            message.effort.push_back(motor.tau_est());
        }
        state_pub_->publish(message);
    }

    void step()
    {
        LowState state;
        std::array<double, 5> torque{};
        bool have_state = false, command_fresh = false;
        const auto now_steady = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> guard(mutex_);
            have_state = have_state_;
            if (have_state) state = state_;
            torque = torque_;
            command_fresh = have_command_
                && std::chrono::duration<double>(now_steady - last_command_).count()
                    <= command_timeout_;
            if (have_state_ && std::chrono::duration<double>(now_steady - last_state_).count()
                    > state_timeout_) have_state = false;
        }
        if (!have_state) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "No fresh rt/lowstate; arm_sdk write inhibited");
            return;
        }
        publishState(state);

        const bool deadman_fresh = !deadman_required_
            || (deadman_ && std::chrono::duration<double>(now_steady - last_deadman_).count()
                <= deadman_timeout_);
        const bool active = output_enabled_ && command_fresh && deadman_fresh;
        if (active && !was_active_) {
            for (size_t i = 0; i < kHeldIndices.size(); ++i) {
                held_position_[i] = state.motor_state().at(kHeldIndices[i]).q();
            }
        }
        was_active_ = active;
        const float step = static_cast<float>(weight_rate_ * dt_);
        weight_ = active ? std::min(1.0F, weight_ + step)
                         : std::max(0.0F, weight_ - step);

        LowCmd command;
        for (size_t i = 0; i < kHeldIndices.size(); ++i) {
            setMotor(command, kHeldIndices[i], held_position_[i],
                     static_cast<float>(hold_kp_), static_cast<float>(hold_kd_), 0.0F);
        }
        for (size_t i = 0; i < kControlledIndices.size(); ++i) {
            const float current_q = state.motor_state().at(kControlledIndices[i]).q();
            if (active) {
                setMotor(command, kControlledIndices[i], current_q, 0.0F, 0.0F,
                         static_cast<float>(torque[i]));
            } else {
                setMotor(command, kControlledIndices[i], current_q,
                         static_cast<float>(hold_kp_), static_cast<float>(hold_kd_), 0.0F);
            }
        }
        command.motor_cmd().at(kWeightIndex).q(weight_);
        publisher_->Write(command);

        if (output_enabled_ && !active) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "arm_sdk releasing: command_fresh=%d deadman_fresh=%d weight=%.2f",
                command_fresh, deadman_fresh, weight_);
        }
    }

    std::mutex mutex_;
    LowState state_{};
    std::array<double, 5> torque_{};
    std::array<float, 6> held_position_{};
    bool have_state_{false}, have_command_{false}, deadman_{false};
    bool output_enabled_{false}, deadman_required_{true}, was_active_{false};
    double command_timeout_{0.10}, state_timeout_{0.10}, deadman_timeout_{0.10};
    double weight_rate_{0.50};
    double hold_kp_{40.0}, hold_kd_{1.5}, max_abs_torque_{20.0}, dt_{0.005};
    float weight_{0.0F};
    std::chrono::steady_clock::time_point last_state_{}, last_command_{}, last_deadman_{};

    unitree::robot::ChannelPublisherPtr<LowCmd> publisher_;
    unitree::robot::ChannelSubscriberPtr<LowState> subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

constexpr std::array<int, 5> G1ArmSdkAdapter::kControlledIndices;
constexpr std::array<int, 6> G1ArmSdkAdapter::kHeldIndices;

}  // namespace g1_right_arm_reach

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<g1_right_arm_reach::G1ArmSdkAdapter>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("g1_arm_sdk_adapter"), "%s", error.what());
    }
    rclcpp::shutdown();
    return 0;
}
