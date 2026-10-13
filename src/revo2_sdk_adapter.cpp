#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <stark-sdk.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace g1_right_arm_reach {

/**
 * Exclusive BrainCo SDK backend for one Revo2 device.
 *
 * A single SDK handle owns both position commands and tactile reads so the
 * serial/CAN transport is never opened concurrently by two processes. Do not
 * run this node together with BrainCo's ros2_control hardware driver.
 */
class Revo2SdkAdapter final : public rclcpp::Node {
public:
    Revo2SdkAdapter() : Node("revo2_sdk_adapter")
    {
        declare_parameter("port", "");
        declare_parameter("protocol", "auto");
        declare_parameter("slave_id", 127);
        declare_parameter("io_rate", 50.0);
        // Position+speed uses positive magnitudes; direction follows position.
        declare_parameter("finger_speeds", std::vector<int64_t>(6, 1000));
        declare_parameter("command_timeout", 0.20);
        declare_parameter("deadman_required", true);
        declare_parameter("deadman_timeout", 0.10);
        declare_parameter("deadman_topic", "/hand_deadman");
        declare_parameter("enable_position_output", false);
        declare_parameter("enable_tactile", true);
        declare_parameter("normal_force_full_scale", 1000.0);
        declare_parameter("joint_names", std::vector<std::string>{
            "right_thumb_proximal_joint", "right_thumb_metacarpal_joint",
            "right_index_proximal_joint", "right_middle_proximal_joint",
            "right_ring_proximal_joint", "right_pinky_proximal_joint"});
        declare_parameter("upper_limits", std::vector<double>{
            1.0297443, 1.5707963, 1.4137167,
            1.4137167, 1.4137167, 1.4137167});

        slave_id_ = static_cast<uint8_t>(get_parameter("slave_id").as_int());
        command_timeout_ = get_parameter("command_timeout").as_double();
        deadman_required_ = get_parameter("deadman_required").as_bool();
        deadman_timeout_ = get_parameter("deadman_timeout").as_double();
        output_enabled_ = get_parameter("enable_position_output").as_bool();
        if (command_timeout_ < 0.0 || deadman_timeout_ <= 0.0) {
            throw std::runtime_error("Revo2 safety timeouts are invalid");
        }
        tactile_enabled_ = get_parameter("enable_tactile").as_bool();
        full_scale_ = get_parameter("normal_force_full_scale").as_double();
        joint_names_ = get_parameter("joint_names").as_string_array();
        const auto limits = get_parameter("upper_limits").as_double_array();
        const auto speeds = get_parameter("finger_speeds").as_integer_array();
        if (!std::isfinite(full_scale_) || full_scale_ <= 0.0
            || joint_names_.size() != 6 || limits.size() != 6 || speeds.size() != 6) {
            throw std::runtime_error("invalid Revo2 SDK adapter parameters");
        }
        for (size_t i = 0; i < 6; ++i) {
            if (!std::isfinite(limits[i]) || limits[i] <= 0.0) {
                throw std::runtime_error("Revo2 upper limits must be finite and positive");
            }
            upper_limits_[i] = limits[i];
            if (speeds[i] < 1 || speeds[i] > 1000) {
                throw std::runtime_error("Revo2 finger_speeds must be in [1, 1000]");
            }
            finger_speeds_[i] = static_cast<uint16_t>(speeds[i]);
        }

        StarkProtocolType protocol = STARK_PROTOCOL_TYPE_AUTO;
        const std::string protocol_name = get_parameter("protocol").as_string();
        if (protocol_name == "modbus") protocol = STARK_PROTOCOL_TYPE_MODBUS;
        else if (protocol_name == "canfd") protocol = STARK_PROTOCOL_TYPE_CAN_FD;
        else if (protocol_name != "auto") {
            throw std::runtime_error("protocol must be auto, modbus, or canfd");
        }
        const std::string port = get_parameter("port").as_string();
        CDetectedDeviceList* devices = stark_auto_detect(
            true, port.empty() ? nullptr : port.c_str(), protocol);
        if (!devices || devices->count == 0) {
            if (devices) free_detected_device_list(devices);
            throw std::runtime_error("BrainCo SDK found no Revo2 device");
        }

        const CDetectedDevice* selected = nullptr;
        for (uintptr_t i = 0; i < devices->count; ++i) {
            if (devices->devices[i].slave_id == slave_id_) {
                selected = &devices->devices[i];
                break;
            }
        }
        if (!selected) {
            free_detected_device_list(devices);
            throw std::runtime_error("requested Revo2 slave_id was not detected");
        }
        protocol_ = static_cast<uint8_t>(selected->protocol);
        hardware_type_ = selected->hardware_type;
        if (static_cast<uint8_t>(hardware_type_) < 10
            || static_cast<uint8_t>(hardware_type_) >= 20) {
            free_detected_device_list(devices);
            throw std::runtime_error("position+speed backend requires a Revo2 device");
        }
        if (tactile_enabled_
            && !stark_is_touch_device(static_cast<uint8_t>(hardware_type_))) {
            free_detected_device_list(devices);
            throw std::runtime_error("detected Revo2 SKU has no tactile sensor");
        }
        if (tactile_enabled_ && hardware_type_ != STARK_HARDWARE_TYPE_REVO2_TOUCH) {
            free_detected_device_list(devices);
            throw std::runtime_error(
                "classic five-finger tactile reader selected, but detected SKU uses "
                "Pressure/Force3D/ArrayPressure API; confirm SKU before enabling tactile");
        }
        handle_ = init_from_detected(selected);
        free_detected_device_list(devices);
        if (!handle_) throw std::runtime_error("BrainCo init_from_detected failed");

        stark_set_finger_unit_mode(handle_, slave_id_, FINGER_UNIT_MODE_NORMALIZED);
        if (tactile_enabled_) stark_enable_touch_sensor(handle_, slave_id_, 0x1f);

        command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/hand_commands", rclcpp::QoS(1),
            std::bind(&Revo2SdkAdapter::commandCallback, this, std::placeholders::_1));
        deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
            get_parameter("deadman_topic").as_string(), 10,
            [this](const std_msgs::msg::Bool::SharedPtr value) {
                deadman_ = value->data;
                last_deadman_time_ = now();
            });
        state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        tactile_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/revo2/tactile_force_summary", 10);
        calibrate_service_ = create_service<std_srvs::srv::Trigger>(
            "/revo2/calibrate_tactile",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr response) {
                if (!tactile_enabled_) {
                    response->success = false;
                    response->message = "tactile acquisition is disabled";
                    return;
                }
                stark_calibrate_touch_sensor(handle_, slave_id_, 0x1f);
                response->success = true;
                response->message = "calibration requested; keep all fingers unloaded";
            });
        reset_service_ = create_service<std_srvs::srv::Trigger>(
            "/revo2/reset_tactile",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr response) {
                if (!tactile_enabled_) {
                    response->success = false;
                    response->message = "tactile acquisition is disabled";
                    return;
                }
                stark_reset_touch_sensor(handle_, slave_id_, 0x1f);
                response->success = true;
                response->message = "tactile channels reset";
            });

        const double rate = get_parameter("io_rate").as_double();
        if (!std::isfinite(rate) || rate <= 0.0) {
            throw std::runtime_error("io_rate must be positive");
        }
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / rate)),
            std::bind(&Revo2SdkAdapter::ioLoop, this));
        RCLCPP_WARN(get_logger(),
            "Exclusive Revo2 SDK backend is %s; tactile=%s. Do not run BrainCo "
            "ros2_control hardware simultaneously.",
            output_enabled_ ? "OUTPUT ENABLED" : "output locked",
            tactile_enabled_ ? "enabled" : "disabled");
    }

    ~Revo2SdkAdapter() override
    {
        if (handle_) close_device_handler(handle_, protocol_);
    }

private:
    void commandCallback(const sensor_msgs::msg::JointState::SharedPtr command)
    {
        std::array<uint16_t, 6> positions{};
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            const auto iterator =
                std::find(command->name.begin(), command->name.end(), joint_names_[i]);
            if (iterator == command->name.end()) return;
            const size_t index = std::distance(command->name.begin(), iterator);
            if (index >= command->position.size()
                || !std::isfinite(command->position[index])) return;
            const double normalized = std::clamp(
                command->position[index] / upper_limits_[i], 0.0, 1.0);
            positions[i] = static_cast<uint16_t>(std::lround(1000.0 * normalized));
        }
        pending_positions_ = positions;
        have_pending_command_ = true;
        have_command_ = true;
        last_command_time_ = now();
    }

    void publishMotorState()
    {
        CMotorStatusData* status = stark_get_motor_status(handle_, slave_id_);
        if (!status) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Revo2 motor-state read failed");
            return;
        }
        sensor_msgs::msg::JointState message;
        message.header.stamp = now();
        message.name = joint_names_;
        message.position.resize(6);
        for (size_t i = 0; i < 6; ++i) {
            message.position[i] =
                (static_cast<double>(status->positions[i]) / 1000.0) * upper_limits_[i];
        }
        free_motor_status_data(status);
        state_pub_->publish(message);
    }

    void publishTactile()
    {
        if (!tactile_enabled_) return;
        CTouchFingerData* data = stark_get_touch_status(handle_, slave_id_);
        if (!data) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Revo2 tactile read failed");
            return;
        }
        std_msgs::msg::Float64MultiArray message;
        message.data.resize(5);
        bool valid = true;
        for (size_t i = 0; i < 5; ++i) {
            const CTouchFingerItem& item = data->items[i];
            valid = valid && item.status == TOUCH_SENSOR_STATUS_NORMAL;
            const double mean = (static_cast<double>(item.normal_force1)
                + item.normal_force2 + item.normal_force3) / 3.0;
            message.data[i] = std::clamp(mean / full_scale_, 0.0, 1.0);
        }
        free_touch_finger_data(data);
        if (valid) tactile_pub_->publish(message);
    }

    void ioLoop()
    {
        const rclcpp::Time current_time = now();
        const bool command_fresh = have_command_
            && (current_time - last_command_time_).seconds() <= command_timeout_;
        const bool deadman_fresh = !deadman_required_
            || (deadman_ && last_deadman_time_.nanoseconds() > 0
                && (current_time - last_deadman_time_).seconds() <= deadman_timeout_);
        const bool output_allowed = output_enabled_ && command_fresh && deadman_fresh;

        if (have_pending_command_) {
            if (output_allowed) {
                stark_set_finger_positions_and_speeds(
                    handle_, slave_id_, pending_positions_.data(),
                    finger_speeds_.data(),
                    pending_positions_.size());
            }
            have_pending_command_ = false;
        }
        if (output_enabled_ && !output_allowed) {
            // BrainCo exposes an explicit zero-speed command. Send it while
            // locked so an in-flight position target is actively stopped, and
            // require a fresh command after deadman or command timeout.
            const std::array<int16_t, 6> stop_speeds{};
            stark_set_finger_speeds(
                handle_, slave_id_, stop_speeds.data(), stop_speeds.size());
            have_command_ = false;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Revo2 output stopped: command_fresh=%d deadman_fresh=%d",
                command_fresh, deadman_fresh);
        }
        publishMotorState();
        publishTactile();
    }

    DeviceHandler* handle_{nullptr};
    uint8_t slave_id_{127}, protocol_{0};
    StarkHardwareType hardware_type_{STARK_HARDWARE_TYPE_REVO2_BASIC};
    std::vector<std::string> joint_names_;
    std::array<double, 6> upper_limits_{};
    std::array<uint16_t, 6> pending_positions_{};
    std::array<uint16_t, 6> finger_speeds_{};
    double full_scale_{1000.0}, command_timeout_{0.20};
    double deadman_timeout_{0.10};
    bool output_enabled_{false}, tactile_enabled_{true};
    bool deadman_required_{true}, deadman_{false};
    bool have_pending_command_{false}, have_command_{false};
    rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_deadman_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tactile_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_, reset_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace g1_right_arm_reach

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<g1_right_arm_reach::Revo2SdkAdapter>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("revo2_sdk_adapter"), "%s", error.what());
    }
    rclcpp::shutdown();
    return 0;
}
