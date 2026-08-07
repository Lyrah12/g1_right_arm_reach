#include "g1_right_arm_reach/msg/end_effector_command.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class TargetPublisherNode final : public rclcpp::Node {
public:
    TargetPublisherNode() : Node("target_publisher_node")
    {
        declare_parameter("target", "0.30,-0.10,0.0,0.0");
        declare_parameter("rate", 10.0);
        parse(get_parameter("target").as_string());
        publisher_ = create_publisher<g1_right_arm_reach::msg::EndEffectorCommand>(
            "/end_effector_command", 10);
        const double rate = get_parameter("rate").as_double();
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / rate)),
            [this]() { publisher_->publish(command_); });
    }

private:
    void parse(const std::string& text)
    {
        std::stringstream stream(text);
        std::string token;
        std::vector<double> values;
        while (std::getline(stream, token, ',')) values.push_back(std::stod(token));
        if (values.size() != 4) throw std::runtime_error("target must be x,y,z,roll");
        command_.target_x = values[0];
        command_.target_y = values[1];
        command_.target_z = values[2];
        command_.target_roll = values[3];
        command_.command_id = 0;
    }

    g1_right_arm_reach::msg::EndEffectorCommand command_;
    rclcpp::Publisher<g1_right_arm_reach::msg::EndEffectorCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
