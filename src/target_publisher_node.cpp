/** @file target_publisher_node.cpp
 *  Test utility: publishes a configurable 3D target point on /target_point at a
 *  fixed rate.  Used to exercise the reach node without the full simulation.
 *
 *  Usage:
 *    ros2 run g1_right_arm_reach target_publisher_node
 *    ros2 run g1_right_arm_reach target_publisher_node --ros-args -p target:="[0.30,-0.10,0.88]"
 *
 *  TARGET FRAME: world coordinates (x=fwd, y=left, z=up), targets the right
 *  rubber hand WRIST JOINT origin.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <string>
#include <sstream>
#include <vector>

class TargetPublisherNode : public rclcpp::Node {
public:
    // ========================================================================
    // MODULE: Constructor — parameter loading, target parsing, pub + timer
    // ========================================================================

    TargetPublisherNode()
        : Node("target_publisher_node")
    {
        // Declare ROS2 parameters with defaults.
        this->declare_parameter("target", "0.20,-0.05,0.88");
        this->declare_parameter("rate", 10.0);  // publishing frequency in Hz

        std::string target_str = this->get_parameter("target").as_string();
        double rate_hz = this->get_parameter("rate").as_double();

        // Parse the "x,y,z" string into a Point message.
        parseTarget(target_str);

        // Set up ROS2 publisher and periodic timer.
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/target_point", 10);

        int period_ms = static_cast<int>(1000.0 / rate_hz);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&TargetPublisherNode::publishTarget, this));

        RCLCPP_INFO(this->get_logger(),
            "TargetPublisher ready. Publishing target: [%.3f, %.3f, %.3f] at %.1f Hz",
            target_.x, target_.y, target_.z, rate_hz);

        // Show how to change the target at runtime.
        RCLCPP_INFO(this->get_logger(),
            "Change target: ros2 param set /target_publisher_node target \"x,y,z\"");

        // Register a parameter callback so target changes take effect immediately.
        [[maybe_unused]] auto _param_cb = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto& p : params) {
                    if (p.get_name() == "target") {
                        parseTarget(p.as_string());
                        RCLCPP_INFO(this->get_logger(),
                            "Target updated: [%.3f, %.3f, %.3f]",
                            target_.x, target_.y, target_.z);
                    }
                }
                return result;
            });
    }

private:
    // ========================================================================
    // MODULE: Parameter parsing — convert "x,y,z" string to Point fields
    // ========================================================================

    // Parse a comma-separated string (optionally wrapped in brackets) into the
    // target_ Point's x, y, z fields.  Malformed tokens are logged and skipped.
    void parseTarget(const std::string& str)
    {
        std::string cleaned = str;
        // Strip optional square brackets: "[0.30,-0.40,0.20]" -> "0.30,-0.40,0.20"
        if (!cleaned.empty() && cleaned.front() == '[') cleaned.erase(0, 1);
        if (!cleaned.empty() && cleaned.back() == ']') cleaned.pop_back();

        std::stringstream ss(cleaned);
        std::string token;
        std::vector<double> values;

        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse token: '%s'", token.c_str());
            }
        }

        if (values.size() >= 3) {
            target_.x = values[0];
            target_.y = values[1];
            target_.z = values[2];
        }
    }

    // ========================================================================
    // MODULE: Publishing — timer-driven output of the target point
    // ========================================================================

    // Timer callback: publishes the current target point on /target_point.
    void publishTarget()
    {
        publisher_->publish(target_);
    }

    // ========================================================================
    // MODULE: State — ROS2 handles and the target point
    // ========================================================================

    geometry_msgs::msg::Point target_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================================
// MODULE: Main — ROS2 entry point
// ============================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
