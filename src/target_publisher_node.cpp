/** @file target_publisher_node.cpp
 *  Test utility: publishes a configurable 3-DOF target position on /target_pose
 *  (PoseStamped) and /target_point (Point) at a fixed rate.
 *
 *  Usage:
 *    ros2 run g1_right_arm_reach target_publisher_node
 *    ros2 run g1_right_arm_reach target_publisher_node --ros-args -p target:="[0.30,-0.10,0.88]"
 *    ros2 run g1_right_arm_reach target_publisher_node --ros-args -p "pose:='{position:{x:0.3,y:-0.1,z:0.1},orientation:{x:0,y:0,z:0,w:1}}'"
 *
 *  TARGET FRAME: shoulder-relative coordinates (origin at right shoulder joint).
 *    +X forward, +Y left, +Z up, relative to the robot's right shoulder.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <sstream>
#include <vector>

class TargetPublisherNode : public rclcpp::Node {
public:
    // ========================================================================
    // MODULE: Constructor
    // ========================================================================

    TargetPublisherNode()
        : Node("target_publisher_node")
    {
        this->declare_parameter("target", "0.18,-0.05,-0.10");
        this->declare_parameter("rate", 10.0);
        this->declare_parameter("publish_pose", true);  // true=PoseStamped, false=Point

        std::string target_str = this->get_parameter("target").as_string();
        double rate_hz = this->get_parameter("rate").as_double();
        publish_pose_ = this->get_parameter("publish_pose").as_bool();

        parseTarget(target_str);

        // Set up publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10);
        point_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/target_point", 10);

        int period_ms = static_cast<int>(1000.0 / rate_hz);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&TargetPublisherNode::publishTarget, this));

        RCLCPP_INFO(this->get_logger(),
            "TargetPublisher ready. Publishing %s: pos=[%.3f, %.3f, %.3f] ori=[%.3f,%.3f,%.3f,%.3f] at %.1f Hz "
            "(shoulder-relative)",
            publish_pose_ ? "PoseStamped" : "Point",
            target_pose_.position.x, target_pose_.position.y, target_pose_.position.z,
            target_pose_.orientation.x, target_pose_.orientation.y,
            target_pose_.orientation.z, target_pose_.orientation.w,
            rate_hz);

        RCLCPP_INFO(this->get_logger(),
            "Change target: ros2 param set /target_publisher_node target \"x,y,z\"");
        RCLCPP_INFO(this->get_logger(),
            "NOTE: Coordinates are SHOULDER-RELATIVE (origin = right shoulder joint)");

        [[maybe_unused]] auto _param_cb = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto& p : params) {
                    if (p.get_name() == "target") {
                        parseTarget(p.as_string());
                        RCLCPP_INFO(this->get_logger(),
                            "Target updated: [%.3f, %.3f, %.3f]",
                            target_pose_.position.x,
                            target_pose_.position.y,
                            target_pose_.position.z);
                    }
                }
                return result;
            });
    }

private:
    // ========================================================================
    // MODULE: Parameter parsing
    // ========================================================================

    void parseTarget(const std::string& str)
    {
        std::string cleaned = str;
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
            target_pose_.position.x = values[0];
            target_pose_.position.y = values[1];
            target_pose_.position.z = values[2];
        }
        // Default orientation: identity (hand pointing forward)
        target_pose_.orientation.x = 0.0;
        target_pose_.orientation.y = 0.0;
        target_pose_.orientation.z = 0.0;
        target_pose_.orientation.w = 1.0;
    }

    // ========================================================================
    // MODULE: Publishing
    // ========================================================================

    void publishTarget()
    {
        auto now = this->now();

        if (publish_pose_) {
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = now;
            pose_msg.header.frame_id = "right_shoulder";
            pose_msg.pose = target_pose_;
            pose_pub_->publish(pose_msg);
        } else {
            auto point_msg = geometry_msgs::msg::Point();
            point_msg.x = target_pose_.position.x;
            point_msg.y = target_pose_.position.y;
            point_msg.z = target_pose_.position.z;
            point_pub_->publish(point_msg);
        }
    }

    // ========================================================================
    // MODULE: State
    // ========================================================================

    geometry_msgs::msg::Pose target_pose_;
    bool publish_pose_{true};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================================
// MODULE: Main
// ============================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
