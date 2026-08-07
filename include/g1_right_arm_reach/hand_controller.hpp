#ifndef G1_RIGHT_ARM_REACH__HAND_CONTROLLER_HPP_
#define G1_RIGHT_ARM_REACH__HAND_CONTROLLER_HPP_

#include "g1_right_arm_reach/grasp_library.hpp"
#include "g1_right_arm_reach/msg/hand_command.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <Eigen/Dense>

#include <cstdint>
#include <string>
#include <vector>

namespace g1_right_arm_reach {

/** Revo2 six-motor finger controller. It intentionally has no wrist state. */
class HandController final : public rclcpp::Node {
public:
    HandController();

private:
    void handCommandCallback(const msg::HandCommand::SharedPtr command);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state);
    void tactileCallback(const std_msgs::msg::Float64MultiArray::SharedPtr tactile);
    void controlLoop();
    bool setTarget(const Eigen::VectorXd& target, double duration,
                   const std::string& grasp_name);
    void applyTactileStabilization(Eigen::VectorXd& target);
    void publish(const Eigen::VectorXd& target);

    GraspLibrary grasp_library_;
    std::vector<std::string> joint_names_;
    Eigen::VectorXd lower_limits_, upper_limits_;
    Eigen::VectorXd actual_position_, desired_position_, final_position_, active_target_;
    Eigen::VectorXd tactile_force_;
    std::string active_grasp_{"open"};
    double dt_{0.02}, interpolation_remaining_{0.0};
    double command_timeout_{0.0}, tactile_timeout_{0.20}, tactile_target_{0.35};
    double tactile_max_{0.75}, tactile_gain_{0.20}, deadman_timeout_{0.10};
    bool have_state_{false}, have_command_{false};
    bool output_enabled_{false}, tactile_enabled_{false}, tactile_valid_{false};
    bool deadman_required_{false}, deadman_{false};
    bool have_command_id_{false};
    int32_t last_command_id_{0};
    rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_tactile_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_deadman_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<msg::HandCommand>::SharedPtr command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tactile_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_command_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace g1_right_arm_reach

#endif
