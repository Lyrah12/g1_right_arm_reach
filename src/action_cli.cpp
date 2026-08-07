#include "g1_right_arm_reach/msg/action_command.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

using g1_right_arm_reach::msg::ActionCommand;

namespace {
double number(const char* text) { return std::stod(text); }

void usage()
{
    std::cout
        << "action_cli reach x y z roll\n"
        << "action_cli grasp pattern [duration]\n"
        << "action_cli reach_and_grasp x y z roll pattern [duration]\n"
        << "action_cli home | open\n";
}
}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (argc < 2) { usage(); return 2; }

    ActionCommand command;
    command.action_id = static_cast<int32_t>(
        std::chrono::steady_clock::now().time_since_epoch().count() & 0x7fffffff);
    // A normal reach should settle in 3--5 s. Eight seconds is a hard
    // performance deadline, not a substitute for slow controller tuning.
    command.convergence_timeout = 8.0;
    command.settle_time = 0.2;
    command.grasp_duration = 0.5;

    try {
        const std::string type = argv[1];
        if (type == "reach" && argc == 6) {
            command.action_type = type;
            command.target_x = number(argv[2]);
            command.target_y = number(argv[3]);
            command.target_z = number(argv[4]);
            command.target_roll = number(argv[5]);
        } else if (type == "grasp" && (argc == 3 || argc == 4)) {
            command.action_type = type;
            command.grasp_type = argv[2];
            if (argc == 4) command.grasp_duration = number(argv[3]);
        } else if (type == "reach_and_grasp" && (argc == 7 || argc == 8)) {
            command.action_type = type;
            command.target_x = number(argv[2]);
            command.target_y = number(argv[3]);
            command.target_z = number(argv[4]);
            command.target_roll = number(argv[5]);
            command.grasp_type = argv[6];
            if (argc == 8) command.grasp_duration = number(argv[7]);
        } else if (type == "home" && argc == 2) {
            command.action_type = "home";
        } else if (type == "open" && argc == 2) {
            command.action_type = "grasp";
            command.grasp_type = "open";
            command.grasp_duration = 0.3;
        } else {
            usage();
            return 2;
        }
    } catch (const std::exception& error) {
        std::cerr << "Invalid argument: " << error.what() << '\n';
        return 2;
    }

    auto node = std::make_shared<rclcpp::Node>("action_cli");
    auto publisher = node->create_publisher<ActionCommand>("/action_command", 10);
    // Wait for the sequencer to subscribe before publishing anything. If the
    // launch is not up yet the command would otherwise be dropped silently and
    // the user would wait forever with no hand movement — so fail loudly
    // instead. Once subscribed, repeat-publish: a single publish can still be
    // dropped by the DDS discovery race even after the subscription is seen.
    const auto connect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (publisher->get_subscription_count() == 0
           && std::chrono::steady_clock::now() < connect_deadline) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (publisher->get_subscription_count() == 0) {
        std::cerr << "ERROR: no subscriber on /action_command after 15s.\n"
                  << "Start the launch (ros2 launch g1_right_arm_reach "
                     "mujoco_demo.launch.py) first, then retry.\n";
        rclcpp::shutdown();
        return 1;
    }
    const auto publish_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
    int count = 0;
    while (std::chrono::steady_clock::now() < publish_deadline) {
        publisher->publish(command);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++count;
    }
    std::cout << "Published action id=" << command.action_id
              << " (" << count << " copies)\n";
    rclcpp::shutdown();
    return 0;
}
