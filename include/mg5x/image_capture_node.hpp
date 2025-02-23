#pragma once

#include <rclcpp/rclcpp.hpp>

#ifdef CAPTURE_TESTING
#include <std_msgs/msg/string.hpp>
#else
#include <px4_msgs/msg/vehicle_command.hpp>
#endif

namespace mg5x {

class session_handler;

class image_capture_node : public rclcpp::Node {
public:
    image_capture_node(session_handler &handler);

private:
    static constexpr auto node_name = "image_capture_node";
    static constexpr auto qos_history_depth = 10U;

    session_handler &handler;

#ifdef CAPTURE_TESTING
    void capture_image_callback(const std_msgs::msg::String &message);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr image_cap_sub;
#else
    void capture_image_callback(const px4_msgs::msg::VehicleCommand::UniquePtr &message);
    rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_sub;
#endif
};

}
