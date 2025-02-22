#pragma once

#include <rclcpp/rclcpp.h>
#include <std_msgs/msg/string.hpp>  // TODO: Remove

namespace mg5x {

class session_handler;

class image_capture_node : public rclcpp::Node {
public:
    image_capture_node(session_handler &handler);

private:
    static constexpr auto qos_history_depth = 10U;
    static constexpr auto manual_controls_topic = "test_topic"; // TODO: replace with actual topic

    void capture_image_callback();

    session_handler &handler;
    rclcpp:Subscription<std_msgs::msg::String>::SharedPtr image_cap_sub; // TODO: Change to actual subscription
};

}
