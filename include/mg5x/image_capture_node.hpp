#pragma once

#include <rclcpp/rclcpp.h>
#include <string>

namespace mg5x {

class image_capture_node : public rclcpp::Node {
public:
    image_capture_node();

private:
    void mission_start_callback();
    void waypoint_callback();

    static constexpr auto NODE_NAME = "image_capture_node";
    
    rclcpp::Subscription<> mission_sub;     // Trigger for button assignment
    rclcpp::Subscription<> waypoint_sub;    // Waypoint reached
    std::string session_directory;
    int counter;
};

}
