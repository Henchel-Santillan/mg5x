#include "image_capture_node.hpp"
#include "session.hpp"

#include <thread>
#include <fmt/format.h>

namespace mg5x {

image_capture_node::image_capture_node(session_handler &handler)
    : rclcpp::Node("image_capture_node"),
      handler(handler),
      image_cap_sub(this->create_subscription<std_msgs::msg::String>(manual_controls_topic, 
        qos_history_depth, std::bind(&image_capture_node::capture_image_callback, this, std::placeholders::_1)))
{
}

void image_capture_node::capture_image_callback(const std_msgs::msg::String &message) {
    (void) message;
    const auto session_info = handler.current_session_info();
    //const auto new_image_name = fmt::format("Image{}.jpg", session_info.counter);
    const auto test_file = fmt::format("Image{}.txt", session_info.counter);
    handler.bump();

    auto image_capture_thread = std::thread([&](const filesystem::path &path, const std::string &image_name) {
        const auto image_path = path / image_name;
        //const auto command = fmt::format("rpicam-still -o {}", image_path.string());
        const auto command = fmt::format("touch {}.txt", image_path.string());
        std::system(command.c_str());   // TODO: Add error handling
    }, session_info.path, test_file);   // TODO: change back to new_image_name

    image_capture_thread.detach();
}

}
