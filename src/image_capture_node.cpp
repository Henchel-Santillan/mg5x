#include "image_capture_node.hpp"
#include "session.hpp"

namespace mg5x {

image_capture_node::image_capture_node(session_handler &handler)
    : handler(handler),
      image_cap_sub(this->create_subscription(manual_controls_topic, 
        qos_history_depth, std::bind(&image_capture_node::capture_image_callback, this, std::placeholders::_1)))
{
}

void image_capture_node::capture_image_callback() {
    const auto session_info = handler.current_session_info();
    const auto new_image_name = fmt::format("Image{}.jpg", session_info.counter);
    handler.bump();

    std::thread([&](const filesystem::path &path, const std::string &image_name) {
        const auto image_path = path / image_name;
        const auto command = fmt::format("rpicam-still -o {}", image_path);
        std::system(command);   // TODO: Add error handling
    }, session_info.path, new_image_name).detach();
}

}
