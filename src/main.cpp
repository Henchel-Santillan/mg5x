#include "image_capture_node.hpp"
#include "session.hpp"

namespace filesystem = std::filesystem;

int main(int argc, char **argv) {
    filesystem::path root(filesystem::path(std::getenv("HOME")) / "images");
    if (!filesystem::exists(root)) {
        filesystem::create_directories(root);
        filesystem::permissions(root, filesystem::perms::owner_all);
    }

    // Currently, a new session is started on restart of the application
    mg5x::session_handler handler(root);
    handler.new_session();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mg5x::image_capture_node>(handler));
    rclcpp::shutdown();
    return 0;
}
