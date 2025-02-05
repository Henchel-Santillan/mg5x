#include "image_capture_node.hpp"

#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <limits>
#include <random>

namespace mg5x {

image_capture_node::image_capture_node()
    : rclcpp::Node(NODE_NAME),
      session_directory(""),
      counter(0)
{
    mission_sub = this->create_subscription<>();
    waypoint_sub = this->create_subscription<>();
}

void image_capture_node::mission_start_callback() {
    // Generate (pseudo)-random 16-bit signed integers for session ID
    std::random_device rd;
    std::array<int, std::mt19937::state_size> seed_data{};
    std::generate(seed_data.begin(), seed_data.end(), std::ref(rd));
    std::seed_seq seq(seed_data.begin(), seed_data.end());
    std::mt19937 generator;
    std::uniform_int_distribution dist{std::numeric_limits<short>::min(),
                                       std::numeric_limits<short>::max()};

    const auto nonce = static_cast<short>(dist(generator));

    using system_clock = std::chrono::system_clock;
    const auto now = system_clock::now();
    const auto time_point = system_clock::to_time_t(system_clock::now());

    auto dir_name = std::format("{}_{}", std::to_string(nonce), std::ctime(&time_point));
    std::replace_if(dir_name.begin(), dir_name.end(), ::isspace, '_');

    using filesystem = std::filesystem;
    auto path = filesystem::path{std::getenv("HOME")} / "images" / dir_name;
    if (!filesystem::exists(path) && filesystem::create_directories(path)) {
        // This is the directory where images will be saved for the current session
        session_directory = path.string();
        counter = 0;    // Reset the counter
    }
}

void image_capture_node::waypoint_callback() {
    // Use libcamera to capture an image
    std::system(std::format("rpicam-still -o {}/image{}.jpg", session_directory, counter));
    counter++;
}

}
