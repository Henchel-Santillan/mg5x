#include "session.hpp"

#include <array>
#include <chrono>
#include <iostream>
#include <limits>
#include <random>

#include <fmt/chrono.h>
#include <fmt/format.h>

namespace mg5x {

// Pass filesystem::path(std::getenv("HOME")) / "images" as the base_dir

session_handler::session_handler(const filesystem::path &root)
    : base_dir(),
      current_session()
{
    // Generate time-stamped directory
    const auto now = std::chrono::system_clock::now();
    const auto dir_name = fmt::format("{:%Y_%m_%d}", now);
    base_dir = root / dir_name;

    try {
        if (!filesystem::exists(base_dir)) {
            filesystem::create_directories(base_dir);
            filesystem::permissions(base_dir, filesystem::perms::owner_all);
        }
    } catch (const filesystem::filesystem_error &error) {
        const auto code = error.code();
        auto error_string = fmt::format("Error {}: {}\n", code.value(), error.what());
        std::cerr << error_string << std::endl;
    }
}

session_info session_handler::current_session_info() const noexcept {
    return current_session;
}

bool session_handler::new_session() {
    // Generate random session id
    std::random_device rd;
    std::array<int, std::mt19937::state_size> seed_data{};
    std::generate(seed_data.begin(), seed_data.end(), std::ref(rd));
    std::seed_seq seq(seed_data.begin(), seed_data.end());
    std::mt19937 generator;
    std::uniform_int_distribution dist{std::numeric_limits<std::size_t>::min(),
                                       std::numeric_limits<std::size_t>::max()};

    current_session.id = static_cast<std::size_t>(dist(generator));

    // Reset the counter
    current_session.counter = 0;
    
    current_session.path = base_dir / filesystem::path(fmt::format("Images_{:%H_%M_%S}_{}", current_session.id));
    if (!filesystem::exists(current_session.path) && 
            !filesystem::create_directory(current_session.path)) {
        return false;
    }

    filesystem::permissions(current_session.path, filesystem::perms::owner_all);
    return true;
}

void session_handler::bump() {
    current_session.counter++;
}

}
