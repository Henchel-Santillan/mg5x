#include "session.hpp"

#include <chrono>
#include <iostream>

#include <fmt/chrono.h>
#include <fmt/format.h>

namespace mg5x {

namespace {
std::time_t get_now_as_time_t() {
    return std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
}
}

session_handler::session_handler(const filesystem::path &root)
    : base_dir(),
      current_session()
{
    // Generate time-stamped directory
    const auto dir_name = fmt::format("{:%Y-%m-%d}", fmt::localtime(get_now_as_time_t()));
    base_dir = root / dir_name;

    try {
        if (!filesystem::exists(base_dir)) {
            filesystem::create_directories(base_dir);
            filesystem::permissions(base_dir, filesystem::perms::owner_all);
        }
    } catch (const filesystem::filesystem_error &error) {
        const auto code = error.code();
        const auto error_string = fmt::format("Error {}: {}\n", code.value(), error.what());
        std::cerr << error_string << std::endl;
    }
}

session_info session_handler::current_session_info() const noexcept {
    return current_session;
}

bool session_handler::new_session() {
    // Reset the counter
    current_session.counter = 0;
    
    current_session.path = base_dir / filesystem::path(fmt::format("Images_{:%H-%M-%S}", fmt::localtime(get_now_as_time_t())));
    try {
        if (!filesystem::exists(current_session.path) && 
                !filesystem::create_directory(current_session.path)) {
            return false;
        }

        filesystem::permissions(current_session.path, filesystem::perms::owner_all);

    } catch (const filesystem::filesystem_error &error) {
        const auto code = error.code();
        const auto error_string = fmt::format("Error {}: {}\n", code.value(), error.what());
        std::cerr << error_string << std::endl;
        return false;
    }

    return true;
}

void session_handler::bump_session_counter() {
    current_session.counter++;
}

}
