#pragma once

#include <filesystem>
#include <string>

namespace mg5x {

namespace filesystem = std::filesystem;

struct session_info {
    session_info() {}
    session_info(const std::size_t counter, const filesystem::path &path)
        : counter(counter),
          path(path) {}

    std::size_t counter;
    filesystem::path path;
};

class session_handler {
public:
    explicit session_handler(const filesystem::path &root);
    [[nodiscard]] session_info current_session_info() const noexcept;
    bool new_session();
    void bump_session_counter();

private:
    filesystem::path base_dir;
    session_info current_session;
};

}
