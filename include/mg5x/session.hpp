#pragma once

#include <filesystem>
#include <string>

namespace mg5x {

namespace filesystem = std::filesystem;

struct session_info {
    session_info() {}
    session_info(const std::size_t id, const std::size_t counter, const filesystem::path &path)
        : id(id),
          counter(counter),
          path(path) {}

    std::size_t id;
    std::size_t counter;
    filesystem::path path;
};

class session_handler {
public:
    explicit session_handler(const filesystem::path &root);
    [[nodiscard]] session_info current_session_info() const noexcept;
    bool new_session();
    void bump();

private:
    filesystem::path base_dir;
    session_info current_session;
};

}
