#pragma once

#include <libcamera/libcamera.h>

#include <memory>
#include <optional>
#include <string_view>
#include <vector>

namespace mg5x {

struct stream_params {
    constexpr stream_params() {}

    unsigned int width;
    unsigned int height;
    unsigned int stride;
    unsigned int buffer_count;

    std::optional<libcamera::PixelFormat> pixel_format = std::nullopt;
};

class mg5x_camera {
    using CameraSPtr = std::shared_ptr<libcamera::Camera>
    using ConfigStatus = libcamera::CameraConfiguration::Status;

public:
    explicit mg5x_camera(CameraSPtr camera);
    
    // Disable copy semantics
    mg5x_camera(const mg5x_camera&) = delete;
    mg5x_camera& operator=(const mg5x_camera&) = delete;

    mg5x_camera& operator=(mg5x_camera &&other) noexcept;
    mg5x_camera(mg5x_camera &&other) noexcept;
    ~mg5x_camera();

    bool start();
    bool still_capture(string_view save_location);
    bool stop();

    ConfigStatus edit_camera_configuration(const stream_params &params);

    // TODO: Find default camera ID for Camera Module V3

private:
    CameraSPtr camera;

    std::unique_ptr<libcamera::CameraConfiguration> configuration;
    std::unique_ptr<libcamera::FrameBufferAllocator> buffer_allocator;
    std::vector<std::unique_ptr<libcamera::Request>> requests;
};

}
