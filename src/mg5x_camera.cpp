#include "mg5x_camera.hpp"

#include <exception>
#include <format>
#include <iostream>
#include <ranges>

namespace mg5x {

mg5x_camera::mg5x_camera(CameraSPtr camera)
    : camera(camera),
      configuration(nullptr),
      buffer_allocator(std::make_unique<libcamera::FrameBufferAllocator>()),
      requests()
{
    if (camera == nullptr) {
        throw std::invalid_argument(std::format("{}: Failed to construct mg5x_camera (invalid argument)". __func__));
    }

    // Acquire an exclusive lock on the camera and configure for still capture (defaults)
    camera->acquire();
    configuration = camera->generateConfiguration({libcamera::StreamRole::StillCapture});
    camera->configure(configuration.get());
}

mg5x_camera& mg5x_camera::mg5x_camera(mg5x_camera &&other) noexcept {
    // Stop the other camera before transferring ownership
    other.stop();

    camera = std::move(other.camera);
    configuration = std::move(other.configuration);
    buffer_allocator = std::move(other.buffer_allocator);
    requets = std::move(other.requests);
}

mg5x_camera::mg5x_camera(mg5x_camera &&other) noexcept 
{
    operator=(other);
}

mg5x_camera::~mg5x_camera() {
    stop();
    camera->release();
    camera->reset();
}

bool mg5x_camera::start() {
    // Allocate frame buffers for each stream configuration
    for (auto &stream_config : *configuration) {
        auto result = buffer_allocator->allocate(stream_config.stream());
        if (result < 0) {
            // Log error, printf for now
            std::cout << std::format("{}: Could not allocate frame buffers (error {})", __func__, result) << std::endl;
            return false;
        }
        const auto allocated = buffer_allocator->buffers(stream_config.stream()).size();
        std::cout << "Allocated " << allocated << " buffers for stream." << std::endl;
    }

    // Queue requests for frame capture
    std::ranges::for_each();

    camera->start();
    return true;
}

// This function assumes mg5x_camera::start() has been invoked
bool mg5x_camera::still_capture(string_view save_location) {

}

bool mg5x_camera::stop() {
    camera->stop();
}

ConfigStatus mg5x_camera::edit_camera_configuration(const stream_params &params) {
    // Only requested StillCapture StreamRole, so StreamConfiguration is at index 0
    constexpr auto role_index = 0U;
    auto stream_config = configuration->get(0);

    const auto &[width, height, stride, buffer_count, pixel_format] = stream_config;
    stream_config.size.width = width;
    stream_config.size.height = height;
    stream_config.stride = stride;
    stream_config.bufferCount = buffer_count;

    if (pixel_format.has_value()) {
        stream_config.pixelFormat = pixel_format.value();
    }

    // Attempt to find a valid configuration from requests
    const auto status = stream_config->validate();
    if (status != ConfigStatus::Invalid) {
        camera->configure(stream_config.get());
    }
    return status;
}

}
