#include "image_capture_node.hpp"
#include "session.hpp"

#include <thread>
#include <fmt/format.h>
#include <curl/curl.h>

extern "C" {
#include <errno.h>
}

namespace mg5x {

image_capture_node::image_capture_node(session_handler &handler, const path &tuning_file_path)
    : rclcpp::Node(node_name),
      handler(handler),
      tuning_file_path(tuning_file_path),
      capturing{false}
{
#ifdef CAPTURE_TESTING
    image_cap_sub = this->create_subscription<std_msgs::msg::String>("test_topic", 
        qos_history_depth, std::bind(&image_capture_node::capture_image_callback, this, std::placeholders::_1));
#else
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    RCLCPP_INFO(this->get_logger(), "Creating subscription to topic /fmu/out/manual_control_setpoint");
    manual_control_sub = this->create_subscription<ManualControlSetpoint>("/fmu/out/manual_control_setpoint", 
        qos, std::bind(&image_capture_node::capture_image_callback, this, std::placeholders::_1));
#endif
}

#ifdef CAPTURE_TESTING
void image_capture_node::capture_image_callback(const std_msgs::msg::String &message) {
    (void) message;
    spawn_and_detach_imcap_thread();
}
#else
void image_capture_node::capture_image_callback(const ManualControlSetpoint::UniquePtr &message) {
    const auto buttons = message->buttons; 

    // Note: 1024 is 2^10
    if ((buttons & 1024) != 0) {
        RCLCPP_INFO(this->get_logger(), "RB Button pressed, trying image capture...");
        spawn_and_detach_imcap_thread();
    }
}
#endif

// PRIVATE
void image_capture_node::spawn_and_detach_imcap_thread() {
    if (!capturing.exchange(true)) {
        const auto info = handler.current_session_info();
        const auto new_image_name = fmt::format("Image{}.jpg", info.counter);
        handler.bump_session_counter();
    
        RCLCPP_INFO(this->get_logger(), "Starting image capture...");

        auto image_capture_thread = std::thread([this, new_image_name](const filesystem::path &path) {
            const auto image_path = path / new_image_name;
            const auto command = fmt::format("rpicam-still -o {} -n -t 3 --tuning-file {}", image_path.string(),
                tuning_file_path.string());
            auto result = std::system(command.c_str());
            if (result != 0) {
                std::cerr << strerror(result) << std::endl;
            }

            // Upload the image to the API
            upload_image_to_api(image_path);

            capturing.store(false);
        }, info.path);

        // Allow the thread to run independently
        image_capture_thread.detach();
    }
}

void image_capture_node::upload_image_to_api(const filesystem::path &image_path) {
    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    if(curl) {
        std::string url = "http://example.com/upload"; // Replace with your server URL
        std::string filePath = image_path.string(); // Use the captured image path

        struct curl_httppost *formpost=NULL;
        struct curl_httppost *lastptr=NULL;
        struct curl_slist *headerlist=NULL;
        static const char buf[] = "Expect:";

        curl_formadd(&formpost,
                     &lastptr,
                     CURLFORM_COPYNAME, "imagefile",
                     CURLFORM_FILE, filePath.c_str(),
                     CURLFORM_END);

        headerlist = curl_slist_append(headerlist, buf);

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);

        res = curl_easy_perform(curl);

        if(res != CURLE_OK) {
            std::cerr << "cURL error: " << curl_easy_strerror(res) << std::endl;
        } else {
            RCLCPP_INFO(this->get_logger(), "Image uploaded successfully.");
        }

        curl_easy_cleanup(curl);
        curl_formfree(formpost);
        curl_slist_free_all(headerlist);
    }

    curl_global_cleanup();
}

} // namespace mg5x
