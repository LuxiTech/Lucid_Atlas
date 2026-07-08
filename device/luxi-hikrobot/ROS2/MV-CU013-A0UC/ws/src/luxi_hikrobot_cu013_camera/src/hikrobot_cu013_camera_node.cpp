#include "luxitech/mvs/mvs_camera.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace {

using luxitech::mvs::MvsCamera;
using luxitech::mvs::MvsError;

std::string normalize_auto_mode(const std::string& mode) {
    std::string value = mode;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    if (value == "on" || value == "true" || value == "1") {
        return "Continuous";
    }
    if (value == "off" || value == "false" || value == "0") {
        return "Off";
    }
    if (value == "once") {
        return "Once";
    }
    if (value == "continuous") {
        return "Continuous";
    }
    return mode;
}

sensor_msgs::msg::Image mat_to_image_msg(const cv::Mat& bgr, const std_msgs::msg::Header& header) {
    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.height = static_cast<std::uint32_t>(bgr.rows);
    msg.width = static_cast<std::uint32_t>(bgr.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(bgr.cols * bgr.elemSize());
    msg.data.resize(static_cast<std::size_t>(msg.step) * msg.height);

    if (bgr.isContinuous()) {
        std::memcpy(msg.data.data(), bgr.data, msg.data.size());
        return msg;
    }

    for (int row = 0; row < bgr.rows; ++row) {
        std::memcpy(msg.data.data() + static_cast<std::size_t>(row) * msg.step,
                    bgr.ptr(row),
                    msg.step);
    }
    return msg;
}

class HikrobotCu013CameraNode final : public rclcpp::Node {
public:
    HikrobotCu013CameraNode() : Node("hikrobot_cu013_camera_node") {
        camera_index_ = declare_parameter<int>("camera_index", 0);
        serial_number_ = declare_parameter<std::string>("serial_number", "");
        frame_id_ = declare_parameter<std::string>("frame_id", "hikrobot_cu013_optical_frame");
        output_image_topic_ = declare_parameter<std::string>("output_image_topic", "/hikrobot/cu013/image_raw");
        image_transport_topic_ = declare_parameter<std::string>("image_transport_topic", "/hikrobot/cu013/rgb_img");
        compressed_image_topic_ = declare_parameter<std::string>("compressed_image_topic", "/hikrobot/cu013/rgb_img/compressed");
        camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/hikrobot/cu013/camera_info");
        publish_compressed_ = declare_parameter<bool>("publish_compressed", true);
        compressed_jpeg_quality_ = declare_parameter<int>("compressed_jpeg_quality", 55);
        compressed_width_ = declare_parameter<int>("compressed_width", 640);
        compressed_height_ = declare_parameter<int>("compressed_height", 512);
        width_ = declare_parameter<int>("width", -1);
        height_ = declare_parameter<int>("height", -1);
        offset_x_ = declare_parameter<int>("offset_x", -1);
        offset_y_ = declare_parameter<int>("offset_y", -1);
        exposure_auto_ = normalize_auto_mode(declare_parameter<std::string>("exposure_auto", "Off"));
        exposure_time_us_ = declare_parameter<double>("exposure_time_us", 25000.0);
        gain_auto_ = normalize_auto_mode(declare_parameter<std::string>("gain_auto", "Off"));
        gain_ = declare_parameter<double>("gain", 15.0);
        pixel_format_ = declare_parameter<std::string>("pixel_format", "");
        external_trigger_enabled_ = declare_parameter<bool>("external_trigger_enabled", false);
        trigger_source_ = declare_parameter<std::string>("trigger_source", "Line0");
        trigger_activation_ = declare_parameter<std::string>("trigger_activation", "RisingEdge");
        publish_rate_ = declare_parameter<double>("publish_rate", 30.0);
        grab_timeout_ms_ = declare_parameter<int>("grab_timeout_ms", 1000);

        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            output_image_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        image_transport_pub_ = image_transport::create_publisher(this, image_transport_topic_, rmw_qos_profile_default);
        compressed_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            compressed_image_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);

        open_camera();

        parameter_callback_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                return on_parameters(parameters);
            });

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this]() { publish_frame(); });
    }

    ~HikrobotCu013CameraNode() override {
        try {
            camera_.close();
        } catch (...) {
        }
    }

private:
    void open_camera() {
        RCLCPP_INFO(get_logger(), "MVS SDK version: %s", MvsCamera::sdk_version().c_str());

        const auto devices = MvsCamera::enumerate_devices();
        if (devices.empty()) {
            throw MvsError("No Hikrobot camera found.");
        }

        for (const auto& device : devices) {
            RCLCPP_INFO(get_logger(), "camera[%zu] transport=%s serial=%s model=%s user_name=%s",
                        device.index,
                        device.transport.c_str(),
                        device.serial_number.empty() ? "-" : device.serial_number.c_str(),
                        device.model_name.empty() ? "-" : device.model_name.c_str(),
                        device.user_defined_name.empty() ? "-" : device.user_defined_name.c_str());
        }

        camera_.open(static_cast<std::size_t>(std::max(0, camera_index_)), serial_number_);
        apply_trigger_mode();

        if (width_ > 0) {
            camera_.set_int("Width", width_);
        }
        if (height_ > 0) {
            camera_.set_int("Height", height_);
        }
        if (offset_x_ >= 0) {
            camera_.set_int("OffsetX", offset_x_);
        }
        if (offset_y_ >= 0) {
            camera_.set_int("OffsetY", offset_y_);
        }
        if (!pixel_format_.empty()) {
            camera_.set_enum("PixelFormat", pixel_format_);
        }

        apply_camera_parameters();
        camera_.start_grabbing();

        RCLCPP_INFO(get_logger(),
                    "MV-CU013-A0UC publishing %s, %s, and %s at %.2f Hz, trigger=%s source=%s activation=%s, width=%d height=%d offset=(%d,%d), exposure_auto=%s exposure=%.1f us gain_auto=%s gain=%.2f compressed=%s quality=%d compressed_size=%dx%d",
                    output_image_topic_.c_str(),
                    image_transport_topic_.c_str(),
                    compressed_image_topic_.c_str(),
                    publish_rate_,
                    external_trigger_enabled_ ? "external" : "continuous",
                    trigger_source_.c_str(),
                    trigger_activation_.c_str(),
                    width_,
                    height_,
                    offset_x_,
                    offset_y_,
                    exposure_auto_.c_str(),
                    exposure_time_us_,
                    gain_auto_.c_str(),
                    gain_,
                    publish_compressed_ ? "true" : "false",
                    compressed_jpeg_quality_,
                    compressed_width_,
                    compressed_height_);
    }

    void apply_camera_parameters() {
        if (!exposure_auto_.empty()) {
            camera_.set_auto_exposure(exposure_auto_);
        }
        if (exposure_auto_ == "Off" && exposure_time_us_ > 0.0) {
            camera_.set_manual_exposure(static_cast<float>(exposure_time_us_));
        }

        if (!gain_auto_.empty()) {
            camera_.set_gain_auto(gain_auto_);
        }
        if (gain_auto_ == "Off" && gain_ >= 0.0) {
            camera_.set_manual_gain(static_cast<float>(gain_));
        }
    }

    void apply_trigger_mode() {
        if (external_trigger_enabled_) {
            camera_.configure_line_trigger(trigger_source_, trigger_activation_);
        } else {
            camera_.configure_continuous_output();
        }
    }

    void reconfigure_trigger_mode() {
        const bool was_grabbing = camera_.is_grabbing();
        if (was_grabbing) {
            camera_.stop_grabbing();
        }
        apply_trigger_mode();
        if (was_grabbing) {
            camera_.start_grabbing();
        }

        RCLCPP_INFO(get_logger(), "trigger mode changed: %s source=%s activation=%s",
                    external_trigger_enabled_ ? "external" : "continuous",
                    trigger_source_.c_str(),
                    trigger_activation_.c_str());
    }

    template <typename ApplyFn>
    void apply_while_not_grabbing(ApplyFn&& apply) {
        const bool was_grabbing = camera_.is_grabbing();
        if (was_grabbing) {
            camera_.stop_grabbing();
        }
        try {
            apply();
        } catch (...) {
            if (was_grabbing) {
                camera_.start_grabbing();
            }
            throw;
        }
        if (was_grabbing) {
            camera_.start_grabbing();
        }
    }

    rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        try {
            for (const auto& parameter : parameters) {
                const auto& name = parameter.get_name();
                if (name == "width") {
                    width_ = parameter.as_int();
                    if (width_ > 0) {
                        apply_while_not_grabbing([this]() { camera_.set_int("Width", width_); });
                    }
                } else if (name == "height") {
                    height_ = parameter.as_int();
                    if (height_ > 0) {
                        apply_while_not_grabbing([this]() { camera_.set_int("Height", height_); });
                    }
                } else if (name == "offset_x") {
                    offset_x_ = parameter.as_int();
                    if (offset_x_ >= 0) {
                        apply_while_not_grabbing([this]() { camera_.set_int("OffsetX", offset_x_); });
                    }
                } else if (name == "offset_y") {
                    offset_y_ = parameter.as_int();
                    if (offset_y_ >= 0) {
                        apply_while_not_grabbing([this]() { camera_.set_int("OffsetY", offset_y_); });
                    }
                } else if (name == "exposure_auto") {
                    exposure_auto_ = normalize_auto_mode(parameter.as_string());
                    camera_.set_auto_exposure(exposure_auto_);
                } else if (name == "exposure_time_us") {
                    exposure_time_us_ = parameter.as_double();
                    camera_.set_manual_exposure(static_cast<float>(exposure_time_us_));
                    exposure_auto_ = "Off";
                } else if (name == "gain_auto") {
                    gain_auto_ = normalize_auto_mode(parameter.as_string());
                    camera_.set_gain_auto(gain_auto_);
                } else if (name == "gain") {
                    gain_ = parameter.as_double();
                    camera_.set_manual_gain(static_cast<float>(gain_));
                    gain_auto_ = "Off";
                } else if (name == "pixel_format") {
                    pixel_format_ = parameter.as_string();
                    if (!pixel_format_.empty()) {
                        apply_while_not_grabbing([this]() { camera_.set_enum("PixelFormat", pixel_format_); });
                    }
                } else if (name == "external_trigger_enabled") {
                    external_trigger_enabled_ = parameter.as_bool();
                    reconfigure_trigger_mode();
                } else if (name == "trigger_source") {
                    trigger_source_ = parameter.as_string();
                    reconfigure_trigger_mode();
                } else if (name == "trigger_activation") {
                    trigger_activation_ = parameter.as_string();
                    reconfigure_trigger_mode();
                } else if (name == "publish_compressed") {
                    publish_compressed_ = parameter.as_bool();
                } else if (name == "compressed_jpeg_quality") {
                    compressed_jpeg_quality_ = static_cast<int>(
                        std::clamp(parameter.as_int(), static_cast<int64_t>(30), static_cast<int64_t>(95)));
                } else if (name == "compressed_width") {
                    compressed_width_ = static_cast<int>(
                        std::clamp(parameter.as_int(), static_cast<int64_t>(0), static_cast<int64_t>(4096)));
                } else if (name == "compressed_height") {
                    compressed_height_ = static_cast<int>(
                        std::clamp(parameter.as_int(), static_cast<int64_t>(0), static_cast<int64_t>(4096)));
                }
            }
        } catch (const std::exception& error) {
            result.successful = false;
            result.reason = error.what();
        }

        return result;
    }

    void publish_frame() {
        try {
            const auto frame = camera_.grab_frame_bgr(static_cast<unsigned int>(std::max(1, grab_timeout_ms_)));

            std_msgs::msg::Header header;
            header.stamp = now();
            header.frame_id = frame_id_;

            const auto image_msg = mat_to_image_msg(frame.bgr, header);
            image_pub_->publish(image_msg);
            image_transport_pub_.publish(image_msg);
            publish_compressed_frame(frame.bgr, header);

            sensor_msgs::msg::CameraInfo camera_info;
            camera_info.header = header;
            camera_info.width = static_cast<std::uint32_t>(frame.bgr.cols);
            camera_info.height = static_cast<std::uint32_t>(frame.bgr.rows);
            camera_info.distortion_model = "plumb_bob";
            camera_info_pub_->publish(camera_info);
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to grab frame: %s", error.what());
        }
    }

    void publish_compressed_frame(const cv::Mat& bgr, const std_msgs::msg::Header& header) {
        if (!publish_compressed_) {
            return;
        }

        cv::Mat encode_bgr = bgr;
        cv::Mat resized;
        if (compressed_width_ > 0 && compressed_height_ > 0 &&
            (bgr.cols != compressed_width_ || bgr.rows != compressed_height_)) {
            cv::resize(bgr, resized, cv::Size(compressed_width_, compressed_height_), 0.0, 0.0, cv::INTER_AREA);
            encode_bgr = resized;
        }

        std::vector<unsigned char> encoded;
        const int quality = std::clamp(compressed_jpeg_quality_, 30, 95);
        const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, quality};
        if (!cv::imencode(".jpg", encode_bgr, encoded, params)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to encode compressed RGB frame.");
            return;
        }

        sensor_msgs::msg::CompressedImage msg;
        msg.header = header;
        msg.format = "jpeg";
        msg.data.assign(encoded.begin(), encoded.end());
        compressed_image_pub_->publish(msg);
    }

    MvsCamera camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    image_transport::Publisher image_transport_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    int camera_index_{0};
    std::string serial_number_;
    std::string frame_id_;
    std::string output_image_topic_;
    std::string image_transport_topic_;
    std::string compressed_image_topic_;
    std::string camera_info_topic_;
    bool publish_compressed_{true};
    int compressed_jpeg_quality_{55};
    int compressed_width_{640};
    int compressed_height_{512};
    int width_{-1};
    int height_{-1};
    int offset_x_{-1};
    int offset_y_{-1};
    std::string exposure_auto_;
    double exposure_time_us_{25000.0};
    std::string gain_auto_;
    double gain_{15.0};
    std::string pixel_format_;
    bool external_trigger_enabled_{false};
    std::string trigger_source_{"Line0"};
    std::string trigger_activation_{"RisingEdge"};
    double publish_rate_{30.0};
    int grab_timeout_ms_{1000};
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<HikrobotCu013CameraNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("hikrobot_cu013_camera_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
