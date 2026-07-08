#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "luxitech/mvs/mv_sdk_api.hpp"

namespace luxitech::mvs {

class MvsError : public std::runtime_error {
public:
    explicit MvsError(const std::string& message);
};

struct DeviceDescriptor {
    std::size_t index{};
    std::string transport;
    std::string serial_number;
    std::string model_name;
    std::string user_defined_name;
};

struct FrameResult {
    cv::Mat bgr;
    MV_FRAME_OUT_INFO_EX info{};
};

struct IntRange {
    std::int64_t current{};
    std::int64_t minimum{};
    std::int64_t maximum{};
    std::int64_t increment{1};
};

struct FloatRange {
    float current{};
    float minimum{};
    float maximum{};
};

class MvsCamera {
public:
    MvsCamera();
    ~MvsCamera();

    MvsCamera(const MvsCamera&) = delete;
    MvsCamera& operator=(const MvsCamera&) = delete;

    static std::string sdk_version();
    static std::vector<DeviceDescriptor> enumerate_devices();

    void open(std::size_t index = 0, const std::string& serial_number = {});
    void close();

    bool is_open() const;
    bool is_grabbing() const;

    void configure_continuous_output();
    void configure_software_trigger();
    void configure_line_trigger(const std::string& line_name = "Line0", const std::string& activation = "RisingEdge");

    void set_auto_exposure(const std::string& mode = "Continuous", float lower_us = -1.0f, float upper_us = -1.0f);
    void set_manual_exposure(float exposure_us);
    void set_gain_auto(const std::string& mode = "Continuous");
    void set_manual_gain(float gain_value);

    void set_enum(const std::string& key, const std::string& value);
    void set_int(const std::string& key, std::int64_t value);
    void set_float(const std::string& key, float value);
    void set_command(const std::string& key);

    std::int64_t get_int(const std::string& key) const;
    IntRange get_int_range(const std::string& key) const;
    float get_float(const std::string& key) const;
    FloatRange get_float_range(const std::string& key) const;
    std::string get_enum_symbolic(const std::string& key) const;
    std::vector<std::pair<unsigned int, std::string>> get_supported_enum_entries(const std::string& key) const;

    void start_grabbing();
    void stop_grabbing();
    void trigger_software();

    FrameResult grab_frame_bgr(unsigned int timeout_ms = 1000) const;
    void save_frame(const cv::Mat& frame_bgr, const std::string& output_path) const;

private:
    static void ensure_sdk_initialized();
    static void release_sdk();

    static std::string decode_chars(const unsigned char* data, std::size_t len);
    static std::string transport_name(unsigned int tlayer_type);
    static cv::Mat convert_to_bgr(void* handle, const unsigned char* raw, std::size_t raw_size, const MV_FRAME_OUT_INFO_EX& info);

    void optimize_network_if_needed() const;
    void require_open() const;

    void* handle_{nullptr};
    unsigned int transport_type_{0};
    bool is_open_{false};
    bool is_grabbing_{false};
};

}  // namespace luxitech::mvs
