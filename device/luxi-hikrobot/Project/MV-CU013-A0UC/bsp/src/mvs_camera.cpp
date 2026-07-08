#include "luxitech/mvs/mvs_camera.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <vector>

namespace luxitech::mvs {

namespace {

std::mutex g_sdk_mutex;
bool g_sdk_initialized = false;
std::size_t g_sdk_ref_count = 0;

std::string format_error(const std::string& what, unsigned int code) {
    std::ostringstream oss;
    oss << what << " failed: 0x" << std::hex << std::setw(8) << std::setfill('0') << code;
    return oss.str();
}

}  // namespace

MvsError::MvsError(const std::string& message) : std::runtime_error(message) {}

MvsCamera::MvsCamera() {
    ensure_sdk_initialized();
}

MvsCamera::~MvsCamera() {
    try {
        close();
    } catch (...) {
    }
    release_sdk();
}

void MvsCamera::ensure_sdk_initialized() {
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (!g_sdk_initialized) {
        const int ret = MV_CC_Initialize();
        if (ret != 0) {
            throw MvsError(format_error("MV_CC_Initialize", static_cast<unsigned int>(ret)));
        }
        g_sdk_initialized = true;
    }
    ++g_sdk_ref_count;
}

void MvsCamera::release_sdk() {
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (g_sdk_ref_count == 0) {
        return;
    }
    --g_sdk_ref_count;
}

std::string MvsCamera::sdk_version() {
    const unsigned int version = MV_CC_GetSDKVersion();
    std::ostringstream oss;
    oss << ((version >> 24) & 0xff) << '.'
        << ((version >> 16) & 0xff) << '.'
        << ((version >> 8) & 0xff) << '.'
        << (version & 0xff);
    return oss.str();
}

std::string MvsCamera::decode_chars(const unsigned char* data, std::size_t len) {
    const char* begin = reinterpret_cast<const char*>(data);
    const auto* end = std::find(begin, begin + len, '\0');
    return std::string(begin, end);
}

std::string MvsCamera::transport_name(unsigned int tlayer_type) {
    if (tlayer_type == MV_GIGE_DEVICE) {
        return "GigE";
    }
    if (tlayer_type == MV_USB_DEVICE) {
        return "USB3";
    }
    std::ostringstream oss;
    oss << "Unknown(0x" << std::hex << tlayer_type << ")";
    return oss.str();
}

std::vector<DeviceDescriptor> MvsCamera::enumerate_devices() {
    ensure_sdk_initialized();
    MV_CC_DEVICE_INFO_LIST list{};
    const unsigned int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list);
    release_sdk();
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_EnumDevices", ret));
    }

    std::vector<DeviceDescriptor> devices;
    devices.reserve(list.nDeviceNum);

    for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
        const auto* info = list.pDeviceInfo[i];
        if (info == nullptr) {
            continue;
        }

        DeviceDescriptor desc;
        desc.index = i;
        desc.transport = transport_name(info->nTLayerType);

        if (info->nTLayerType == MV_GIGE_DEVICE) {
            desc.serial_number = decode_chars(info->SpecialInfo.stGigEInfo.chSerialNumber, sizeof(info->SpecialInfo.stGigEInfo.chSerialNumber));
            desc.model_name = decode_chars(info->SpecialInfo.stGigEInfo.chModelName, sizeof(info->SpecialInfo.stGigEInfo.chModelName));
            desc.user_defined_name = decode_chars(info->SpecialInfo.stGigEInfo.chUserDefinedName, sizeof(info->SpecialInfo.stGigEInfo.chUserDefinedName));
        } else if (info->nTLayerType == MV_USB_DEVICE) {
            desc.serial_number = decode_chars(info->SpecialInfo.stUsb3VInfo.chSerialNumber, sizeof(info->SpecialInfo.stUsb3VInfo.chSerialNumber));
            desc.model_name = decode_chars(info->SpecialInfo.stUsb3VInfo.chModelName, sizeof(info->SpecialInfo.stUsb3VInfo.chModelName));
            desc.user_defined_name = decode_chars(info->SpecialInfo.stUsb3VInfo.chUserDefinedName, sizeof(info->SpecialInfo.stUsb3VInfo.chUserDefinedName));
        }

        devices.push_back(std::move(desc));
    }

    return devices;
}

void MvsCamera::open(std::size_t index, const std::string& serial_number) {
    if (is_open_) {
        close();
    }

    MV_CC_DEVICE_INFO_LIST list{};
    const unsigned int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_EnumDevices", ret));
    }
    if (list.nDeviceNum == 0) {
        throw MvsError("No Hikrobot camera found.");
    }

    const MV_CC_DEVICE_INFO* selected = nullptr;
    if (!serial_number.empty()) {
        for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
            const auto* info = list.pDeviceInfo[i];
            if (!info) {
                continue;
            }
            std::string sn;
            if (info->nTLayerType == MV_GIGE_DEVICE) {
                sn = decode_chars(info->SpecialInfo.stGigEInfo.chSerialNumber, sizeof(info->SpecialInfo.stGigEInfo.chSerialNumber));
            } else if (info->nTLayerType == MV_USB_DEVICE) {
                sn = decode_chars(info->SpecialInfo.stUsb3VInfo.chSerialNumber, sizeof(info->SpecialInfo.stUsb3VInfo.chSerialNumber));
            }
            if (sn == serial_number) {
                selected = info;
                break;
            }
        }
        if (!selected) {
            throw MvsError("Camera with requested serial number was not found.");
        }
    } else {
        if (index >= list.nDeviceNum || list.pDeviceInfo[index] == nullptr) {
            throw MvsError("Camera index is out of range.");
        }
        selected = list.pDeviceInfo[index];
    }

    transport_type_ = selected->nTLayerType;

    void* handle = nullptr;
    unsigned int create_ret = MV_CC_CreateHandle(&handle, selected);
    if (create_ret != MV_OK) {
        throw MvsError(format_error("MV_CC_CreateHandle", create_ret));
    }

    const unsigned int open_ret = MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
    if (open_ret != MV_OK) {
        MV_CC_DestroyHandle(handle);
        throw MvsError(format_error("MV_CC_OpenDevice", open_ret));
    }

    handle_ = handle;
    is_open_ = true;
    optimize_network_if_needed();
}

void MvsCamera::close() {
    if (is_grabbing_) {
        stop_grabbing();
    }

    if (handle_ != nullptr) {
        if (is_open_) {
            MV_CC_CloseDevice(handle_);
        }
        MV_CC_DestroyHandle(handle_);
    }

    handle_ = nullptr;
    is_open_ = false;
    is_grabbing_ = false;
    transport_type_ = 0;
}

bool MvsCamera::is_open() const {
    return is_open_;
}

bool MvsCamera::is_grabbing() const {
    return is_grabbing_;
}

void MvsCamera::require_open() const {
    if (!is_open_ || handle_ == nullptr) {
        throw MvsError("Camera is not open.");
    }
}

void MvsCamera::optimize_network_if_needed() const {
    if (transport_type_ != MV_GIGE_DEVICE) {
        return;
    }
    const unsigned int packet = MV_CC_GetOptimalPacketSize(handle_);
    if (packet > 0) {
        const unsigned int ret = MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", packet);
        if (ret != MV_OK) {
            throw MvsError(format_error("MV_CC_SetIntValueEx(GevSCPSPacketSize)", ret));
        }
    }
}

void MvsCamera::set_enum(const std::string& key, const std::string& value) {
    require_open();
    const unsigned int ret = MV_CC_SetEnumValueByString(handle_, key.c_str(), value.c_str());
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_SetEnumValueByString(" + key + "=" + value + ")", ret));
    }
}

void MvsCamera::set_int(const std::string& key, std::int64_t value) {
    require_open();
    const unsigned int ret = MV_CC_SetIntValueEx(handle_, key.c_str(), value);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_SetIntValueEx(" + key + ")", ret));
    }
}

void MvsCamera::set_float(const std::string& key, float value) {
    require_open();
    const unsigned int ret = MV_CC_SetFloatValue(handle_, key.c_str(), value);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_SetFloatValue(" + key + ")", ret));
    }
}

void MvsCamera::set_command(const std::string& key) {
    require_open();
    const unsigned int ret = MV_CC_SetCommandValue(handle_, key.c_str());
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_SetCommandValue(" + key + ")", ret));
    }
}

std::int64_t MvsCamera::get_int(const std::string& key) const {
    require_open();
    MVCC_INTVALUE_EX value{};
    const unsigned int ret = MV_CC_GetIntValueEx(handle_, key.c_str(), &value);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_GetIntValueEx(" + key + ")", ret));
    }
    return value.nCurValue;
}

IntRange MvsCamera::get_int_range(const std::string& key) const {
    require_open();
    MVCC_INTVALUE_EX value{};
    const unsigned int ret = MV_CC_GetIntValueEx(handle_, key.c_str(), &value);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_GetIntValueEx(" + key + ")", ret));
    }

    IntRange range;
    range.current = value.nCurValue;
    range.minimum = value.nMin;
    range.maximum = value.nMax;
    range.increment = std::max<std::int64_t>(1, value.nInc);
    return range;
}

float MvsCamera::get_float(const std::string& key) const {
    require_open();
    MVCC_FLOATVALUE value{};
    const unsigned int ret = MV_CC_GetFloatValue(handle_, key.c_str(), &value);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_GetFloatValue(" + key + ")", ret));
    }
    return value.fCurValue;
}

FloatRange MvsCamera::get_float_range(const std::string& key) const {
    require_open();
    MVCC_FLOATVALUE value{};
    const unsigned int ret = MV_CC_GetFloatValue(handle_, key.c_str(), &value);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_GetFloatValue(" + key + ")", ret));
    }

    FloatRange range;
    range.current = value.fCurValue;
    range.minimum = value.fMin;
    range.maximum = value.fMax;
    return range;
}

std::string MvsCamera::get_enum_symbolic(const std::string& key) const {
    require_open();
    MVCC_ENUMVALUE value{};
    const unsigned int value_ret = MV_CC_GetEnumValue(handle_, key.c_str(), &value);
    if (value_ret != MV_OK) {
        throw MvsError(format_error("MV_CC_GetEnumValue(" + key + ")", value_ret));
    }

    MVCC_ENUMENTRY entry{};
    entry.nValue = value.nCurValue;
    const unsigned int entry_ret = MV_CC_GetEnumEntrySymbolic(handle_, key.c_str(), &entry);
    if (entry_ret != MV_OK) {
        throw MvsError(format_error("MV_CC_GetEnumEntrySymbolic(" + key + ")", entry_ret));
    }

    return std::string(entry.chSymbolic);
}

std::vector<std::pair<unsigned int, std::string>> MvsCamera::get_supported_enum_entries(const std::string& key) const {
    require_open();
    MVCC_ENUMVALUE value{};
    const unsigned int ret = MV_CC_GetEnumValue(handle_, key.c_str(), &value);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_GetEnumValue(" + key + ")", ret));
    }

    std::vector<std::pair<unsigned int, std::string>> result;
    result.reserve(value.nSupportedNum);
    for (unsigned int i = 0; i < value.nSupportedNum; ++i) {
        MVCC_ENUMENTRY entry{};
        entry.nValue = value.nSupportValue[i];
        if (MV_CC_GetEnumEntrySymbolic(handle_, key.c_str(), &entry) == MV_OK) {
            result.emplace_back(entry.nValue, std::string(entry.chSymbolic));
        }
    }
    return result;
}

void MvsCamera::configure_continuous_output() {
    require_open();
    const unsigned int ret = MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous");
    if (ret != MV_OK) {
        // Some cameras do not expose this node. TriggerMode is the important one here.
    }
    set_enum("TriggerMode", "Off");
}

void MvsCamera::configure_software_trigger() {
    set_enum("TriggerMode", "On");
    set_enum("TriggerSource", "Software");
}

void MvsCamera::configure_line_trigger(const std::string& line_name, const std::string& activation) {
    set_enum("TriggerMode", "On");
    set_enum("TriggerSource", line_name);
    set_enum("TriggerActivation", activation);
}

void MvsCamera::set_auto_exposure(const std::string& mode, float lower_us, float upper_us) {
    set_enum("ExposureAuto", mode);
    if (lower_us >= 0.0f) {
        set_float("AutoExposureTimeLowerLimit", lower_us);
    }
    if (upper_us >= 0.0f) {
        set_float("AutoExposureTimeUpperLimit", upper_us);
    }
}

void MvsCamera::set_manual_exposure(float exposure_us) {
    set_enum("ExposureAuto", "Off");
    set_float("ExposureTime", exposure_us);
}

void MvsCamera::set_gain_auto(const std::string& mode) {
    set_enum("GainAuto", mode);
}

void MvsCamera::set_manual_gain(float gain_value) {
    set_enum("GainAuto", "Off");
    set_float("Gain", gain_value);
}

void MvsCamera::start_grabbing() {
    require_open();
    const unsigned int ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_StartGrabbing", ret));
    }
    is_grabbing_ = true;
}

void MvsCamera::stop_grabbing() {
    require_open();
    const unsigned int ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_StopGrabbing", ret));
    }
    is_grabbing_ = false;
}

void MvsCamera::trigger_software() {
    set_command("TriggerSoftware");
}

cv::Mat MvsCamera::convert_to_bgr(void* handle, const unsigned char* raw, std::size_t raw_size, const MV_FRAME_OUT_INFO_EX& info) {
    const int width = static_cast<int>(info.nWidth);
    const int height = static_cast<int>(info.nHeight);

    if (info.enPixelType == PixelType_Gvsp_Mono8) {
        const cv::Mat gray(height, width, CV_8UC1, const_cast<unsigned char*>(raw));
        cv::Mat bgr;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    if (info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
        const cv::Mat rgb(height, width, CV_8UC3, const_cast<unsigned char*>(raw));
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }

    MV_CC_PIXEL_CONVERT_PARAM convert{};
    convert.nWidth = info.nWidth;
    convert.nHeight = info.nHeight;
    convert.enSrcPixelType = info.enPixelType;
    convert.pSrcData = const_cast<unsigned char*>(raw);
    convert.nSrcDataLen = static_cast<unsigned int>(raw_size);
    convert.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    std::vector<unsigned char> dst(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3u);
    convert.pDstBuffer = dst.data();
    convert.nDstBufferSize = static_cast<unsigned int>(dst.size());

    const unsigned int ret = MV_CC_ConvertPixelType(handle, &convert);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_ConvertPixelType", ret));
    }

    const cv::Mat rgb(height, width, CV_8UC3, dst.data());
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr.clone();
}

FrameResult MvsCamera::grab_frame_bgr(unsigned int timeout_ms) const {
    require_open();
    if (!is_grabbing_) {
        throw MvsError("Camera is not grabbing.");
    }

    MVCC_INTVALUE payload{};
    const unsigned int payload_ret = MV_CC_GetIntValue(handle_, "PayloadSize", &payload);
    if (payload_ret != MV_OK) {
        throw MvsError(format_error("MV_CC_GetIntValue(PayloadSize)", payload_ret));
    }

    std::vector<unsigned char> raw(payload.nCurValue);
    MV_FRAME_OUT_INFO_EX info{};
    const unsigned int ret = MV_CC_GetOneFrameTimeout(handle_, raw.data(), payload.nCurValue, &info, timeout_ms);
    if (ret != MV_OK) {
        throw MvsError(format_error("MV_CC_GetOneFrameTimeout", ret));
    }

    FrameResult result;
    result.info = info;
    result.bgr = convert_to_bgr(handle_, raw.data(), info.nFrameLen, info);
    return result;
}

void MvsCamera::save_frame(const cv::Mat& frame_bgr, const std::string& output_path) const {
    std::filesystem::path path(output_path);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    if (!cv::imwrite(path.string(), frame_bgr)) {
        throw MvsError("Failed to save image to " + path.string());
    }
}

}  // namespace luxitech::mvs
