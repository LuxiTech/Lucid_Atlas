#include "luxitech/mvs/mvs_camera.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using luxitech::mvs::DeviceDescriptor;
using luxitech::mvs::FloatRange;
using luxitech::mvs::FrameResult;
using luxitech::mvs::IntRange;
using luxitech::mvs::MvsCamera;
using luxitech::mvs::MvsError;

namespace {

constexpr int kPreviewWidth = 1024;
constexpr int kPreviewHeight = 640;
constexpr int kPanelWidth = 460;
constexpr int kWindowWidth = kPreviewWidth + kPanelWidth;
constexpr int kWindowHeight = kPreviewHeight;
constexpr int kPanelPadding = 14;
constexpr int kTrackHeight = 20;
constexpr int kSliderRowHeight = 54;
constexpr int kSectionGap = 10;
constexpr int kButtonHeight = 24;
constexpr int kButtonGap = 6;
constexpr int kButtonMinWidth = 74;

struct Options {
    std::size_t index = 0;
    std::string serial;
    unsigned int timeout_ms = 300;
    bool headless = false;
    std::string save_path = "output/adj_first_frame.png";
};

struct EnumControl {
    std::string label;
    std::string key;
    std::vector<std::string> options;
    int selected = 0;
    bool restart_required = false;
    std::vector<cv::Rect> option_rects;
};

struct SliderControl {
    std::string label;
    std::string key;
    double minimum = 0.0;
    double maximum = 1.0;
    double increment = 1.0;
    double committed = 0.0;
    double pending = 0.0;
    bool is_float = false;
    bool restart_required = false;
    bool enabled = true;
    bool dragging = false;
    cv::Rect row_rect;
    cv::Rect track_rect;
};

struct AppState {
    Options options;
    DeviceDescriptor device;
    MvsCamera camera;
    std::vector<EnumControl> enums;
    std::vector<SliderControl> sliders;
    FrameResult last_frame;
    bool has_frame = false;
    bool should_close = false;
    std::string last_status = "Ready";
    std::string window_name = "02-adj_visualization";
    std::string last_save_path;
    int panel_content_height = kWindowHeight;
    int panel_scroll_y = 0;
    int consecutive_frame_errors = 0;
    cv::Rect scrollbar_track_rect;
    cv::Rect scrollbar_thumb_rect;
    bool scrollbar_dragging = false;
    int scrollbar_drag_offset_y = 0;
};

Options parse_args(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const std::string& name) -> const char* {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value after " + name);
            }
            return argv[++i];
        };

        if (arg == "--index") {
            options.index = static_cast<std::size_t>(std::stoul(require_value(arg)));
        } else if (arg == "--serial") {
            options.serial = require_value(arg);
        } else if (arg == "--timeout-ms") {
            options.timeout_ms = static_cast<unsigned int>(std::stoul(require_value(arg)));
        } else if (arg == "--save") {
            options.save_path = require_value(arg);
        } else if (arg == "--headless") {
            options.headless = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: 02-adj_visualization [options]\n"
                << "  --index N\n"
                << "  --serial SN\n"
                << "  --timeout-ms VALUE\n"
                << "  --save PATH\n"
                << "  --headless\n";
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return options;
}

std::string format_double(double value, bool is_float) {
    std::ostringstream oss;
    if (is_float) {
        oss.setf(std::ios::fixed);
        oss.precision(2);
    } else {
        oss.precision(0);
    }
    oss << value;
    return oss.str();
}

cv::Mat fit_frame_to_window(const cv::Mat& frame_bgr) {
    cv::Mat canvas(kPreviewHeight, kPreviewWidth, CV_8UC3, cv::Scalar(10, 10, 10));
    if (frame_bgr.empty()) {
        return canvas;
    }

    const double scale_x = static_cast<double>(kPreviewWidth) / static_cast<double>(frame_bgr.cols);
    const double scale_y = static_cast<double>(kPreviewHeight) / static_cast<double>(frame_bgr.rows);
    const double scale = std::min(scale_x, scale_y);

    const int resized_width = std::max(1, static_cast<int>(frame_bgr.cols * scale));
    const int resized_height = std::max(1, static_cast<int>(frame_bgr.rows * scale));

    cv::Mat resized;
    cv::resize(frame_bgr, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_AREA);

    const int offset_x = (kPreviewWidth - resized_width) / 2;
    const int offset_y = (kPreviewHeight - resized_height) / 2;
    resized.copyTo(canvas(cv::Rect(offset_x, offset_y, resized_width, resized_height)));
    return canvas;
}

std::vector<std::string> collect_options(MvsCamera& camera, const std::string& key, const std::vector<std::string>& preferred_order) {
    std::vector<std::pair<unsigned int, std::string>> raw;
    try {
        raw = camera.get_supported_enum_entries(key);
    } catch (...) {
        return {};
    }

    std::vector<std::string> values;
    values.reserve(raw.size());
    for (const auto& item : raw) {
        if (!item.second.empty()) {
            values.push_back(item.second);
        }
    }

    if (preferred_order.empty()) {
        return values;
    }

    std::vector<std::string> ordered;
    for (const auto& wanted : preferred_order) {
        auto it = std::find(values.begin(), values.end(), wanted);
        if (it != values.end()) {
            ordered.push_back(*it);
        }
    }
    for (const auto& value : values) {
        if (std::find(ordered.begin(), ordered.end(), value) == ordered.end()) {
            ordered.push_back(value);
        }
    }
    return ordered;
}

int index_of_option(const std::vector<std::string>& options, const std::string& current_value) {
    auto it = std::find(options.begin(), options.end(), current_value);
    if (it == options.end()) {
        return 0;
    }
    return static_cast<int>(std::distance(options.begin(), it));
}

void reload_controls_from_camera(AppState& app) {
    app.enums.clear();
    app.sliders.clear();

    const auto add_enum = [&](const std::string& label,
                              const std::string& key,
                              const std::vector<std::string>& preferred,
                              bool restart_required) {
        EnumControl control;
        control.label = label;
        control.key = key;
        control.options = collect_options(app.camera, key, preferred);
        if (control.options.empty()) {
            return;
        }
        control.selected = index_of_option(control.options, app.camera.get_enum_symbolic(key));
        control.restart_required = restart_required;
        app.enums.push_back(std::move(control));
    };

    const auto add_int = [&](const std::string& label,
                             const std::string& key,
                             bool restart_required) {
        SliderControl control;
        control.label = label;
        control.key = key;
        IntRange range{};
        try {
            range = app.camera.get_int_range(key);
        } catch (...) {
            return;
        }
        control.minimum = static_cast<double>(range.minimum);
        control.maximum = static_cast<double>(range.maximum);
        control.increment = static_cast<double>(range.increment);
        control.committed = static_cast<double>(range.current);
        control.pending = control.committed;
        control.restart_required = restart_required;
        app.sliders.push_back(std::move(control));
    };

    const auto add_float = [&](const std::string& label,
                               const std::string& key) {
        SliderControl control;
        control.label = label;
        control.key = key;
        FloatRange range{};
        try {
            range = app.camera.get_float_range(key);
        } catch (...) {
            return;
        }
        control.minimum = static_cast<double>(range.minimum);
        control.maximum = static_cast<double>(range.maximum);
        if (key == "ExposureTime") {
            control.minimum = std::max(control.minimum, 1000.0);
            control.maximum = std::min(control.maximum, 5000000.0);
        }
        control.increment = 0.0;
        control.committed = std::clamp(static_cast<double>(range.current), control.minimum, control.maximum);
        control.pending = control.committed;
        control.is_float = true;
        app.sliders.push_back(std::move(control));
    };

    add_enum("PixelFormat", "PixelFormat", {"BayerGB8", "Mono8", "RGB8Packed"}, true);
    add_enum("AcquisitionMode", "AcquisitionMode", {"Continuous", "MultiFrame", "SingleFrame"}, true);
    add_enum("TriggerMode", "TriggerMode", {"Off", "On"}, false);
    add_enum("TriggerSource", "TriggerSource", {"Line0", "Line1", "Line2", "Line3", "Software"}, false);
    add_enum("TriggerActivation", "TriggerActivation", {"RisingEdge", "FallingEdge", "AnyEdge", "LevelHigh", "LevelLow"}, false);
    add_enum("ExposureAuto", "ExposureAuto", {"Off", "Once", "Continuous"}, false);
    add_enum("GainAuto", "GainAuto", {"Off", "Once", "Continuous"}, false);

    add_int("Width", "Width", true);
    add_int("Height", "Height", true);
    add_int("OffsetX", "OffsetX", true);
    add_int("OffsetY", "OffsetY", true);
    add_float("ExposureTime(us)", "ExposureTime");
    add_float("AutoExpLower(us)", "AutoExposureTimeLowerLimit");
    add_float("AutoExpUpper(us)", "AutoExposureTimeUpperLimit");
    add_int("AutoTargetBrightness", "AutoTargetBrightness", false);
    add_float("Gain", "Gain");

    auto exposure_auto = std::find_if(app.enums.begin(), app.enums.end(), [](const EnumControl& control) {
        return control.key == "ExposureAuto";
    });
    auto gain_auto = std::find_if(app.enums.begin(), app.enums.end(), [](const EnumControl& control) {
        return control.key == "GainAuto";
    });
    for (auto& slider : app.sliders) {
        if (slider.key == "ExposureTime" && exposure_auto != app.enums.end()) {
            slider.enabled = exposure_auto->options[exposure_auto->selected] == "Off";
        }
        if ((slider.key == "AutoExposureTimeLowerLimit" ||
             slider.key == "AutoExposureTimeUpperLimit" ||
             slider.key == "AutoTargetBrightness") &&
            exposure_auto != app.enums.end()) {
            slider.enabled = exposure_auto->options[exposure_auto->selected] != "Off";
        }
        if (slider.key == "Gain" && gain_auto != app.enums.end()) {
            slider.enabled = gain_auto->options[gain_auto->selected] == "Off";
        }
    }
}

void apply_enum_selection(AppState& app, EnumControl& control, int selected_index) {
    const std::string key = control.key;
    const std::string value = control.options[selected_index];
    const bool restart = control.restart_required && app.camera.is_grabbing();
    if (restart) {
        app.camera.stop_grabbing();
    }
    control.selected = selected_index;
    app.camera.set_enum(key, value);
    if (restart) {
        app.camera.start_grabbing();
    }
    reload_controls_from_camera(app);
    app.last_status = key + " = " + value;
}

double quantize_slider_value(const SliderControl& control, double value) {
    const double min_value = control.minimum;
    const double max_value = control.maximum;
    double clamped = std::max(min_value, std::min(max_value, value));
    if (!control.is_float) {
        const double steps = std::round((clamped - min_value) / std::max(1.0, control.increment));
        clamped = min_value + steps * std::max(1.0, control.increment);
    }
    return std::max(min_value, std::min(max_value, clamped));
}

void apply_slider_value(AppState& app, SliderControl& control) {
    if (!control.enabled) {
        return;
    }
    const std::string key = control.key;
    const bool is_float = control.is_float;
    const double value = control.pending;
    const bool restart = control.restart_required && app.camera.is_grabbing();
    if (restart) {
        app.camera.stop_grabbing();
    }
    if (is_float) {
        app.camera.set_float(key, static_cast<float>(value));
    } else {
        app.camera.set_int(key, static_cast<std::int64_t>(value));
    }
    if (restart) {
        app.camera.start_grabbing();
    }
    reload_controls_from_camera(app);
    app.last_status = key + " = " + format_double(value, is_float);
}

int draw_enum_control(cv::Mat& panel, int y, EnumControl& control) {
    cv::putText(panel, control.label, cv::Point(kPanelPadding, y + 14), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
    y += 22;
    control.option_rects.clear();

    int x = kPanelPadding;
    int row_top = y;
    for (std::size_t i = 0; i < control.options.size(); ++i) {
        const int text_width = static_cast<int>(control.options[i].size()) * 8 + 16;
        const int button_width = std::max(kButtonMinWidth, text_width);
        if (x + button_width > kPanelWidth - kPanelPadding) {
            x = kPanelPadding;
            row_top += kButtonHeight + kButtonGap;
        }
        const cv::Rect rect(x, row_top, button_width, kButtonHeight);
        control.option_rects.push_back(rect);

        const bool selected = static_cast<int>(i) == control.selected;
        const cv::Scalar fill = selected ? cv::Scalar(40, 120, 220) : cv::Scalar(56, 56, 56);
        cv::rectangle(panel, rect, fill, cv::FILLED);
        cv::rectangle(panel, rect, cv::Scalar(100, 100, 100), 1);
        cv::putText(panel, control.options[i], cv::Point(rect.x + 8, rect.y + 16), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
        x += button_width + kButtonGap;
    }

    return row_top + kButtonHeight + kSectionGap;
}

int draw_slider_control(cv::Mat& panel, int y, SliderControl& control) {
    control.row_rect = cv::Rect(0, y, kPanelWidth, kSliderRowHeight);
    const cv::Scalar title_color = control.enabled ? cv::Scalar(230, 230, 230) : cv::Scalar(120, 120, 120);
    const cv::Scalar value_color = control.enabled ? cv::Scalar(200, 220, 255) : cv::Scalar(120, 120, 120);
    cv::putText(panel, control.label, cv::Point(kPanelPadding, y + 14), cv::FONT_HERSHEY_SIMPLEX, 0.48, title_color, 1, cv::LINE_AA);
    cv::putText(panel, format_double(control.pending, control.is_float), cv::Point(kPanelWidth - 130, y + 14), cv::FONT_HERSHEY_SIMPLEX, 0.46, value_color, 1, cv::LINE_AA);

    const int track_y = y + 24;
    control.track_rect = cv::Rect(kPanelPadding, track_y, kPanelWidth - 2 * kPanelPadding, kTrackHeight);
    cv::rectangle(panel, control.track_rect, cv::Scalar(50, 50, 50), cv::FILLED);

    const double denom = std::max(1e-6, control.maximum - control.minimum);
    const double ratio = std::clamp((control.pending - control.minimum) / denom, 0.0, 1.0);
    const int knob_x = control.track_rect.x + static_cast<int>(ratio * control.track_rect.width);
    const cv::Scalar accent = control.enabled ? cv::Scalar(60, 170, 255) : cv::Scalar(90, 90, 90);
    cv::rectangle(panel, cv::Rect(control.track_rect.x, control.track_rect.y, std::max(1, knob_x - control.track_rect.x), control.track_rect.height), accent, cv::FILLED);
    cv::circle(panel, cv::Point(knob_x, control.track_rect.y + control.track_rect.height / 2), 8, cv::Scalar(240, 240, 240), cv::FILLED);
    cv::rectangle(panel, control.track_rect, cv::Scalar(90, 90, 90), 1);

    return y + kSliderRowHeight;
}

cv::Mat render_sidebar(AppState& app) {
    cv::Mat panel(1600, kPanelWidth, CV_8UC3, cv::Scalar(28, 28, 28));
    int y = kPanelPadding + 4;

    cv::putText(panel, "02-adj_visualization", cv::Point(kPanelPadding, y), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    y += 22;
    cv::putText(panel, "Preview 1024x640 + live control panel", cv::Point(kPanelPadding, y), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(170, 170, 170), 1, cv::LINE_AA);
    y += 20;
    cv::putText(panel, "Camera: " + app.device.model_name, cv::Point(kPanelPadding, y), cv::FONT_HERSHEY_SIMPLEX, 0.44, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    y += 18;
    cv::putText(panel, "SN: " + app.device.serial_number, cv::Point(kPanelPadding, y), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
    y += 20;
    cv::putText(panel, "s: save frame   r: reload   q: quit", cv::Point(kPanelPadding, y), cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(150, 190, 255), 1, cv::LINE_AA);
    y += 16;
    cv::putText(panel, "wheel: scroll panel", cv::Point(kPanelPadding, y), cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(150, 190, 255), 1, cv::LINE_AA);
    y += 16;
    cv::putText(panel, app.last_status, cv::Point(kPanelPadding, y), cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(170, 220, 170), 1, cv::LINE_AA);
    y += 16;

    cv::line(panel, cv::Point(kPanelPadding, y), cv::Point(kPanelWidth - kPanelPadding, y), cv::Scalar(70, 70, 70), 1);
    y += 12;

    cv::putText(panel, "Enums", cv::Point(kPanelPadding, y), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 215, 120), 1, cv::LINE_AA);
    y += 10;
    for (auto& control : app.enums) {
        y = draw_enum_control(panel, y, control);
    }

    y += 2;
    cv::line(panel, cv::Point(kPanelPadding, y), cv::Point(kPanelWidth - kPanelPadding, y), cv::Scalar(70, 70, 70), 1);
    y += 14;
    cv::putText(panel, "Numeric", cv::Point(kPanelPadding, y), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 215, 120), 1, cv::LINE_AA);
    y += 8;
    for (auto& control : app.sliders) {
        y = draw_slider_control(panel, y, control);
    }

    app.panel_content_height = std::max(y + kPanelPadding, kWindowHeight);
    app.panel_scroll_y = std::clamp(app.panel_scroll_y, 0, std::max(0, app.panel_content_height - kWindowHeight));

    cv::Mat visible = panel(cv::Rect(0, app.panel_scroll_y, kPanelWidth, kWindowHeight)).clone();
    app.scrollbar_track_rect = cv::Rect();
    app.scrollbar_thumb_rect = cv::Rect();
    if (app.panel_content_height > kWindowHeight) {
        const int track_x = kPanelWidth - 10;
        app.scrollbar_track_rect = cv::Rect(track_x - 4, 0, 8, kWindowHeight);
        cv::rectangle(visible, app.scrollbar_track_rect, cv::Scalar(55, 55, 55), cv::FILLED);

        const double ratio = static_cast<double>(kWindowHeight) / static_cast<double>(app.panel_content_height);
        const int thumb_height = std::max(40, static_cast<int>(kWindowHeight * ratio));
        const int max_scroll = std::max(1, app.panel_content_height - kWindowHeight);
        const double thumb_ratio = static_cast<double>(app.panel_scroll_y) / static_cast<double>(max_scroll);
        const int thumb_top = static_cast<int>((kWindowHeight - thumb_height) * thumb_ratio);
        app.scrollbar_thumb_rect = cv::Rect(track_x - 3, thumb_top, 6, thumb_height);
        cv::rectangle(visible, app.scrollbar_thumb_rect, cv::Scalar(140, 140, 140), cv::FILLED);
    }

    return visible;
}

void refresh_frame(AppState& app) {
    app.last_frame = app.camera.grab_frame_bgr(app.options.timeout_ms);
    app.has_frame = true;
    app.consecutive_frame_errors = 0;
}

bool trigger_mode_is_off(const AppState& app) {
    auto it = std::find_if(app.enums.begin(), app.enums.end(), [](const EnumControl& control) {
        return control.key == "TriggerMode";
    });
    if (it == app.enums.end() || it->options.empty()) {
        return true;
    }
    return it->options[it->selected] == "Off";
}

void restart_preview_stream(AppState& app) {
    if (app.camera.is_grabbing()) {
        app.camera.stop_grabbing();
    }
    app.camera.configure_continuous_output();
    reload_controls_from_camera(app);
    app.camera.start_grabbing();
    try {
        app.camera.set_command("AcquisitionStart");
    } catch (...) {
    }
    app.last_status = "Preview stream restarted";
}

void update_slider_pending_from_point(SliderControl& slider, const cv::Point& point) {
    const double ratio = std::clamp(
        static_cast<double>(point.x - slider.track_rect.x) / static_cast<double>(slider.track_rect.width),
        0.0,
        1.0);
    slider.pending = quantize_slider_value(slider, slider.minimum + ratio * (slider.maximum - slider.minimum));
}

void set_scroll_from_thumb_top(AppState& app, int thumb_top) {
    if (app.scrollbar_thumb_rect.height <= 0 || app.scrollbar_track_rect.height <= 0) {
        return;
    }
    const int max_thumb_top = std::max(0, app.scrollbar_track_rect.height - app.scrollbar_thumb_rect.height);
    const int clamped_thumb_top = std::clamp(thumb_top, 0, max_thumb_top);
    const int max_scroll = std::max(0, app.panel_content_height - kWindowHeight);
    if (max_thumb_top == 0 || max_scroll == 0) {
        app.panel_scroll_y = 0;
        return;
    }
    const double ratio = static_cast<double>(clamped_thumb_top) / static_cast<double>(max_thumb_top);
    app.panel_scroll_y = static_cast<int>(std::round(ratio * static_cast<double>(max_scroll)));
}

void save_current_frame(AppState& app) {
    if (!app.has_frame) {
        refresh_frame(app);
    }
    app.camera.save_frame(app.last_frame.bgr, app.options.save_path);
    app.last_save_path = app.options.save_path;
    app.last_status = "Saved frame to " + app.last_save_path;
}

void mouse_callback(int event, int x, int y, int flags, void* userdata) {
    auto& app = *static_cast<AppState*>(userdata);
    if (x < kPreviewWidth) {
        if (event == cv::EVENT_LBUTTONUP) {
            for (auto& slider : app.sliders) {
                slider.dragging = false;
            }
        }
        return;
    }

    const int local_x = x - kPreviewWidth;
    const cv::Point visible_point(local_x, y);
    const cv::Point point(local_x, y + app.panel_scroll_y);

    if (event == cv::EVENT_LBUTTONDOWN) {
        if (app.scrollbar_thumb_rect.contains(visible_point)) {
            app.scrollbar_dragging = true;
            app.scrollbar_drag_offset_y = visible_point.y - app.scrollbar_thumb_rect.y;
            return;
        }
        if (app.scrollbar_track_rect.contains(visible_point)) {
            set_scroll_from_thumb_top(app, visible_point.y - app.scrollbar_thumb_rect.height / 2);
            return;
        }

        for (auto& control : app.enums) {
            for (std::size_t i = 0; i < control.option_rects.size(); ++i) {
                if (control.option_rects[i].contains(point)) {
                    try {
                        apply_enum_selection(app, control, static_cast<int>(i));
                    } catch (const std::exception& error) {
                        app.last_status = error.what();
                    }
                    return;
                }
            }
        }

        for (auto& slider : app.sliders) {
            if (slider.enabled && slider.row_rect.contains(point)) {
                slider.dragging = true;
                update_slider_pending_from_point(slider, point);
                return;
            }
            if (!slider.enabled && slider.row_rect.contains(point)) {
                if (slider.key == "ExposureTime") {
                    app.last_status = "ExposureTime disabled while ExposureAuto != Off";
                } else if (slider.key == "Gain") {
                    app.last_status = "Gain disabled while GainAuto != Off";
                }
                return;
            }
        }
    } else if (event == cv::EVENT_MOUSEWHEEL) {
        const int wheel_delta = cv::getMouseWheelDelta(flags);
        if (wheel_delta != 0) {
            app.panel_scroll_y = std::clamp(
                app.panel_scroll_y - (wheel_delta > 0 ? 50 : -50),
                0,
                std::max(0, app.panel_content_height - kWindowHeight));
        }
    } else if (event == cv::EVENT_MOUSEMOVE) {
        if (app.scrollbar_dragging) {
            set_scroll_from_thumb_top(app, visible_point.y - app.scrollbar_drag_offset_y);
            return;
        }
        for (auto& slider : app.sliders) {
            if (slider.dragging) {
                update_slider_pending_from_point(slider, point);
                return;
            }
        }
    } else if (event == cv::EVENT_LBUTTONUP) {
        app.scrollbar_dragging = false;
        for (auto& slider : app.sliders) {
            if (slider.dragging) {
                slider.dragging = false;
                try {
                    apply_slider_value(app, slider);
                } catch (const std::exception& error) {
                    app.last_status = error.what();
                }
                return;
            }
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        AppState app;
        app.options = parse_args(argc, argv);

        std::cout << "MVS SDK version: " << MvsCamera::sdk_version() << '\n';
        const auto devices = MvsCamera::enumerate_devices();
        if (devices.empty()) {
            std::cout << "No Hikrobot camera found.\n";
            return 1;
        }

        std::cout << "Detected cameras:\n";
        for (const auto& device : devices) {
            std::cout << "  [" << device.index
                      << "] transport=" << device.transport
                      << " serial=" << (device.serial_number.empty() ? "-" : device.serial_number)
                      << " model=" << (device.model_name.empty() ? "-" : device.model_name)
                      << " user_name=" << (device.user_defined_name.empty() ? "-" : device.user_defined_name)
                      << '\n';
        }

        if (!app.options.serial.empty()) {
            auto it = std::find_if(devices.begin(), devices.end(), [&](const DeviceDescriptor& device) {
                return device.serial_number == app.options.serial;
            });
            if (it != devices.end()) {
                app.device = *it;
            }
        } else if (app.options.index < devices.size()) {
            app.device = devices[app.options.index];
        } else {
            throw std::runtime_error("Camera index is out of range.");
        }

        app.camera.open(app.options.index, app.options.serial);
        app.camera.configure_continuous_output();
        reload_controls_from_camera(app);
        app.last_status = "Startup: AcquisitionMode=Continuous, TriggerMode=Off";
        app.camera.start_grabbing();
        try {
            app.camera.set_command("AcquisitionStart");
        } catch (...) {
        }
        refresh_frame(app);

        if (app.options.headless) {
            save_current_frame(app);
            std::cout << app.last_status << '\n';
            app.camera.close();
            return 0;
        }

        cv::namedWindow(app.window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(app.window_name, kWindowWidth, kWindowHeight);
        cv::setMouseCallback(app.window_name, mouse_callback, &app);

        while (!app.should_close) {
            try {
                refresh_frame(app);
            } catch (const std::exception& error) {
                ++app.consecutive_frame_errors;
                app.last_status = error.what();
                if (trigger_mode_is_off(app) && app.consecutive_frame_errors >= 3) {
                    try {
                        restart_preview_stream(app);
                    } catch (const std::exception& restart_error) {
                        app.last_status = restart_error.what();
                    }
                }
            }

            cv::Mat composed(kWindowHeight, kWindowWidth, CV_8UC3, cv::Scalar(24, 24, 24));
            fit_frame_to_window(app.last_frame.bgr).copyTo(composed(cv::Rect(0, 0, kPreviewWidth, kPreviewHeight)));
            render_sidebar(app).copyTo(composed(cv::Rect(kPreviewWidth, 0, kPanelWidth, kWindowHeight)));
            cv::imshow(app.window_name, composed);

            const int key = cv::waitKey(15) & 0xFF;
            if (key == 'q') {
                app.should_close = true;
            } else if (key == 's') {
                try {
                    save_current_frame(app);
                } catch (const std::exception& error) {
                    app.last_status = error.what();
                }
            } else if (key == 'r') {
                try {
                    reload_controls_from_camera(app);
                    app.last_status = "Reloaded settings from camera";
                } catch (const std::exception& error) {
                    app.last_status = error.what();
                }
            }
        }

        app.camera.close();
        cv::destroyAllWindows();
        return 0;
    } catch (const MvsError& error) {
        std::cerr << "MVS error: " << error.what() << '\n';
        return 2;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 3;
    }
}
