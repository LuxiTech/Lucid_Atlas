#include "luxitech/mvs/mvs_camera.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using luxitech::mvs::DeviceDescriptor;
using luxitech::mvs::FloatRange;
using luxitech::mvs::FrameResult;
using luxitech::mvs::IntRange;
using luxitech::mvs::MvsCamera;
using luxitech::mvs::MvsError;

namespace {

constexpr int kPreviewWidth = 1280;
constexpr int kPreviewHeight = 1024;
constexpr int kPanelWidth = 0;
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

struct CommandQueue {
    std::mutex mutex;
    std::queue<std::string> lines;
    std::atomic<bool> done{false};
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
        oss << value;
    } else {
        oss << static_cast<long long>(std::llround(value));
    }
    return oss.str();
}

cv::Mat fit_frame_to_window(const cv::Mat& frame_bgr) {
    cv::Mat canvas(kPreviewHeight, kPreviewWidth, CV_8UC3, cv::Scalar(10, 10, 10));
    if (frame_bgr.empty()) {
        return canvas;
    }

    const int copy_width = std::min(kPreviewWidth, frame_bgr.cols);
    const int copy_height = std::min(kPreviewHeight, frame_bgr.rows);
    frame_bgr(cv::Rect(0, 0, copy_width, copy_height))
        .copyTo(canvas(cv::Rect(0, 0, copy_width, copy_height)));
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

void print_help() {
    std::cout
        << "\nCommands:\n"
        << "  help\n"
        << "  list\n"
        << "  set <Key> <Value>\n"
        << "  save [path]\n"
        << "  reload\n"
        << "  quit\n"
        << "\nExamples:\n"
        << "  set ExposureAuto Off\n"
        << "  set ExposureTime 8000\n"
        << "  set GainAuto Off\n"
        << "  set GainAuto On       # alias of Continuous\n"
        << "  set Gain 0\n"
        << "  set TriggerMode Off\n"
        << "  set PixelFormat BayerGB8\n"
        << std::endl;
}

void print_key_help() {
    std::cout
        << "\nOpenCV keys:\n"
        << "  [ / ]  ExposureTime -/+ 1000 us\n"
        << "  - / =  Gain -/+ 0.5\n"
        << "  a      toggle ExposureAuto Off/Continuous\n"
        << "  g      toggle GainAuto Off/Continuous\n"
        << "  p      cycle PixelFormat\n"
        << "  l      list current settings\n"
        << "  h      show this help\n"
        << "  s      save current frame\n"
        << "  r      reload settings\n"
        << "  q      quit\n"
        << std::endl;
}

void print_controls(const AppState& app) {
    std::cout << "\nEnums:\n";
    for (const auto& control : app.enums) {
        std::cout << "  " << control.key << " = ";
        if (!control.options.empty() && control.selected >= 0 && control.selected < static_cast<int>(control.options.size())) {
            std::cout << control.options[control.selected];
        } else {
            std::cout << "-";
        }
        std::cout << "  [";
        for (std::size_t i = 0; i < control.options.size(); ++i) {
            if (i > 0) {
                std::cout << ", ";
            }
            std::cout << control.options[i];
        }
        std::cout << "]\n";
    }

    std::cout << "Numeric:\n";
    for (const auto& control : app.sliders) {
        std::cout << "  " << control.key << " = " << format_double(control.committed, control.is_float)
                  << "  range=[" << format_double(control.minimum, control.is_float)
                  << ", " << format_double(control.maximum, control.is_float) << "]";
        if (!control.is_float) {
            std::cout << " inc=" << format_double(control.increment, false);
        }
        if (!control.enabled) {
            std::cout << " disabled";
        }
        std::cout << '\n';
    }
    std::cout << std::endl;
}

EnumControl* find_enum(AppState& app, const std::string& key) {
    auto it = std::find_if(app.enums.begin(), app.enums.end(), [&](const EnumControl& control) {
        return control.key == key || control.label == key;
    });
    return it == app.enums.end() ? nullptr : &(*it);
}

SliderControl* find_slider(AppState& app, const std::string& key) {
    auto it = std::find_if(app.sliders.begin(), app.sliders.end(), [&](const SliderControl& control) {
        return control.key == key || control.label == key;
    });
    return it == app.sliders.end() ? nullptr : &(*it);
}

std::string to_lower_ascii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

std::string normalize_enum_value(const std::string& key, const std::string& value) {
    const std::string lower = to_lower_ascii(value);
    if ((key == "ExposureAuto" || key == "GainAuto") &&
        (lower == "on" || lower == "true" || lower == "1")) {
        return "Continuous";
    }
    if ((key == "ExposureAuto" || key == "GainAuto") &&
        (lower == "off" || lower == "false" || lower == "0")) {
        return "Off";
    }
    return value;
}

void apply_set_command(AppState& app, const std::string& key, const std::string& value) {
    if (auto* control = find_enum(app, key)) {
        const std::string normalized_value = normalize_enum_value(control->key, value);
        const std::string normalized_lower = to_lower_ascii(normalized_value);
        auto it = std::find_if(control->options.begin(), control->options.end(), [&](const std::string& option) {
            return to_lower_ascii(option) == normalized_lower;
        });
        if (it == control->options.end()) {
            throw std::runtime_error("Unsupported enum value for " + key + ": " + value);
        }
        apply_enum_selection(app, *control, static_cast<int>(std::distance(control->options.begin(), it)));
        std::cout << app.last_status << std::endl;
        return;
    }

    if (auto* control = find_slider(app, key)) {
        control->pending = quantize_slider_value(*control, std::stod(value));
        apply_slider_value(app, *control);
        std::cout << app.last_status << std::endl;
        return;
    }

    throw std::runtime_error("Unknown parameter key: " + key);
}

void apply_numeric_delta(AppState& app, const std::string& key, double delta) {
    auto* control = find_slider(app, key);
    if (control == nullptr) {
        throw std::runtime_error("Unknown numeric parameter: " + key);
    }
    control->pending = quantize_slider_value(*control, control->committed + delta);
    apply_slider_value(app, *control);
    std::cout << app.last_status << std::endl;
}

void toggle_enum_between(AppState& app, const std::string& key, const std::string& first, const std::string& second) {
    auto* control = find_enum(app, key);
    if (control == nullptr || control->options.empty()) {
        throw std::runtime_error("Unknown enum parameter: " + key);
    }

    const std::string current =
        control->selected >= 0 && control->selected < static_cast<int>(control->options.size())
            ? control->options[control->selected]
            : "";
    const std::string target = current == first ? second : first;
    auto it = std::find(control->options.begin(), control->options.end(), target);
    if (it == control->options.end()) {
        throw std::runtime_error("Unsupported enum value for " + key + ": " + target);
    }

    apply_enum_selection(app, *control, static_cast<int>(std::distance(control->options.begin(), it)));
    std::cout << app.last_status << std::endl;
}

void cycle_enum(AppState& app, const std::string& key, int direction) {
    auto* control = find_enum(app, key);
    if (control == nullptr || control->options.empty()) {
        throw std::runtime_error("Unknown enum parameter: " + key);
    }

    const int count = static_cast<int>(control->options.size());
    const int next = (control->selected + direction + count) % count;
    apply_enum_selection(app, *control, next);
    std::cout << app.last_status << std::endl;
}

void process_command(AppState& app, const std::string& line) {
    std::istringstream iss(line);
    std::string command;
    iss >> command;
    if (command.empty()) {
        return;
    }

    if (command == "help" || command == "h" || command == "?") {
        print_help();
    } else if (command == "list" || command == "ls") {
        reload_controls_from_camera(app);
        print_controls(app);
    } else if (command == "set") {
        std::string key;
        std::string value;
        iss >> key >> value;
        if (key.empty() || value.empty()) {
            throw std::runtime_error("Usage: set <Key> <Value>");
        }
        apply_set_command(app, key, value);
    } else if (command == "save" || command == "s") {
        std::string path;
        iss >> path;
        if (!path.empty()) {
            app.options.save_path = path;
        }
        save_current_frame(app);
        std::cout << app.last_status << std::endl;
    } else if (command == "reload" || command == "r") {
        reload_controls_from_camera(app);
        app.last_status = "Reloaded settings from camera";
        std::cout << app.last_status << std::endl;
    } else if (command == "quit" || command == "q" || command == "exit") {
        app.should_close = true;
    } else if (command == "a") {
        toggle_enum_between(app, "ExposureAuto", "Off", "Continuous");
    } else if (command == "g") {
        toggle_enum_between(app, "GainAuto", "Off", "Continuous");
    } else if (command == "p") {
        cycle_enum(app, "PixelFormat", 1);
    } else if (command == "[") {
        apply_set_command(app, "ExposureAuto", "Off");
        apply_numeric_delta(app, "ExposureTime", -1000.0);
    } else if (command == "]") {
        apply_set_command(app, "ExposureAuto", "Off");
        apply_numeric_delta(app, "ExposureTime", 1000.0);
    } else if (command == "-") {
        apply_set_command(app, "GainAuto", "Off");
        apply_numeric_delta(app, "Gain", -0.5);
    } else if (command == "=" || command == "+") {
        apply_set_command(app, "GainAuto", "Off");
        apply_numeric_delta(app, "Gain", 0.5);
    } else {
        throw std::runtime_error("Unknown command: " + command);
    }
}

void stdin_reader(CommandQueue& commands) {
    std::string line;
    while (std::getline(std::cin, line)) {
        {
            std::lock_guard<std::mutex> lock(commands.mutex);
            commands.lines.push(line);
        }
        if (line == "quit" || line == "q" || line == "exit") {
            break;
        }
    }
    commands.done = true;
}

void drain_commands(AppState& app, CommandQueue& commands) {
    while (true) {
        std::string line;
        {
            std::lock_guard<std::mutex> lock(commands.mutex);
            if (commands.lines.empty()) {
                break;
            }
            line = std::move(commands.lines.front());
            commands.lines.pop();
        }
        try {
            process_command(app, line);
        } catch (const std::exception& error) {
            app.last_status = error.what();
            std::cerr << "Command error: " << error.what() << std::endl;
        }
    }
}

void handle_key(AppState& app, int key) {
    try {
        if (key == 'q') {
            app.should_close = true;
        } else if (key == 's') {
            save_current_frame(app);
            std::cout << app.last_status << std::endl;
        } else if (key == 'r') {
            reload_controls_from_camera(app);
            app.last_status = "Reloaded settings from camera";
            std::cout << app.last_status << std::endl;
        } else if (key == '[') {
            apply_set_command(app, "ExposureAuto", "Off");
            apply_numeric_delta(app, "ExposureTime", -1000.0);
        } else if (key == ']') {
            apply_set_command(app, "ExposureAuto", "Off");
            apply_numeric_delta(app, "ExposureTime", 1000.0);
        } else if (key == '-') {
            apply_set_command(app, "GainAuto", "Off");
            apply_numeric_delta(app, "Gain", -0.5);
        } else if (key == '=' || key == '+') {
            apply_set_command(app, "GainAuto", "Off");
            apply_numeric_delta(app, "Gain", 0.5);
        } else if (key == 'a') {
            toggle_enum_between(app, "ExposureAuto", "Off", "Continuous");
        } else if (key == 'g') {
            toggle_enum_between(app, "GainAuto", "Off", "Continuous");
        } else if (key == 'p') {
            cycle_enum(app, "PixelFormat", 1);
        } else if (key == 'l') {
            reload_controls_from_camera(app);
            print_controls(app);
        } else if (key == 'h' || key == '?') {
            print_key_help();
        }
    } catch (const std::exception& error) {
        app.last_status = error.what();
        std::cerr << "Key error: " << error.what() << std::endl;
    }
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
        print_help();
        print_key_help();
        print_controls(app);

        CommandQueue commands;
        std::thread input_thread(stdin_reader, std::ref(commands));

        while (!app.should_close) {
            drain_commands(app, commands);

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

            cv::imshow(app.window_name, fit_frame_to_window(app.last_frame.bgr));

            const int key = cv::waitKey(15);
            if (key >= 0) {
                handle_key(app, key & 0xFF);
            }
        }

        commands.done = true;
        if (input_thread.joinable()) {
            if (std::cin.eof()) {
                input_thread.join();
            } else {
                input_thread.detach();
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
