#include "luxitech/mvs/mvs_camera.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

using luxitech::mvs::MvsCamera;
using luxitech::mvs::MvsError;

namespace {

constexpr int kPreviewWindowWidth = 1024;
constexpr int kPreviewWindowHeight = 640;

struct Options {
    std::size_t index = 0;
    std::string serial;
    std::int64_t width = -1;
    std::int64_t height = -1;
    std::string exposure_auto;
    float exposure_us = -1.0f;
    std::string gain_auto;
    float gain = -1.0f;
    unsigned int timeout_ms = 1000;
    bool headless = false;
    std::string save_path = "output/first_frame.png";
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
        } else if (arg == "--width") {
            options.width = std::stoll(require_value(arg));
        } else if (arg == "--height") {
            options.height = std::stoll(require_value(arg));
        } else if (arg == "--exposure-auto") {
            options.exposure_auto = require_value(arg);
        } else if (arg == "--exposure-us") {
            options.exposure_us = std::stof(require_value(arg));
        } else if (arg == "--gain-auto") {
            options.gain_auto = require_value(arg);
        } else if (arg == "--gain") {
            options.gain = std::stof(require_value(arg));
        } else if (arg == "--timeout-ms") {
            options.timeout_ms = static_cast<unsigned int>(std::stoul(require_value(arg)));
        } else if (arg == "--save") {
            options.save_path = require_value(arg);
        } else if (arg == "--headless") {
            options.headless = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: 01-mini_visualization [options]\n"
                << "  --index N\n"
                << "  --serial SN\n"
                << "  --width W --height H\n"
                << "  --exposure-auto Off|Once|Continuous\n"
                << "  --exposure-us VALUE\n"
                << "  --gain-auto Off|Once|Continuous\n"
                << "  --gain VALUE\n"
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

cv::Mat fit_frame_to_window(const cv::Mat& frame_bgr) {
    cv::Mat canvas(kPreviewWindowHeight, kPreviewWindowWidth, frame_bgr.type(), cv::Scalar::all(0));
    if (frame_bgr.empty()) {
        return canvas;
    }

    const double scale_x = static_cast<double>(kPreviewWindowWidth) / static_cast<double>(frame_bgr.cols);
    const double scale_y = static_cast<double>(kPreviewWindowHeight) / static_cast<double>(frame_bgr.rows);
    const double scale = std::min(scale_x, scale_y);

    const int resized_width = std::max(1, static_cast<int>(frame_bgr.cols * scale));
    const int resized_height = std::max(1, static_cast<int>(frame_bgr.rows * scale));

    cv::Mat resized;
    cv::resize(frame_bgr, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_AREA);

    const int offset_x = (kPreviewWindowWidth - resized_width) / 2;
    const int offset_y = (kPreviewWindowHeight - resized_height) / 2;
    resized.copyTo(canvas(cv::Rect(offset_x, offset_y, resized_width, resized_height)));
    return canvas;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_args(argc, argv);

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

        MvsCamera camera;
        camera.open(options.index, options.serial);
        camera.configure_continuous_output();

        if (options.width > 0) {
            camera.set_int("Width", options.width);
        }
        if (options.height > 0) {
            camera.set_int("Height", options.height);
        }
        if (!options.exposure_auto.empty()) {
            camera.set_auto_exposure(options.exposure_auto);
        }
        if (options.exposure_us > 0.0f) {
            camera.set_manual_exposure(options.exposure_us);
        }
        if (!options.gain_auto.empty()) {
            camera.set_gain_auto(options.gain_auto);
        }
        if (options.gain >= 0.0f) {
            camera.set_manual_gain(options.gain);
        }

        camera.start_grabbing();
        if (options.headless) {
            std::cout << "Camera configured to continuous output. Running in headless mode.\n";
        } else {
            std::cout << "Camera configured to continuous output. Press q in the preview window to exit.\n";
        }

        bool first_saved = false;
        const std::string window_name = "Hikrobot Preview";
        if (!options.headless) {
            cv::namedWindow(window_name, cv::WINDOW_NORMAL);
            cv::resizeWindow(window_name, kPreviewWindowWidth, kPreviewWindowHeight);
        }
        while (true) {
            const auto frame = camera.grab_frame_bgr(options.timeout_ms);
            if (!first_saved) {
                camera.save_frame(frame.bgr, options.save_path);
                std::cout << "Saved first frame to: " << options.save_path << '\n';
                first_saved = true;
            }

            if (options.headless) {
                break;
            }

            cv::imshow(window_name, fit_frame_to_window(frame.bgr));
            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q') {
                break;
            }
        }

        camera.close();
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
