// Copyright 2025-2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// M20 native camera driver for DimOS.

#include <lcm/lcm-cpp.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include "dimos_native_module.hpp"

#include "sensor_msgs/CameraInfo.hpp"
#include "sensor_msgs/Image.hpp"

static std::atomic<bool> g_running{true};

static void signal_handler(int) {
    g_running.store(false);
}

static void publish_black_image(
    lcm::LCM& lcm,
    const std::string& topic,
    const std::string& frame_id
) {
    constexpr int width = 640;
    constexpr int height = 480;
    constexpr int channels = 3;

    sensor_msgs::Image msg;
    msg.header = dimos::make_header(frame_id, 0.0);
    msg.height = height;
    msg.width = width;
    msg.encoding = "rgb8";
    msg.is_bigendian = 0;
    msg.step = width * channels;
    msg.data_length = height * msg.step;
    msg.data.resize(msg.data_length, 0);

    lcm.publish(topic, &msg);
}

static void publish_camera_info(
    lcm::LCM& lcm,
    const std::string& topic,
    const std::string& frame_id
) {
    sensor_msgs::CameraInfo msg;
    msg.header = dimos::make_header(frame_id, 0.0);
    msg.height = 480;
    msg.width = 640;
    msg.distortion_model = "plumb_bob";

    //todo: Replace these placeholder intrinsics with calibrated M20 values.
    msg.k[0] = 1.0;
    msg.k[4] = 1.0;
    msg.k[8] = 1.0;
    msg.r[0] = 1.0;
    msg.r[4] = 1.0;
    msg.r[8] = 1.0;
    msg.p[0] = 1.0;
    msg.p[5] = 1.0;
    msg.p[10] = 1.0;

    lcm.publish(topic, &msg);
}

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    const std::string image_topic = mod.topic("color_image");
    const std::string camera_info_topic = mod.topic("camera_info");
    const std::string ip = mod.arg("ip", "192.168.1.20");
    const float camera_hz = mod.arg_float("camera_hz", 30.0f);
    const float camera_info_hz = mod.arg_float("camera_info_hz", 1.0f);
    const std::string camera_frame_id = mod.arg("camera_frame_id", "camera_optical");

    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    lcm::LCM lcm;
    if (!lcm.good()) {
        std::fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }

    std::printf("[m20-camera] Starting native camera driver\n");
    std::printf("[m20-camera] ip: %s\n", ip.c_str());
    std::printf("[m20-camera] color_image topic: %s\n", image_topic.c_str());
    std::printf("[m20-camera] camera_info topic: %s\n", camera_info_topic.c_str());

    //todo: Initialize the real M20 camera C++ SDK here.
    //todo: Register callbacks or blocking reads for raw M20 camera frames here.

    auto last_camera = std::chrono::steady_clock::now();
    auto last_camera_info = std::chrono::steady_clock::now();
    const auto camera_period = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / std::max(camera_hz, 1.0f))
    );
    const auto camera_info_period = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / std::max(camera_info_hz, 0.1f))
    );

    while (g_running.load()) {
        lcm.handleTimeout(5);
        const auto now = std::chrono::steady_clock::now();

        if (now - last_camera >= camera_period) {
            //todo: Convert real M20 camera frames to sensor_msgs::Image.
            publish_black_image(lcm, image_topic, camera_frame_id);
            last_camera = now;
        }

        if (now - last_camera_info >= camera_info_period) {
            //todo: Publish real M20 camera intrinsics.
            publish_camera_info(lcm, camera_info_topic, camera_frame_id);
            last_camera_info = now;
        }
    }

    //todo: Stop and release the real M20 camera SDK here.
    std::printf("[m20-camera] Shutting down\n");
    return 0;
}
