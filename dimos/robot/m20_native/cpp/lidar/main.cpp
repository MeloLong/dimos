// Copyright 2025-2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// M20 native lidar driver for DimOS.

#include <lcm/lcm-cpp.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include "dimos_native_module.hpp"

#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

static std::atomic<bool> g_running{true};

static void signal_handler(int) {
    g_running.store(false);
}

static sensor_msgs::PointField make_field(const std::string& name, int32_t offset) {
    sensor_msgs::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    return field;
}

static void publish_empty_pointcloud(
    lcm::LCM& lcm,
    const std::string& topic,
    const std::string& frame_id
) {
    sensor_msgs::PointCloud2 msg;
    msg.header = dimos::make_header(frame_id, 0.0);
    msg.height = 1;
    msg.width = 0;
    msg.is_bigendian = 0;
    msg.is_dense = 1;
    msg.fields_length = 4;
    msg.fields.resize(4);
    msg.fields[0] = make_field("x", 0);
    msg.fields[1] = make_field("y", 4);
    msg.fields[2] = make_field("z", 8);
    msg.fields[3] = make_field("intensity", 12);
    msg.point_step = 16;
    msg.row_step = 0;
    msg.data_length = 0;

    lcm.publish(topic, &msg);
}

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    const std::string lidar_topic = mod.topic("lidar");
    const std::string ip = mod.arg("ip", "192.168.1.20");
    const float lidar_hz = mod.arg_float("lidar_hz", 20.0f);
    const std::string lidar_frame_id = mod.arg("lidar_frame_id", "lidar");

    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    lcm::LCM lcm;
    if (!lcm.good()) {
        std::fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }

    std::printf("[m20-lidar] Starting native lidar driver\n");
    std::printf("[m20-lidar] ip: %s\n", ip.c_str());
    std::printf("[m20-lidar] lidar topic: %s\n", lidar_topic.c_str());

    //todo: Initialize the real M20 lidar C++ SDK here.
    //todo: Register callbacks or blocking reads for raw M20 lidar packets here.

    auto last_lidar = std::chrono::steady_clock::now();
    const auto lidar_period = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / std::max(lidar_hz, 1.0f))
    );

    while (g_running.load()) {
        lcm.handleTimeout(5);
        const auto now = std::chrono::steady_clock::now();

        if (now - last_lidar >= lidar_period) {
            //todo: Convert real M20 lidar frames to sensor_msgs::PointCloud2.
            publish_empty_pointcloud(lcm, lidar_topic, lidar_frame_id);
            last_lidar = now;
        }
    }

    //todo: Stop and release the real M20 lidar SDK here.
    std::printf("[m20-lidar] Shutting down\n");
    return 0;
}
