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
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/utility/sync_queue.hpp>

#include "dimos_native_module.hpp"

#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

using namespace robosense::lidar;

using PointCloudMsg = PointCloudT<PointXYZI>;

static std::atomic<bool> g_running{true};
static SyncQueue<std::shared_ptr<PointCloudMsg>> g_free_clouds;
static SyncQueue<std::shared_ptr<PointCloudMsg>> g_ready_clouds;

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

static double timestamp_seconds(double rs_timestamp) {
    // rs_driver docs say timestamp is in seconds for point clouds. Some builds
    // report nanoseconds; normalize very large values defensively.
    if (rs_timestamp > 1e12) {
        return rs_timestamp * 1e-9;
    }
    return rs_timestamp;
}

static sensor_msgs::PointCloud2 to_pointcloud2(
    const PointCloudMsg& cloud,
    const std::string& frame_id
) {
    sensor_msgs::PointCloud2 msg;
    msg.header = dimos::make_header(frame_id, timestamp_seconds(cloud.timestamp));
    msg.height = cloud.height == 0 ? 1 : static_cast<int32_t>(cloud.height);
    msg.width = static_cast<int32_t>(cloud.points.size());
    msg.is_bigendian = 0;
    msg.is_dense = cloud.is_dense ? 1 : 0;
    msg.fields_length = 4;
    msg.fields.resize(4);
    msg.fields[0] = make_field("x", 0);
    msg.fields[1] = make_field("y", 4);
    msg.fields[2] = make_field("z", 8);
    msg.fields[3] = make_field("intensity", 12);
    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.data_length = msg.row_step * msg.height;
    msg.data.resize(msg.data_length);

    uint8_t* dst = msg.data.data();
    for (const auto& point : cloud.points) {
        const float x = point.x;
        const float y = point.y;
        const float z = point.z;
        const float intensity = static_cast<float>(point.intensity);
        std::memcpy(dst + 0, &x, sizeof(float));
        std::memcpy(dst + 4, &y, sizeof(float));
        std::memcpy(dst + 8, &z, sizeof(float));
        std::memcpy(dst + 12, &intensity, sizeof(float));
        dst += msg.point_step;
    }

    return msg;
}

static void publish_pointcloud(
    lcm::LCM& lcm,
    const std::string& topic,
    const PointCloudMsg& cloud,
    const std::string& fallback_frame_id
) {
    const std::string& frame_id = cloud.frame_id.empty() ? fallback_frame_id : cloud.frame_id;
    sensor_msgs::PointCloud2 msg = to_pointcloud2(cloud, frame_id);

    lcm.publish(topic, &msg);
}

static std::shared_ptr<PointCloudMsg> get_cloud_callback() {
    std::shared_ptr<PointCloudMsg> cloud = g_free_clouds.pop();
    if (cloud) {
        cloud->points.clear();
        return cloud;
    }
    return std::make_shared<PointCloudMsg>();
}

static void put_cloud_callback(std::shared_ptr<PointCloudMsg> cloud) {
    if (cloud) {
        g_ready_clouds.push(cloud);
    }
}

static void exception_callback(const Error& error) {
    std::fprintf(stderr, "[m20-lidar] rs_driver warning: %s\n", error.toString().c_str());
}

static RSDriverParam make_driver_param(dimos::NativeModule& mod) {
    RSDriverParam param;
    param.input_type = InputType::ONLINE_LIDAR;
    param.lidar_type = strToLidarType(mod.arg("lidar_type", "RSM1"));
    param.input_param.msop_port = static_cast<uint16_t>(mod.arg_int("msop_port", 6699));
    param.input_param.difop_port = static_cast<uint16_t>(mod.arg_int("difop_port", 7788));
    param.input_param.host_address = mod.arg("host_address", "0.0.0.0");
    param.input_param.group_address = mod.arg("group_address", "0.0.0.0");
    param.decoder_param.min_distance = mod.arg_float("min_distance", 0.2f);
    param.decoder_param.max_distance = mod.arg_float("max_distance", 200.0f);
    param.decoder_param.start_angle = mod.arg_float("start_angle", 0.0f);
    param.decoder_param.end_angle = mod.arg_float("end_angle", 360.0f);
    param.decoder_param.wait_for_difop = mod.arg_bool("wait_for_difop", false);
    return param;
}

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    const std::string lidar_topic = mod.topic("lidar");
    const std::string lidar_frame_id = mod.arg("lidar_frame_id", "lidar");
    const int publish_timeout_ms = std::max(mod.arg_int("publish_timeout_ms", 100), 1);

    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    lcm::LCM lcm;
    if (!lcm.good()) {
        std::fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }

    std::printf("[m20-lidar] Starting native lidar driver\n");
    std::printf("[m20-lidar] lidar topic: %s\n", lidar_topic.c_str());

    RSDriverParam param = make_driver_param(mod);
    std::printf(
        "[m20-lidar] type=%s msop=%u difop=%u host=%s group=%s frame=%s\n",
        lidarTypeToStr(param.lidar_type).c_str(),
        param.input_param.msop_port,
        param.input_param.difop_port,
        param.input_param.host_address.c_str(),
        param.input_param.group_address.c_str(),
        lidar_frame_id.c_str()
    );

    LidarDriver<PointCloudMsg> driver;
    driver.regExceptionCallback(exception_callback);
    driver.regPointCloudCallback(get_cloud_callback, put_cloud_callback);
    if (!driver.init(param)) {
        std::fprintf(stderr, "[m20-lidar] rs_driver init failed\n");
        return 1;
    }
    if (!driver.start()) {
        std::fprintf(stderr, "[m20-lidar] rs_driver start failed\n");
        driver.stop();
        return 1;
    }

    while (g_running.load()) {
        lcm.handleTimeout(5);

        std::shared_ptr<PointCloudMsg> cloud =
            g_ready_clouds.popWait(static_cast<unsigned int>(publish_timeout_ms * 1000));
        if (cloud) {
            publish_pointcloud(lcm, lidar_topic, *cloud, lidar_frame_id);
            g_free_clouds.push(cloud);
        }
    }

    driver.stop();
    std::printf("[m20-lidar] Shutting down\n");
    return 0;
}
