// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// DeepRobotics M20 "drdds" -> dimos LCM bridge (NativeModule binary).
//
// Runs *on the M20*. The robot's onboard stack publishes standard ROS 2-typed
// samples (sensor_msgs/PointCloud2, sensor_msgs/Imu, nav_msgs/Odometry, ...) on
// a Fast-DDS fork ("drdds") under the ROS `rt/` topic namespace. This binary
// subscribes to the topics we care about and republishes each sample as the
// structurally-identical dimos_lcm type, so the rest of dimos consumes them on
// the LCM bus like any other module's output.
//
// The Python wrapper (module.py) passes each output port's LCM channel string
// via `--<port> <topic>#<msg_type>` and the drdds source topic names via
// `--lidar_topic` / `--imu_topic` / `--odom_topic`.
//
// Usage:
//   ./m20_dds_bridge \
//       --lidar    '/m20/lidar#sensor_msgs.PointCloud2' \
//       --imu      '/m20/imu#sensor_msgs.Imu' \
//       --odometry '/m20/odometry#nav_msgs.Odometry' \
//       --lidar_topic /LIDAR/POINTS --imu_topic /IMU --odom_topic /ODOM \
//       --domain 0

#include "drdds/core/drdds_core.h"

#include "dridl/sensor_msgs/msg/PointCloud2.h"
#include "dridl/sensor_msgs/msg/PointCloud2PubSubTypes.h"
#include "dridl/sensor_msgs/msg/Imu.h"
#include "dridl/sensor_msgs/msg/ImuPubSubTypes.h"
#include "dridl/nav_msgs/msg/Odometry.h"
#include "dridl/nav_msgs/msg/OdometryPubSubTypes.h"

#include <lcm/lcm-cpp.hpp>

#include "dimos_native_module.hpp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

static std::atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;

static void on_signal(int) { g_running.store(false); }

// drdds std_msgs::msg::Header -> dimos_lcm std_msgs::Header. The DDS header
// already carries the sensor's own frame_id and stamp; preserve them exactly
// (integer sec/nanosec, no float round-trip). `frame_override`, when non-empty,
// replaces the source frame_id (handy when the robot leaves it blank).
static std_msgs::Header to_lcm_header(const std_msgs::msg::Header& h,
                                      const std::string& frame_override) {
    static std::atomic<int32_t> seq{0};
    std_msgs::Header out;
    out.seq = seq.fetch_add(1, std::memory_order_relaxed);
    out.stamp.sec = h.stamp().sec();
    out.stamp.nsec = static_cast<int32_t>(h.stamp().nanosec());
    out.frame_id = frame_override.empty() ? h.frame_id() : frame_override;
    return out;
}

static std::string g_frame_override;

// sensor_msgs/PointCloud2: a flat byte blob described by `fields`. The drdds and
// dimos_lcm layouts are identical, so this is a straight field + buffer copy.
static void on_pointcloud(const sensor_msgs::msg::PointCloud2* m, const std::string& chan) {
    if (!g_lcm || m == nullptr) { return; }

    sensor_msgs::PointCloud2 pc;
    pc.header = to_lcm_header(m->header(), g_frame_override);
    pc.height = m->height();
    pc.width = m->width();
    pc.is_bigendian = m->is_bigendian();
    pc.point_step = m->point_step();
    pc.row_step = m->row_step();
    pc.is_dense = m->is_dense();

    const auto& fields = m->fields();
    pc.fields_length = static_cast<int32_t>(fields.size());
    pc.fields.resize(fields.size());
    for (size_t i = 0; i < fields.size(); ++i) {
        pc.fields[i].name = fields[i].name();
        pc.fields[i].offset = fields[i].offset();
        pc.fields[i].datatype = static_cast<int8_t>(fields[i].datatype());
        pc.fields[i].count = fields[i].count();
    }

    const auto& data = m->data();
    pc.data_length = static_cast<int32_t>(data.size());
    pc.data.resize(data.size());
    if (!data.empty()) {
        std::memcpy(pc.data.data(), data.data(), data.size());
    }

    g_lcm->publish(chan, &pc);
}

// sensor_msgs/Imu. Orientation is copied x/y/z/w straight through (standard
// sensor_msgs/Imu order). If gravity ends up on the wrong axis on real hardware,
// the publisher may be filling w/x/y/z (Unitree does) — reorder here if so.
static void on_imu(const sensor_msgs::msg::Imu* m, const std::string& chan) {
    if (!g_lcm || m == nullptr) { return; }

    sensor_msgs::Imu out;
    out.header = to_lcm_header(m->header(), g_frame_override);

    out.orientation.x = m->orientation().x();
    out.orientation.y = m->orientation().y();
    out.orientation.z = m->orientation().z();
    out.orientation.w = m->orientation().w();
    out.angular_velocity.x = m->angular_velocity().x();
    out.angular_velocity.y = m->angular_velocity().y();
    out.angular_velocity.z = m->angular_velocity().z();
    out.linear_acceleration.x = m->linear_acceleration().x();
    out.linear_acceleration.y = m->linear_acceleration().y();
    out.linear_acceleration.z = m->linear_acceleration().z();

    for (int i = 0; i < 9; ++i) {
        out.orientation_covariance[i] = m->orientation_covariance()[i];
        out.angular_velocity_covariance[i] = m->angular_velocity_covariance()[i];
        out.linear_acceleration_covariance[i] = m->linear_acceleration_covariance()[i];
    }

    g_lcm->publish(chan, &out);
}

// nav_msgs/Odometry: pose+twist with covariance, identical nesting on both sides.
static void on_odometry(const nav_msgs::msg::Odometry* m, const std::string& chan) {
    if (!g_lcm || m == nullptr) { return; }

    nav_msgs::Odometry out;
    out.header = to_lcm_header(m->header(), g_frame_override);
    out.child_frame_id = m->child_frame_id();

    const auto& p = m->pose().pose();
    out.pose.pose.position.x = p.position().x();
    out.pose.pose.position.y = p.position().y();
    out.pose.pose.position.z = p.position().z();
    out.pose.pose.orientation.x = p.orientation().x();
    out.pose.pose.orientation.y = p.orientation().y();
    out.pose.pose.orientation.z = p.orientation().z();
    out.pose.pose.orientation.w = p.orientation().w();

    const auto& t = m->twist().twist();
    out.twist.twist.linear.x = t.linear().x();
    out.twist.twist.linear.y = t.linear().y();
    out.twist.twist.linear.z = t.linear().z();
    out.twist.twist.angular.x = t.angular().x();
    out.twist.twist.angular.y = t.angular().y();
    out.twist.twist.angular.z = t.angular().z();

    const auto& pcov = m->pose().covariance();
    const auto& tcov = m->twist().covariance();
    for (int i = 0; i < 36; ++i) {
        out.pose.covariance[i] = pcov[i];
        out.twist.covariance[i] = tcov[i];
    }

    g_lcm->publish(chan, &out);
}

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    const int domain = mod.arg_int("domain", 0);
    const std::string network = mod.arg("network", "");
    g_frame_override = mod.arg("frame_id", "");

    // drdds source topics (ROS names). DrDDSChannel prepends the "rt" namespace,
    // so "/LIDAR/POINTS" subscribes to the wire topic "rt/LIDAR/POINTS".
    const std::string lidar_src = mod.arg("lidar_topic", "/LIDAR/POINTS");
    const std::string imu_src = mod.arg("imu_topic", "/IMU");
    const std::string odom_src = mod.arg("odom_topic", "/ODOM");

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    DrDDSManager::Init(domain, network);

    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[m20_dds] LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    // Only subscribe to ports the wrapper actually wired up. Each channel owns a
    // drdds DataReader for its whole lifetime; hold them until shutdown.
    std::unique_ptr<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>> lidar_chan;
    std::unique_ptr<DrDDSChannel<sensor_msgs::msg::ImuPubSubType>> imu_chan;
    std::unique_ptr<DrDDSChannel<nav_msgs::msg::OdometryPubSubType>> odom_chan;

    if (mod.has("lidar")) {
        const std::string chan = mod.topic("lidar");
        lidar_chan = std::make_unique<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>>(
            [chan](const sensor_msgs::msg::PointCloud2* m) { on_pointcloud(m, chan); },
            lidar_src, domain, false, "rt");
        fprintf(stderr, "[m20_dds] lidar: rt%s -> %s\n", lidar_src.c_str(), chan.c_str());
    }
    if (mod.has("imu")) {
        const std::string chan = mod.topic("imu");
        imu_chan = std::make_unique<DrDDSChannel<sensor_msgs::msg::ImuPubSubType>>(
            [chan](const sensor_msgs::msg::Imu* m) { on_imu(m, chan); },
            imu_src, domain, false, "rt");
        fprintf(stderr, "[m20_dds] imu: rt%s -> %s\n", imu_src.c_str(), chan.c_str());
    }
    if (mod.has("odometry")) {
        const std::string chan = mod.topic("odometry");
        odom_chan = std::make_unique<DrDDSChannel<nav_msgs::msg::OdometryPubSubType>>(
            [chan](const nav_msgs::msg::Odometry* m) { on_odometry(m, chan); },
            odom_src, domain, false, "rt");
        fprintf(stderr, "[m20_dds] odometry: rt%s -> %s\n", odom_src.c_str(), chan.c_str());
    }

    if (!lidar_chan && !imu_chan && !odom_chan) {
        fprintf(stderr, "[m20_dds] no output ports wired; nothing to bridge\n");
        return 1;
    }

    fprintf(stderr, "[m20_dds] bridging domain %d ...\n", domain);
    while (g_running.load()) {
        lcm.handleTimeout(100);
    }

    fprintf(stderr, "[m20_dds] shutting down\n");
    g_lcm = nullptr;
    lidar_chan.reset();
    imu_chan.reset();
    odom_chan.reset();
    DrDDSManager::Delete();
    return 0;
}
