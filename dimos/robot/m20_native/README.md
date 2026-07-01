# M20 Native Sensors

Native C++ sensor driver wrappers for M20 hardware.

The Python modules declare DimOS streams and launch native executables. The C++
executables own the hardware SDK calls and publish DimOS LCM messages.

## Layout

```text
m20_sensors.py                         Python NativeModule wrapper
blueprints/basic/m20_native_sensors.py  Runnable DimOS blueprint
cpp/lidar/main.cpp                      Native lidar driver entry point
cpp/lidar/CMakeLists.txt                Native lidar build config
cpp/camera/main.cpp                     Native camera driver entry point
cpp/camera/CMakeLists.txt               Native camera build config
```

## Build

```bash
cd dimos/robot/m20_native/cpp/lidar
cmake -B build
cmake --build build --target install -j

cd ../camera
cmake -B build
cmake --build build --target install -j
```

The binaries are installed to:

```text
dimos/robot/m20_native/cpp/lidar/result/bin/m20_lidar_native
dimos/robot/m20_native/cpp/camera/result/bin/m20_camera_native
```

## Run

```bash
uv run dimos run m20-native-sensors --robot-ip <M20_IP>
```

## SDK Integration Points

Search for `#todo` in `cpp/lidar/main.cpp`, `cpp/camera/main.cpp`, and
`m20_sensors.py`.

The native lidar executable should:

- Initialize the M20 C++ SDK.
- Read or subscribe to lidar frames.
- Convert lidar data to `sensor_msgs::PointCloud2`.
- Release lidar SDK resources on shutdown.

The native camera executable should:

- Initialize the M20 C++ SDK.
- Read or subscribe to camera frames.
- Convert camera data to `sensor_msgs::Image`.
- Publish calibrated camera intrinsics as `sensor_msgs::CameraInfo`.
- Release camera SDK resources on shutdown.
