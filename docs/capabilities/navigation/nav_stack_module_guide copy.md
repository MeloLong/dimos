# Nav Stack Module Guide

这份手册从代码实现角度说明 DimOS 的模块、蓝图和自动连接机制，并以 [`create_nav_stack()`](/dimos/navigation/nav_stack/main.py) 为例分析一个完整导航栈是如何构建出来的。

适合阅读对象：

- 想把 Nav Stack 接到新机器人上的开发者
- 想理解 `Blueprint`、`Module`、`In[T]`、`Out[T]` 如何协作的开发者
- 想修改 `nav_stack/main.py` 或替换其中某个 planner 的开发者

## Table of Contents

- [Core Concepts](#core-concepts)
  - [Module](#module)
  - [NativeModule](#nativemodule)
  - [Blueprint](#blueprint)
- [Modules](#modules)
  - [How Python Modules Are Built](#how-python-modules-are-built)
  - [How Native Modules Call Cpp](#how-native-modules-call-cpp)
  - [LCM as the Data Bridge](#lcm-as-the-data-bridge)
  - [PGO](#pgo)
  - [TerrainAnalysis](#terrainanalysis)
  - [TerrainMapExt](#terrainmapext)
  - [FarPlanner](#farplanner)
  - [SimplePlanner](#simpleplanner)
  - [LocalPlanner](#localplanner)
  - [PathFollower](#pathfollower)
- [Autoconnect Model](#autoconnect-model)
- [Remapping](#remapping)
- [Lifecycle](#lifecycle)
  - [Python Module Lifecycle Example](#python-module-lifecycle-example)
  - [NativeModule Lifecycle Example](#nativemodule-lifecycle-example)
  - [Lifecycle in `create_nav_stack()`](#lifecycle-in-create_nav_stack)
- [Nav Stack Blueprint Structure](#nav-stack-blueprint-structure)
  - [1. Accept Stack-Level Options](#1-accept-stack-level-options)
  - [2. Build Default Module Configs](#2-build-default-module-configs)
  - [3. Instantiate Core Modules](#3-instantiate-core-modules)
  - [4. Select Optional Planner and Extensions](#4-select-optional-planner-and-extensions)
  - [5. Autoconnect and Remap](#5-autoconnect-and-remap)
- [Runtime Flow](#runtime-flow)
- [SLAM and Costmap](#slam-and-costmap)
  - [Building a SLAM Module from Zero](#building-a-slam-module-from-zero)
  - [1. Data Source](#1-data-source)
  - [2. Python Wrapper and Capability Interfaces](#2-python-wrapper-and-capability-interfaces)
  - [3. Native Process and Packet Decoding](#3-native-process-and-packet-decoding)
  - [4. SLAM Processing](#4-slam-processing)
  - [5. Outputs and TF](#5-outputs-and-tf)
  - [6. Connecting SLAM to Nav Stack](#6-connecting-slam-to-nav-stack)
  - [Costmap Interface](#costmap-interface)
  - [CostMapper Construction](#costmapper-construction)
  - [Connecting Map to Costmap](#connecting-map-to-costmap)
  - [Nav Stack vs CostMapper](#nav-stack-vs-costmapper)
- [Parallel and Serial Edges](#parallel-and-serial-edges)
- [Time Alignment](#time-alignment)
  - [Latest-Value Cache](#latest-value-cache)
  - [Timestamp Lookup](#timestamp-lookup)
  - [Per-Module Synchronizer](#per-module-synchronizer)
  - [Nav Stack Practical Rule](#nav-stack-practical-rule)
- [Adding a Module to Nav Stack](#adding-a-module-to-nav-stack)
  - [1. Define Module I/O](#1-define-module-io)
  - [2. Add Blueprint to `modules`](#2-add-blueprint-to-modules)
  - [3. Add Remapping if Needed](#3-add-remapping-if-needed)
  - [4. Check Downstream Conflicts](#4-check-downstream-conflicts)
- [Common Patterns](#common-patterns)
  - [Stack-Level Defaults](#stack-level-defaults)
  - [Per-Module Overrides](#per-module-overrides)
  - [Keep Velocity Output Separate](#keep-velocity-output-separate)
  - [Be Explicit About Odometry](#be-explicit-about-odometry)
- [Minimal Usage Example](#minimal-usage-example)
- [Debugging Checklist](#debugging-checklist)

## Core Concepts

DimOS 的导航不是一个单体程序，而是一组独立模块组成的数据流图。

### Module

`Module` 是最小运行单元。每个模块声明自己的输入流、输出流、配置和生命周期方法。

```python
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

class ExampleModule(Module):
    scan: In[PointCloud2]
    filtered_scan: Out[PointCloud2]
```

模块之间默认不直接互相调用方法，而是通过 stream 传消息。一个模块 `publish()`，多个下游模块可以同时订阅。

### NativeModule

`NativeModule` 是 `Module` 的一种特殊形式：Python 负责声明接口、配置和生命周期，真正的算法运行在 native binary 中。

Nav Stack 里大部分重计算模块都是 `NativeModule`：

| Module | Implementation |
|--------|----------------|
| `PGO` | C++ native |
| `TerrainAnalysis` | C++ native |
| `FarPlanner` | C++ native |
| `LocalPlanner` | C++ native |
| `PathFollower` | C++ native |
| `TarePlanner` | C++ native |
| `TerrainMapExt` | Python module |
| `SimplePlanner` | Python module |

这让 Python 层保持轻量，C++ 层承担点云、规划和控制等高频计算。

### Blueprint

`Blueprint` 是“如何创建一个模块”的说明书。调用 `SomeModule.blueprint(...)` 不会立刻运行模块，而是生成一个可以被 coordinator 构建的配置对象。

```python
from dimos.navigation.nav_stack.modules.local_planner.local_planner import LocalPlanner

local_planner_bp = LocalPlanner.blueprint(max_speed=1.0)
```

多个 blueprint 可以通过 `autoconnect()` 组合成一个大 blueprint。

```python
from dimos.core.coordination.blueprints import autoconnect

nav = autoconnect(
    module_a.blueprint(),
    module_b.blueprint(),
    module_c.blueprint(),
)
```

## Modules

这一章把 Nav Stack 里的模块集中放在一起说明。每个模块都从三个角度看：

- **如何构建**：它是纯 Python `Module`，还是调用 C++/native binary 的 `NativeModule`。
- **输入输出**：它声明了哪些 `In[T]` / `Out[T]` stream，以及在 `create_nav_stack()` 里是否被 remap。
- **主流程**：模块真正工作的算法主体在哪里，消息进入后大致怎样被处理。

先抓住一个判断规则：

```text
class X(Module)
  -> 算法主体通常在当前 Python 文件里，找 start()、callback、loop。

class X(NativeModule)
  -> Python 文件通常只是 wrapper，算法主体在 executable 指向的 native binary 里。
```

### How Python Modules Are Built

纯 Python 模块继承 `Module`。它的构建一般由四部分组成。

第一，声明配置：

```python
class MyModuleConfig(ModuleConfig):
    rate_hz: float = 10.0
```

第二，声明输入输出：

```python
class MyModule(Module):
    config: MyModuleConfig

    scan: In[PointCloud2]
    result: Out[PointCloud2]
```

第三，在 `start()` 里订阅输入、启动线程或 async task：

```python
@rpc
def start(self) -> None:
    super().start()
    self.register_disposable(Disposable(self.scan.subscribe(self._on_scan)))
    self._running = True
    self._thread = threading.Thread(target=self._process_loop, daemon=True)
    self._thread.start()
```

第四，在 callback / loop 里写主流程，在 `stop()` 里清理：

```python
def _on_scan(self, msg: PointCloud2) -> None:
    self._latest_scan = msg

def _process_loop(self) -> None:
    while self._running:
        ...
        self.result.publish(output)

@rpc
def stop(self) -> None:
    self._running = False
    self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
    super().stop()
```

纯 Python 模块的算法入口通常就是：

- `start()`：生命周期入口
- `_on_xxx()`：输入消息 callback
- `_process_loop()` / `_planning_loop()`：后台主循环
- `publish()`：输出结果

Nav Stack 里主要的纯 Python 算法模块是 `TerrainMapExt` 和 `SimplePlanner`。

### How Native Modules Call Cpp

调用 C++ 的模块继承 `NativeModule`。Python 文件只负责把这个 native binary 包装成 DimOS 模块。

它的构建一般由两部分组成。

第一，声明 native 配置：

```python
class MyNativeConfig(NativeModuleConfig):
    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/my_native_module"
    build_command: str | None = "nix build .#default --no-write-lock-file"
    shutdown_timeout: float = 10.0

    max_speed: float = 1.0
```

第二，声明 typed streams：

```python
class MyNativeModule(NativeModule):
    config: MyNativeConfig

    scan: In[PointCloud2]
    cmd_vel: Out[Twist]
```

当 coordinator 调用 `build()` 时，`NativeModule` 会检查 `executable` 是否存在；如果不存在且配置了 `build_command`，就先构建 binary。

当 coordinator 调用 `start()` 时，`NativeModule` 会：

1. 收集每个 `In[T]` / `Out[T]` 被 blueprint 分配到的 LCM topic。
2. 把 topic 作为 CLI 参数传给 native binary。
3. 把 config 字段也转成 CLI 参数。
4. 启动 subprocess。
5. 启动 watchdog 线程监控进程。

启动命令大致长这样：

```bash
result/bin/my_native_module \
  --scan /registered_scan#sensor_msgs.PointCloud2 \
  --cmd_vel /nav_cmd_vel#geometry_msgs.Twist \
  --max_speed 1.0
```

所以 NativeModule 的主流程不在 Python wrapper 里，而在 native process 中：

```text
native process subscribes configured LCM input topics
  -> runs C++ algorithm loop
  -> publishes configured LCM output topics
```

Nav Stack 里多数重计算模块都是这种形式：`PGO`、`TerrainAnalysis`、`FarPlanner`、`LocalPlanner`、`PathFollower`、`TarePlanner`。

### LCM as the Data Bridge

对 native 模块来说，`lcm::LCM` 是连接 **DimOS 数据流** 和 **C++ 算法主体** 的核心。Python 侧声明的是 `In[T]` / `Out[T]`，但 C++ 进程真正看到的是一组 LCM channel 名称；算法通过 `lcm.subscribe(...)` 接收输入，通过 `lcm.publish(...)` 发布输出。

完整链路是：

```text
Python Module declaration
  -> In[PointCloud2] / Out[PointCloud2]
  -> NativeModule.start() collects LCM topics
  -> native binary receives --port_name <topic>
  -> C++ lcm.subscribe(topic, callback)
  -> algorithm callback / loop
  -> C++ lcm.publish(output_topic, &msg)
  -> downstream DimOS modules receive typed stream data
```

以 `TerrainAnalysis` 为例，Python wrapper 声明：

```python
class TerrainAnalysis(NativeModule):
    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: Out[PointCloud2]
```

启动时，`NativeModule` 会把这些端口变成命令行参数，形式类似：

```bash
result/bin/terrain_analysis \
  --registered_scan /registered_scan#sensor_msgs.PointCloud2 \
  --odometry /corrected_odometry#nav_msgs.Odometry \
  --terrain_map /terrain_map#sensor_msgs.PointCloud2
```

C++ 侧用 `dimos::NativeModule` 解析这些 topic：

```cpp
dimos::NativeModule mod(argc, argv);

std::string odometry_topic = mod.topic("odometry");
std::string registered_scan_topic = mod.topic("registered_scan");
std::string terrain_map_topic = mod.topic("terrain_map");
```

然后创建 LCM 实例并订阅输入：

```cpp
lcm::LCM lcm;
TerrainAnalysisHandler handler;

lcm.subscribe(
    odometry_topic,
    &TerrainAnalysisHandler::odometryHandler,
    &handler);

lcm.subscribe(
    registered_scan_topic,
    &TerrainAnalysisHandler::laserCloudHandler,
    &handler);
```

这里的 callback 就是算法数据入口。`odometryHandler(...)` 收到 `nav_msgs::Odometry` 后缓存机器人位姿；`laserCloudHandler(...)` 收到 `sensor_msgs::PointCloud2` 后解析点云、裁剪点云，并设置 `newlaserCloud = true`。主循环通过 `lcm.handleTimeout(...)` 驱动这些 callback：

```cpp
while (running) {
    lcm.handleTimeout(10);

    if (newlaserCloud) {
        newlaserCloud = false;
        // run terrain analysis
    }
}
```

这点很重要：`subscribe(...)` 只是注册 callback，真正触发 callback 的是 `lcm.handle(...)` / `lcm.handleTimeout(...)`。如果主循环不调用 handle，消息不会被派发到算法函数里。

输出方向则相反。算法把内部结果组装回 DimOS 消息类型，然后发布到 Python `Out[...]` 对应的 topic：

```cpp
sensor_msgs::PointCloud2 terrainCloud2 =
    smartnav::build_pointcloud2(terrainCloudElev, "map", laserCloudTime);

lcm.publish(terrain_map_topic, &terrainCloud2);
```

所以 `terrain_map: Out[PointCloud2]` 在 Python 里看起来是一个 stream，在 C++ 里就是 `terrain_map_topic` 这个 LCM channel。下游模块不关心它来自 Python 还是 C++，只要 channel 和消息类型匹配，就能被 `autoconnect()` 接上。

`PointCloud2` 这类消息还会有一层内部数据适配。C++ 算法通常不直接操作 `PointCloud2.data`，而是先转成自己的点结构：

```text
sensor_msgs::PointCloud2
  -> parse_pointcloud2()
  -> vector<PointXYZI> / PCL point cloud
  -> algorithm
  -> build_pointcloud2()
  -> sensor_msgs::PointCloud2
```

因此 native 模块的输入输出适配可以总结为：

| Layer | Responsibility |
|-------|----------------|
| Python `In[T]` / `Out[T]` | 声明模块边界和 typed stream |
| `NativeModule.start()` | 把端口映射成 LCM topic CLI 参数 |
| `dimos::NativeModule` C++ helper | 从 `argv` 读取 topic 和配置 |
| `lcm::LCM` | 订阅输入、派发 callback、发布输出 |
| message helpers | 在 `PointCloud2` / `Odometry` 和算法内部结构之间转换 |

### PGO

**Type:** `NativeModule`

**Python wrapper:** `/dimos/navigation/nav_stack/modules/pgo/pgo.py`

**Native algorithm:** local C++ under `/dimos/navigation/nav_stack/modules/pgo/cpp/`

**Build config:**

```python
class PGOConfig(NativeModuleConfig):
    cwd: str | None = ".../pgo/cpp"
    executable: str = "result/bin/pgo"
    build_command: str | None = "nix build .#default --no-write-lock-file"
```

**Inputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `registered_scan` | `PointCloud2` | world-frame LiDAR scan |
| `odometry` | `Odometry` | raw SLAM odometry |

**Outputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `corrected_odometry` | `Odometry` | PGO-corrected pose |
| `global_map` | `PointCloud2` | remapped to `global_map_pgo` |
| `pgo_tf` | `Odometry` | correction transform used by Python wrapper |

**Main flow:**

```text
registered_scan + raw odometry
  -> keyframe selection
  -> scan registration / ICP loop closure
  -> pose graph optimization
  -> publish corrected_odometry
  -> publish global_map
  -> publish pgo_tf
```

Python wrapper 有一个小的辅助流程：收到 `pgo_tf` 后，把它发布到 DimOS TF tree，让其他模块可以查询 `map -> odom` 这类 transform。

### TerrainAnalysis

**Type:** `NativeModule`

**Python wrapper:** `/dimos/navigation/nav_stack/modules/terrain_analysis/terrain_analysis.py`

**Native algorithm:** `github:dimensionalOS/dimos-module-terrain-analysis`

**Build config:**

```python
class TerrainAnalysisConfig(NativeModuleConfig):
    executable: str = "result/bin/terrain_analysis"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-terrain-analysis/v0.1.1 --no-write-lock-file"
    )
```

**Inputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `registered_scan` | `PointCloud2` | LiDAR points |
| `odometry` | `Odometry` | remapped to `corrected_odometry` in `create_nav_stack()` |

**Outputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `terrain_map` | `PointCloud2` | local classified terrain |

**Main flow:**

```text
registered_scan + corrected_odometry
  -> voxelize / crop around vehicle
  -> estimate ground height
  -> classify ground vs obstacle
  -> filter dynamic / stale points
  -> publish terrain_map
```

这个模块是局部和全局规划共同依赖的基础感知层。`terrain_map` 后续会同时进入 `TerrainMapExt`、`LocalPlanner`，也会被 global planner 使用。

### TerrainMapExt

**Type:** pure Python `Module`

**Source:** `/dimos/navigation/nav_stack/modules/terrain_map_ext/terrain_map_ext.py`

**Build config:**

```python
class TerrainMapExtConfig(ModuleConfig):
    scan_voxel_size: float = 0.1
    decay_time: float = 4.0
    vehicle_height: float = 1.5
    terrain_voxel_size: float = 2.0
```

**Inputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `registered_scan` | `PointCloud2` | fresh scan; triggers processing |
| `odometry` | `Odometry` | remapped to `corrected_odometry` |
| `terrain_map` | `PointCloud2` | local terrain from `TerrainAnalysis` |

**Outputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `terrain_map_ext` | `PointCloud2` | larger rolling terrain map |

**Construction pattern:**

```text
start()
  -> subscribe registered_scan / odometry / terrain_map
  -> start _process_loop() thread
```

**Main flow:**

```text
_on_odom()
  -> cache latest vehicle position

_on_local_terrain()
  -> cache latest local terrain cloud

_on_scan()
  -> crop scan by height/range
  -> mark new scan available

_process_loop()
  -> wait for new scan
  -> roll terrain voxel grid around vehicle
  -> insert cropped scan points
  -> downsample cells
  -> decay old points
  -> estimate ground elevation on planar grid
  -> run terrain connectivity filter
  -> merge local terrain near robot
  -> publish terrain_map_ext
```

这个模块没有严格同步三路输入，而是 latest-cache 风格：scan 到达时使用当前缓存的 odometry 和 local terrain。

### FarPlanner

**Type:** `NativeModule`

**Python wrapper:** `/dimos/navigation/nav_stack/modules/far_planner/far_planner.py`

**Native algorithm:** `github:dimensionalOS/dimos-module-far-planner`

**Build config:**

```python
class FarPlannerConfig(NativeModuleConfig):
    executable: str = "result/bin/far_planner_native"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-far-planner/v0.5.0 --no-write-lock-file"
    )
```

**Inputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `terrain_map_ext` | `PointCloud2` | extended terrain map |
| `terrain_map` | `PointCloud2` | local terrain |
| `registered_scan` | `PointCloud2` | current scan |
| `odometry` | `Odometry` | remapped to `corrected_odometry` when `planner="far"` |
| `goal` | `PointStamped` | final navigation goal |
| `stop_movement` | `Bool` | cancel/stop signal |

**Outputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `way_point` | `PointStamped` | intermediate target for `LocalPlanner` |
| `goal_path` | `Path` | global path visualization |
| `graph_nodes` | `GraphNodes3D` | planner debug graph |
| `graph_edges` | `LineSegments3D` | planner debug graph |
| `contour_polygons` | `ContourPolygons3D` | boundary/contour debug |
| `nav_boundary` | `LineSegments3D` | navigation boundary |

**Main flow:**

```text
terrain_map_ext + terrain_map + registered_scan + corrected_odometry + goal
  -> build / update traversability representation
  -> compute global route toward goal
  -> choose near-term waypoint
  -> publish way_point for LocalPlanner
  -> publish goal_path and debug graph outputs
```

`FarPlanner` 是默认全局规划器，偏长距离和较大地图场景。它不直接输出速度，只输出 `way_point` 让局部规划器追踪。

### SimplePlanner

**Type:** pure Python `Module`

**Source:** `/dimos/navigation/nav_stack/modules/simple_planner/simple_planner.py`

**Build config:**

```python
class SimplePlannerConfig(ModuleConfig):
    cell_size: float = 0.3
    obstacle_height_threshold: float = 0.15
    inflation_radius: float = 0.2
    lookahead_distance: float = 1.0
    replan_rate: float = 5.0
    waypoint_rate: float = 30.0
```

**Inputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `terrain_map_ext` | `PointCloud2` | persistent world view |
| `terrain_map` | `PointCloud2` | fresh local terrain |
| `goal` | `PointStamped` | final navigation goal |
| `stop_movement` | `Bool` | cancel/stop signal |

**Outputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `way_point` | `PointStamped` | lookahead target for `LocalPlanner` |
| `goal_path` | `Path` | full A* path |
| `costmap_cloud` | `PointCloud2` | blocked/inflated cells for visualization |

**Construction pattern:**

```text
start()
  -> subscribe goal / stop_movement / terrain_map_ext / terrain_map
  -> start _planning_loop()
  -> start _waypoint_loop()
```

**Main flow:**

```text
_on_goal()
  -> store goal
  -> reset progress / stuck state
  -> clear cached path

_on_terrain_map_ext()
  -> rebuild costmap from persistent terrain

_on_terrain_map()
  -> layer fresh local terrain onto current costmap

_planning_loop()
  -> _replan_once()
      -> query robot pose from TF
      -> check goal reached
      -> update stuck detector
      -> run A* on current costmap
      -> cache path
      -> publish goal_path
      -> publish costmap_cloud

_waypoint_loop()
  -> query robot pose from TF
  -> choose lookahead point on cached path
  -> publish way_point
```

`SimplePlanner` 是 Nav Stack 内部的 A* global planner，不是 Simple Navigation 那套系统。它的设计重点是低频重算完整路径、高频滑动 waypoint，避免每一帧都跑 A*。

### LocalPlanner

**Type:** `NativeModule`

**Python wrapper:** `/dimos/navigation/nav_stack/modules/local_planner/local_planner.py`

**Native algorithm:** `github:dimensionalOS/dimos-module-local-planner`

**Build config:**

```python
class LocalPlannerConfig(NativeModuleConfig):
    executable: str = "result/bin/local_planner"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-local-planner/v0.6.0 --no-write-lock-file"
    )
```

**Inputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `registered_scan` | `PointCloud2` | live obstacle input |
| `odometry` | `Odometry` | raw odometry, not remapped |
| `terrain_map` | `PointCloud2` | terrain from `TerrainAnalysis` |
| `joy_cmd` | `Twist` | manual command input |
| `way_point` | `PointStamped` | from global planner |
| `goal_pose` | `PoseStamped` | alternate goal input |
| `speed` | `Float32` | speed override |
| `navigation_boundary` | `PolygonStamped` | optional boundary |
| `added_obstacles` | `PointCloud2` | injected obstacles |
| `check_obstacle` | `Bool` | obstacle checking toggle |
| `cancel_goal` | `Bool` | cancel signal |

**Outputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `path` | `Path` | local path for `PathFollower` |
| `effective_cmd_vel` | `Twist` | debug/effective velocity |
| `free_paths` | `PointCloud2` | candidate free paths |
| `slow_down` | `Int8` | slowdown signal for `PathFollower` |
| `goal_reached` | `Bool` | navigation reached signal |

**Main flow:**

```text
registered_scan + raw odometry + terrain_map + way_point
  -> evaluate candidate local paths
  -> reject paths blocked by obstacles / terrain
  -> choose best feasible local trajectory
  -> publish path
  -> publish slow_down / goal_reached
  -> publish free_paths debug output when enabled
```

`LocalPlanner` 属于局部控制层，所以保留 raw `odometry`。它更关心机器人当前短时运动，而不是全局闭环后的地图一致性。

### PathFollower

**Type:** `NativeModule`

**Python wrapper:** `/dimos/navigation/nav_stack/modules/path_follower/path_follower.py`

**Native algorithm:** `github:dimensionalOS/dimos-module-path-follower`

**Build config:**

```python
class PathFollowerConfig(NativeModuleConfig):
    executable: str = "result/bin/path_follower"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-path-follower/v0.2.0 --no-write-lock-file"
    )
```

**Inputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `path` | `Path` | local path from `LocalPlanner` |
| `odometry` | `Odometry` | raw odometry |
| `speed` | `Float32` | speed override |
| `slow_down` | `Int8` | slowdown signal |
| `safety_stop` | `Int8` | emergency/safety stop |

**Outputs:**

| Stream | Type | Notes |
|--------|------|-------|
| `cmd_vel` | `Twist` | remapped to `nav_cmd_vel` in `create_nav_stack()` |

**Main flow:**

```text
path + raw odometry + speed / slow_down / safety_stop
  -> choose lookahead target on path
  -> compute linear velocity and yaw command
  -> apply speed caps, slowdown, safety stop
  -> publish cmd_vel
```

`PathFollower` 的 Python 文件没有 pure pursuit / control law 实现。Python 只把 `path`、`odometry`、`slow_down` 等 stream 对应的 LCM topic 传给 native binary。由于 `cmd_vel` 被 remap 成 `nav_cmd_vel`，它不会直接覆盖机器人最终速度；通常还会经过 `MovementManager` 做 teleop/nav 仲裁。


## Autoconnect Model

`autoconnect()` 根据 **stream 名称 + 消息类型** 自动连接模块。

如果一个模块输出：

```python
terrain_map: Out[PointCloud2]
```

另一个模块输入：

```python
terrain_map: In[PointCloud2]
```

那么它们会自动连接到同一个 stream。

这意味着模块关系不是显式写成 `A.call(B)`，而是声明式的数据流：

```text
TerrainAnalysis.terrain_map
  -> LocalPlanner.terrain_map
  -> TerrainMapExt.terrain_map
```

同一个输出 stream 可以被多个模块订阅，因此并行 fan-out 是自然发生的：

```text
registered_scan
  -> PGO
  -> TerrainAnalysis
  -> TerrainMapExt
  -> LocalPlanner
  -> FarPlanner
```

## Remapping

当模块内部字段名和希望连接的外部 stream 名不同，就用 `remappings()`。

Nav Stack 的关键 remapping 在 [`main.py`](/dimos/navigation/nav_stack/main.py)：

```python
remappings = [
    (PathFollower, "cmd_vel", "nav_cmd_vel"),
    (TerrainAnalysis, "odometry", "corrected_odometry"),
    (TerrainMapExt, "odometry", "corrected_odometry"),
    (PGO, "global_map", "global_map_pgo"),
]
```

含义：

| Remapping | Purpose |
|-----------|---------|
| `PathFollower.cmd_vel -> nav_cmd_vel` | 避免直接覆盖机器人最终 `cmd_vel`，让 `MovementManager` 做 nav/teleop mux |
| `TerrainAnalysis.odometry -> corrected_odometry` | 地形分析使用 PGO 修正后的 odometry |
| `TerrainMapExt.odometry -> corrected_odometry` | 扩展地图使用 PGO 修正后的 odometry |
| `PGO.global_map -> global_map_pgo` | 把 PGO 地图和其他 global map 区分开 |

如果 `planner == "far"`，还会额外 remap：

```python
(FarPlanner, "odometry", "corrected_odometry")
```

这表示全局规划器使用全局一致的修正位姿。

## Lifecycle

Blueprint 负责描述“有哪些模块、如何配置、如何连线”，但模块生命周期由 `ModuleCoordinator` 统一执行。

一次完整启动大致是：

```text
ModuleCoordinator.build(blueprint)
  -> start coordinator workers
  -> deploy all modules
  -> connect streams
  -> connect module refs
  -> call build() on every module
  -> call start() on every module
  -> loop until stopped
  -> call stop() on every module in reverse deployment order
```

几个要点：

- `build()`：用于一次性准备工作，例如 native binary build、模型下载、docker build。
- `start()`：开始运行模块，例如订阅输入、启动线程、启动 native subprocess。
- `stop()`：停止模块，例如取消订阅、停止线程、关闭 subprocess、释放 transports。
- `build_all_modules()` 和 `start_all_modules()` 都是并行调用所有模块。
- `stop()` 按模块部署顺序反向执行，尽量让后接入的模块先停。

### Python Module Lifecycle Example

`TerrainMapExt` 是 Python module。它的生命周期写在模块类里：

```python
class TerrainMapExt(Module):
    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: In[PointCloud2]
    terrain_map_ext: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.registered_scan.subscribe(self._on_scan)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.terrain_map.subscribe(self._on_local_terrain)))

        self._running = True
        self._thread = threading.Thread(target=self._process_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()
```

这里的生命周期含义：

- `start()` 订阅三个输入 stream，并启动后台处理线程。
- `register_disposable(...)` 让订阅在 `stop()` 时自动清理。
- `stop()` 先关掉循环标志，再等待线程退出，最后调用 `super().stop()` 清理通用资源。

这类模块的“生命周期配置”主要体现在你如何实现 `start()` / `stop()`。

### NativeModule Lifecycle Example

`PathFollower`、`LocalPlanner`、`PGO` 等是 `NativeModule`。它们的 lifecycle 主要通过 config 描述 native binary 怎么构建、怎么启动、怎么停止。

以 `PathFollower` 为例：

```python
class PathFollowerConfig(NativeModuleConfig):
    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/path_follower"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-path-follower/v0.2.0 --no-write-lock-file"
    )
    shutdown_timeout: float = 10.0

    max_speed: float = 1.0
    max_yaw_rate: float = 45.0


class PathFollower(NativeModule):
    config: PathFollowerConfig

    path: In[NavPath]
    odometry: In[Odometry]
    slow_down: In[Int8]
    cmd_vel: Out[Twist]
```

当 coordinator 调用 `PathFollower.build()` 时：

- 如果 `executable` 不存在，并且配置了 `build_command`，会运行 build command。
- 如果 `executable` 已存在，默认不重复 build。
- 如果设置 `auto_build=True` 或全局 `build_native=True`，即使 executable 已存在也会 rebuild。

当 coordinator 调用 `PathFollower.start()` 时，`NativeModule` 会：

1. 收集 blueprint 分配给每个 input/output port 的 LCM topic。
2. 把 topic 和 config fields 转成 CLI args。
3. 启动 subprocess。
4. 启动 watchdog 线程监控 native process。

启动命令形态类似：

```bash
result/bin/path_follower \
  --path /path#nav_msgs.Path \
  --odometry /odometry#nav_msgs.Odometry \
  --slow_down /slow_down#std_msgs.Int8 \
  --cmd_vel /nav_cmd_vel#geometry_msgs.Twist \
  --maxSpeed 1.0 \
  --maxYawRate 60.0
```

注意：因为 Nav Stack remap 了 `(PathFollower, "cmd_vel", "nav_cmd_vel")`，native binary 看到的 `--cmd_vel` topic 实际会是 `nav_cmd_vel` 对应的 LCM channel。

当 coordinator 调用 `PathFollower.stop()` 时：

- 先给 native subprocess 发 `SIGTERM`。
- 如果超过 `shutdown_timeout` 仍未退出，再发 `SIGKILL`。
- 等 watchdog 线程退出。
- 最后调用 `super().stop()` 清理 DimOS module 资源。

### Lifecycle in `create_nav_stack()`

`create_nav_stack()` 本身不手动调用生命周期方法。它只生成 blueprint：

```python
return autoconnect(*modules).remappings(remappings)
```

真正生命周期发生在运行 blueprint 时：

```python
from dimos.navigation.nav_stack.main import create_nav_stack

coordinator = create_nav_stack().build()
coordinator.loop()
```

或通过 CLI：

```bash
dimos run <blueprint-name>
```

因此，在 Nav Stack 中“配置生命周期”通常有两层：

| Layer | What You Configure |
|-------|--------------------|
| Blueprint layer | 是否加入模块、模块 config 参数、worker 数、stream remapping |
| Module layer | `build()`、`start()`、`stop()` 行为，或 `NativeModuleConfig` 的 `executable` / `build_command` / `shutdown_timeout` |

## Nav Stack Blueprint Structure

`create_nav_stack()` 是 Nav Stack 的核心组装函数。它的结构可以拆成五步。

### 1. Accept Stack-Level Options

函数签名提供栈级别开关：

```python
def create_nav_stack(
    *,
    use_tare: bool = False,
    use_terrain_map_ext: bool = True,
    planner: str = "far",
    vehicle_height: float | None = None,
    max_speed: float | None = None,
    waypoint_threshold: float | None = None,
    terrain_voxel_size: float = 0.2,
    replan_rate: float = 0.5,
    record: bool = False,
    ...
) -> Blueprint:
```

这些参数不是单个模块独有的，而是会被分发给多个模块。例如：

| Option | Propagates To |
|--------|---------------|
| `vehicle_height` | `TerrainAnalysis`、`TerrainMapExt`、`FarPlanner` 或 `SimplePlanner` |
| `max_speed` | `LocalPlanner`、`PathFollower` |
| `waypoint_threshold` | `LocalPlanner`、`PathFollower`、`SimplePlanner` |
| `planner` | 选择 `FarPlanner` 或 `SimplePlanner` |
| `record` | 是否加入 `NavRecord` |

这让用户可以用少量参数控制整个导航栈，而不用分别知道每个模块的内部配置名。

### 2. Build Default Module Configs

`create_nav_stack()` 会先复制用户传入的 per-module config：

```python
far_planner_config = {**(far_planner or {})}
local_planner_config = {**(local_planner or {})}
path_follower_config = {**(path_follower or {})}
simple_planner_config = {**(simple_planner or {})}
```

然后使用 `setdefault()` 注入栈级默认值。用户显式传入的配置优先级更高。

例如：

```python
far_planner_config.setdefault("is_static_env", False)
```

如果用户传入：

```python
create_nav_stack(far_planner={"is_static_env": True})
```

用户值会保留，不会被默认值覆盖。

### 3. Instantiate Core Modules

基础模块列表在 `modules` 中构造：

```python
modules = [
    TerrainAnalysis.blueprint(...),
    LocalPlanner.blueprint(...),
    PathFollower.blueprint(...),
    PGO.blueprint(...),
]
```

它们构成 Nav Stack 的基础骨架：

```text
PGO
TerrainAnalysis
LocalPlanner
PathFollower
```

这四个模块无论选择哪个 global planner 都会存在。

### 4. Select Optional Planner and Extensions

全局规划器二选一：

```python
if planner == "simple":
    modules.append(SimplePlanner.blueprint(...))
elif planner == "far":
    modules.append(FarPlanner.blueprint(...))
else:
    raise Exception(f"invalid planner: {planner}")
```

可选扩展：

```python
if use_terrain_map_ext:
    modules.append(TerrainMapExt.blueprint(...))

if use_tare:
    modules.append(TarePlanner.blueprint(...))

if record:
    modules.append(NavRecord.blueprint(...))
```

所以 `create_nav_stack()` 生成的 blueprint 不是固定图，而是由参数控制的可变模块图。

### 5. Autoconnect and Remap

最后一行完成组装：

```python
return autoconnect(*modules).remappings(remappings)
```

`autoconnect(*modules)` 负责按 stream 名称和类型连接大部分边。

`remappings(remappings)` 负责修改少数关键 stream 名称，让数据流符合 Nav Stack 的设计。

## Runtime Flow

默认 `planner="far"` 时，主要数据流如下：

```text
registered_scan + odometry
  -> PGO
      -> corrected_odometry
      -> global_map_pgo

registered_scan + corrected_odometry
  -> TerrainAnalysis
      -> terrain_map

registered_scan + corrected_odometry + terrain_map
  -> TerrainMapExt
      -> terrain_map_ext

terrain_map_ext + terrain_map + registered_scan + corrected_odometry + goal
  -> FarPlanner
      -> way_point
      -> goal_path

registered_scan + raw odometry + terrain_map + way_point
  -> LocalPlanner
      -> path
      -> slow_down
      -> goal_reached

path + raw odometry + slow_down
  -> PathFollower
      -> nav_cmd_vel
```

这里有两个关键分层：

- **Global layer** 使用 `corrected_odometry`，包括 `TerrainAnalysis`、`TerrainMapExt`、`FarPlanner`。
- **Local/control layer** 使用 raw `odometry`，包括 `LocalPlanner` 和 `PathFollower`。

这样做的目的，是让全局地图和全局规划享受 PGO 修正后的长期一致性，同时让局部控制保持实时、平滑和贴近机器人当前运动状态。


## SLAM and Costmap

导航栈通常需要两个基础能力：

- **SLAM**：提供机器人位姿和注册到世界坐标系的点云。
- **Costmap**：把 3D 点云/地图压成 2D 可规划代价图。

在 DimOS 里，这两个能力也是普通模块，通过 typed streams 和蓝图连接到导航模块。注意这里有一个重要区别：

- **Nav Stack** 主要消费 `registered_scan`、`odometry`、`terrain_map`、`terrain_map_ext`，不直接依赖 `CostMapper.global_costmap`。
- **Simple Navigation** 使用 `VoxelGridMapper` / `RayTracingVoxelMap` + `CostMapper` 生成 `global_costmap`，再交给 A* 重规划器。

所以 `SLAM + Costmap` 可以看成两种常见链路：

```text
FastLio2 SLAM -> Nav Stack
  lidar -> registered_scan
  odometry -> odometry

Robot lidar -> Voxel/RayTracing map -> CostMapper -> ReplanningAStarPlanner
  lidar -> global_map -> global_costmap
```

### Building a SLAM Module from Zero

以 [`FastLio2`](/dimos/hardware/sensors/lidar/fastlio2/module.py) 为例，一个 SLAM 模块从 0 构建时可以按这条链路理解：

```text
Data source
  -> Python module wrapper
  -> native process
  -> packet decoding / sensor normalization
  -> SLAM algorithm
  -> DimOS output streams
  -> blueprint connection / remapping
```

这条链路里，Python 不做 SLAM 计算，也不手写解析裸 UDP bytes。Python 负责“模块是什么、有哪些输入输出、如何启动”；C++ native binary 负责“怎么从硬件拿数据、怎么处理、怎么发布结果”。

### 1. Data Source

`FastLio2` 是 source module：它没有 DimOS 输入 stream，不订阅其他模块的 `In[...]`。它的数据源来自外部 Livox LiDAR 网络。

| Source | Meaning |
|--------|---------|
| Livox Mid-360 hardware | 点云和 IMU 的真实来源 |
| `host_ip` | 本机在 LiDAR 网段上的 IP |
| `lidar_ip` | LiDAR 设备 IP |
| SDK ports | 命令、状态、点云、IMU、日志等 UDP 端口 |
| FAST-LIO config | SLAM 算法参数，最终传给 C++ binary |
| `mount` | LiDAR 相对机器人/地面坐标系的安装位姿 |

传统 ROS 链路通常是：

```text
Livox hardware
  -> Livox SDK / livox_ros_driver2
  -> ROS PointCloud2 / Imu
  -> FAST-LIO / Point-LIO ROS node
```

DimOS 里的 `FastLio2` 链路是：

```text
Livox hardware
  -> Livox SDK2 inside fastlio2_native
  -> internal CustomMsg / Imu
  -> FAST-LIO2
  -> DimOS LCM PointCloud2 / Odometry
```

所以“数据源”不是 DimOS stream，而是硬件网络。`FastLio2` 模块边界内部包含了驱动、解包和 SLAM。

### 2. Python Wrapper and Capability Interfaces

Python wrapper 的第一步是定义配置。`FastLio2Config` 继承 `NativeModuleConfig`，说明这个模块需要启动一个 native executable：

```python
class FastLio2Config(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/fastlio2_native"
    build_command: str | None = "nix build .#fastlio2_native"

    host_ip: str = "192.168.1.5"
    lidar_ip: str = "192.168.1.155"
    frequency: float = 10.0
    mount: Pose = Pose()

    frame_id: str = FRAME_ODOM
    child_frame_id: str = FRAME_BODY

    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0
    map_freq: float = 0.0

    config: Path = Path("mid360.yaml")
    config_path: str | None = None
    init_pose: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
```

这里有两类配置：

| Config | Role |
|--------|------|
| `cwd` / `executable` / `build_command` | 告诉 `NativeModule` 在哪里构建和启动 C++ binary |
| `host_ip` / `lidar_ip` / ports | 告诉 Livox SDK 如何和 LiDAR 通信 |
| `config` / `config_path` | FAST-LIO2 算法配置文件 |
| `mount` / `init_pose` | 把传感器安装位姿传给 native binary |
| `frame_id` / `child_frame_id` | 输出 odometry 和 TF 的坐标系 |
| `pointcloud_freq` / `odom_freq` / `map_freq` | 控制输出频率 |

`model_post_init()` 会把用户传入的 `config` 解析成绝对路径 `config_path`，并把 `mount` 转成 C++ 命令行参数需要的 `init_pose`：

```python
self.config_path = str(cfg.resolve())
self.init_pose = [
    m.x,
    m.y,
    m.z,
    m.orientation.x,
    m.orientation.y,
    m.orientation.z,
    m.orientation.w,
]
```

然后定义模块本体：

```python
class FastLio2(NativeModule, perception.Lidar, perception.Odometry, mapping.GlobalPointcloud):
    config: FastLio2Config

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    global_map: Out[PointCloud2]
```

这个类声明分成两层：

| Part | Meaning |
|------|---------|
| `NativeModule` | 运行方式：Python wrapper 启动并管理 C++ native binary |
| `perception.Lidar` | 能力接口：提供 `lidar: Out[PointCloud2]` |
| `perception.Odometry` | 能力接口：提供 `odometry: Out[Odometry]` |
| `mapping.GlobalPointcloud` | 能力接口：提供 `global_map: Out[PointCloud2]` |

这些 capability interface 是很薄的 `Protocol`，用于表达“这个模块提供什么能力”。例如：

```python
class Lidar(Protocol):
    lidar: Out[PointCloud2]

class Odometry(Protocol):
    odometry: Out[OdometryMsg]

class GlobalPointcloud(Protocol):
    global_map: Out[PointCloud2]
```

因此 `FastLio2` 的 DimOS 输入输出边界是：

| Direction | Stream | Type | Meaning |
|-----------|--------|------|---------|
| Input | *(none)* | — | 不订阅 DimOS stream，直接读取 Livox 硬件网络 |
| Output | `lidar` | `PointCloud2` | 注册到世界/里程计坐标系的点云 |
| Output | `odometry` | `Odometry` | SLAM 估计出的连续里程计 |
| Output | `global_map` | `PointCloud2` | 可选累计地图，`map_freq > 0` 时发布 |

### 3. Native Process and Packet Decoding

`FastLio2.start()` 先检查网络配置，再调用 `NativeModule.start()`：

```python
@rpc
def start(self) -> None:
    self._validate_network()
    super().start()
    self.register_disposable(
        Disposable(self.odometry.transport.subscribe(self._on_odom_for_tf, self.odometry))
    )
```

`NativeModule.start()` 会启动 `result/bin/fastlio2_native`，并把配置和 output stream 的 topic 传给 C++ 程序。C++ 程序看到的是类似：

```text
--lidar <LCM topic for FastLio2.lidar>
--odometry <LCM topic for FastLio2.odometry>
--global_map <LCM topic for FastLio2.global_map>
--host_ip ...
--lidar_ip ...
--config_path ...
--init_pose ...
```

C++ 入口在 [`fastlio2/cpp/main.cpp`](/dimos/hardware/sensors/lidar/fastlio2/cpp/main.cpp)。启动后做三件事：

```text
1. read NativeModule CLI args
2. initialize FAST-LIO2 core
3. initialize Livox SDK2 and register callbacks
```

Livox SDK 回调注册如下：

```cpp
SetLivoxLidarPointCloudCallBack(on_point_cloud, nullptr);
SetLivoxLidarImuDataCallback(on_imu_data, nullptr);
SetLivoxLidarInfoChangeCallback(on_info_change, nullptr);
LivoxLidarSdkStart();
```

这里要注意：C++ 代码没有自己从 socket 里逐字节解析裸 UDP。Livox SDK2 负责底层网络通信、设备发现和 packet 封装，然后把 SDK 定义好的 `LivoxLidarEthernetPacket*` 传给回调。

点云 packet 的处理逻辑在 `on_point_cloud(...)`：

```text
LivoxLidarEthernetPacket*
  -> timestamp
  -> dot_num
  -> data_type
  -> data
```

根据 `data_type`，代码把 `data->data` 转成不同的 Livox SDK raw struct：

| Livox data type | Cast target | Unit conversion |
|-----------------|-------------|-----------------|
| `DATA_TYPE_CARTESIAN_HIGH` | `LivoxLidarCartesianHighRawPoint*` | `x/y/z / 1000.0`, mm to m |
| `DATA_TYPE_CARTESIAN_LOW` | `LivoxLidarCartesianLowRawPoint*` | `x/y/z / 100.0`, cm to m |

然后每个 Livox 点被转换成 SLAM 内部点：

```cpp
custom_messages::CustomPoint cp;
cp.x = ...;
cp.y = ...;
cp.z = ...;
cp.reflectivity = ...;
cp.tag = ...;
cp.line = 0;
cp.offset_time = ...;
```

IMU packet 的处理逻辑在 `on_imu_data(...)`：把 `data->data` 转成 `LivoxLidarImuRawPoint*`，读取 `gyro_x/y/z` 和 `acc_x/y/z`，构造内部 `custom_messages::Imu`，再送进 FAST-LIO2。

### 4. SLAM Processing

回调线程只负责把点云/IMU 解成 SLAM 内部输入。真正的 SLAM 主循环在 native binary 的 main loop 里运行。

点云处理流程：

```text
on_point_cloud callback
  -> append CustomPoint to g_accumulated_points
  -> main loop drains accumulated points at frame rate
  -> build custom_messages::CustomMsg
  -> fast_lio.feed_lidar(lidar_msg)
```

IMU 处理流程：

```text
on_imu_data callback
  -> build custom_messages::Imu
  -> fast_lio.feed_imu(imu_msg)
```

主循环不断执行 FAST-LIO2 的处理步骤，得到：

```text
registered point cloud
odometry / pose estimate
optional accumulated global voxel map
```

所以完整处理链路是：

```text
Livox SDK packet callbacks
  -> CustomPoint / Imu
  -> CustomMsg frame
  -> FAST-LIO2 state estimation
  -> registered scan + odometry + optional map
```

### 5. Outputs and TF

C++ native binary 最终把结果发布到 `NativeModule` 传入的 LCM topic。Python wrapper 中声明的 `Out[...]` stream 就对应这些 topic。

| Output stream | Published by | Meaning |
|---------------|--------------|---------|
| `lidar: Out[PointCloud2]` | C++ `publish_lidar(...)` | 注册后的点云，供导航栈做地形分析和局部规划 |
| `odometry: Out[Odometry]` | C++ `publish_odometry(...)` | SLAM 位姿，供导航栈定位机器人 |
| `global_map: Out[PointCloud2]` | C++ map publisher | 可选累计地图，通常需要 remap 避免和 PGO 输出冲突 |

`FastLio2` 还在 Python 层订阅自己的 `odometry` 输出，用于发布 TF：

```python
self.odometry.transport.subscribe(self._on_odom_for_tf, self.odometry)
```

`_on_odom_for_tf()` 会把 odometry 转成：

```text
frame_id = config.frame_id          # usually odom
child_frame_id = config.child_frame_id  # usually body
translation = odometry.pose.position
rotation = odometry.pose.orientation
```

这样其他模块除了订阅 `odometry` stream，也可以通过 TF 查询机器人位姿。

### 6. Connecting SLAM to Nav Stack

Nav Stack 期望输入名字是：

| Expected Stream | Type |
|-----------------|------|
| `registered_scan` | `PointCloud2` |
| `odometry` | `Odometry` |

但 `FastLio2` 输出点云 stream 名叫 `lidar`，所以需要 remap：

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.nav_stack.main import create_nav_stack

robot_nav = (
    autoconnect(
        FastLio2.blueprint(
            host_ip="192.168.1.5",
            lidar_ip="192.168.1.155",
            mount=ROBOT.internal_odom_offsets["mid360_link"],
            map_freq=1.0,
        ),
        create_nav_stack(
            planner="simple",
            vehicle_height=ROBOT.height_clearance,
        ),
    )
    .remappings([
        (FastLio2, "lidar", "registered_scan"),
        (FastLio2, "global_map", "global_map_fastlio"),
    ])
)
```

这里发生了两件事：

- `FastLio2.lidar -> registered_scan`：让 `PGO`、`TerrainAnalysis`、`LocalPlanner`、global planner 都能收到 scan。
- `FastLio2.odometry -> odometry`：名字和类型本来就匹配，不需要 remap。

`global_map` 通常 remap 成 `global_map_fastlio`，是为了避免和 `PGO.global_map -> global_map_pgo` 冲突。

### Costmap Interface

`CostMapper` 是 Simple Navigation 里的代价地图模块，接口在 `/dimos/mapping/costmapper.py`。

**Inputs:**

| Stream | Type | Meaning |
|--------|------|---------|
| `global_map` | `PointCloud2` | 全局 3D 点云地图 |
| `merged_map` | `PointCloud2` | 可选融合地图；如果存在，优先用它 |

**Outputs:**

| Stream | Type | Meaning |
|--------|------|---------|
| `global_costmap` | `OccupancyGrid` | 2D occupancy/cost grid |

`CostMapper` 的主流程：

```text
global_map + optional merged_map
  -> select merged_map if available, otherwise global_map
  -> run occupancy algorithm
  -> apply initial safe radius
  -> publish global_costmap
```

默认算法是 `height_cost`，会根据点云高度变化/坡度生成 cost：

| Cost | Meaning |
|------|---------|
| `0` | free / easy terrain |
| `1..99` | increasing cost |
| `100` | obstacle / impassable |
| `-1` | unknown |

### CostMapper Construction

`CostMapper` 是纯 Python `Module`。它的配置：

```python
class Config(ModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = Field(default_factory=HeightCostConfig)
    initial_safe_radius_meters: float = 0.0
```

模块声明：

```python
class CostMapper(Module):
    global_map: In[PointCloud2]
    merged_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]
```

启动时，它使用 `combine_latest()` 监听：

```text
global_map observable
merged_map observable with initial None
  -> choose map
  -> calculate costmap
  -> publish global_costmap
```

也就是说，`merged_map` 是可选输入。如果没有模块发布 `merged_map`，`CostMapper` 会用 `global_map` 继续工作。

典型构造：

```python
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig

CostMapper.blueprint(
    config=HeightCostConfig(
        resolution=0.05,
        can_pass_under=1.2,
        can_climb=0.10,
    ),
    initial_safe_radius_meters=0.8,
)
```

### Connecting Map to Costmap

Simple Navigation 的 Go2 链路是：

```python
unitree_go2 = autoconnect(
    unitree_go2_basic,
    VoxelGridMapper.blueprint(emit_every=5),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    MovementManager.blueprint(),
)
```

自动连接关系：

```text
unitree_go2_basic.lidar
  -> VoxelGridMapper.lidar

VoxelGridMapper.global_map
  -> CostMapper.global_map

CostMapper.global_costmap
  -> ReplanningAStarPlanner.global_costmap
```

G1 simple nav 里则是：

```python
unitree_g1_nav_simple = autoconnect(
    _unitree_g1_onboard,
    RayTracingVoxelMap.blueprint(voxel_size=0.05),
    CostMapper.blueprint(...),
    ReplanningAStarPlanner.blueprint(...),
    MovementManager.blueprint(),
)
```

对应连接：

```text
FastLio2.lidar
  -> RayTracingVoxelMap.lidar

FastLio2.odometry
  -> RayTracingVoxelMap.odometry

RayTracingVoxelMap.global_map
  -> CostMapper.global_map

CostMapper.global_costmap
  -> ReplanningAStarPlanner.global_costmap
```

`RayTracingVoxelMap` 需要 `odometry`，因为它要根据机器人位姿做 ray tracing clearing；`VoxelGridMapper` 只吃 `lidar`，因为 Go2 原生点云链路已经是适合它累积的格式。

### Nav Stack vs CostMapper

Nav Stack 里没有把 `CostMapper.global_costmap` 当作主导航接口。它的地形/规划链路是：

```text
registered_scan + corrected_odometry
  -> TerrainAnalysis
      -> terrain_map

registered_scan + corrected_odometry + terrain_map
  -> TerrainMapExt
      -> terrain_map_ext

terrain_map / terrain_map_ext
  -> FarPlanner or SimplePlanner
  -> LocalPlanner
```

对比：

| System | Map Representation | Planner Input |
|--------|--------------------|---------------|
| Simple Navigation | `global_map -> global_costmap` | `OccupancyGrid` |
| Nav Stack | `terrain_map / terrain_map_ext` | `PointCloud2` terrain layers |

如果你在接新机器人：

- 使用 **Nav Stack**：优先让 SLAM 输出 `registered_scan` 和 `odometry`，不需要先接 `CostMapper`。
- 使用 **Simple Navigation**：需要 `global_map -> CostMapper -> global_costmap` 这条链路。

## Parallel and Serial Edges

Nav Stack 是数据流图，不是一条严格串行 pipeline。

同一个输入可以 fan-out 到多个模块：

```text
registered_scan
  -> PGO
  -> TerrainAnalysis
  -> TerrainMapExt
  -> LocalPlanner
  -> FarPlanner
```

但模块间也有明确依赖：

```text
PGO.corrected_odometry -> TerrainAnalysis
TerrainAnalysis.terrain_map -> LocalPlanner
FarPlanner.way_point -> LocalPlanner
LocalPlanner.path -> PathFollower
```

因此它既有并行处理，也有串行依赖。运行时不是由一个中央函数顺序调用所有模块，而是每个模块独立订阅输入、处理消息、发布输出。

## Time Alignment

`autoconnect()` 只负责把 stream 接起来，不负责把多个输入按时间戳同步成一组。

如果一个模块订阅了两个或更多输入：

```python
class MyModule(Module):
    scan: In[PointCloud2]
    odometry: In[Odometry]
```

那么这两个输入是异步到达的。模块需要自己决定如何组合它们。常见策略有三种。

### Latest-Value Cache

模块缓存每个输入的最新值，某个主输入到达时，用其他输入的最新值一起处理。

`TerrainMapExt` 就是这种风格：它订阅 `odometry`、`terrain_map` 和 `registered_scan`。`odometry` 更新缓存的车辆位置，`terrain_map` 更新缓存的局部地形；当新的 `registered_scan` 到达时，处理线程使用当前缓存的车辆位姿和局部地形生成 `terrain_map_ext`。

这种方式延迟低、实现简单，适合导航里很多“实时控制 / rolling map”场景，但它不是严格的时间戳对齐。

### Timestamp Lookup

如果模块需要在某个消息时间点查询位姿，可以使用 TF buffer。

TF buffer 支持按时间查询：

```python
tf = self.tf.get(
    "map",
    "body",
    time_point=scan.ts,
    time_tolerance=0.05,
)
```

这表示：找离 `scan.ts` 最近、误差在 `0.05s` 内的 transform。

如果不传 `time_point`，TF 查询返回最新 transform。Nav Stack 的 `SimplePlanner` 查询 robot pose 时就是 latest pose 风格，而不是严格用某个 scan timestamp 做同步。

### Per-Module Synchronizer

如果必须严格同步多路输入，模块应该自己维护小型时间窗口：

```python
from collections import deque

class SyncedModule(Module):
    scan: In[PointCloud2]
    odometry: In[Odometry]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._odom_buffer = deque(maxlen=100)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_buffer.append(msg)

    def _on_scan(self, scan: PointCloud2) -> None:
        odom = min(
            self._odom_buffer,
            key=lambda msg: abs((msg.ts or 0.0) - (scan.ts or 0.0)),
            default=None,
        )
        if odom is None:
            return
        if abs((odom.ts or 0.0) - (scan.ts or 0.0)) > 0.05:
            return
        self._process(scan, odom)
```

这种方式适合传感器融合、标定、精确回放验证等场景。代价是会引入等待、丢帧或更复杂的 buffer 管理。

### Nav Stack Practical Rule

Nav Stack 里大部分模块采用的是“异步 stream + 模块内部缓存 / native 模块内部处理”的模型：

- `TerrainMapExt`：Python 层缓存最新 odometry 和 local terrain，由 scan 触发处理。
- `SimplePlanner`：缓存 costmap，循环查询 latest TF pose，并按自己的频率重规划。
- `PGO`、`TerrainAnalysis`、`FarPlanner`、`LocalPlanner`、`PathFollower`：Python wrapper 只声明 typed streams；具体多输入融合和时序处理在 native binary 内部完成。

因此，默认不要假设 DimOS 会自动做类似 ROS `message_filters` 的 exact/approximate sync。需要严格时间同步时，把同步策略写在模块内部，并明确设置 timestamp tolerance。

## Adding a Module to Nav Stack

添加模块通常分四步。

### 1. Define Module I/O

```python
class MyNavModule(Module):
    terrain_map: In[PointCloud2]
    my_output: Out[PointCloud2]
```

输入输出名字要尽量沿用已有 stream 名称。这样可以被 `autoconnect()` 自动连接。

### 2. Add Blueprint to `modules`

```python
modules.append(MyNavModule.blueprint(...))
```

如果它是基础模块，就加入初始 `modules = [...]`。如果它是可选能力，就加一个 `use_my_module` 参数控制。

### 3. Add Remapping if Needed

如果模块字段名和现有 stream 名不一致，就添加 remapping：

```python
remappings.append((MyNavModule, "input_name", "existing_stream_name"))
```

优先通过一致的 stream 名避免 remapping；remapping 用在确实需要语义区分的地方。

### 4. Check Downstream Conflicts

添加模块时重点检查：

- 是否有多个模块发布同名同类型 output
- 是否有模块误接到了不该接的 stream
- 是否需要 raw `odometry` 还是 `corrected_odometry`
- 输出是否应该被 `MovementManager` mux，而不是直接变成 `cmd_vel`

## Common Patterns

### Stack-Level Defaults

如果一个参数会影响多个模块，放在 `create_nav_stack()` 顶层参数里。

例如：

```python
create_nav_stack(max_speed=1.0)
```

内部同时传给：

- `LocalPlanner.max_speed`
- `LocalPlanner.autonomy_speed`
- `PathFollower.max_speed`
- `PathFollower.autonomy_speed`

### Per-Module Overrides

如果参数只属于某个模块，使用 per-module config dict：

```python
create_nav_stack(
    local_planner={"goal_clearance": 0.8},
    path_follower={"max_yaw_rate": 45.0},
)
```

### Keep Velocity Output Separate

导航栈内部输出 `nav_cmd_vel`，不要直接占用最终 `cmd_vel`。最终 `cmd_vel` 应由 robot-level blueprint 里的 `MovementManager` 或控制仲裁模块生成。

### Be Explicit About Odometry

新增模块时要先判断它属于哪一层：

- 需要全局地图一致性：用 `corrected_odometry`
- 需要实时局部控制：用 raw `odometry`

如果字段名叫 `odometry`，但实际要接 `corrected_odometry`，就在 `create_nav_stack()` 里 remap。

## Minimal Usage Example

新机器人如果已经有 SLAM 模块输出：

- `registered_scan: Out[PointCloud2]`
- `odometry: Out[Odometry]`

并且机器人控制模块输入：

- `cmd_vel: In[Twist]`

可以这样接入：

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack

my_robot_nav = autoconnect(
    MySlamModule.blueprint(),
    create_nav_stack(
        planner="far",
        vehicle_height=0.8,
        max_speed=1.0,
    ),
    MovementManager.blueprint(),
    MyRobotControl.blueprint(),
)
```

如果 SLAM 输出的点云 stream 名叫 `lidar`，而 Nav Stack 需要 `registered_scan`，就 remap：

```python
my_robot_nav = my_robot_nav.remappings([
    (MySlamModule, "lidar", "registered_scan"),
])
```

## Debugging Checklist

如果 Nav Stack 没动起来，按这个顺序检查：

1. `registered_scan` 是否有数据
2. `odometry` 是否有数据
3. `PGO` 是否输出 `corrected_odometry`
4. `TerrainAnalysis` 是否输出 `terrain_map`
5. `TerrainMapExt` 是否输出 `terrain_map_ext`
6. global planner 是否收到 `goal`
7. global planner 是否输出 `way_point`
8. `LocalPlanner` 是否输出 `path`
9. `PathFollower` 是否输出 `nav_cmd_vel`
10. `MovementManager` 是否把 `nav_cmd_vel` mux 成最终 `cmd_vel`

大多数接线问题都来自 stream 名称不匹配、odometry remapping 错误，或者多个模块同时发布同名输出。
