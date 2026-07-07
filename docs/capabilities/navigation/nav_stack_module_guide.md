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
- [Autoconnect Model](#autoconnect-model)
  - [Subscription Path](#subscription-path)
  - [Remapping](#remapping)
- [SLAM and Costmap](#slam-and-costmap)
  - [Mental Model](#mental-model)
  - [Simple Navigation](#simple-navigation)
  - [Nav Stack](#nav-stack)
  - [Costmap Difference](#costmap-difference)
- [FastLio2 Integration](#fastlio2-integration)
  - [What DimOS Wraps](#what-dimos-wraps)
  - [Python Wrapper](#python-wrapper)
  - [Field Initialization Path](#field-initialization-path)
  - [Native Runtime](#native-runtime)
  - [How It Plugs Into Blueprints](#how-it-plugs-into-blueprints)
- [Time Alignment](#time-alignment)
  - [Latest-Value Cache](#latest-value-cache)
  - [Timestamp Lookup](#timestamp-lookup)
  - [Per-Module Synchronizer](#per-module-synchronizer)
  - [Nav Stack Practical Rule](#nav-stack-practical-rule)
- [Minimal Usage Example](#minimal-usage-example)

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

如果只看调用形式，`blueprint(...)` 很像一个“传参函数”：

```python
FastLio2.blueprint(host_ip="192.168.123.164", config="default.yaml")
```

但它比普通传参函数多做了一件事：**它不仅保存配置参数，还保存模块拓扑信息。**

这里的“模块拓扑信息”主要包括：

- 这个 blueprint 对应哪个模块类
- 这个模块声明了哪些 `In[T]` / `Out[T]`
- 这些端口的名称和消息类型是什么
- 模块是否还依赖其他 `Spec` 或模块引用

所以更准确地说：

```text
blueprint(...) = 模块构建说明书
               = 参数 + 端口声明 + 模块连接元信息
```

这也是为什么 `blueprint` 返回的不是模块实例，而是一个还可以继续参与组合的对象。它后面还能接：

- `autoconnect(...)`
- `.remappings(...)`
- `.global_config(...)`
- `.disabled_modules(...)`

从源码角度看，`blueprint` 的动作分两层：

第一层，保存参数：

```text
FastLio2.blueprint(host_ip=..., config=...)
  -> Blueprint.create(FastLio2, **kwargs)
  -> kwargs 被保存到 BlueprintAtom.kwargs
```

第二层，提取模块拓扑信息：

```text
BlueprintAtom.create(FastLio2, kwargs)
  -> 读取 FastLio2 的类型注解
  -> 找出 lidar / odometry / global_map 这些 Out[T]
  -> 找出模块依赖的其他引用
  -> 一并放进 Blueprint
```

这就是为什么 blueprint 不是“仅仅把参数先存起来”，而是把“这个模块以后如何接入系统”也一起描述了。

以 `FastLio2` 和导航框架结合为例：

`FastLio2` 自己声明的输出是：

```python
class FastLio2(...):
    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    global_map: Out[PointCloud2]
```

而 Nav Stack 里的很多模块需要的输入名字是：

```python
registered_scan: In[PointCloud2]
odometry: In[Odometry]
```

这里立刻会遇到一个拓扑问题：

- `FastLio2` 输出叫 `lidar`
- Nav Stack 输入叫 `registered_scan`
- 类型虽然都是 `PointCloud2`，但名字不同，不能直接按默认规则自动连接

这时候 blueprint 保存下来的“端口名 + 端口类型”信息就派上用了。比如 G1 onboard 导航蓝图里是这样接的：

```python
autoconnect(
    _unitree_g1_onboard,
    create_nav_stack(...),
).remappings(
    [
        (FastLio2, "lidar", "registered_scan"),
        (FastLio2, "global_map", "global_map_fastlio"),
    ]
)
```

这段的含义不是“调用 FastLio2 的某个函数”，而是：

```text
FastLio2 这个模块的输出端口 lidar
  -> 在 blueprint 层改名为 registered_scan
  -> 于是可以接到 Nav Stack 里所有声明 registered_scan: In[PointCloud2] 的模块
```

连接后的拓扑大致是：

```text
FastLio2.lidar
  --remap--> registered_scan
  -> TerrainAnalysis.registered_scan
  -> PGO.registered_scan
  -> TerrainMapExt.registered_scan
  -> LocalPlanner.registered_scan
```

同样地：

```text
FastLio2.global_map
  --remap--> global_map_fastlio
```

这样做是为了避免它和别的 `global_map` 端口混在一起。

所以在导航和 FastLio2 的结合里，blueprint 同时承担了两类职责：

1. **传参职责**
   例如：
   - `host_ip`
   - `lidar_ip`
   - `config`
   - `map_freq`

2. **拓扑描述职责**
   例如：
   - `FastLio2` 有哪些输出端口
   - 这些端口默认叫什么
   - 是否要 remap 成 `registered_scan` 或 `global_map_fastlio`
   - 最后应接到 Nav Stack 的哪些模块上

可以把它记成一句话：

```text
参数决定“模块怎么创建”
拓扑信息决定“模块怎么接入系统”
blueprint 同时保存这两部分
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

**Python 封装层面的订阅**

在纯 Python `Module` 中，`In[T]` 字段本身就是模块内部可用的输入 stream。模块需要在 `start()` 里主动调用：

```python
self.scan.subscribe(self._on_scan)
```

这个调用最终会走到 `In.subscribe()`：

```python
def subscribe(self, cb: Callable[[T], Any]) -> Callable[[], None]:
    return self.transport.subscribe(cb, self)
```

也就是说，Python 层订阅的真实含义是：

```text
In[T] stream
  -> stream.transport
  -> transport.subscribe(callback, stream)
  -> incoming LCM message is decoded into T
  -> callback(msg: T)
```

例如 `TerrainMapExt` 是纯 Python 模块，它会在 `start()` 中订阅多个输入：

```python
self.register_disposable(Disposable(self.registered_scan.subscribe(self._on_scan)))
self.register_disposable(Disposable(self.odometry.subscribe(self._on_odom)))
self.register_disposable(Disposable(self.terrain_map.subscribe(self._on_local_terrain)))
```

收到消息后，Python callback 直接拿到 typed message，例如 `PointCloud2` 或 `Odometry`，然后缓存最新值或触发处理线程。

但 `NativeModule` 的 Python wrapper 通常不是这样消费业务输入。以 `TerrainAnalysis` 为例，Python 只声明：

```python
registered_scan: In[PointCloud2]
odometry: In[Odometry]
terrain_map: Out[PointCloud2]
```

它的 `start()` 只调用 `super().start()`，不会在 Python 里写：

```python
self.registered_scan.subscribe(...)
self.odometry.subscribe(...)
```

原因是 native 模块的业务输入由 C++ 进程订阅。Python wrapper 的职责是把 `registered_scan`、`odometry`、`terrain_map` 对应的 transport topic 收集出来，并作为 CLI 参数传给 native binary。对应逻辑在 `NativeModule._collect_topics()`：

```python
for name in list(self.inputs) + list(self.outputs):
    stream = getattr(self, name, None)
    transport = getattr(stream, "_transport", None)
    topic = getattr(transport, "topic", None)
    topics[name] = str(topic)
```

所以 Python 封装层有两种订阅模式：

| Module type | Python wrapper does |
|-------------|---------------------|
| pure Python `Module` | 在 `start()` 里调用 `In.subscribe(callback)`，Python callback 直接处理消息 |
| `NativeModule` | 不在 Python 里订阅业务输入；收集 topic 并交给 C++，由 C++ `lcm.subscribe(...)` 处理 |

### How Native Modules Call Cpp

调用 C++ 的模块继承 `NativeModule`。Python 文件只负责把 native binary 包装成 DimOS 模块：声明可执行文件、构建命令、配置参数，以及这个 native process 对外暴露的 `In[T]` / `Out[T]` 端口。

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

当 coordinator 调用 `start()` 时，`NativeModule` 会启动 subprocess。它不会在 Python 里执行 C++ 算法，也通常不会在 Python 里订阅业务输入；业务输入输出的互联由 blueprint/autoconnect 决定，具体的 topic 传参和 C++ 订阅机制在下一章 `Autoconnect Model` 中说明。

更完整地说，native 模块的参数链路是：

```text
SomeNativeModule.blueprint(**kwargs)
  -> Blueprint.create(module, **kwargs) 只保存 kwargs
  -> coordinator 真正实例化模块
  -> Configurable.__init__() 读取 type(self).config
  -> config_type(**kwargs) 生成 Pydantic config
  -> NativeModule.start() 把 topic + config 都转成 CLI
  -> C++ main(argc, argv) 用 dimos::NativeModule 逐个取回
```

这里有两个关键点。

第一，**字段名不是随意的**。
`SomeNativeModule.blueprint(max_speed=1.0)` 传进去的 key，必须能被该模块的 `config` 类型接住。DimOS 这里不是松散字典，而是：

```python
config_type = get_type_hints(type(self))["config"]
self.config = config_type(**kwargs)
```

而 `BaseConfig` 使用的是：

```python
model_config = {"arbitrary_types_allowed": True, "extra": "forbid"}
```

所以：

- 传了 `FastLio2Config` 没有的字段，会直接报错
- 没传的字段，走 config class 里的默认值
- 类型不对，会在 Pydantic 初始化时校验

第二，**topic 参数和普通配置参数是两路来源**。

| 参数类型 | Python 来源 | C++ 获取方式 |
|---------|-------------|---------------|
| stream topic | `In[T]` / `Out[T]` 经 `autoconnect()` 分配 transport topic | `mod.topic("port_name")` |
| config field | `FastLio2Config(...)` 这类 Pydantic 配置对象 | `mod.arg(...)` / `mod.arg_float(...)` / `mod.arg_bool(...)` |

也就是说，C++ 看到的 `--lidar`、`--odometry` 这类参数，并不是 `FastLio2Config` 字段，而是 Python wrapper 从 stream transport 里收集出来的；而 `--host_ip`、`--msr_freq`、`--config_path` 才是 config 字段转换出来的。

Python 侧的 CLI 组装逻辑在 `NativeModule.start()`：

```python
topics = self._collect_topics()

cmd = [self.config.executable]
for name, topic_str in topics.items():
    cmd.extend([f"--{name}", topic_str])
cmd.extend(self.config.to_cli_args())
cmd.extend(self.config.extra_args)
```

其中 `_collect_topics()` 负责收集端口对应的 transport topic，`to_cli_args()` 负责把 config 字段转成：

```text
--field_name value
```

如果字段不适合直接作为 CLI 参数，还可以在 config 中做两种处理：

- `cli_exclude`：不要直接传这个字段
- `cli_name_override`：Python 字段名和 CLI 参数名不一样时做改名

`FastLio2Config` 就是典型例子：它不直接传 `config: Path` 和 `mount: Pose`，而是在 `model_post_init()` 里把它们转换成更适合 C++ 消费的 `config_path: str` 和 `init_pose: list[float]`。

最后，C++ 侧通常不会再定义一个自动同步的“参数 schema”。它只是用一个很轻量的 helper 解析 argv，例如 [`dimos/hardware/sensors/lidar/common/dimos_native_module.hpp`](/dimos/hardware/sensors/lidar/common/dimos_native_module.hpp)：

```cpp
for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg.size() > 2 && arg[0] == '-' && arg[1] == '-' && i + 1 < argc) {
        args_[arg.substr(2)] = argv[++i];
    }
}
```

之后靠名字取值：

```cpp
std::string lidar_topic = mod.topic("lidar");
std::string host_ip = mod.arg("host_ip", "192.168.1.5");
float msr_freq = mod.arg_float("msr_freq", 50.0f);
bool debug = mod.arg_bool("debug", false);
```

所以 native 模块参数是否能对上，本质上取决于一件事：**Python 发出去的 CLI key，和 C++ 读取时写的 key，名字必须一致。**

**LCM 在 C++ native module 里的作用**

对 C++ native module 来说，LCM 是它和 DimOS stream graph 之间的运行时通信层。Python wrapper 只负责声明：

```python
registered_scan: In[PointCloud2]
odometry: In[Odometry]
terrain_map: Out[PointCloud2]
```

真正收发消息发生在 C++ 进程里：

```text
Python In[T] / Out[T]
  -> autoconnect() 分配 transport topic
  -> NativeModule.start() 传给 C++：--registered_scan /... --terrain_map /...
  -> C++ mod.topic("registered_scan") 取回 LCM channel
  -> lcm.subscribe(...) 订阅输入
  -> lcm.publish(...) 发布输出
```

换句话说，`lcm::LCM` 是 native module 的消息总线对象。它不理解 DimOS 的 Python `In[T]` / `Out[T]` 类型系统，但它拿到的是已经包含消息类型后缀的 LCM channel，例如：

```text
/registered_scan#sensor_msgs.PointCloud2
/odometry#nav_msgs.Odometry
/terrain_map#sensor_msgs.PointCloud2
```

**订阅输入**

C++ 侧通常先从 CLI 中取出输入端口对应的 topic：

```cpp
dimos::NativeModule mod(argc, argv);

std::string scan_topic = mod.topic("registered_scan");
std::string odom_topic = mod.topic("odometry");
```

然后创建 LCM 对象并注册 callback：

```cpp
lcm::LCM lcm;
if (!lcm.good()) {
    return 1;
}

Handlers handlers;
lcm.subscribe(odom_topic, &Handlers::on_odometry, &handlers);
lcm.subscribe(scan_topic, &Handlers::on_registered_scan, &handlers);
```

callback 的签名由 LCM C++ binding 约定，第三个参数就是解码后的 typed message：

```cpp
void on_odometry(
    const lcm::ReceiveBuffer*,
    const std::string&,
    const nav_msgs::Odometry* msg
) {
    // msg 已经是 nav_msgs::Odometry
}
```

只注册 `subscribe()` 还不够，主循环里必须持续调用 `handle()` 或 `handleTimeout()`，LCM 才会真正分发收到的消息到 callback：

```cpp
while (g_running.load()) {
    while (lcm.handleTimeout(0) > 0) {}

    // consume callback-filled buffers, run planner, publish outputs
}
```

`PGO` 就是这种模式：订阅 `odometry` 和 `registered_scan`，callback 缓存最新 odom 或把 scan+pose 放进队列，主循环再从队列中取数据做优化。

`TerrainAnalysis` 也一样：C++ 侧订阅 `odometry` 和 `registered_scan`，主循环调用 `lcm.handleTimeout(10)` 收消息；当新的 scan 到达后，算法生成并发布 `terrain_map`。

**发布输出**

输出端口也通过 `mod.topic("port_name")` 取回 topic：

```cpp
std::string terrain_map_topic = mod.topic("terrain_map");
```

算法生成消息后，直接用同一个 LCM 对象发布：

```cpp
sensor_msgs::PointCloud2 terrain_cloud;
// fill terrain_cloud ...
lcm.publish(terrain_map_topic, &terrain_cloud);
```

发布的消息类型必须和 Python wrapper 里的 `Out[T]` 一致。例如：

| Python output | C++ message type | C++ publish |
|---------------|------------------|-------------|
| `terrain_map: Out[PointCloud2]` | `sensor_msgs::PointCloud2` | `lcm.publish(terrain_map_topic, &msg)` |
| `odometry: Out[Odometry]` | `nav_msgs::Odometry` | `lcm.publish(odometry_topic, &msg)` |
| `cmd_vel: Out[Twist]` | `geometry_msgs::Twist` | `lcm.publish(cmd_vel_topic, &msg)` |

FastLio2 是一个只发布、不订阅 DimOS 业务输入的例子。它的 LiDAR/IMU 数据来自 Livox SDK callback，FAST-LIO 算完后用：

```cpp
g_lcm->publish(g_lidar_topic, &point_cloud);
g_lcm->publish(g_odometry_topic, &odometry);
```

把 `lidar: Out[PointCloud2]` 和 `odometry: Out[Odometry]` 发回 DimOS stream graph。它的主循环里也调用了 `handleTimeout(0)`，但当前 FastLio2 C++ 文件没有注册业务 `subscribe()`，所以 LCM 在这里主要承担输出发布职责。

因此，native planner 的 C++ 使用方式可以记成：

```text
输入端口：mod.topic("input_name") -> lcm.subscribe(topic, callback)
输出端口：mod.topic("output_name") -> lcm.publish(topic, &message)
主循环：handleTimeout()/handle() 驱动订阅 callback
```

## Autoconnect Model

`autoconnect()` 是模块互联发生的地方。模块不会在代码里写 `A.call(B)`，而是声明自己的 `Out[T]` 和 `In[T]`；blueprint 在 build 时根据 **stream 名称 + 消息类型** 把它们接到同一个 transport topic 上。

如果一个模块输出：

```python
terrain_map: Out[PointCloud2]
```

另一个模块输入：

```python
terrain_map: In[PointCloud2]
```

那么它们会自动连接到同一个 stream。

```text
TerrainAnalysis.terrain_map
  -> LocalPlanner.terrain_map
  -> TerrainMapExt.terrain_map
```

同一个输出可以被多个输入订阅，因此 fan-out 是自然发生的：

```text
registered_scan
  -> PGO
  -> TerrainAnalysis
  -> TerrainMapExt
  -> LocalPlanner
  -> FarPlanner
```

### Subscription Path

订阅机制要分成两种情况看：纯 Python 模块和 `NativeModule`。它们都由 `autoconnect()` 决定连接关系，但实际消费消息的位置不同。

纯 Python 模块会在 Python 的 `start()` 中调用 `In.subscribe(...)`：

```python
self.register_disposable(Disposable(self.scan.subscribe(self._on_scan)))
```

这行可以拆开理解：

```python
unsubscribe = self.scan.subscribe(self._on_scan)
disposable = Disposable(unsubscribe)
self.register_disposable(disposable)
```

`self._on_scan` 是模块作者自己定义的 callback，不是框架自动生成的函数。`subscribe(...)` 会注册这个 callback，并返回一个 `unsubscribe` 函数；`Disposable(unsubscribe)` 只是把取消订阅函数保存起来，等模块停止或清理时调用。

这条链路是：

```text
autoconnect connects Out[T] to In[T]
  -> In[T] has a transport
  -> module start() calls In.subscribe(callback)
  -> transport.subscribe(callback, stream)
  -> incoming LCM message is decoded into T
  -> Python callback(msg: T) runs
```

`NativeModule` 不会在 Python wrapper 里订阅业务输入。它会把 autoconnect 分配好的 topic 收集出来，作为命令行参数传给 C++ 进程。

```python
def _collect_topics(self) -> dict[str, str]:
    topics: dict[str, str] = {}
    for name in list(self.inputs) + list(self.outputs):
        stream = getattr(self, name, None)
        transport = getattr(stream, "_transport", None)
        topic = getattr(transport, "topic", None)
        if topic is not None:
            topics[name] = str(topic)
    return topics
```

`NativeModule.start()` 再把这些 topic 变成 CLI 参数：

```python
cmd = [self.config.executable]
for name, topic_str in topics.items():
    cmd.extend([f"--{name}", topic_str])
cmd.extend(self.config.to_cli_args())
```

因此，一个外部话题进入 C++ 的完整路径是：

```text
upstream Out[T]
  -> autoconnect/remapping connects it to NativeModule.In[T]
  -> In[T] transport has an LCM topic
  -> NativeModule._collect_topics() reads the topic
  -> NativeModule.start() launches C++ with --input_name <topic>
  -> C++ mod.topic("input_name") gets the topic string
  -> C++ lcm.subscribe(topic, callback)
```

以 `TerrainAnalysis` 为例，Python wrapper 声明：

```python
class TerrainAnalysis(NativeModule):
    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: Out[PointCloud2]
```

启动 C++ 时会得到类似：

```bash
result/bin/terrain_analysis \
  --registered_scan /registered_scan#sensor_msgs.PointCloud2 \
  --odometry /corrected_odometry#nav_msgs.Odometry \
  --terrain_map /terrain_map#sensor_msgs.PointCloud2
```

C++ 入口先解析 topic：

```cpp
dimos::NativeModule mod(argc, argv);

std::string odometry_topic = mod.topic("odometry");
std::string registered_scan_topic = mod.topic("registered_scan");
std::string terrain_map_topic = mod.topic("terrain_map");
```

然后用 LCM 订阅输入：

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

`subscribe(...)` 只是注册 callback，真正触发 callback 的是主循环里的 `handle`：

```cpp
while (running) {
    lcm.handleTimeout(10);

    if (newlaserCloud) {
        newlaserCloud = false;
        // run terrain analysis
    }
}
```

输出方向相反：C++ 把算法结果组装成 DimOS 消息类型，并发布到 `Out[T]` 对应的 topic。

```cpp
sensor_msgs::PointCloud2 terrainCloud2 =
    smartnav::build_pointcloud2(terrainCloudElev, "map", laserCloudTime);

lcm.publish(terrain_map_topic, &terrainCloud2);
```

所以 `terrain_map: Out[PointCloud2]` 在 Python 里是一个 typed stream，在 C++ 里就是 `terrain_map_topic` 这个 LCM channel。

### Remapping

`remappings()` 的作用是在 blueprint build 阶段改变某个模块端口对应的 stream 名称。它不是运行时消息转发，也不是 C++ 里的逻辑；它会影响 `autoconnect()` 给这个端口分配哪个 topic。

例如 `create_nav_stack()` 中的典型 remapping：

```python
remappings = [
    (PathFollower, "cmd_vel", "nav_cmd_vel"),
    (TerrainAnalysis, "odometry", "corrected_odometry"),
    (TerrainMapExt, "odometry", "corrected_odometry"),
    (PGO, "global_map", "global_map_pgo"),
]
```

含义是：

| Remapping | Effect |
|-----------|--------|
| `PathFollower.cmd_vel -> nav_cmd_vel` | `PathFollower` 仍然在代码里叫 `cmd_vel`，但实际发布到 `nav_cmd_vel` 对应的 topic |
| `TerrainAnalysis.odometry -> corrected_odometry` | `TerrainAnalysis` 代码里仍然读取 `odometry` 端口，但该端口连接的是 `corrected_odometry` stream |
| `TerrainMapExt.odometry -> corrected_odometry` | 扩展地形图使用 PGO 修正后的位姿 |
| `PGO.global_map -> global_map_pgo` | 避免 PGO 输出的地图和其他 `global_map` stream 冲突 |

对 native 模块来说，remapping 最终体现在 CLI 参数的值上。`TerrainAnalysis` 的 C++ 代码仍然写：

```cpp
std::string odometry_topic = mod.topic("odometry");
```

但因为 Python blueprint 把 `TerrainAnalysis.odometry` remap 到 `corrected_odometry`，所以 `--odometry` 后面传入的是 `corrected_odometry` 对应的 LCM topic。C++ 不需要知道 remapping 存在，它只订阅 Python 传进来的 topic。

这就是 remapping 的核心价值：**模块内部端口名保持稳定，blueprint 可以在外部改变连接对象**。同一个模块可以在不同 stack 里接 raw `odometry`，也可以接 `corrected_odometry`，不用改 Python wrapper 或 C++ 源码。

## SLAM and Costmap

**SLAM 输出的空间信息如何进入导航框架，并最终变成 planner 可以用的地图或代价表达。**

DimOS 里有两条容易混淆的链路：

- **Simple Navigation**：先用 `RayTracingVoxelMap` 或 `VoxelGridMapper` 生成 `global_map`，再由独立 `CostMapper` 把 `global_map` 转成 `global_costmap`，最后交给 `ReplanningAStarPlanner`。
- **Nav Stack**：不走 `global_map -> CostMapper -> global_costmap`，而是从 `registered_scan + odometry` 生成 `terrain_map / terrain_map_ext`，再由 `SimplePlanner`、`FarPlanner`、`LocalPlanner` 在内部形成代价表达和路径。

### Mental Model

两套导航都需要 SLAM/odometry，但它们消费 SLAM 的方式不同。最重要的是看清楚：**哪个模块生成哪种地图，地图再输入给哪个下游模块。**

**Simple Navigation** 的地图链路是：

```text
RayTracingVoxelMap or VoxelGridMapper
  -> global_map: PointCloud2
  -> CostMapper.global_map
  -> CostMapper.global_costmap: OccupancyGrid
  -> ReplanningAStarPlanner.global_costmap
```

其中：

- G1 Simple Navigation 常用 `RayTracingVoxelMap`：输入 `lidar + odometry`，输出 `global_map`。
- Go2 Simple Navigation 常用 `VoxelGridMapper`：输入 `lidar`，输出 `global_map`。
- `CostMapper` 只接收 `global_map / merged_map`，输出 `global_costmap`。
- `ReplanningAStarPlanner` 接收 `global_costmap + odom/odometry` 做 A* 规划。

**Nav Stack** 的地图链路是：

```text
registered_scan + odometry
  -> TerrainAnalysis
  -> terrain_map: PointCloud2

registered_scan + odometry + terrain_map
  -> TerrainMapExt
  -> terrain_map_ext: PointCloud2

terrain_map + terrain_map_ext
  -> SimplePlanner / FarPlanner
  -> way_point + goal_path

terrain_map + registered_scan + way_point
  -> LocalPlanner
  -> path
```

其中：

- `terrain_map` 是局部、更新更快的地形点云层。
- `terrain_map_ext` 是范围更大、衰减更慢的扩展地形点云层。
- `SimplePlanner` 会把 `terrain_map_ext + terrain_map` 转成自己内部的 2D `Costmap`，再跑 A*。
- `FarPlanner` 直接消费 `terrain_map_ext / terrain_map / registered_scan / odometry` 做远程规划。
- `LocalPlanner` 不消费 `SimplePlanner.costmap_cloud`，而是直接消费 `terrain_map + registered_scan + way_point` 做局部避障和路径生成。

按“地图产物 -> 下游消费者”看，可以总结成：

| Map product | Produced by | Consumed by | Used for |
|-------------|-------------|-------------|----------|
| `global_map` | `RayTracingVoxelMap` or `VoxelGridMapper` | `CostMapper.global_map` | Simple Navigation 的 3D 地图输入 |
| `global_costmap` | `CostMapper` | `ReplanningAStarPlanner.global_costmap` | Simple Navigation 的 2D A* 代价地图 |
| `terrain_map` | `TerrainAnalysis` | `TerrainMapExt`, `SimplePlanner`, `FarPlanner`, `LocalPlanner` | Nav Stack 的快速局部地形层 |
| `terrain_map_ext` | `TerrainMapExt` | `SimplePlanner`, `FarPlanner` | Nav Stack 的扩展地形层 |
| `costmap_cloud` | `SimplePlanner` | visualization only | 可视化内部 blocked cells，不是下游 planner 的标准输入 |

### Simple Navigation

Simple Navigation 是一条比较传统的“3D voxel map -> 2D costmap -> A* planner”链路。

```text
lidar / odometry
  -> RayTracingVoxelMap or VoxelGridMapper
  -> global_map: PointCloud2
  -> CostMapper.global_map
  -> global_costmap: OccupancyGrid
  -> ReplanningAStarPlanner.global_costmap
```

它的核心特点是：**先由 Voxel Map 模块生成 `global_map`，再由独立 `CostMapper` 生成 `global_costmap`。**

Simple Navigation 不是直接把原始 scan 送进 `CostMapper`。`CostMapper` 前面一定有一层 Voxel Map / 3D map builder，负责把输入点云变成 `global_map: PointCloud2`。

源码里主要有两种 Voxel Map 模块。

**RayTracingVoxelMap**

源码接口：

```python
class RayTracingVoxelMap(NativeModule, mapping.GlobalPointcloud):
    lidar: In[PointCloud2]
    odometry: In[Odometry]
    global_map: Out[PointCloud2]
    local_map: Out[PointCloud2]
```

它的含义是：

- 输入 `lidar`：当前点云。
- 输入 `odometry`：当前机器人位置，用作 ray tracing 的 origin。
- 输出 `global_map`：累计后的全局 voxel map，给 `CostMapper`。
- 输出 `local_map`：局部 voxel map，目前不是 Simple Navigation 主链路里的 costmap 输入。

它用在 G1 Simple Navigation：

```python
unitree_g1_nav_simple = autoconnect(
    _unitree_g1_onboard,
    RayTracingVoxelMap.blueprint(voxel_size=voxel_resolution),
    CostMapper.blueprint(...),
    ReplanningAStarPlanner.blueprint(...),
    ...
)
```

对应链路是：

```text
FastLio2.lidar
  -> RayTracingVoxelMap.lidar

FastLio2.odometry
  -> RayTracingVoxelMap.odometry

RayTracingVoxelMap.global_map
  -> CostMapper.global_map
```

**VoxelGridMapper**

源码接口：

```python
class VoxelGridMapper(StreamModule[PointCloud2, PointCloud2]):
    lidar: In[PointCloud2]
    global_map: Out[PointCloud2]
```

它的含义是：

- 输入 `lidar`：点云输入。
- 输出 `global_map`：累积后的 voxel grid map，给 `CostMapper`。
- 它不订阅 `odometry`，因此默认输入点云已经在可累积的坐标系下。

它用在 Go2 Simple Navigation：

```python
unitree_go2 = autoconnect(
    unitree_go2_basic,
    VoxelGridMapper.blueprint(emit_every=5),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    ...
)
```

对应链路是：

```text
unitree_go2_basic.lidar
  -> VoxelGridMapper.lidar

VoxelGridMapper.global_map
  -> CostMapper.global_map
```

两者对比如下：

| Robot / chain | Voxel map module | Inputs | Outputs | Meaning |
|---------------|------------------|--------|---------|---------|
| G1 Simple Navigation | `RayTracingVoxelMap` | `lidar: In[PointCloud2]`, `odometry: In[Odometry]` | `global_map: Out[PointCloud2]`, `local_map: Out[PointCloud2]` | 用 odometry 作为 ray origin，做 ray tracing voxel map 和 clearing |
| Go2 Simple Navigation | `VoxelGridMapper` | `lidar: In[PointCloud2]` | `global_map: Out[PointCloud2]` | 把输入点云累积成 voxel grid map；假设输入点云已经在可累积坐标系下 |

所以 Simple Navigation 的完整结构应理解为：

```text
lidar / registered point cloud
  -> RayTracingVoxelMap or VoxelGridMapper
  -> global_map: PointCloud2
  -> CostMapper.global_map
  -> global_costmap: OccupancyGrid
  -> ReplanningAStarPlanner.global_costmap

odom / odometry
  -> ReplanningAStarPlanner
```

其中 `CostMapper` 消费的是 Voxel Map 模块输出的 `global_map`，不是直接消费 `lidar`。

`CostMapper` 的接口很窄：

```python
class CostMapper(Module):
    global_map: In[PointCloud2]
    merged_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]
```

```text
global_map + optional merged_map
  -> choose merged_map if available, otherwise global_map
  -> occupancy / height-cost algorithm
  -> global_costmap
```

`CostMapper` 不订阅 `odometry`，不做 SLAM，也不做路径规划；它只负责把点云地图压成 `OccupancyGrid`。下游 `ReplanningAStarPlanner` 同时订阅代价地图和当前位姿：

```python
class ReplanningAStarPlanner(Module, NavigationInterface):
    odom: In[PoseStamped]
    odometry: In[Odometry]
    global_costmap: In[OccupancyGrid]
```

因此 Simple Navigation 的职责边界可以压缩成：

- `FastLio2 / robot lidar` 输出点云和位姿。
- `RayTracingVoxelMap / VoxelGridMapper` 把点云整理成 `global_map`。
- `CostMapper` 把 `global_map` 转成 `global_costmap`。
- `ReplanningAStarPlanner` 用 `global_costmap + odom/odometry` 做路径规划。

可以记成：

```text
lidar / registered point cloud
  -> RayTracingVoxelMap or VoxelGridMapper
  -> global_map
  -> CostMapper
  -> global_costmap
  -> ReplanningAStarPlanner

odom / odometry
  -> ReplanningAStarPlanner
```

一句话总结：

```text
Simple Navigation 的 costmap 是显式产物：
global_map -> CostMapper -> global_costmap
```

### Nav Stack

Nav Stack 是另一套框架：它不把 `global_costmap` 作为主地图接口，而是使用 `terrain_map / terrain_map_ext` 作为导航地图层。

`create_nav_stack()` 是 Nav Stack 的组合入口。它只创建导航模块本体，不创建 `FastLio2`；SLAM/传感器模块在外层 robot blueprint 接入。

```python
return autoconnect(*modules).remappings(remappings)
```

简化后的结构是：

```python
modules = [
    TerrainAnalysis.blueprint(...),
    LocalPlanner.blueprint(...),
    PathFollower.blueprint(...),
    PGO.blueprint(...),
]

if planner == "simple":
    modules.append(SimplePlanner.blueprint(...))
elif planner == "far":
    modules.append(FarPlanner.blueprint(...))

if use_terrain_map_ext:
    modules.append(TerrainMapExt.blueprint(...))

if use_tare:
    modules.append(TarePlanner.blueprint(...))

nav_stack = autoconnect(*modules).remappings([
    (PathFollower, "cmd_vel", "nav_cmd_vel"),
    (TerrainAnalysis, "odometry", "corrected_odometry"),
    (TerrainMapExt, "odometry", "corrected_odometry"),
    (PGO, "global_map", "global_map_pgo"),
])
```

外层 robot blueprint 负责把 SLAM 接进来。以 G1 onboard nav 为例：

```python
autoconnect(
    _unitree_g1_onboard,  # includes FastLio2
    create_nav_stack(...),
).remappings([
    (FastLio2, "lidar", "registered_scan"),
    (FastLio2, "global_map", "global_map_fastlio"),
])
```

因此 Nav Stack 的输入前提是：

- 上游提供 `registered_scan: PointCloud2`
- 上游提供 `odometry: Odometry`
- `PGO` 可进一步输出 `corrected_odometry`

主链路是：

```text
FastLio2.lidar
  --remap--> registered_scan

FastLio2.odometry
  -> odometry

registered_scan + odometry
  -> PGO
  -> corrected_odometry

registered_scan + corrected_odometry
  -> TerrainAnalysis
  -> terrain_map

registered_scan + corrected_odometry + terrain_map
  -> TerrainMapExt
  -> terrain_map_ext

terrain_map + terrain_map_ext + goal
  -> SimplePlanner or FarPlanner
  -> way_point + goal_path

way_point + terrain_map + registered_scan + odometry
  -> LocalPlanner
  -> path

path + odometry + slow_down
  -> PathFollower
  -> cmd_vel
```

Nav Stack 的关键点是：`terrain_map` 和 `terrain_map_ext` 都还是 `PointCloud2`。它们不是 `CostMapper` 输出的 `OccupancyGrid`，而是地形点云层。

| Module | Inputs | Outputs | Role |
|--------|--------|---------|------|
| `TerrainAnalysis` | `registered_scan`, `odometry` | `terrain_map` | 从 scan + pose 生成局部地形图 |
| `PGO` | `registered_scan`, `odometry` | `corrected_odometry`, `global_map`, `pgo_tf` | 修正位姿，给地形模块和 planner 更稳定的 pose |
| `TerrainMapExt` | `registered_scan`, `odometry`, `terrain_map` | `terrain_map_ext` | 扩展地形图，范围更大、衰减更慢 |
| `SimplePlanner` | `terrain_map_ext`, `terrain_map`, `goal` | `way_point`, `goal_path`, `costmap_cloud` | 内部生成 2D costmap 并跑 A* |
| `FarPlanner` | `terrain_map_ext`, `terrain_map`, `registered_scan`, `odometry`, `goal` | `way_point`, `goal_path`, graph/debug outputs | 原生远程规划器 |
| `LocalPlanner` | `registered_scan`, `odometry`, `terrain_map`, `way_point` | `path`, `slow_down`, `goal_reached` | 近场避障和局部路径 |
| `PathFollower` | `path`, `odometry`, `slow_down` | `cmd_vel` | 把路径转换为速度命令 |

Nav Stack 可以记成一句话：

```text
Nav Stack 的 costmap 不是独立模块产物：
registered_scan + odometry -> terrain maps -> planner-internal cost
```

## FastLio2 Integration

这一章专门说明：**DimOS 是怎么把 FAST-LIO2 适配成一个可被 blueprint 组合、可被 Python 直接调用的模块**。

可以把它看成四层：

```text
FAST-LIO-NON-ROS + Livox SDK2
  -> dimos native binary: fastlio2_native
  -> Python NativeModule wrapper: FastLio2
  -> blueprint / autoconnect / remapping
```

### What DimOS Wraps

DimOS 这里封装的不是 ROS 版 FAST-LIO，而是：

```text
FAST-LIO-NON-ROS
  + Livox SDK2
  + dimos LCM message publishing
```

编译入口在：

- [`dimos/hardware/sensors/lidar/fastlio2/module.py`](/dimos/hardware/sensors/lidar/fastlio2/module.py)
- [`dimos/hardware/sensors/lidar/fastlio2/cpp/main.cpp`](/dimos/hardware/sensors/lidar/fastlio2/cpp/main.cpp)

`fastlio2_native` 会把 Livox SDK2 直接绑定到 `FAST-LIO-NON-ROS`，然后把注册点云和里程计发布成 DimOS/LCM 消息。

所以从 DimOS 的角度看，FAST-LIO 不是一个外部 ROS 节点，而是一个被 DimOS 自己启动和管理的 native module。

### Python Wrapper

Python 层的入口是 `FastLio2`：

```python
class FastLio2(NativeModule, perception.Lidar, perception.Odometry, mapping.GlobalPointcloud):
    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    global_map: Out[PointCloud2]
```

这个类做了三件事：

1. 声明模块能力和 stream 接口
   `lidar`、`odometry`、`global_map` 这三个输出会被其他模块自动连接。

2. 声明如何构建和启动 native binary
   `FastLio2Config` 里写明：

```python
cwd: str | None = "cpp"
executable: str = "result/bin/fastlio2_native"
build_command: str | None = "nix build .#fastlio2_native"
```

3. 把 Python 侧配置转换成 C++ 启动参数
   `model_post_init()` 会把：

- `config` 解析成绝对路径 `config_path`
- `mount: Pose` 转成 `init_pose = [x, y, z, qx, qy, qz, qw]`

`FastLio2.start()` 还做了两件运行时工作：

- `_validate_network()`：检查 `host_ip` 是否真的在本机网卡上，并尝试绑定 UDP socket
- 订阅自己的 `odometry` 输出，把它转成 TF 发布出去

因此 Python wrapper 不是算法主体，它更像一个“模块描述 + 启动管理 + 输出适配层”。

### Field Initialization Path

这一节专门回答两个问题：

- `FastLio2Config` 的字段从哪来
- 这些字段是怎么一步步传到 C++ 里的

先看 `FastLio2` 的定义：

```python
class FastLio2(NativeModule, perception.Lidar, perception.Odometry, mapping.GlobalPointcloud):
    config: FastLio2Config

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    global_map: Out[PointCloud2]
```

这里的 `config: FastLio2Config` 就决定了：`FastLio2.blueprint(...)` 接收的配置字段，最终都要由 `FastLio2Config` 来解释。

`FastLio2Config` 的字段来源分三层：

| 来源 | 例子 | 作用 |
|------|------|------|
| `FastLio2Config` 自己定义 | `host_ip`、`lidar_ip`、`frequency`、`mount`、`config`、`map_freq` | FAST-LIO2/Livox 业务参数 |
| 继承自 `NativeModuleConfig` | `executable`、`build_command`、`cwd`、`extra_args`、`cli_exclude` | native 进程启动控制 |
| 继承自 `ModuleConfig` | `frame_id`、`g`、RPC/TF 相关字段 | 普通模块通用配置 |

因此，字段不是“凭空出现”的，而是 Python config class 继承链的一部分。

真正初始化发生在模块实例化时，而不是调用 `blueprint()` 的那一刻。链路是：

```text
FastLio2.blueprint(host_ip="192.168.123.164", config="default.yaml", ...)
  -> Blueprint.create(FastLio2, **kwargs)
  -> 只保存 kwargs，不创建模块
  -> coordinator 创建 FastLio2(config_args=kwargs)
  -> Configurable.__init__()
  -> FastLio2Config(**kwargs)
```

所以 `blueprint(...)` 更像“先把参数记下来”，真正的 config 构造在模块 build/deploy 阶段才发生。

以这个调用为例：

```python
FastLio2.blueprint(
    host_ip="192.168.123.164",
    lidar_ip="192.168.123.120",
    mount=G1.internal_odom_offsets["mid360_link"],
    map_freq=1.0,
    config="default.yaml",
)
```

Pydantic 会按下面的规则构造 `FastLio2Config`：

1. `host_ip`、`lidar_ip`、`map_freq` 直接覆盖默认值
2. `config="default.yaml"` 先按 `Path` 解析
3. `mount=Pose(...)` 保留成 Python `Pose` 对象
4. 其他没传的字段，例如 `msr_freq`、`pointcloud_freq`，继续使用类里的默认值
5. 如果传入未知字段，例如 `foo=1`，会因为 `extra="forbid"` 直接报错

构造完成后，`FastLio2Config.model_post_init()` 还会做一次“派生字段”计算：

```python
cfg = self.config
if not cfg.is_absolute():
    cfg = _CONFIG_DIR / cfg
self.config_path = str(cfg.resolve())

m = self.mount
self.init_pose = [
    m.x, m.y, m.z,
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w,
]
```

也就是说：

- `config` 是给 Python 用的原始 YAML 路径字段
- `config_path` 是真正传给 C++/FAST-LIO 的绝对路径
- `mount` 是给 Python 用的结构化 `Pose`
- `init_pose` 是真正传给 C++ 的扁平化数值数组

这里要注意两种不同机制：

- `config="default.yaml"` 不是把 YAML 里面每个字段都由 DimOS 拆开再传给 FAST-LIO
- 它只是先在 Python 里解析成绝对路径 `config_path`
- 然后把这个路径交给 native/FAST-LIO 一侧继续读取配置文件

而像 `host_ip`、`lidar_ip`、`frame_id` 这类字段，则是 DimOS 直接转成 CLI 参数，C++ wrapper 立即读取并赋给本地变量。

所以 native binary 启动时拿到的是两类东西：一类是“直接可用的标量参数”，另一类是“配置文件路径”。

这也是为什么 `FastLio2Config` 里有：

```python
cli_exclude: frozenset[str] = frozenset({"config", "mount"})
```

因为传给 C++ 的不是：

```text
--config default.yaml
--mount <Pose object>
```

而是：

```text
--config_path /abs/path/to/default.yaml
--init_pose x,y,z,qx,qy,qz,qw
```

接着，`NativeModule.start()` 会把它们组装成真正的启动命令。可以把 FastLio2 理解成大概会生成：

```bash
result/bin/fastlio2_native \
  --lidar /registered_scan#sensor_msgs.PointCloud2 \
  --odometry /odometry#nav_msgs.Odometry \
  --global_map /global_map_fastlio#sensor_msgs.PointCloud2 \
  --host_ip 192.168.123.164 \
  --lidar_ip 192.168.123.120 \
  --frequency 10.0 \
  --frame_id odom \
  --child_frame_id body \
  --map_freq 1.0 \
  --config_path /.../fastlio2/config/default.yaml \
  --init_pose 0.0,0.0,0.0,0.0,0.0,0.0,1.0
```

这里要注意，FastLio2 里其实有两种完全不同的参数传递方式：

- `--lidar`、`--odometry`、`--global_map` 来自 stream topic，不来自 config 字段
- `--host_ip`、`--lidar_ip`、`--frame_id`、`--init_pose` 这类是 DimOS 直接传标量值
- `--config_path` 这类只传“配置文件路径”，真正的配置项由 FAST-LIO 内部继续读取

所以 FastLio2 的参数传递，本质上是两部分合并：

```text
autoconnect 决定 topic 参数
FastLio2Config 决定业务配置参数
NativeModule.start() 把两者拼成同一个 argv
```

### Native Runtime

真正的 FAST-LIO 工作主体在 [`dimos/hardware/sensors/lidar/fastlio2/cpp/main.cpp`](/dimos/hardware/sensors/lidar/fastlio2/cpp/main.cpp)。

启动时，`fastlio2_native` 会从 `NativeModule` CLI 读取这些关键参数：

```text
--lidar
--odometry
--global_map
--config_path
--host_ip
--lidar_ip
--frame_id
--child_frame_id
--init_pose
```

这里要把两类参数分开看。

第一类是 **topic 参数**，对应 Python wrapper 里声明的 `Out[T]` 端口：

```cpp
g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
g_odometry_topic = mod.has("odometry") ? mod.topic("odometry") : "";
g_map_topic = mod.has("global_map") ? mod.topic("global_map") : "";
```

它们对应的是：

| Python 端口名 | C++ 读取方式 | 含义 |
|--------------|--------------|------|
| `lidar` | `mod.topic("lidar")` | 注册点云输出 topic |
| `odometry` | `mod.topic("odometry")` | 里程计输出 topic |
| `global_map` | `mod.topic("global_map")` | 累积地图输出 topic |

第二类是 **配置参数**，但这里内部又分两种：

- 一种是 **直接赋值型参数**：C++ wrapper 一读到就立刻用
- 一种是 **配置文件路径型参数**：只把 YAML 路径继续传给 FAST-LIO 内核

源码上可以直接看到这两种分流：

```cpp
std::string config_path = mod.arg("config_path", "");
double msr_freq = mod.arg_float("msr_freq", 50.0f);
std::string host_ip = mod.arg("host_ip", "192.168.1.5");
g_frame_id = mod.arg_required("frame_id");
std::string init_str = mod.arg("init_pose", "");
```

它们对应的是：

| Python 字段 | CLI 形式 | C++ 读取方式 |
|-------------|----------|---------------|
| `host_ip` | `--host_ip 192.168...` | `mod.arg("host_ip", ...)`，wrapper 直接使用 |
| `msr_freq` | `--msr_freq 50.0` | `mod.arg_float("msr_freq", ...)`，再传给 `FastLio(...)` |
| `frame_id` | `--frame_id odom` | `mod.arg_required("frame_id")`，wrapper 直接使用 |
| `config_path` | `--config_path /abs/path.yaml` | `mod.arg("config_path", "")`，再交给 `FastLio(config_path, ...)` |
| `init_pose` | `--init_pose x,y,z,qx,qy,qz,qw` | `mod.arg("init_pose", "")` 后手动拆分 |

其中 `config_path` 和 `host_ip` 的语义是不一样的：

```text
host_ip
  -> DimOS 传一个具体字符串
  -> C++ wrapper 直接赋给 host_ip 变量并立即使用

config_path
  -> DimOS 只传 YAML 文件路径
  -> C++ wrapper 读到路径
  -> 再把这条路径交给 FastLio(config_path, ...)
  -> FAST-LIO 内部再根据这个文件读取自身参数
```

所以 `config="default.yaml"` 更准确的理解是：

```text
DimOS 负责定位配置文件
FAST-LIO 负责消费配置文件内容
```

这说明 C++ 侧并没有一个和 Python 自动同步生成的 `FastLio2Config` 类。它是按名字逐个取值：

- topic 用 `topic("name")`
- 字符串参数用 `arg("name")`
- 浮点数用 `arg_float("name")`
- 整数用 `arg_int("name")`
- 布尔值用 `arg_bool("name")`

所以 Python/C++ 适配是否成功，核心不是“共享了同一个配置类”，而是：

```text
Python 发出的 CLI key
==
C++ 读取时写的 key
```

这里最关键的适配点有三处。

第一，输出 topic 不是写死的，而是从 `NativeModule` 参数里取：

```cpp
g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
g_odometry_topic = mod.has("odometry") ? mod.topic("odometry") : "";
g_map_topic = mod.has("global_map") ? mod.topic("global_map") : "";
```

这意味着 C++ 不需要知道 blueprint 怎么 remap，也不需要知道下游模块是谁；它只管往 Python 给它的 topic 发布。

第二，Livox 数据不是 ROS 驱动转出来的，而是 SDK2 直接回调：

```cpp
SetLivoxLidarPointCloudCallBack(on_point_cloud, nullptr);
SetLivoxLidarImuDataCallback(on_imu_data, nullptr);
SetLivoxLidarInfoChangeCallback(on_info_change, nullptr);
```

`on_point_cloud()` 会把 `LivoxLidarEthernetPacket` 解成 `custom_messages::CustomPoint`，`on_imu_data()` 会解成 `custom_messages::Imu`，然后喂给 `FastLio`：

```cpp
g_fastlio->feed_imu(imu_msg);
fast_lio.feed_lidar(lidar_msg);
```

第三，C++ 最终把 FAST-LIO 结果转成 DimOS 消息并发布：

```cpp
publish_lidar(filtered, ts);
publish_odometry(fast_lio.get_odometry(), ts);
publish_lidar(map_cloud, ts, g_map_topic);
```

对应的 DimOS 输出语义是：

| Output | Type | Meaning |
|--------|------|---------|
| `lidar` | `PointCloud2` | 注册后的世界系点云 |
| `odometry` | `Odometry` | FAST-LIO 输出的连续里程计 |
| `global_map` | `PointCloud2` | 可选累计体素地图，只有 `map_freq > 0` 时才有意义 |

另外，`init_pose` 也在 C++ 里真正生效：它会同时作用在发布的点云和 odometry 上，所以传感器安装位姿和初始坐标系偏移不是文档参数，而是会进入最终输出坐标系的。

### How It Plugs Into Blueprints

`FastLio2` 被接入 DimOS 的方式和普通模块一样：直接进 `autoconnect()`。

例如 G1 的 onboard blueprint：

```python
_unitree_g1_onboard = autoconnect(
    FastLio2.blueprint(
        host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
        lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
        mount=G1.internal_odom_offsets["mid360_link"],
        map_freq=1.0,
        config="default.yaml",
    ).remappings([(FastLio2, "global_map", "global_map_fastlio")]),
    G1HighLevelDdsSdk.blueprint(),
    unitree_g1_vis,
)
```

这里有三个适配动作：

- `FastLio2.blueprint(...)`：把硬件网络、YAML、安装位姿这些都配置给 Python wrapper
- `.remappings([(FastLio2, "global_map", "global_map_fastlio")])`：避免 `global_map` 和其他模块冲突
- `autoconnect(...)`：把 `lidar`、`odometry`、`global_map_fastlio` 自动接到下游模块

如果接到 Nav Stack，通常再补一个 remap：

```python
.remappings([
    (FastLio2, "lidar", "registered_scan"),
])
```

因为 Nav Stack 需要的输入名是 `registered_scan`，而 `FastLio2` 自己输出名叫 `lidar`。

因此“DimOS 适配 FAST-LIO”这件事，本质上是：

- 用 `NativeModule` 把 FAST-LIO2 native binary 生命周期纳入 DimOS
- 用 Python wrapper 把它声明成标准模块接口
- 用 `autoconnect()` 和 `remappings()` 把它接入不同导航栈

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
