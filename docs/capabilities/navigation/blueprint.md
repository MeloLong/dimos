# Blueprint 机制

DimOS 的 `Blueprint` 是一张“可部署的模块图”。它不直接运行算法，也不直接启动进程，而是描述：

- 这个系统由哪些 `Module` 组成
- 每个模块启动时需要哪些配置参数
- 每个模块声明了哪些 `In[T]` / `Out[T]` stream
- 哪些 stream 需要被重命名或指定特殊 transport
- 哪些模块之间存在 RPC/Spec 依赖
- 运行前需要检查哪些系统条件

真正把 `Blueprint` 变成运行中系统的是 `ModuleCoordinator.build()`。因此，像 `unitree_g1_nav_simple.py` 这样的文件通常只是定义一个 blueprint 变量，供 `dimos run unitree-g1-nav-simple` 加载；直接运行这个 Python 文件通常不会启动所有模块。

## 为什么需要 Blueprint

机器人系统不是一个单体程序，而是一组独立模块组成的数据流图。以导航为例，感知、建图、代价地图、全局规划、局部控制、可视化都可以是独立模块。它们应该能被替换、重组、禁用、复用，而不应该互相硬编码调用关系。

如果没有 `Blueprint`，每个机器人栈都需要手写很多样板逻辑：

```text
创建模块实例
创建 transport/topic
把 A 的输出接到 B 的输入
启动 worker
启动 native process
注入 RPC 依赖
按顺序 build/start/stop
处理配置覆盖
```

`Blueprint` 把这些事情拆成两层：

```text
模块作者：
  只在模块类里声明 In[T] / Out[T] / config / Spec

系统集成者：
  用 autoconnect(...) 组合模块
  用 remappings(...) 修正命名差异
  用 global_config(...) 调整运行环境
```

这样做的核心收益是：模块保持局部、可测试、可复用；系统组合保持显式、可读、可替换。

## 设计原理

### Blueprint 是不可变的系统描述

源码位置：`dimos/core/coordination/blueprints.py`。

`Blueprint` 和 `BlueprintAtom` 都是 frozen dataclass。一个 `BlueprintAtom` 对应一个模块节点：

```python
@dataclass(frozen=True)
class BlueprintAtom:
    kwargs: dict[str, Any]
    module: type[ModuleBase]
    streams: tuple[StreamRef, ...]
    module_refs: tuple[ModuleRef, ...]
```

一个 `Blueprint` 则包含多个 atom，以及 transport、remapping、global config 等图级别信息：

```python
@dataclass(frozen=True)
class Blueprint:
    blueprints: tuple[BlueprintAtom, ...]
    transport_map: Mapping[tuple[str, type], PubSubTransport[Any]]
    global_config_overrides: Mapping[str, Any]
    remapping_map: Mapping[tuple[type[ModuleBase], str], str | type[ModuleBase] | type[Spec]]
```

这种设计让 blueprint 更像配置值，而不是运行对象。组合 blueprint 时不会修改旧对象，而是返回一个新对象。这对机器人栈很重要，因为一个基础 blueprint 可以被多个更大的系统复用。

### 模块接口来自类型注解

每个模块通过类型注解声明自己的数据接口：

```python
class TerrainAnalysis(NativeModule):
    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: Out[PointCloud2]
```

`BlueprintAtom.create()` 会读取模块类的 annotations，提取 `In[T]` / `Out[T]`：

```python
if origin in (In, Out):
    direction = "in" if origin == In else "out"
    type_ = get_args(annotation)[0]
    streams.append(StreamRef(name=name, type=type_, direction=direction))
```

这意味着模块的接口不是额外维护的一份配置，而是直接来自源码中的类型声明。优点是：

- 模块接口和实现放在一起，不容易漂移
- autoconnect 可以基于名称和消息类型自动连接
- native module 也可以使用同一套 Python wrapper 接口
- 文档、CLI help、可视化图都可以从同一个结构中生成

### autoconnect 合并模块图

`autoconnect()` 的职责是把多个 `Blueprint` 合并成一张更大的图：

```python
unitree_g1_nav_simple = autoconnect(
    _unitree_g1_onboard,
    RayTracingVoxelMap.blueprint(...),
    CostMapper.blueprint(...),
    ReplanningAStarPlanner.blueprint(...),
    MovementManager.blueprint(),
    unitree_g1_vis,
)
```

它做的是静态合并：

- 合并所有 `BlueprintAtom`
- 合并 transport override
- 合并 global config override
- 合并 remapping
- 去除重复模块，后出现的模块覆盖先出现的模块

所以 `autoconnect()` 并不会启动模块。它只是得到一张更完整的系统描述。

### 连接规则是“名称 + 类型”

真正连接 stream 的逻辑在 `ModuleCoordinator._connect_streams()`：

```python
streams[remapped_name, conn.type].append((bp.module, conn.name))
```

也就是说，两个 stream 会被连接，是因为它们在 blueprint 中拥有相同的：

```text
(stream name, message type)
```

例如：

```python
class A(Module):
    terrain_map: Out[PointCloud2]

class B(Module):
    terrain_map: In[PointCloud2]
```

它们会共享同一个 transport。连接不是通过直接调用发生的，而是通过给这些 stream 设置同一个 transport：

```python
instance.set_transport(original_name, transport)
```

这种方式天然支持 fan-out：

```text
TerrainAnalysis.terrain_map
  -> LocalPlanner.terrain_map
  -> TerrainMapExt.terrain_map
  -> SimplePlanner.terrain_map
```

一个输出可以被多个输入消费，模块之间仍然不需要知道彼此存在。

### remapping 解决命名差异

自动连接依赖 stream 名称，但不同模块的命名不一定完全一致。`remappings()` 用来把某个模块的端口名称映射成图里的另一个名字。

例如 G1 nav onboard 中：

```python
.remappings([
    (FastLio2, "lidar", "registered_scan"),
    (FastLio2, "global_map", "global_map_fastlio"),
    (MovementManager, "way_point", "_mgr_way_point_unused"),
])
```

含义是：

- `FastLio2.lidar` 在这张图里当作 `registered_scan`
- `FastLio2.global_map` 避免和其他 `global_map` 冲突
- `MovementManager.way_point` 被挪到 unused stream，避免抢 planner 的 `way_point`

remapping 的价值在于：模块可以保持自己的自然命名，而系统集成时再做适配。

### transport 是运行时通信层

Blueprint 本身只知道 stream 的名称和类型；真正通信使用哪个 transport，在 build 时确定。

默认逻辑在 `_get_transport_for()`：

```python
use_pickled = getattr(stream_type, "lcm_encode", None) is None
topic = f"/{name}" if _is_name_unique(blueprint, name) else f"/{short_id()}"
transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, stream_type)
```

如果消息类型支持 LCM 编码，就使用 `LCMTransport`；否则使用 `pLCMTransport`。也可以通过 `.transports(...)` 显式指定，比如图像点云走 shared memory。

这让模块只关心 typed stream，不关心底层是 LCM、SHM、ROS bridge、DDS 还是别的 transport。

### ModuleRef 处理 RPC 依赖

Blueprint 不只连接数据流，也会处理模块之间的 RPC 依赖。

如果模块声明：

```python
class MySkillContainer(Module):
    _navigator: NavigatorSpec
```

`BlueprintAtom.create()` 会把它识别成 `ModuleRef`。随后 `_connect_module_refs()` 会在 blueprint 中寻找满足该 Spec 的模块，并把目标模块 proxy 注入到当前模块：

```python
setattr(base_instance, ref_name, target_instance)
base_instance.set_module_ref(ref_name, target_instance)
```

所以 Blueprint 管理两类关系：

```text
In/Out stream:
  数据流，pub/sub 关系

ModuleRef/Spec:
  控制流，RPC 调用关系
```

这两个关系被分开建模，避免把高频数据流和低频控制调用混在一起。

## 从 Blueprint 到运行系统

`dimos run <blueprint-name>` 会从 `dimos/robot/all_blueprints.py` 找到对应变量，然后调用：

```python
ModuleCoordinator.build(blueprint, kwargs)
```

build 的核心流程是：

```text
1. 应用 global_config override
2. 运行 configurator 和 requirement checks
3. 校验 stream 名称/类型冲突
4. 部署所有模块到 worker
5. 为同名同类型 stream 分配 transport
6. 注入 ModuleRef / Spec 依赖
7. 调用所有模块 build()
8. 调用所有模块 start()
9. 输出/可视化 blueprint graph
```

源码里的顺序是：

```python
_deploy_all_modules(...)
coordinator._connect_streams(blueprint)
_connect_module_refs(blueprint, coordinator)
coordinator.build_all_modules()
coordinator.start_all_modules()
```

这解释了为什么一个 blueprint 文件本身通常不能启动机器人。文件里的变量只是“图定义”；启动逻辑在 CLI 和 `ModuleCoordinator`。

## Blueprint 的作用

Blueprint 在 DimOS 中承担的是系统集成层角色。

它让模块作者可以专注在单个模块：

```python
class MyModule(Module):
    scan: In[PointCloud2]
    odometry: In[Odometry]
    result: Out[Path]
```

也让系统作者可以专注在组合：

```python
my_robot_nav = autoconnect(
    robot_base,
    perception_stack,
    planner_stack,
    visualization_stack,
).remappings([...]).global_config(...)
```

它的作用可以总结为：

- **声明系统结构**：哪些模块组成一个机器人 stack
- **推导数据连接**：基于 stream 名称和消息类型自动连线
- **集中配置**：模块 config、global config、CLI override 都能进入同一张图
- **隔离模块实现**：模块不需要知道消费者是谁、生产者是谁
- **支持替换和复用**：同一模块可用于不同机器人，同一子图可嵌入更大系统
- **支持 native module**：C++ binary 通过 Python wrapper 暴露同样的 `In[T]` / `Out[T]`
- **支持运行前校验**：在启动 worker 和硬件前发现 stream 类型冲突、配置错误、系统环境问题

## 特点

### 组合优先

Blueprint 可以嵌套组合。一个 robot blueprint 可以包含 sensor blueprint、nav stack blueprint、visualization blueprint。`autoconnect()` 会把它们摊平成一张图。

### 类型驱动

stream 连接必须同时匹配名称和消息类型。这样可以避免两个同名但语义不同的 stream 被错误连接。

### 不可变

`.remappings()`、`.global_config()`、`.transports()` 都返回新的 `Blueprint`，不会原地修改旧对象。这使得基础 blueprint 可以安全复用。

### 晚绑定

Blueprint 定义时不会创建 worker，不会打开硬件，不会启动 native process。真正运行发生在 `ModuleCoordinator.build()`。这让 blueprint 可以被 CLI、测试、可视化、配置检查共同使用。

### 模块解耦

模块只声明自己的接口，不声明“我要连接哪个具体模块”。这让一个模块可以被放进很多不同图里。

### 对 native 和 Python 模块统一

Python 模块在 `start()` 中订阅 `In[T]`；native module 则由 Python wrapper 收集 topic 参数并传给 C++ binary。对 Blueprint 来说，两者都是声明了 `In[T]` / `Out[T]` 的模块节点。

## 心智模型

可以把 Blueprint 理解成机器人系统的电路图：

```text
Module class
  -> 通过类型注解暴露引脚 In[T] / Out[T]

BlueprintAtom
  -> 一个模块节点 + 初始化参数 + 引脚列表

autoconnect()
  -> 把多个节点放到一张图里

ModuleCoordinator.build()
  -> 给同名同类型引脚接线
  -> 把模块部署到 worker
  -> 启动整个系统
```

所以，Blueprint 不是“程序入口”，而是“系统结构”。`dimos run` 才是把这张结构图变成运行中机器人系统的入口。
