# Navigation 中的 LCM

这篇文档从 navigation blueprint 的角度解释 DimOS 里的 LCM：模块是怎么启动的，`In[T]` / `Out[T]` 怎么被接起来，`LCMTransport` / `pLCMTransport` 和共享内存是什么关系，以及 `unitree_g1_onboard.py` 这类机器人栈里数据到底怎么流。

如果只记一句话：

```text
Blueprint 负责把模块接成图，LCM 是默认的数据总线，worker 进程负责承载模块运行。
```

## 关键结论

- DimOS 模块默认部署到 Python worker 子进程，不是简单地在主进程里开线程。
- worker 使用 `multiprocessing` 的 `forkserver` context 启动，避免 CUDA、native library、GIL 竞争等问题互相污染。
- 一个 worker 进程可以承载多个模块；设置 `dedicated_worker = True` 的模块会独占 worker。
- 模块内部仍然可以自己使用线程、asyncio loop 或 native subprocess。
- 模块间数据通信使用 typed stream：`In[T]` / `Out[T]`。
- 默认 transport 是 `LCMTransport` 或 `pLCMTransport`。
- 共享内存不是默认 LCM 通信的一部分，需要显式使用 `SHMTransport` / `pSHMTransport` / `JpegShmTransport`。
- `NativeModule` 会在 Python worker 中启动一个额外原生进程，并把 LCM topic 通过 CLI 或 stdin config 传给原生程序。

## 以 G1 onboard 为例

源码：[`unitree_g1_onboard.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/robot/unitree/g1/blueprints/primitive/unitree_g1_onboard.py)

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
).global_config(n_workers=12, robot_model="unitree_g1")
```

这张 blueprint 主要包含三类模块：

- `FastLio2`: LiDAR SLAM。它是 `NativeModule`，Python 侧是 wrapper，真正 SLAM 在 native executable 中跑。
- `G1HighLevelDdsSdk`: G1 高层运动控制。它是 Python module，内部使用 Unitree DDS SDK。
- `unitree_g1_vis`: 可视化相关 blueprint，订阅 odometry、pointcloud、map、path 等 stream。

运行形态可以理解成：

```text
dimos run / ModuleCoordinator 主进程
  -> Python worker pool，n_workers=12
       -> FastLio2 Python wrapper
            -> fastlio2_native 原生子进程
       -> G1HighLevelDdsSdk Python module
       -> visualization modules
```

所以 `FastLio2` 这里有两层进程：

```text
Python worker process
  -> NativeModule wrapper
       -> subprocess.Popen(...) 启动 native executable
```

而 `G1HighLevelDdsSdk` 主要在 Python worker 里运行，并通过 Unitree DDS SDK 和机器人通信。

## 启动流程

真正把 blueprint 变成运行中系统的是 `ModuleCoordinator.build()`。

源码：[`module_coordinator.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/core/coordination/module_coordinator.py)

大致流程：

```text
ModuleCoordinator.build(blueprint)
  -> 应用 GlobalConfig / blueprint args
  -> 启动 WorkerManagerPython
  -> 创建 n_workers 个 PythonWorker 子进程
  -> 并行部署 blueprint 中的模块
  -> 根据 stream name + type 自动连接 transport
  -> 注入 Spec / ModuleRef 依赖
  -> 并行调用所有模块的 build()
  -> 并行调用所有模块的 start()
```

对应核心代码：

```python
coordinator.start()
_deploy_all_modules(blueprint, coordinator, global_config, blueprint_args)
coordinator._connect_streams(blueprint)
_connect_module_refs(blueprint, coordinator)

coordinator.build_all_modules()
coordinator.start_all_modules()
```

`WorkerManagerPython.start()` 会启动 worker pool：

```python
for _ in range(self._n_workers):
    worker = PythonWorker()
    worker.start_process()
    self._workers.append(worker)
```

`PythonWorker.start_process()` 使用 forkserver：

```python
ctx = multiprocessing.get_context("forkserver")
parent_conn, child_conn = ctx.Pipe()
self._process = ctx.Process(
    target=_worker_entrypoint,
    args=(child_conn, self._worker_id),
    daemon=True,
)
self._process.start()
```

这意味着模块实例是在 worker 子进程里创建的，而不是在 coordinator 主进程里创建的。

## 进程、线程和 asyncio 的关系

DimOS 的默认隔离单位是 worker 进程。模块部署后，coordinator 拿到的是模块 proxy，真正实例在 worker 中。

控制调用路径大致是：

```text
coordinator / proxy
  -> multiprocessing Pipe
  -> worker process
  -> module.start() / module.stop() / @rpc method
```

模块内部还有一个 asyncio loop thread。`ModuleBase.__init__()` 会调用 `get_loop()`，如果当前线程没有 running loop，就创建一个新 event loop，并在 daemon thread 中 `run_forever()`。

所以模块内部的 async handler 会跑在该模块自己的 loop 上：

```text
worker process
  -> module instance
       -> asyncio event loop thread
       -> optional user threads
       -> optional native subprocess
```

例如 `G1HighLevelDdsSdk` 使用 `threading.Timer` 做 velocity timeout；`NativeModule` 使用 watchdog thread 监控原生进程 stdout/stderr 和退出状态。

## Stream 如何自动连接

模块通过类型标注声明数据接口：

```python
class FastLio2(NativeModule):
    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    global_map: Out[PointCloud2]
```

```python
class G1HighLevelDdsSdk(Module):
    cmd_vel: In[Twist]
```

`BlueprintAtom.create()` 会读取这些 annotations，提取 `In[T]` / `Out[T]`。随后 `ModuleCoordinator._connect_streams()` 按下面的 key 聚合：

```text
(remapped_stream_name, message_type)
```

也就是说，只有名字和类型都相同的 stream 才会共享同一个 transport。

例子：

```python
class A(Module):
    odometry: Out[Odometry]

class B(Module):
    odometry: In[Odometry]
```

它们会被连接到同一个 transport。

如果名字相同但类型不同，比如：

```text
odometry: Out[Odometry]
odometry: In[PoseStamped]
```

blueprint build 阶段会报 stream type conflict。

## Remapping 的作用

自动连接依赖 stream name，但不同模块的自然命名可能不一样，或者同名流需要避让。`unitree_g1_onboard.py` 中：

```python
.remappings([(FastLio2, "global_map", "global_map_fastlio")])
```

含义是：

```text
FastLio2.global_map
  -> 在这张 blueprint 图里改名为 global_map_fastlio
```

这样可视化或下游模块可以订阅 `global_map_fastlio`，同时避免它和其他模块的 `global_map` 混在一起。

## 默认 Transport 选择

默认 transport 选择逻辑在 `_get_transport_for()`：

```python
use_pickled = getattr(stream_type, "lcm_encode", None) is None
topic = f"/{name}" if _is_name_unique(blueprint, name) else f"/{short_id()}"
transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, stream_type)
```

规则：

- 如果消息类型有 `lcm_encode()`，使用 `LCMTransport`。
- 如果没有 `lcm_encode()`，使用 `pLCMTransport`。
- 如果 stream name 在图中唯一，topic 默认是 `/{name}`。
- 如果 stream name 不唯一，topic 会变成短随机名，避免误接。

所以导航里常见消息：

- `PointCloud2`
- `Odometry`
- `Path`
- `Pose`
- `PoseStamped`
- `Twist`

通常会走 `LCMTransport`，因为这些消息有 LCM 编码。

## `LCMTransport`

源码：[`transport.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/core/transport.py)

`LCMTransport` 用于有 LCM schema / LCM 编码能力的消息：

```text
PointCloud2 / Odometry / Path / Twist
  -> msg.lcm_encode()
  -> LCM pub/sub
  -> MsgType.lcm_decode(...)
  -> Python message object
```

优点：

- 消息 schema 稳定
- 二进制格式紧凑
- Python / C++ / TypeScript / Lua 等多语言互通方便
- native module 可以直接用同一个 topic 收发消息

这也是 navigation stack 默认偏向使用 LCM 消息类型的原因。

## `pLCMTransport`

`pLCMTransport` 的 `p` 是 pickle。它用于没有 `lcm_encode()` 的 Python 对象。

数据路径：

```text
任意 Python 对象
  -> pickle.dumps(...)
  -> LCM pub/sub
  -> pickle.loads(...)
  -> Python 对象
```

优点是方便，缺点是：

- 不适合跨语言
- schema 不稳定
- payload 和性能通常不如原生 LCM 消息
- 更适合 Python 内部调试或临时对象流

在 navigation 主链路里，优先使用 `dimos.msgs.*` 这类支持 LCM 编码的消息。

## 共享内存和 LCM 的关系

LCM 和共享内存是两套 transport 机制。

默认：

```text
In[T] / Out[T]
  -> LCMTransport 或 pLCMTransport
```

显式指定时才会使用共享内存：

```text
In[T] / Out[T]
  -> SHMTransport / pSHMTransport / JpegShmTransport
```

共享内存实现适合大 payload，例如高频图像、大点云、压缩图像帧。共享内存 pub/sub 的基本机制是：

```text
publisher 写入 shared memory frame
subscriber topic fanout thread 读取 frame
callback 收到 decoded message
```

相关实现：

- [`SHMTransport`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/core/transport.py)
- [`shmpubsub.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/protocol/pubsub/impl/shmpubsub.py)

在 `unitree_g1_onboard.py` 里没有显式 `.transports(...)`，因此该 blueprint 自身的 stream 默认会走 LCM / pLCM。更大的导航 blueprint 可以根据需要显式覆盖某些大流量 stream 的 transport。

## Python 模块里如何收发消息

模块代码通常不直接调用 LCM API，而是使用 stream：

```python
self.cmd_vel.subscribe(self.move)
self.odometry.publish(odom)
```

`Out.publish()` 背后会调用：

```python
self._transport.broadcast(self, msg)
```

`In.subscribe()` 背后会调用：

```python
self.transport.subscribe(cb, self)
```

所以模块作者只关心 typed message，不关心底层 transport。

`G1HighLevelDdsSdk.start()` 里有一个典型例子：

```python
if self.cmd_vel._transport is not None:
    self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
```

含义是：

```text
收到 cmd_vel: Twist
  -> 调用 self.move(twist)
  -> Unitree LocoClient.Move / SetVelocity
  -> 机器人执行运动
```

## NativeModule 里如何使用 LCM

`NativeModule` 的作用是：让 C/C++/Rust 等原生程序也能作为 DimOS 模块接入 blueprint。

Python wrapper 声明 stream：

```python
class FastLio2(NativeModule):
    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    global_map: Out[PointCloud2]
```

blueprint build 时，这些 stream 已经被分配好 transport。`NativeModule.start()` 会收集 topic：

```python
topics = self._collect_topics()
```

然后把 topic 作为 CLI 参数传给原生程序：

```python
cmd = [self.config.executable]
for name, topic_str in topics.items():
    cmd.extend([f"--{name}", topic_str])
```

所以 native executable 实际启动时类似：

```text
fastlio2_native
  --lidar /lidar
  --odometry /odometry
  --global_map /global_map_fastlio
  --host_ip 192.168.123.164
  --lidar_ip 192.168.123.120
```

原生程序再直接用 LCM 订阅或发布这些 topic。这样 Python 模块和 native 模块共享同一张数据图。

## G1 onboard 的数据流

`unitree_g1_onboard.py` 的核心链路可以简化成：

```text
Livox Mid-360
  -> FastLio2 native subprocess
       -> lidar: PointCloud2
       -> odometry: Odometry
       -> global_map_fastlio: PointCloud2

unitree_g1_vis
  <- lidar / odometry / global_map_fastlio
  -> Rerun visualization

cmd_vel: Twist
  -> G1HighLevelDdsSdk
       -> Unitree DDS SDK
       -> G1 robot
```

注意这里有两种 DDS/LCM 概念容易混：

- DimOS 模块间默认数据流：LCM / pLCM。
- G1 机器人控制 SDK：Unitree DDS SDK，用来和机器人底层通信。

也就是说，`G1HighLevelDdsSdk` 是 DimOS 模块图中的一个 Python module，但它内部把收到的 `cmd_vel` 转成 Unitree DDS 命令发给机器人。

## RPC 和 Stream 的区别

Stream 用来持续传数据：

```text
odometry
pointcloud
cmd_vel
path
costmap
```

RPC 用来调用方法：

```python
@rpc
def start(self) -> None: ...

@rpc
def stop(self) -> None: ...

@rpc
def move(self, twist: Twist, duration: float = 0.0) -> bool: ...
```

coordinator 调用模块 RPC 时，常见路径是：

```text
coordinator proxy
  -> multiprocessing Pipe
  -> worker process
  -> real module method
```

模块也会启动自己的 RPC transport，例如 LCM RPC，用于外部工具或其他模块远程调用。

因此可以这样区分：

```text
Stream:
  高频 / 连续 / pub-sub 数据流

RPC:
  低频 / 请求响应 / 控制调用
```

## 调试建议

查看正在跑的 blueprint：

```bash
dimos status
```

查看日志：

```bash
dimos log
dimos log -f
```

查看 LCM topic：

```bash
dimos lcmspy
```

查看某个 topic 内容：

```bash
dimos topic echo /odometry
```

手动发送 topic：

```bash
dimos topic send /cmd_vel "Twist(...)"
```

如果是 MCP-enabled blueprint，也可以查看 MCP 状态：

```bash
dimos mcp status
dimos mcp modules
```

## 心智模型

可以把 DimOS navigation 的运行方式压缩成三层：

```text
Blueprint:
  描述模块图，声明哪些 stream 该自动连接

Worker / ModuleCoordinator:
  启动 worker 进程，部署模块，调用 build/start/stop

Transport:
  负责真正传数据，默认 LCM/pLCM，需要时可换成 SHM/ROS/DDS
```

再短一点：

```text
Blueprint 负责接线
Worker 负责运行
LCM 负责送消息
```
