# LCM in DimOS

DimOS uses [LCM (Lightweight Communications and Marshalling)](https://github.com/lcm-proj/lcm) as its default inter-process pub/sub mechanism.

如果只记一句话，可以记成：

```text
LCM 是底层消息总线
LCMTransport 是“强类型 LCM 消息”
pLCMTransport 是“pickle 后走 LCM”
```

在 DimOS 里，模块作者通常只写：

- `In[T]`
- `Out[T]`

真正的数据如何跨进程流动，通常由 blueprint build 时自动分配的 transport 决定，而默认 transport 往往就是 `LCMTransport` 或 `pLCMTransport`。

## LCM 是什么

LCM 是一个轻量级 pub/sub 通信系统，常见于机器人和实时系统。它的核心特点是：

- 基于 topic/channel 的发布订阅模型
- 消息是二进制编码
- 适合多进程、多语言模块通信
- 默认实现偏本地/局域网低延迟消息分发

在 DimOS 里，LCM 的作用不是“框架入口”，而是模块图中的默认数据总线。模块之间不直接互相调用发送传感器数据，而是：

```text
producer module
  -> publish(topic, message)
  -> LCM channel
  -> subscriber module callback
```

## DimOS 里的两种 LCM transport

源码位置：[`dimos/core/transport.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/core/transport.py:79)

### `LCMTransport`

`LCMTransport` 用于 **消息类型本身支持 LCM 编码** 的情况。

它要求消息类型具备：

- `lcm_encode()`
- `lcm_decode()`

它内部包装的是 `dimos.protocol.pubsub.impl.lcmpubsub.LCM`，也就是“真正的 LCM pubsub”：

```python
class LCMTransport(PubSubTransport[T]):
    def __init__(self, topic: str, type: type, **kwargs) -> None:
        super().__init__(LCMTopic(topic, type))
        self.lcm = LCM(**kwargs)
```

发送时会调用消息对象自己的 `lcm_encode()`：

```python
def encode(self, msg: DimosMsg | bytes, _: LCMTopicProto) -> bytes:
    return msg.lcm_encode()
```

接收时会用 topic 上记录的消息类型去 `lcm_decode()`：

```python
return topic.lcm_type.lcm_decode(msg)
```

所以它是：

```text
Python/Dimos message object
  -> lcm_encode()
  -> bytes
  -> LCM channel
  -> lcm_decode()
  -> Python/Dimos message object
```

典型适用对象：

- `PointCloud2`
- `Odometry`
- `Image`
- `PoseStamped`
- 其他 `dimos.msgs.*` / `dimos_lcm.*` 消息

### `pLCMTransport`

`pLCMTransport` 里的 `p` 是 **pickle**。

它用于 **消息类型没有 `lcm_encode()`** 的情况。源码里默认逻辑是：

```python
use_pickled = getattr(stream_type, "lcm_encode", None) is None
transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, stream_type)
```

`pLCMTransport` 内部用的是 `PickleLCM`：

```python
class pLCMTransport(PubSubTransport[T]):
    def __init__(self, topic: str, **kwargs) -> None:
        super().__init__(topic)
        self.lcm = PickleLCM(**kwargs)
```

它的编码器是：

```python
def encode(self, msg: MsgT, _: TopicT) -> bytes:
    return pickle.dumps(msg)

def decode(self, msg: bytes, _: TopicT) -> MsgT:
    return pickle.loads(msg)
```

所以它是：

```text
arbitrary Python object
  -> pickle.dumps()
  -> bytes
  -> LCM channel
  -> pickle.loads()
  -> arbitrary Python object
```

它的好处是简单直接，几乎任何 Python 对象都能传。

它的代价是：

- 只能在 Python 世界里自然工作
- 不像原生 LCM 消息那样天然多语言互通
- 性能和可移植性通常不如强类型 LCM 消息

## 两者的本质区别

| Transport | 编码方式 | 适用对象 | 跨语言互通 |
|---------|---------|---------|---------|
| `LCMTransport` | `lcm_encode` / `lcm_decode` | `dimos.msgs.*` 这类 LCM 消息 | 强 |
| `pLCMTransport` | `pickle.dumps` / `pickle.loads` | 任意 Python 对象 | 弱 |

可以把它理解成：

```text
LCMTransport = LCM 原生消息总线
pLCMTransport = 借 LCM 通道传 Python pickle
```

## 在 blueprint 里怎么被选中

源码位置：[`dimos/core/coordination/module_coordinator.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/core/coordination/module_coordinator.py:574)

当 `ModuleCoordinator.build()` 给 stream 分配 transport 时，如果你没有手动 override，会走默认规则：

```python
use_pickled = getattr(stream_type, "lcm_encode", None) is None
topic = f"/{name}" if _is_name_unique(blueprint, name) else f"/{short_id()}"
transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, stream_type)
```

含义是：

1. 如果消息类型有 `lcm_encode()`，优先用 `LCMTransport`
2. 否则退回到 `pLCMTransport`
3. 默认 topic 往往是 `/{stream_name}`

例如：

```python
class TerrainAnalysis(NativeModule):
    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: Out[PointCloud2]
```

因为 `PointCloud2` 和 `Odometry` 都是 LCM 消息，所以这几个 stream 默认会走 `LCMTransport`。

## 在模块里怎么用

模块通常不会直接操作 LCM API，而是通过 stream API：

```python
self.odometry.subscribe(self._on_odom)
self.path.publish(path_msg)
```

也可以更底层一点，直接订阅 transport：

```python
self.register_disposable(
    Disposable(self.odometry.transport.subscribe(self._on_odom_for_tf, self.odometry))
)
```

这意味着：

```text
在 odometry 绑定的 transport 上注册一个 callback
收到消息时执行 _on_odom_for_tf
模块 stop() 时自动取消订阅
```

`FastLio2` 就用了这种方式来监听自己输出的 `odometry`，再额外发布 TF：[module.py](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/hardware/sensors/lidar/fastlio2/module.py:167)

## 在 native module 里怎么用

对 native module 来说，Python wrapper 不直接处理业务数据；它负责把 stream 对应的 topic 收集出来，然后通过 CLI 参数传给 C++ 进程。

例如：

```python
topics = self._collect_topics()
cmd.extend([f"--{name}", topic_str])
```

C++ 里再用：

```cpp
std::string odom_topic = mod.topic("odometry");
lcm.subscribe(odom_topic, &Handlers::on_odometry, &handlers);
```

然后发布输出：

```cpp
lcm.publish(terrain_map_topic, &terrain_cloud);
```

这就是为什么 native module 能无缝接进同一张模块图：Python 侧仍然是 `In[T]` / `Out[T]`，只是 C++ 进程直接在同一个 LCM topic 上收发。

## `LCMTransport` 和共享内存的关系

它们**不是共享内存机制**。

`LCMTransport` / `pLCMTransport` 都是：

- pub/sub
- topic-based
- 通过 LCM backend 发送消息字节流

如果你想用共享内存，DimOS 里是另一套 transport：

- `SHMTransport`
- `pSHMTransport`
- `JpegShmTransport`

也就是说：

```text
LCMTransport / pLCMTransport = 消息总线
SHMTransport / pSHMTransport = 共享内存通道
```

通常大部分控制、状态、路径、姿态、里程计消息用 LCM 很自然；很大的图像或点云，如果你特别在意复制成本和吞吐量，才会考虑显式切到 SHM。

## topic 长什么样

LCM 在 DimOS 中通常会把 topic 和消息类型一起表示成类似：

```text
/odometry#nav_msgs.Odometry
/terrain_map#sensor_msgs.PointCloud2
/lidar#sensor_msgs.PointCloud2
```

`Topic.__str__()` 的实现会把它渲染成：

```python
return f"{self.pattern}#{self.lcm_type.msg_name}"
```

这也是为什么 native module 的 CLI 参数经常长成：

```bash
--odometry /odometry#nav_msgs.Odometry
--terrain_map /terrain_map#sensor_msgs.PointCloud2
```

## 什么时候该选哪一个

优先选 `LCMTransport`，如果：

- 你的消息是 `dimos.msgs.*` / `dimos_lcm.*`
- 你希望 native module 和 Python module 都能直接互通
- 你希望保留更清晰的消息 schema

使用 `pLCMTransport`，如果：

- 你要传的是临时的 Python 对象
- 这个对象没有现成的 LCM 消息类型
- 你只关心 Python 内部先跑起来

如果是大 payload 且在同机高频传输，进一步考虑：

- `SHMTransport`
- `pSHMTransport`

## 心智模型

在 DimOS 里可以把它想成这样：

```text
Module
  -> Out[T].publish(msg)
  -> Transport.encode(msg)
  -> topic
  -> pub/sub backend
  -> subscriber side decode(msg)
  -> In[T].subscribe(callback)
```

而 `LCMTransport` 和 `pLCMTransport` 只是这个链路里两种不同的编码策略：

```text
LCMTransport: typed LCM bytes
pLCMTransport: pickled Python bytes
```

## 相关源码

- [`dimos/core/transport.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/core/transport.py:79)
- [`dimos/protocol/pubsub/impl/lcmpubsub.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/protocol/pubsub/impl/lcmpubsub.py:75)
- [`dimos/protocol/pubsub/encoders.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/protocol/pubsub/encoders.py:87)
- [`dimos/core/coordination/module_coordinator.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/core/coordination/module_coordinator.py:574)
- [`dimos/core/native_module.py`](/home/longyuxiang/LYX/Progress/Dimensional/dimos/dimos/core/native_module.py:221)

## 进一步阅读

- [Transports](/home/longyuxiang/LYX/Progress/Dimensional/dimos/docs/usage/transports/index.md)
- [Blueprints](/home/longyuxiang/LYX/Progress/Dimensional/dimos/docs/usage/blueprints.md)
- [Modules](/home/longyuxiang/LYX/Progress/Dimensional/dimos/docs/usage/modules.md)
