# DimOS Memory

`memory2` 是 DimOS 的持久化 observation 存储和流式处理层。它把实时机器人
数据流保存为可查询的数据集，并保留时间戳、位姿、标签和编码后的原始数据。

当你需要录制一次机器人运行、检查数据集、在 Rerun 中渲染，或同时处理历史与
实时 observation 时，应从这里开始。

## 快速开始

导航录制通过把记录器与机器人蓝图组合完成。`NavRecord` 只会记录实际连接到其
`In` 端口的数据流：

```bash
mkdir -p recordings
dimos run m20-dan-nav nav-record \
  --option nav_record.db_path=recordings/m20-dan-$(date +%Y%m%d_%H%M%S).db
```

使用 `Ctrl-C` 正常停止，确保 SQLite 数据库完成关闭和落盘。`NavRecord` 的默认
输出是 DimOS 项目根目录中的 `nav_recording.db`。同名输出默认会备份旧文件，而不
会静默覆盖。

将已完成的录制渲染为便于分享和回放查看的 Rerun 文件：

```bash
dimos mem rerun recordings/m20-dan-20260715_170000.db \
  --out recordings/m20-dan-20260715_170000.rrd \
  --no-gui
```

去掉 `--no-gui` 后，写完 `.rrd` 会自动打开 Rerun Viewer。`--seconds N` 只渲染
前 `N` 秒；`--root session_name` 会把全部实体放到同一个 Rerun 路径下。

## 核心模型

```text
机器人模块 -> DimOS Stream -> Recorder -> SQLite 数据集 -> 查询 / 渲染
                                  |                         |
                                  +-- 时间戳、位姿、标签、    +-- 导出 .rrd
                                      编码后的 payload
```

核心对象如下：

| 对象 | 职责 |
| --- | --- |
| `Store` | 管理命名数据流及其存储后端。 |
| `Stream` | 单个 observation 流的惰性查询、转换和迭代接口。 |
| `Recorder` | 将已连接 `In` 端口持久化到存储中的 DimOS 模块。 |
| `SqliteStore` | 录制时使用的 SQLite 持久化实现。 |
| `MemoryStore` | 用于实验和测试的内存实现。 |
| `NullStore` | 不保存历史、常量内存的纯实时实现。 |

每条 observation 包含 payload、时间戳、可选位姿、坐标系和标签。payload 会自动
选择编码方式：图像使用 JPEG 压缩，DimOS LCM 消息使用 LCM 编码，其他对象默认
使用 pickle。

## 如何录制蓝图

DimOS 没有会自动抓取全部进程的全局记录器。录制是显式组合的：将记录器模块加到
蓝图中，或在 CLI 上组合已有记录器。只有与记录器 `In` 端口连通的 stream 会被
写入数据库。

上面的 M20 命令组合了两个已注册蓝图：

```text
m20-dan-nav + nav-record
```

`NavRecord` 面向路径、目标、速度命令、里程计和已连通地图流等导航数据。它不是
原始相机包或激光雷达数据包记录器；若需要原始传感器数据或自定义流集合，应增加
专用记录器。

### 添加自定义 Recorder

先定义需要持久化的输入端口，再将记录器与数据生产蓝图组合。流名称和类型匹配时，
`autoconnect()` 会自动完成连接：

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.memory2.module import Recorder
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image


class SessionRecorder(Recorder):
    color_image: In[Image]
    odometry: In[Odometry]


recorded_robot = autoconnect(
    robot_blueprint,
    SessionRecorder.blueprint(db_path="recordings/session.db"),
)
```

优先定义明确的记录器类，而不是无边界地记录所有流。这样可以让数据契约清晰，避免
意外采集高带宽数据，并使后续数据集和回放行为可预测。

## 使用 Python 检查数据集

```python
from dimos.memory2.store.sqlite import SqliteStore

store = SqliteStore(path="recordings/m20-dan-20260715_170000.db")

for name, stream in store.streams.items():
    print(name, stream.summary())

recent_odom = store.streams.odometry.order_by("ts", desc=True).limit(10).to_list()
```

查询默认是惰性的，只有 `.to_list()`、`.first()`、`.count()` 或 `.drain()` 等终端
操作真正消费数据。实时处理应使用 `.drain_thread()`，而不是会物化全部数据的操作：

```python
handle = (
    store.streams.color_image.live()
    .transform(process_frame)
    .save(store.stream("processed_image", Image))
    .drain_thread()
)
```

实时流没有上界。排序、或在实时 transform 后做向量搜索等需要遍历全部输入的操作会
被主动拒绝。应查询已经保存的 stream，或先使用 `.limit()` 限制输入。

## 存储与运维

- 录制结果是 SQLite 数据库，不是视频文件。使用 `dimos mem rerun` 生成用于 Rerun
  回放和分享的 `.rrd` 文件。
- 复制或归档数据库前应先正常停止录制。记录器运行期间 SQLite 可能有 WAL 旁路文件。
- 点云和图像会快速占用磁盘。应明确选择要记录的 stream，并在长时间测试中监控可用
  磁盘空间。
- 运行日志和录制数据彼此独立。DimOS 运行日志位于 `logs/<run-id>/main.jsonl`，不能
  代替 `Recorder` 生成的数据集。
- 数据库应配套保存元数据：机器人型号、软件版本、传输配置、测试地点和操作记录，
  否则后续难以复现和分析。

## 延伸阅读

- [Introduction](intro.md)：Store、Stream、过滤、转换和实时查询。
- [Architecture](architecture.md)：backend、observation、blob、vector 和 notifier 层。
- [Streaming model](streaming.md)：惰性、物化和终端操作。
- [Store implementations](store/README.md)：持久化和内存后端。
- [Codecs](codecs/README.md)：payload 编码规则。
- [Memory capability guide](../../docs/capabilities/memory/index.md)：可视化和分析示例。

英文版本见 [README.md](README.md)。
