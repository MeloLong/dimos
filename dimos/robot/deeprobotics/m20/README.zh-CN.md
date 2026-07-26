# DeepRobotics M20 MuJoCo 集成

本目录包含 DeepRobotics M20 的 DimOS 集成，包括 MuJoCo 仿真、官方运动策略、
实车通信适配和导航蓝图。

> [!IMPORTANT]
> 当前 M20 支持及本文测试结果均属于实验阶段。该仿真可用于导航集成和回归测试，
> 但不能代替实体机器人的标定和安全测试。

语言：[English](/dimos/robot/deeprobotics/m20/README.md) | 中文

## 文档导航

| 文档 | 内容范围 |
| --- | --- |
| 本 README | 安装、架构、配置归属、验证证据和已知限制 |
| [MuJoCo 运行参考](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md) | 蓝图命令、Viewer 操作、YAML 参数、录制和故障排查 |
| [资产来源记录](/dimos/robot/deeprobotics/m20/assets/SOURCE.md) | 上游仓库、固定提交、许可证和 DimOS 对 MJCF 的改动 |

## 范围与来源

模型、网格和运动策略来自官方
[DeepRoboticsLab/sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy)
仓库的提交 `80e3d40084c4ed151ba6f88b0d55cf1d480aa45e`，许可证为
BSD-3-Clause。

公开源代码将这台 16 自由度轮足四足机器人称为 **M20**，没有使用 M20 Pro 名称。
因此，本集成不能证明一台被称为 M20 Pro 的实体产品具有完全相同的结构、传感器、
标定、固件或载荷。

仓库中 ONNX 的 SHA-256 为
`0ac99f3093d4a984d7587b88d57300cbf7ec2f788401dfa1570d1e4800568f6b`，
与固定上游提交中的策略一致。上游实机部署说明也会加载这个策略路径，但已部署机器人
仍可能使用厂商或现场替换过的文件；在宣称二进制一致前，应读取实机哈希。

DimOS 保留官方机器人几何、惯量、关节限制、执行器和策略；删除原场景的地面和灯光，
为合成传感器分配几何组，增加命名相机和传感器，并增加站立关键帧。完整边界见
[资产来源记录](/dimos/robot/deeprobotics/m20/assets/SOURCE.md)。

RGB 相机和 RoboSense Airy 雷达仿真由 DimOS 添加，并不是厂商标定的 M20 传感器模型。
实体 Airy 官方规格为 360 x 90 度、96 通道，解码距离最大 60 m。默认仿真为前后两个
朝外的 180 x 90 度扇区，每台 96 条垂直线、64 个方位采样，以 2 Hz 发布并截断到 10 m，
以保证 CPU 仿真环境可用。公开资料没有提供出厂束角和 M20 安装外参。

## 已包含组件

| 组件 | 位置 | 职责 |
| --- | --- | --- |
| 官方 MJCF 和网格 | `assets/` | M20 运动学、动力学、碰撞、执行器和可视几何 |
| 官方 ONNX 策略 | `assets/deeprobotics_m20_policy.onnx` | 57 维观测到 16 维运动动作 |
| MuJoCo 策略适配 | `dimos/simulation/mujoco/policy.py` | 策略观测、关节映射及腿位置/轮速度混合控制 |
| 仿真连接 | `mujoco_sim.py` | M20 话题发布和仅用于仿真的偏航命令适配 |
| 传感器与规划配置 | `config/mujoco_sim.yaml` | 两个仿真蓝图使用的默认运行参数 |
| Rerun 布局 | `blueprints/basic.py` | 前视相机、3D 布局和仅影响显示的限频 |
| 导航蓝图 | `nav/m20_simple_nav.py`、`nav/m20_dan_nav.py` | 实车与仿真的 Simple A* 和 DAN 组合 |

保留的 `nav/` 文件职责互不重复：

| 路径 | 职责 |
| --- | --- |
| `m20_simple_nav.py` | 实车与仿真的 Simple Nav 蓝图 |
| `m20_dan_nav.py` | 实车与仿真的 DAN 导航蓝图 |
| `moving_obstacle.py` | Simple Nav 仿真使用的可复现 mocap 人物刺激源 |
| `odom2posestamped.py` | DAN 链路需要的里程计到位姿适配器 |
| `m20_map_save.py` | 完整的实体 M20 地图录制蓝图 |
| `map_save/` | `m20-map-save` 使用的原生点云累积模块 |
| `test_m20_*.py` | 当前蓝图、传感器配置与移动人物回归测试 |
| `mujoco_sim.md` | 运行与参数参考 |

## 支持的蓝图

下表只覆盖 `dimos.robot.deeprobotics.m20` 目录负责的蓝图。
`dimos.robot.m20` 和 `dimos.robot.m20_native` 中的同名近似入口属于其他集成，不在本文范围内。

| 蓝图 | 运行目标 | 用途 |
| --- | --- | --- |
| `m20` | 实体 M20，可下发命令 | 基础连接、TF、前后图像可视化与 Rerun |
| `m20-simple-nav` | 实体 M20，可下发命令 | 实体机器人 Simple A* 导航 |
| `m20-dan-nav` | 实体 M20，可下发命令 | 实体机器人 MLS 与 DAN 导航 |
| `m20-map-save` | 实体 M20，可下发命令 | 累积已配准 SLAM 点云，并在正常退出时保存 PCD |
| `m20-dds-rerun` | M20 本体，只读传感器 | 将雷达、IMU 和里程计从 drdds 转接至 LCM 与 Rerun |
| `m20-simple-nav-sim` | MuJoCo | 使用官方 M20 资产、Simple Nav 和移动人物的仿真 |
| `m20-dan-nav-sim` | MuJoCo | 使用官方 M20 资产、MLS 和 DAN 控制的仿真 |

> [!WARNING]
> 四个可下发命令的实车蓝图都会实例化 `M20Connection`，能够向硬件发送命令。运行时必须
> 遵循机器人网络、操作员、遥控急停和空旷区域规范，不能把仿真限制当作实车安全限制。

地图保存蓝图默认写入 `nav/map_save/m20_accumulated_map.pcd`。它是独立的实体建图流程，
不是第二套导航实现。

## 运行架构

1. 遥操作或导航控制器发布 `cmd_vel`。
2. `M20MujocoSimConnection` 将命令转发给 MuJoCo 子进程。
3. `M20OnnxController` 构造官方 57 维观测，每 20 ms 运行一次 ONNX，并输出
   12 个腿部位置目标和 4 个轮子速度目标。
4. MuJoCo 发布里程计、前视 RGB 及前后合并的合成点云。
5. 同一个 DimOS 组合中的建图、规划、控制、TF 和 Rerun 模块消费这些数据。

| 蓝图 | 规划与跟踪链路 | 移动人物 |
| --- | --- | --- |
| `m20-simple-nav-sim` | `CostMapper -> ReplanningAStarPlanner -> LocalPlanner/PController` | 默认开启 |
| `m20-dan-nav-sim` | `MLSPlannerNative -> DanLocalPlanner -> DanHolonomicTC` | 未包含 |

两个仿真蓝图都会关闭 MuJoCo 自带窗口，`--rerun-open` 只控制 Rerun Viewer。
仿真蓝图均不实例化 `M20Connection`，因此不会向实体机器人发送命令。

## 安装

先按仓库的 [Ubuntu 安装指南](/docs/installation/ubuntu.md)准备系统依赖、Nix 和原生构建环境。
随后为源码工作区安装 CPU ONNX 与仿真依赖：

```sh skip
git clone https://github.com/dimensionalOS/dimos.git
cd dimos
uv sync --extra cpu --extra sim
source "$HOME/.cargo/env"
```

审核尚未合并的 PR 时，应改用贡献者 fork 和待审分支。不要把个人 fork 或临时集成分支
写入正式部署脚本。

启动前确认资产和 Python 环境可用：

```sh skip
test -s dimos/robot/deeprobotics/m20/assets/deeprobotics_m20.xml
test -s dimos/robot/deeprobotics/m20/assets/deeprobotics_m20_policy.onnx
uv run --no-sync python -c \
  'from dimos.simulation.mujoco.model import _get_m20_asset_dir; print(_get_m20_asset_dir())'
```

MuJoCo 模型本身不需要 ROS 2，也不需要另装 DeepRobotics SDK；导航链路仍需要 DimOS
标准的原生建图和规划工具。实体机上的 `m20-dds-rerun` 是独立部署，需要机器人 drdds SDK
和对应的原生 CMake 工具链。

## 启动仿真

切换蓝图前先停止旧协调器：

```sh skip
uv run --no-sync dimos stop
```

使用 headless EGL MuJoCo，并打开原生 Rerun Viewer：

```sh skip
MUJOCO_GL=egl uv run --no-sync dimos --rerun-open native run m20-simple-nav-sim
```

无桌面服务器或 CI 使用：

```sh skip
MUJOCO_GL=egl uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim
```

测试 DAN 链路时替换蓝图名：

```sh skip
MUJOCO_GL=egl uv run --no-sync dimos --rerun-open native run m20-dan-nav-sim
```

如果 SSH 启动的进程无法继承桌面环境，先以 `--rerun-open none` 启动 DimOS，再从桌面环境
单独打开 Viewer：

```sh skip
uv run --no-sync dimos-viewer --connect rerun+http://127.0.0.1:9877/proxy
```

目标点测试、录制及进程清理见 [运行参考](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md)。

## 配置归属

默认运行配置是 [`config/mujoco_sim.yaml`](/dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml)。持久修改应写入该文件，
随后重启 DimOS。仿真蓝图导入时会校验 YAML；未知字段和非法范围会使启动失败，不会被静默忽略。

| YAML 节 | 使用者 | 配置内容 |
| --- | --- | --- |
| `m20mujocosimconnection` | 两个仿真蓝图 | RGB、合成雷达、人物碰撞开关和仿真偏航适配 |
| `m20movingobstacle` | 仅 Simple Nav | 仿真人物路线、速度、频率及接近机器人时的行为 |
| `mlsplannernative` | 两个仿真蓝图 | 仿真机器人高度和墙体径向净空 |
| `replanningastarplanner` | 仅 Simple Nav | 原始路径诊断、约束平滑、回退及 shadow validator |

临时覆盖使用模块生成名称作为前缀：

```sh skip
uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim \
  --option m20mujocosimconnection.pointcloud_fps=1.0 \
  --option m20movingobstacle.enabled=false
```

YAML 只提供模块参数，不能实例化模块。例如 `M20MovingObstacle` 必须仍在
`m20-simple-nav-sim` 蓝图中，其 YAML 配置才会生效。它驱动的是 mocap 人物，不是机器人
急停或避障安全模块。

并非所有仿真属性都由 YAML 控制：

| 属性 | 配置真源 | 当前状态 |
| --- | --- | --- |
| 传感器安装位置与朝向 | `assets/deeprobotics_m20.xml` | DimOS 近似值，不是实测 M20 外参 |
| 相机光学 TF 与 `CameraInfo` | `tf.py` | 待标定；当前静态 1280 x 720 内参与 640 x 360 仿真图像未统一 |
| 射线生成算法 | `dimos/simulation/mujoco/mujoco_process.py` | 共享实现，修改时必须配套测试 |
| Rerun 布局和仅显示用降采样 | `blueprints/basic.py` | 仿真使用仅前视相机布局 |
| 策略周期、增益与关节映射 | `dimos/simulation/mujoco/policy.py` | 官方策略契约，不是普通调参项 |
| 机器人结构与动力学 | MJCF 与 ONNX 资产 | 没有上游或实机证据时不要修改 |

完整 YAML 当前值与约束见[运行参数参考](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md#parameter-reference)。

## 验证方案

### 自动化测试

测试前停止所有 DimOS、MuJoCo、Rerun、建图和规划进程。执行当前 M20 与共享 MuJoCo 回归集：

```sh skip
MUJOCO_GL=egl uv run --no-sync python -m pytest -q \
  dimos/robot/deeprobotics/m20 \
  dimos/simulation/mujoco/test_m20_policy.py \
  dimos/simulation/mujoco/test_mujoco_process.py \
  dimos/robot/unitree/test_mujoco_connection.py
```

移动人物集成测试会创建 MuJoCo 场景，使用独立 marker：

```sh skip
MUJOCO_GL=egl uv run --no-sync python -m pytest -q -m mujoco \
  dimos/robot/deeprobotics/m20
```

交互目标点测试前先做一次有限时长的完整进程启动：

```sh skip
timeout --signal=INT --kill-after=10s 30s \
  env MUJOCO_GL=egl uv run --no-sync dimos --rerun-open none \
  run m20-simple-nav-sim
```

### 参考测试环境

最新记录于 2026-07-26、提交
`ee818561d9999df73b7ac8baad939a44229dff57`，测试条件如下：

| 项目 | 条件 |
| --- | --- |
| 主机 | Ubuntu 22.04.5 LTS aarch64 VM，14 个 Apple 虚拟 CPU，62 GiB 内存 |
| MuJoCo / Rerun SDK | 3.5.0 / 0.32.0-alpha.1 |
| MuJoCo 渲染 | Headless EGL |
| Viewer | VM 桌面中的原生 Rerun，OpenGL 为 `llvmpipe` 软件渲染 |
| 场景/配置 | 默认 office 场景和仓库内 `mujoco_sim.yaml`；前视 RGB、前后雷达、移动人物开启 |
| 进程规范 | 每次运行前停止已有 DimOS/MuJoCo/Rerun 进程 |

### 参考提交上的测试结果

| 测试项 | 输入与条件 | 结果 |
| --- | --- | --- |
| M20/共享 MuJoCo 自动化测试 | 上述命令，默认 marker | `81 passed, 1 deselected`，7.95 s |
| 显式 MuJoCo 障碍物场景 | `-m mujoco dimos/robot/deeprobotics/m20` | `1 passed, 53 deselected`，7.05 s |
| Simple Nav 冒烟测试 | 运行 `m20-simple-nav-sim` 30 s，EGL，不开 Viewer | 11 个模块全部启动；ONNX 加载成功；RGB 7.70-7.91 FPS；雷达 1.98-1.99 Hz；人物暂停、改道和恢复均触发；退出后无残留进程或监听端口 |
| DAN 冒烟测试 | 运行 `m20-dan-nav-sim` 30 s，EGL，不开 Viewer | 12 个模块、voxel 建图器、MLS 规划器、DAN 控制器、ONNX 和 MuJoCo 均启动；退出后无残留进程或监听端口 |
| 模型契约 | 直接加载 MJCF | `nq=23`、`nv=22`、`nu=16`；关节/执行器及 `obs [1,57] -> actions [1,16]` 均通过 |
| 静止 | 零命令 2 s | 基座由 0.58 m 稳定到 0.5636 m；控制输出有限且保持站立 |
| 前进 | 稳定后执行 `[0.2,0,0]` 3 s | 前进 `+0.4884 m`，横移 `-0.0018 m`，保持站立 |
| 当前传感器时序 | 640 x 360 RGB 8 FPS；两台 64 x 96 雷达 2 Hz；Viewer 开启 | RGB 约 7.73 FPS，P95 帧间隔 133.5 ms，最大 135.6 ms；合并点云约 1.8-2.0 Hz |
| Rerun 布局 | 仿真仅前视 RGB；图像实体从 3D 排除 | 不再出现空后视相机加载页和红色 `3D -> color_image` 类型不匹配；稳定后 Viewer CPU 约 2-3% |
| 导航目标 | 从约 `(-2.457,0.977)` 前往 `(-3.35,-0.51)` | 进入 `path_following`，运动到约 `(-3.270,-0.258)` 并发布 `goal_reached` |
| 普通 teleop 偏航 | `angular.z=+0.8` 与 `-0.8` 各 3 s；放大 2.0，封顶 1.6 | 约 `+0.585 rad` 与 `-1.112 rad`；两方向可转，但响应仍不对称 |

这些是特定场景的结果，不是统计性能保证。直接运动数据采集于官方资产初次集成阶段；此后官方
MJCF、ONNX、策略适配和 20 ms 策略周期均未改变。

## 已知限制

1. **不能证明实机等价。** DimOS 实车通过 Patrol UDP 高层接口控制，仿真则把速度命令输入
   官方低层 ONNX。真实速度、转弯半径、制动、通过性和安全距离均未验证。
2. **横移和正负偏航保真度仍待解决。** 受控 3 s 测试中，`[0,0.2,0]` 只产生
   `+0.0270 m` 横移；`[0,0.5,0]` 虽可横移，但耦合了 `-0.2337 m` 后退。
   `+0.7` 与 `-0.7` 偏航分别得到 `+0.2200` 和 `-0.7311 rad`。追踪结果表明不对称
   轮速目标来自官方 ONNX，而手工镜像轮速可得到近似镜像偏航。因此当前不修改官方策略。
3. **传感器标定是近似值。** 公开资料没有 Airy DIFOP 束角、M20 雷达/相机外参、时序、噪声、
   运动畸变及盲区模型；仿真还主动降低了方位密度、距离和更新率。
4. **相机标定尚未统一。** `tf.py` 仍发布待标定的 1280 x 720 内参，而仿真图像为
   640 x 360；当前图像/3D 叠加不能用于标定级投影测量。
5. **软件渲染仍可能出现孤立卡顿。** 在 `llvmpipe` 下，全局地图重计算或目标切换仍可能
   产生偶发长帧，但此前持续出现的加载和卡顿已消除。
6. **移动人物只是感知刺激源。** 默认关闭碰撞，因为受控 mocap 刚体可能不真实地推倒机器人。
   接近逻辑只让人物暂停和改道，不会停止 M20。
7. **部分全仓测试依赖外部环境。** 一个继承的 `office_lidar` 测试需要私有 LFS 凭据，
   与 M20 无关的 Go2/FastLIO 蓝图参数检查也可能提前失败。应将这些记录为测试环境排除项，
   不能算作 M20 通过项。
8. **限时退出仍有共享内存清理警告。** 30 秒 SIGINT 冒烟测试结束时，Python
   `resource_tracker` 报告 7 个 shared-memory 对象待清理。退出后没有残留子进程或端口，
   但优雅关闭阶段的共享内存生命周期仍需后续调查。

不要为了掩盖上述限制而直接调整关节符号、增益、MJCF 动力学或 ONNX。正确顺序是在官方 runner
复现命令矩阵，并采集同步的实体 M20 里程计、IMU、关节状态和视频，再决定模型或策略改动。

## 录制与回放

需要保存 `.rrd` 时，外部 Rerun 录制器必须在 DimOS 启动前占用 `9877` 端口。
SQLite `nav-record` 必须在蓝图启动时组合，并且只录制显式连接的数据流。完整命令、数据范围和
回放限制见[运行参考](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md#recording-and-replay)。

## 常见问题

| 现象 | 原因或检查项 |
| --- | --- |
| `Unknown robot policy: deeprobotics_m20` | 当前代码早于 M20 集成，或缺少仿真改动 |
| `Error opening file '*.STL'` | 检查 `assets/meshes/`，随后重新安装/同步环境 |
| ONNX 加载失败 | 恢复受跟踪策略，并核对 SHA-256 和 57/16 接口 |
| Headless 模式无 RGB | 设置 `MUJOCO_GL=egl`，并确认 EGL 可用 |
| 原生 Viewer 报 `winit` 并退出 | 进程没有桌面显示环境；使用 `--rerun-open none` 或在桌面会话内单独启动 Viewer |
| 3D 视图下图像显示红色 | 使用仿真专用 Rerun blueprint；图像实体应放在 `Spatial2DView` |
| 点云中出现机器人自身 | 保持 `pointcloud_geom_groups: [0,1]`，排除机器人可视/碰撞组 |
| 第二次启动变慢或端口冲突 | 执行 `dimos stop`，并确认没有残留 MuJoCo、Rerun、voxel 和 MLS 进程 |

## 更新官方资产

1. 在 `assets/SOURCE.md` 固定上游仓库和不可变提交。
2. 重新检查许可证兼容性，并保留上游许可证文件。
3. 替换前比对 MJCF 名称、维度、关节、执行器、惯量、限制和 ONNX 接口。
4. 只有关节顺序和策略契约完全一致时才复用当前控制器，否则应实现独立审核的适配器。
5. 执行两组自动化测试、两个仿真蓝图的有限时长启动、传感器检查、遥操作方向检查和至少一个目标点。
6. 在同一改动中更新英文 README、中文 README、运行参考和资产来源。

只有外观正确、却没有匹配执行器与运动策略契约的模型，不是有效的机器人仿真。
