# DimOS 官方 DeepRobotics M20 MuJoCo 集成指南

本文是将官方 DeepRobotics M20 MuJoCo 模型接入 DimOS 的可复用入口文档。
内容覆盖代码获取、环境安装、模型与策略契约验证，以及两条 M20 导航仿真链路的启动方法。

语言：[English](/dimos/robot/deeprobotics/m20/README.md) | 中文

完成本文后，运行参数调优、录制、回放和完整传感器配置请继续阅读
[M20 MuJoCo 运行指南](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md)。

## 推荐阅读顺序

1. **首次安装：** [范围与型号确认](#范围与型号确认)、[快速安装与启动](#快速安装与启动)、[前置条件](#前置条件)。
2. **集成开发：** [已包含内容](#已包含内容)、[运行架构](#运行架构)、[模型与策略契约](#模型与策略契约)、[配置与临时覆盖](#配置与临时覆盖)。
3. **使用结果前：** [验证清单](#验证清单)、[控制保真度审查](#控制保真度审查)、[已知运动限制](#已知运动限制)、[测试汇总与优化方向](#测试汇总与优化方向)。
4. **运行维护：** [录制与回放](#录制与回放)、[常见问题](#常见问题)、[更新到新版官方资产](#更新到新版官方资产)。


## 范围与型号确认

导入的资产来自官方
[DeepRoboticsLab/sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy)
仓库的提交 `80e3d40084c4ed151ba6f88b0d55cf1d480aa45e`，许可证为 BSD-3-Clause。

官方仓库将该机器人称为 **M20**，并说明其为 16 自由度轮足四足机器人；公开资料中没有
`M20 Pro` 标识。因此本集成一律使用官方的 M20 名称。它不能证明内部称为 M20 Pro 的实体机器人在
尺寸、载荷、传感器或标定上与仿真模型完全一致。

资产清单及为适配 DimOS 所做的改动记录在
[SOURCE.md](/dimos/robot/deeprobotics/m20/assets/SOURCE.md)。不要以未经验证的第三方模型或
Unitree 策略替换这些文件。

## 控制保真度审查

**结论：** 该仿真与官方 M20 Sim-to-Real SDK 的模型坐标和低层 ONNX 策略契约一致，但当前
DimOS 真机导航链路不运行该 ONNX 策略。因此它可用于导航集成与策略级仿真测试，**不能**作为
“DimOS 真机轨迹、动力学或安全行为已一致”的证明。

| 审查项 | 结论 | 证据 |
| --- | --- | --- |
| 运动学、惯量、关节范围、执行器力矩范围 | 一致 | 与官方 `M20.xml` 做去除 CRLF 的差异比对；仅移除地面/灯光、调整网格路径和碰撞可见组，并添加相机、命名传感器和 `home` 关键帧 |
| ONNX 二进制与接口 | 一致 | 与上游 `policy.onnx` 的 SHA-256 均为 `0ac99f3093d4a984d7587b88d57300cbf7ec2f788401dfa1570d1e4800568f6b`；接口为 `obs [1,57] -> actions [1,16]` |
| 低层策略适配 | 一致 | DimOS 与官方 `M20PolicyRunner` 均采用相同关节排列、观测缩放、动作缩放、`Kp=[80,80,80,0]`、`Kd=[2,2,2,0.6]` 和腿位置/轮速度混合控制 |
| 策略频率 | 一致 | 官方为 5 ms 状态机加 4 倍 decimation，即 20 ms；DimOS 为 1 ms 物理步长、20 个子步，即 20 ms |
| 真机编码器坐标 | 设计上正确，但非实测 | 官方 SDK 在进入策略前使用各关节方向和零位偏置把 DDS 编码器值转换到模型坐标；DimOS 已在 MuJoCo 模型坐标中运行，因此不应再次施加硬件偏置 |
| DimOS 仿真与 DimOS 真机控制链 | **不一致，尚未标定** | 仿真将 `cmd_vel` 直接传给 ONNX；真机 `M20Connection` 通过 Patrol UDP 高层接口发送归一化轴指令，并不运行 ONNX |
| 真机速度尺度与符号 | 未验证 | 真实连接当前使用 `max_linear=1.0`、`max_angular=1.5`；官方 RL 键盘路径使用 `0.7/0.5/0.7`。真实连接代码也明确标注横移和偏航符号未验证 |
| 电机与地面接触动力学 | 未验证 | MuJoCo 保留官方刚体、摩擦和力矩限制，但未建模实机固件内环、电机带宽、电流/温度保护、通信延迟、轮胎与地面参数或传感器噪声 |

在获得同一 `cmd_vel` 序列的实机里程计、IMU、关节状态和视频对照前，不得使用该仿真验收真实
速度、转弯半径、制动距离、越障能力或安全距离。建议先以低速直行、横移和原地转向分别验证符号和
尺度，再记录同一段轨迹进行时序对齐和误差比较；需要低层 Sim-to-Real 验证时，应使用官方 SDK 的
`JOINTS_DATA` / `JOINTS_CMD` 链路及已授权的 SDK 模式。

## 已知运动限制

本仓库刻意保持官方 M20 ONNX 不变。VM 直接 MuJoCo 测试已暴露一个待解决问题：前进跟踪稳定，
但 `0.2 m/s` 横移命令的横移响应很小；最大横移命令伴随明显后退耦合；`+0.7` 与 `-0.7` 偏航命令的
响应显著不对称。手工施加互为相反数的四轮速度目标仍呈近似镜像偏航，而官方 ONNX 对正负偏航命令
并不产生镜像的轮子目标。

横移与偏航跟踪目前均视为未验证，不能用于自主导航性能或安全验收。这里明确暴露该限制；在与官方
runner 和实体机器对照确定正确的策略级修复前，官方 ONNX、MJCF 和控制映射均保持不变。

## 测试汇总与优化方向

| 范围 | 结果 | 说明 |
| --- | --- | --- |
| 官方来源与策略 | 通过 | 已审计 BSD-3-Clause 来源；ONNX 与上游逐字节一致，接口为 `obs [1,57] -> actions [1,16]` |
| 模型与控制契约 | 通过 | 已核对 `nq=23`、`nv=22`、`nu=16`、关节树、惯量、限制、力矩范围、映射、PD 增益及 20 ms 策略周期 |
| 渲染与合成传感器 | 通过 | 四个 M20 相机、RGB、非空点云及 `(0, 1)` 几何组下无机器人自扫描 |
| DimOS 集成 | 通过 | 36 个聚焦测试、15 个动态障碍物测试加显式 MuJoCo 检查；Simple Nav 与 DAN 蓝图均完成有限时长启动 |
| 打包与文档 | 通过 | wheel 含 21 个 M20 资产；pre-commit、LFS、大文件和 doclinks 均通过 |
| 静止与前进 | 通过 | 零命令保持站立；`[0.2,0,0]` 3 秒前进 `+0.4884 m`，横向漂移 `-0.0018 m` |
| 横移与偏航跟踪 | 待解决 | 小横移响应很弱；最大横移有后退耦合；正负偏航响应显著不对称 |
| 实车策略 | 未验证 | 官方发布的实机流程加载同一 `policy/policy.onnx`；当前实体机器不可达，尚未完成只读哈希核验 |

当前只验收模型加载、前进主导运动、合成传感器、建图输入、导航连通性和进程生命周期；不能用此仿真
验收横移/偏航跟踪、自主导航性能或安全间距。

后续顺序是：先在官方 runner 复现带符号命令矩阵；再采集实体 M20 的匹配关节、IMU、里程计、视频和
策略哈希；最后才请求修正策略或以平衡横移/正负偏航目标和左右对称数据增强重新训练。未完成对照前，
不要修改关节符号或 PD 增益。

## 已包含内容

| 组件 | 位置 | 用途 |
| --- | --- | --- |
| 官方 MJCF | `dimos/robot/deeprobotics/m20/assets/deeprobotics_m20.xml` | 机器人运动学、惯量、碰撞、执行器、相机及站立关键帧 |
| 官方网格 | `dimos/robot/deeprobotics/m20/assets/meshes/` | 16 自由度 M20 的可视模型 |
| 官方 ONNX 策略 | `dimos/robot/deeprobotics/m20/assets/deeprobotics_m20_policy.onnx` | 行走策略，输入 `obs [1,57]`，输出 `actions [1,16]` |
| DimOS M20 控制器 | `dimos/simulation/mujoco/policy.py` | 将导航速度指令转换为官方 M20 策略观测，并执行腿轮混合控制 |
| 模型加载器 | `dimos/simulation/mujoco/model.py` | 加载 M20 资产，使用 1 ms 物理步长并选择 M20 控制器 |
| 仿真配置 | `dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml` | 传感器与 M20 仿真规划边界 |

这些资产随分支提交，也会被打包进 wheel。ONNX 文件是普通 Git 文件，因此使用
`GIT_LFS_SKIP_SMUDGE=1` 获取代码后仍可以运行本 M20 仿真。

## 快速安装与启动

请使用干净工作区。不要直接向包含其他 M20 导航改动的工作区写入这些文件；应合并或
cherry-pick 官方 M20 集成提交。

```sh skip
git clone https://github.com/T-Markus-Liang/dimos_m20.git ~/work/dimos_m20
cd ~/work/dimos_m20
git fetch origin codex/m20-official-mujoco-model
git switch --track origin/codex/m20-official-mujoco-model
uv sync --extra all
```

在启动蓝图前确认官方模型和策略均已就位：

```sh skip
test -s dimos/robot/deeprobotics/m20/assets/deeprobotics_m20.xml
test -s dimos/robot/deeprobotics/m20/assets/deeprobotics_m20_policy.onnx
uv run --no-sync python -c 'from dimos.simulation.mujoco.model import _get_m20_asset_dir; print(_get_m20_asset_dir())'
```

启动推荐的简单导航仿真：

```sh skip
cd ~/work/dimos_m20
uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim
```

需要测试 DAN 规划器和全向轨迹控制器时，启动：

```sh skip
cd ~/work/dimos_m20
uv run --no-sync dimos --rerun-open none run m20-dan-nav-sim
```

切换蓝图前先停止旧的 DimOS 进程：

```sh skip
cd ~/work/dimos_m20
uv run --no-sync dimos stop
```

## 前置条件

| 条件 | 原因 | 检查命令 |
| --- | --- | --- |
| Linux x86_64 或 Linux aarch64 | DimOS 与 MuJoCo 的受支持运行平台 | `uname -m` |
| Python 与 uv | 安装锁定的 DimOS 环境 | `uv --version` |
| Git | 获取分支及项目历史 | `git --version` |
| Git LFS | DimOS 中其他资产可能依赖 LFS，本 M20 ONNX 不依赖 | `git lfs version` |
| DimOS 原生工具 | 光线追踪需要；DAN 还需要 MLS 规划器 | `nix --version`、`cargo --version` |
| Headless EGL 或显示服务 | MuJoCo RGB/深度渲染需要 | `echo "$DISPLAY"` 或 `echo "$MUJOCO_GL"` |

M20 模型本身不需要 ROS 2 进程，也不需要额外安装 DeepRobotics SDK。DimOS 会直接加载仓库中
提交的 MJCF 与 ONNX 策略。

在新机器上，先完成仓库标准的 DimOS 原生构建环境配置，再判断是否为 M20 模型问题。
`m20-simple-nav-sim` 需要光线追踪原生可执行文件；`m20-dan-nav-sim` 还需要 MLS 规划器可执行文件。
缺少 `nix`、Rust 工具链不兼容或缺少原生构建产物，都属于环境问题而不是 M20 资产集成问题。

## 运行架构

1. 导航规划器或遥操作向 `cmd_vel` 发布速度命令。
2. `M20MujocoSimConnection` 将命令发送给共享内存中的 MuJoCo 进程。
3. `M20OnnxController` 将命令转换为官方 M20 ONNX 策略所需的观测，再输出腿部位置和轮子速度力矩。
4. MuJoCo 的 RGB/深度相机发布 `color_image` 与 `dimos/slam_aligned_points`，供建图和导航模块消费。

模型提供 DimOS 适配器所需的四个相机：

| 相机 | 用途 |
| --- | --- |
| `head_camera` | RGB `color_image` |
| `lidar_front_camera` | 前向合成深度 |
| `lidar_left_camera` | 左侧合成深度 |
| `lidar_right_camera` | 右侧合成深度 |

可视几何使用 MuJoCo 的 `2` 组，碰撞几何使用 `3` 组。默认深度配置只渲染 `(0, 1)` 组，避免
机器人自身进入点云后被地图膨胀为障碍物。

## 模型与策略契约

以下参数是集成契约的一部分。只改模型、策略或控制器中的一项，可能会导致模型能渲染但无法安全运动。

| 契约项 | 值 |
| --- | --- |
| 广义位置 / 速度维度 | `nq=23`、`nv=22` |
| 执行器数量 | `nu=16` |
| 机器人执行器顺序 | FL、FR、HL、HR；每条腿为 hip-x、hip-y、knee、wheel |
| 策略观测 | 57 个值：角速度、投影重力、命令、关节状态、上一帧动作 |
| 策略动作 | 16 个值：12 个腿部位置目标与 4 个轮子速度目标 |
| 物理 / 策略频率 | 1 ms 物理步长，20 ms 策略更新 |
| 仿真初始姿态 | 0.58 m 基座高度的 M20 `home` 站立关键帧 |
| 仿真规划边界 | 0.70 m 高度、0.50 m 径向净空 |

真实 M20 导航配置刻意与仿真不同：它使用 1.00 m 顶部边界和 0.55 m 硬墙净空，以覆盖裸 MuJoCo
本体以外的实体硬件。不要为了与仿真一致而降低真实机器人的安全参数。

## 配置与临时覆盖

修改
[mujoco_sim.yaml](/dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml)
可保存传感器和仿真边界配置，修改后需重启 DimOS。临时测试无需改源码：

```sh skip
cd ~/work/dimos_m20
uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim \
  --option m20mujocosimconnection.pointcloud_fps=1.0 \
  --option m20movingobstacle.enabled=false
```

默认配置发布 640 x 360、10 Hz 的 RGB，以及 2 Hz 的前/左/右合并深度点云。移动 mocap 人物对相机可见，
但默认不会与 M20 发生物理碰撞，因为 mocap 物体质量等效为无穷大。

## 验证清单

修改模型、控制器或配置后，执行以下检查：

```sh skip
cd ~/work/dimos_m20
uv run --no-sync python -m pytest -q \
  dimos/simulation/mujoco/test_m20_policy.py \
  dimos/simulation/mujoco/test_mujoco_process.py \
  dimos/robot/deeprobotics/m20/nav/test_m20_simple_nav_sim.py \
  dimos/robot/deeprobotics/m20/nav/test_m20_dan_nav_sim.py
```

最低通过标准是：模型契约正确、控制器输出为有限值、两个蓝图均解析
`robot_model="deeprobotics_m20"`、RGB 和点云流非空，且 MuJoCo 进程可正常启动和停止。交互测试前先做
有限时长的 headless 启动：

```sh skip
cd ~/work/dimos_m20
timeout --signal=INT --kill-after=10s 30s \
  uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim
```

## 录制与回放

录制 `.rrd` 时必须先启动 Rerun，再启动 DimOS，确保录制器先占用端口 `9877`。完整的启动顺序、
SQLite `nav-record` 的限制与回放命令见
[M20 MuJoCo 运行指南的录制与回放章节](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md#recording-and-replay)。

## 常见问题

| 现象 | 可能原因 | 处理方式 |
| --- | --- | --- |
| `Unknown robot policy: deeprobotics_m20` | 当前代码早于集成提交 | 获取并切换 `codex/m20-official-mujoco-model`，或合并提交 `014403f5` |
| `Error opening file '*.STL'` | 工作区或 wheel 缺少资产 | 检查 `assets/meshes/`，然后执行 `uv sync` |
| ONNX session 无法加载 | 策略缺失、损坏，或替换了其他 M20 策略 | 从受跟踪分支恢复 `deeprobotics_m20_policy.onnx`，并检查其 57/16 接口 |
| 光线追踪报 `nix: not found` | DimOS 原生环境不完整 | 安装/配置 Nix，或使用已完成原生构建的工作区 |
| Cargo 无法解析 `Cargo.lock` | Rust/Cargo 版本低于仓库 lockfile 要求 | 更新 Rust/Cargo 后重新构建 MLS 可执行文件 |
| 机器人出现在自己的点云中 | 深度几何组包含 `2` 或 `3` | 保持 `pointcloud_geom_groups: [0, 1]` |
| M20 在拥挤办公场景中不动 | 出生点有障碍，或运动命令未到达 `cmd_vel` | 从配置的 `(-1, 1)` 出生点测试，检查 `dimos/slam_odom`，再检查 `cmd_vel` |
| 同事称其为 M20 Pro | 公开来源只标识为 M20 | 在宣称仿真保真前，向硬件负责人确认机械与传感器等价性 |

## 更新到新版官方资产

1. 在 `assets/SOURCE.md` 记录上游仓库地址、不可变提交、许可证和精确源文件路径。
2. 替换资产前比对 MJCF 名称、关节顺序、执行器顺序、相机名称、模型维度和 ONNX 输入/输出签名。
3. 只有新策略契约完全一致时，才保留当前 M20 控制器映射；否则需要实现并测试新的控制器契约。
4. 执行验证清单、两个导航蓝图各一次有限时长启动，并检查 RGB 与深度输出。
5. 在同一提交中更新本 README、`nav/mujoco_sim.md` 和来源说明。

不要用仅有可视网格的模型替代匹配的行走策略与执行器契约。外观正确但控制器不兼容的机器人，不能作为可用的导航仿真。
