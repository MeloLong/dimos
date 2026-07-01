# Navigation

DimOS 目前有两套导航实现：**Simple Navigation** 和 **Nav Stack**。它们不是两套机器人框架，而是运行在同一个 DimOS `Module` / `Blueprint` / typed stream 基础设施上的两条导航技术路线。

简单说：

- **Simple Navigation** 是轻量、直接、易调试的 native navigation pipeline。
- **Nav Stack** 是更完整的模块化自主导航栈，覆盖地形分析、全局规划、局部规划、轨迹跟随、PGO 和探索。

这两套系统未来会逐步合并；当前文档先把它们分开描述，方便理解现有代码和蓝图。

## Quick Links

- [Simple Navigation](/docs/capabilities/navigation/native/index.md)
- [Nav Stack](/docs/capabilities/navigation/nav_stack.md)
- [Nav Stack Module Guide](/docs/capabilities/navigation/nav_stack_module_guide.md)

## Relationship

两者共享 DimOS 的运行模型：

- 通过 `Blueprint` 组合模块
- 通过 typed streams 传递 `PointCloud2`、`Odometry`、`OccupancyGrid`、`Twist` 等消息
- 不依赖 ROS Nav2 作为导航框架
- 可以接入 agent skill、viewer、teleop 和 robot control 模块

但它们的导航算法、地图表示、规划链路和主要代码目录不同：

- Simple Navigation 主要由 `VoxelGridMapper` / `RayTracingVoxelMap`、`CostMapper`、`ReplanningAStarPlanner` 组成。
- Nav Stack 主要由 `TerrainAnalysis`、`TerrainMapExt`、`PGO`、`FarPlanner` 或 `SimplePlanner`、`LocalPlanner`、`PathFollower` 组成。

注意：Nav Stack 里也有一个 `SimplePlanner` 选项，它只是 Nav Stack 内部的 A* 全局规划器，不等于这里说的 **Simple Navigation**。

## Simple Navigation

Simple Navigation 是当前 Go2 和 G1 simple nav 蓝图使用的轻量导航链路。它把传感器点云转换成全局 voxel map，再投影成 2D costmap，最后用 A* 重规划器生成运动命令。

典型数据流：

```text
Robot sensors -> voxel map -> costmap -> ReplanningAStarPlanner -> cmd_vel
```

核心模块：

| Module | Role |
|--------|------|
| `VoxelGridMapper` | Go2 常用的稀疏 3D voxel map，支持 column carving |
| `RayTracingVoxelMap` | G1 simple nav 使用的 ray-tracing voxel map |
| `CostMapper` | 将 3D map 转换为 2D occupancy/cost map |
| `ReplanningAStarPlanner` | 基于 costmap 持续重规划路径 |
| `MovementManager` | 管理目标点、teleop 和 nav velocity mux |

主要特点：

- **实现简单**：模块少，链路短，适合快速跑通导航。
- **调试直接**：map、costmap、planner 输出关系清晰。
- **动态障碍响应快**：Go2 的 column carving 会用最新 LiDAR frame 替换局部地图区域，减少 ghost obstacles。
- **适合室内和中小范围场景**：Go2 文档中提到可稳定覆盖较大室内区域，但长期里程计漂移仍会累积。
- **没有完整闭环优化链路**：主要依赖机器人或 SLAM 提供的 odometry，缺少 Nav Stack 里的 PGO corrected odometry。

典型蓝图：

- `unitree_go2`
- `unitree_g1_nav_simple`

## Nav Stack

Nav Stack 是更完整的 autonomous navigation stack。它面向 LiDAR-equipped robot，输入 registered point cloud 和 odometry，内部完成地形分类、地图累积、全局规划、局部轨迹选择、路径跟随和 loop-closure-corrected mapping。

典型数据流：

```text
registered_scan + odometry
  -> PGO + TerrainAnalysis + TerrainMapExt
  -> FarPlanner/SimplePlanner
  -> LocalPlanner
  -> PathFollower
  -> nav_cmd_vel
```

核心模块：

| Module | Role |
|--------|------|
| `PGO` | pose graph optimization，输出 corrected odometry 和 global map |
| `TerrainAnalysis` | 从点云中分类 ground / obstacle / terrain |
| `TerrainMapExt` | 累积和维护扩展地形地图 |
| `FarPlanner` | 默认全局规划器，适合长距离和室外场景 |
| `SimplePlanner` | Nav Stack 内部的轻量 A* 全局规划器 |
| `LocalPlanner` | 局部轨迹选择和避障 |
| `PathFollower` | 将路径转换为速度命令 |
| `MovementManager` | 将 `nav_cmd_vel`、teleop 和 clicked goal 整合到 `cmd_vel` |

主要特点：

- **模块化更强**：规划、地形分析、PGO、轨迹跟随分层清楚。
- **地图和定位更完整**：PGO 提供 `corrected_odometry` 和 `global_map_pgo`。
- **更适合复杂自主导航**：支持 terrain analysis、local/global planning 和 frontier exploration。
- **可配置程度高**：`create_nav_stack()` 支持 planner、terrain、speed、record、TARE 等配置。
- **迁移到新机器人更清晰**：只要提供 `registered_scan`、`odometry`，并接收 `cmd_vel`，就可以组合进新 robot blueprint。

典型入口：

```python
from dimos.navigation.nav_stack.main import create_nav_stack

blueprint = create_nav_stack()
```

## Key Differences

| Dimension | Simple Navigation | Nav Stack |
|-----------|-------------------|-----------|
| 定位 | 主要使用机器人/SLAM odometry | 有 PGO，区分 raw odometry 和 corrected odometry |
| 地图 | voxel map -> 2D costmap | terrain map、terrain_map_ext、global_map_pgo |
| 规划 | `ReplanningAStarPlanner` 持续重规划 | global planner + local planner + path follower |
| 全局规划 | A* costmap-based | `FarPlanner` 或 Nav Stack `SimplePlanner` |
| 局部避障 | 集成在重规划器/costmap 链路中 | 独立 `LocalPlanner` |
| 控制输出 | planner/manager 产生 `cmd_vel` | `PathFollower` 产生 `nav_cmd_vel`，通常经 `MovementManager` mux 成 `cmd_vel` |
| 复杂度 | 低 | 高 |
| 可调试性 | 链路短，容易观察 | 模块多，但每层职责更清楚 |
| 适用场景 | 快速 demo、室内、中小范围、机器人原生点云链路 | 长距离、更复杂地形、需要闭环修正和完整 autonomy stack |
| 典型代码 | `dimos/mapping/*` + `dimos/navigation/replanning_a_star/*` | `dimos/navigation/nav_stack/*` |

## Which One Should I Use?

优先选 **Simple Navigation**，如果你需要：

- 快速跑通 Go2 或 G1 的导航
- 更少模块、更短链路
- 主要在室内或可控环境中导航
- 直接观察 voxel map、costmap 和 A* planner 的效果

优先选 **Nav Stack**，如果你需要：

- 更完整的自主导航能力
- 更好的地形建模和障碍处理
- PGO corrected odometry 和全局地图
- 全局规划、局部规划、路径跟随分层调试
- 将导航栈迁移到新的 LiDAR-equipped robot

## Current Direction

Simple Navigation 和 Nav Stack 现在并存，是因为它们服务于不同阶段的工程需求：前者轻、快、容易跑；后者完整、可扩展、适合逐步成为统一导航栈。

长期方向是把两者的优势合并：保留 Simple Navigation 的易用性和快速反馈，同时吸收 Nav Stack 的模块化、PGO、地形分析和完整 autonomy pipeline。
