# M20 导航测试计划（README）

## 一、测试目标

本次测试旨在验证 M20 当前导航系统的整体性能，收集测试数据，为后续分析导航效果、地图质量以及系统延迟提供依据。

本次测试主要关注以下内容：

* 验证不同 Blueprint 的可用性
* 收集导航过程 Recording 和 Log
* 评估路径跟踪效果
* 评估地图质量
* 分析 Rerun 延迟问题
* 评估系统整体稳定性

---

# Day 1：Blueprint 验证与手动导航测试

## 1. 验证 Blueprint

测试以下两个 Blueprint：

### （A）m20-nav-3d

路径：

```
dimos/robot/deeprobotics/m20/blueprints/basic.py
```

### （B）m20-simple-nav

路径：

```
dimos/robot/deeprobotics/m20/nav/m20_simple_nav.py
```

分别启动：

```bash
dimos run m20-nav-3d
```

```bash
dimos run m20-simple-nav
```

检查内容：

* Blueprint 是否能够正常启动
* Camera 数据是否正常
* LiDAR 数据是否正常
* Rerun 是否正常显示
* Navigation 是否正常初始化

保存启动日志，如出现异常需记录错误信息。

---

## 2. 手动导航录制测试

选择安全、空旷、平坦的测试区域（建议办公室附近 Hangar）。

测试过程中需两人配合完成。

### 人员分工

#### 操作员 A

负责：

* 启动 Blueprint
* 操作 Laptop
* 开始/结束 Recording
* 发送导航目标（Navigation Goal）

#### 操作员 B

负责：

* 手持遥控器
* 全程安全监控
* 必要时立即 Emergency Stop

---

### 测试要求

分别对两个 Blueprint 进行测试。

每个 Blueprint：

* 至少完成 1 组 Recording
* 每组约 10 个 Navigation Goal
* 仅测试平地
* 不测试楼梯、坡道等复杂场景

保存：

* Recording
* Log 文件

重点观察：

* 是否能够正常规划路径
* 是否能够顺利到达目标点
* 路径跟踪是否准确
* 运动是否平滑
* 是否出现规划失败
* 是否出现异常停止
* 是否存在明显控制延迟

---

## 3. Blueprint 对比

比较两个 Blueprint 的测试效果。

若：

```
m20-nav-3d
```

与

```
m20-simple-nav
```

表现一致或更优，

则后续统一使用：

```
m20-nav-3d
```

否则继续使用：

```
m20-simple-nav
```

并保留相关日志供后续分析。

---

# Day 2：自动轨迹测试与数据分析

## 1. 自动测试 Blueprint

基于当前可正常运行的 Blueprint，新建自动测试 Blueprint。

修改内容：

* 去除地图点击导航
* 去除 MLSPlanner
* 直接注入预定义轨迹

建议测试轨迹：

### 直线

* 前进 4 m
* 后退 4 m
* 左移 4 m
* 右移 4 m

### 曲线

* 圆形轨迹（Circle）

运行方式例如：

```bash
dimos run m20-test-lines
```

机器人能够自动完成全部预设轨迹。

---

## 2. 自动轨迹录制

运行自动测试 Blueprint。

保存：

* Recording
* Log 文件

重点观察：

* 路径跟踪是否准确
* 运动是否平滑
* 速度是否稳定
* 是否存在轨迹偏移

若机器人运动异常，应立即停止测试。

---

## 3. Recording 数据分析

回放所有 Recording。

重点分析以下内容：

### （1）路径跟踪性能

检查：

* 是否能够准确跟踪规划轨迹
* 航向是否稳定
* 转弯是否平滑

---

### （2）轨迹质量

重点观察：

* 直线路径是否保持笔直
* 曲线路径是否平滑
* 是否出现明显偏离

---

### （3）地图质量

检查：

* 地图是否稳定
* 是否存在漂移
* 是否存在噪声
* 是否存在缺失区域

---

### （4）延迟分析

重点观察以下延迟情况：

* 导航指令 → 机器人运动
* 机器人运动 → Rerun 显示

结合测试现象，初步分析可能原因，例如：

* 网络通信
* DimOS / Deeprobotics 软件集成
* Zenoh 通信
* 计算资源占用

---

# 最终交付成果

完成测试后，应整理并提交以下内容：

* 两个 Blueprint 的验证结果
* 手动导航测试 Recording
* 自动轨迹测试 Recording
* 测试 Log
* 路径跟踪效果分析
* 地图质量分析
* Rerun 延迟现象及初步分析
* 推荐后续使用的 Blueprint
