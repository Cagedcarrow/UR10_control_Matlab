# UR10_control_Matlab 项目交接说明

## 1. 当前主线

本仓库当前的主线已经不是旧的 `test/virtual` 电流模型链路，而是：

```text
UR10_control_Matlab/realtime_trajectory_fit_gui
```

这条主线的目标是：

1. 在 MATLAB GUI 中统一显示完整场景、机械臂和铲泥轨迹。
2. 让 `sensor_shovel_tcp` 按给定轨迹完成：
   - 切入段
   - 圆弧段
   - 垂直提升段
3. 支持环境位姿调整、轨迹参数调整、YAML 参数保存。
4. 逐步把重计算核心从纯 MATLAB 迁到 C++/MEX，提高拟合求解速度。

---

## 2. 目录结构

### 2.1 核心入口

- `realtime_trajectory_fit_gui/main_realtime_trajectory_fit_gui.m`
  - 主 GUI 入口
  - 当前仍是统一启动入口

### 2.2 GUI 功能分层

- `realtime_trajectory_fit_gui/ui/`
  - 主界面、子窗口、按钮、进度条、状态文本

- `realtime_trajectory_fit_gui/rendering/`
  - 场景渲染
  - 机器人显示
  - 轨迹线显示
  - 碰撞标记显示

- `realtime_trajectory_fit_gui/kinematics/`
  - 当前 MATLAB 主链的轨迹拟合、回放、连续性处理
  - 同时承担 MEX 封装入口调度

- `realtime_trajectory_fit_gui/collision/`
  - 当前 MATLAB 版碰撞检测
  - 包括：
    - 自碰撞
    - 工具-本体碰撞
    - 工具-basin 碰撞

- `realtime_trajectory_fit_gui/trajectory/`
  - 轨迹采样、quintic 插值等辅助逻辑

- `realtime_trajectory_fit_gui/io/`
  - 路径构建
  - YAML / URDF 读写

- `realtime_trajectory_fit_gui/utils/`
  - 位姿、变换、轨迹几何辅助函数

### 2.3 环境模型

- `environmental_model/assembly_with_block_with_basin.urdf`
  - 完整环境主模型

- `environmental_model/ur10_shovel_only.urdf`
  - 供碰撞和简化求解使用的机械臂 + 铲子模型

---

## 3. GUI 现有工作流

当前 GUI 的设计流程是：

1. 打开主界面：

```matlab
main_realtime_trajectory_fit_gui
```

2. 调环境位姿
   - 通过环境位姿子窗口
   - 修改 `block_with_basin` 相对机械臂基座的位置

3. 调轨迹参数
   - 通过轨迹参数子窗口
   - 修改切入角、深度、轨迹截面等

4. 点击 `开始尖端轨迹拟合`
   - 当前语义：只计算，不直接播放

5. 点击 `开始运行`
   - 播放已经求好的轨迹

6. 保存 YAML
   - 当前统一配置文件：
     - `realtime_trajectory_fit_gui/environment_runtime_config.yaml`

---

## 4. 当前拟合算法主线

### 4.1 目标

当前拟合不是简单的 TCP 定点移动，而是面向铲泥任务的分段轨迹：

1. 切入段
   - TCP 沿轨迹切线方向推进

2. 圆弧段
   - TCP 保持沿弧线运动

3. 垂直提升段
   - 从共享端点直接沿世界 `Z` 轴上升

### 4.2 姿态约束

当前姿态规则的核心语义是：

- `sensor_shovel_tcp` 的局部 `-x` 方向沿轨迹切线
- 在切入段 / 圆弧段：
  - 保持与轨迹相容的姿态
- 在提升段：
  - 共享点作为上一段终点、下一段起点
  - 之后直接沿 `Z` 轴垂直提升

### 4.3 MATLAB 主链中的主要步骤

当前 `trackTrajectory` 的核心步骤在：

- `realtime_trajectory_fit_gui/kinematics/rtfg_kinematics.m`

主流程大致是：

1. 生成显示轨迹 `displayTraj`
2. 构造目标位姿序列 `targetPlan`
3. 进行逆运动学求解
4. 生成 anchor 轨迹和 playback 轨迹
5. 做碰撞复检
6. 把结果写回：
   - `state.previewAnchorQSeries`
   - `state.previewQSeries`
   - `state.previewTcpPath`
   - `state.previewMetrics`
   - `state.collisionResults`

---

## 5. 当前碰撞检测逻辑

当前 MATLAB 主链已经接入三类碰撞检测：

1. 机械臂自碰撞
2. 末端工具与机械臂本体碰撞
3. 末端工具与 basin 碰撞

其中：

- 工具碰撞模型：
  - 来自 `ur10_shovel_only.urdf` 中的 STL collision

- 机械臂本体碰撞模型：
  - 当前主要依赖 URDF 中的简化 collision 几何

- basin：
  - 当前使用解析盒体近似
  - 不直接拿显示 mesh 做精确三角面碰撞

碰撞结果在 GUI 中支持：

- 状态栏摘要
- 红点标记
- basin 碰撞壁板高亮

---

## 6. 当前性能瓶颈

当前纯 MATLAB 版慢，主要不是界面问题，而是下面三个重计算环节：

1. `inverseKinematics`
2. `checkCollision`
3. 连续回放轨迹逐点生成

也就是说，真正慢的是：

- 逆运动学
- 最小间隙检测
- 连续轨迹求解

因此后续加速主线确定为：

> 把重计算核心迁到 C++/MEX，MATLAB 只保留 GUI、IO 和渲染。

---

## 7. 当前 C++/MEX 实施状态

### 7.1 已完成部分

已经新增：

```text
realtime_trajectory_fit_gui/mex
```

包含：

- `mex/build_mex.m`
  - MATLAB 编译脚本

- `mex/src/rtfg_solver_mex.cpp`
  - 当前首版 C++ MEX 主入口

- `mex/bin/rtfg_solver_mex.mexa64`
  - 已在本机 MATLAB 2025 上编译成功

- `mex/README.md`
  - 构建与依赖说明

### 7.2 当前 MEX 采用的库

当前首版 MEX 代码直接使用本机可用依赖：

- `Eigen`
- `FCL`
- `orocos-kdl`
- `urdfdom`
- ROS humble 的：
  - `kdl_parser`
  - `urdf`

### 7.3 已验证通过的内容

以下内容已经真实验证过：

1. `mex -setup C++` 可用
2. `build_mex` 可成功编译
3. `rtfg_solver_mex.mexa64` 已成功生成
4. MEX 单点接口能被 MATLAB 调用
5. MEX 能完成最基本的：
   - URDF 读取
   - basin 盒体输入读取
   - 输出结构回传

### 7.4 当前还没有完全打通的地方

必须明确说明：

> 当前 MEX 主链已经“编译成功 + 接口接上”，但**还没有完全替代整条轨迹拟合主链**。

当前真实状态是：

- `trackTrajectory` 已优先尝试调用 `rtfg_solver_mex`
- 但 MEX 对整条轨迹的连续 IK 还不够稳
- 在切入段较早位置就可能失败
- 错误表现类似：
  - `第 1/272 个目标位姿 MEX 求解失败`
  - 或 `第 5/272 个目标位姿 MEX 求解失败`

根因不是语法问题，也不是编译问题，而是：

1. 当前 C++ 里的自写数值 IK 还不够鲁棒
2. 首段目标位姿离当前姿态较远
3. 连续多点轨迹的全量 C++ 求解还没有达到稳定可用状态

### 7.5 当前 GUI 对 MEX 的处理方式

现在的 `trackTrajectory` 已经改成：

1. 优先调用 MEX
2. 如果 MEX 求解失败
3. 显式回退到旧 MATLAB 主链

也就是说：

- GUI 不会因为 MEX 失败而彻底不可用
- 但目前也不能宣称“全量 C++ 已完成”

这个回退逻辑在：

- `realtime_trajectory_fit_gui/kinematics/rtfg_kinematics.m`

---

## 8. 为什么没有直接复用 `test/capsule-8515919`

已经评估过：

```text
test/capsule-8515919
```

结论是：

- 它主要是逆动力学代码
- 不覆盖当前需要的：
  - 逆运动学
  - 自碰撞
  - 工具-本体碰撞
  - 工具-basin 碰撞
  - 连续 playback 轨迹生成

所以当前结论固定为：

> `capsule-8515919` 只作为本地参考，不接入当前主链。

---

## 9. 当前最重要的接手点

如果换其他 AI 或开发者继续接手，最应该先做的是：

### 9.1 优先收口 MEX 的 IK 核心

当前真正卡住的不是构建系统，而是：

- `realtime_trajectory_fit_gui/mex/src/rtfg_solver_mex.cpp`

后续重点应该放在：

1. 提高首段 / 切入段 IK 的鲁棒性
2. 稳定多 seed 连续选解
3. 保证共享端点附近不跳分支
4. 让整条轨迹完整走通

### 9.2 当前最现实的两条路线

后续建议二选一：

#### 路线 A：继续增强当前自写数值 IK

优点：
- 已经有现成 MEX 骨架
- 接口已经接上 GUI
- 不需要大改主链

缺点：
- 还需要较多调试才能达到 MATLAB 版稳定性

#### 路线 B：把 C++ 求解器换成更成熟的 IK 核心

比如：

- `TRAC-IK`
- 或更成熟的 KDL 数值 IK 组合

优点：
- 更有希望较快达到稳态求解

缺点：
- 需要重新整理 MEX 内核和依赖接口

### 9.3 碰撞检测层目前不应该优先重写

当前优先级不是：

- mesh 更精细
- basin 更复杂
- 工业级碰撞框架抽象

而是应该先让：

> “整条轨迹完整求出 + 不跳分支 + GUI 能稳定播放”

先达成。

---

## 10. 本机环境说明

### 10.1 MATLAB

本机 MATLAB 根目录：

```text
/mnt/data_ntfs3g/Matlab2025
```

### 10.2 当前仓库真实 Git 根

注意真实 Git 仓库根不是外层目录，而是：

```text
/home/liuxiaopeng/公共的/MATLAB_ur10_control/UR10_control_Matlab
```

### 10.3 MEX 编译方式

当前已经验证可用：

```matlab
addpath(fullfile(pwd, 'realtime_trajectory_fit_gui', 'mex'));
build_mex
```

---

## 11. 当前交接结论

截至本 README 重写时，项目状态可以准确概括为：

1. `realtime_trajectory_fit_gui` 是当前唯一主线
2. MATLAB GUI、轨迹拟合、环境位姿调整、碰撞显示都已经成型
3. C++/MEX 加速链路已经搭好，并在本机编译成功
4. MEX 已经接入 `trackTrajectory`
5. 但整条轨迹的全量 C++ 连续 IK 求解还没有最终跑通
6. 当前 GUI 采用：
   - 先尝试 MEX
   - 失败后显式回退到 MATLAB 主链

所以当前最准确的状态不是“已完成 C++ 替换”，而是：

> **MEX 框架、编译链和主入口接线已完成；整条轨迹的稳定 C++ 求解仍需继续攻关。**

---

## 12. 推荐接手顺序

建议新的 AI / 开发者按这个顺序继续：

1. 先从 `realtime_trajectory_fit_gui/mex/src/rtfg_solver_mex.cpp` 入手
2. 优先解决首段和切入段 IK 稳定性
3. 跑通整条轨迹的 full-track MEX 求解
4. 再考虑进一步性能优化
5. 最后再考虑完全移除 MATLAB 回退主链

---

## 13. 相关关键文件索引

### 主 GUI

- `realtime_trajectory_fit_gui/main_realtime_trajectory_fit_gui.m`

### 运动学主链

- `realtime_trajectory_fit_gui/kinematics/rtfg_kinematics.m`

### 碰撞逻辑

- `realtime_trajectory_fit_gui/collision/rtfg_collision.m`

### 渲染

- `realtime_trajectory_fit_gui/rendering/rtfg_render.m`

### IO / YAML / URDF

- `realtime_trajectory_fit_gui/io/rtfg_io.m`

### MEX 构建入口

- `realtime_trajectory_fit_gui/mex/build_mex.m`

### MEX 主实现

- `realtime_trajectory_fit_gui/mex/src/rtfg_solver_mex.cpp`

### 环境主模型

- `environmental_model/assembly_with_block_with_basin.urdf`

### 碰撞用机械臂模型

- `environmental_model/ur10_shovel_only.urdf`

---

## 14. 一句话交接说明

如果只看一句话：

> 现在要继续做的不是重新搭 GUI，而是把 `realtime_trajectory_fit_gui/mex/src/rtfg_solver_mex.cpp` 的连续 IK 求解真正做稳定，让整条轨迹在 C++ 里完整跑通。
