# realtime_trajectory_fit_gui MEX 求解核心

## 目的

本目录用于承载 `realtime_trajectory_fit_gui` 的 C++/MEX 重计算核心，目标是把以下高耗时环节迁出纯 MATLAB：

- 逐点逆运动学求解
- 连续 seed 选解
- 自碰撞 / 工具-本体 / 工具-basin 最小间隙检测
- 回放关节轨迹生成
- 连续性与 clearance 统计

## 当前入口

- MEX 主入口：`rtfg_solver_mex`
- MATLAB 编译脚本：`build_mex.m`

## 依赖

当前 Linux 主机首版依赖：

- `Eigen`
- `orocos-kdl`
- `kdl_parser`
- `urdfdom`
- `FCL`

本机实测可用的头文件 / 库路径：

- `/usr/include/eigen3`
- `/usr/include`
- `/opt/ros/humble/include`
- `/usr/lib/x86_64-linux-gnu`
- `/opt/ros/humble/lib`
- `/opt/ros/humble/lib/x86_64-linux-gnu`

## 编译

在 MATLAB 中执行：

```matlab
addpath(fullfile(pwd, 'realtime_trajectory_fit_gui', 'mex'));
build_mex
```

编译成功后，MEX 产物会输出到：

```text
realtime_trajectory_fit_gui/mex/bin/
```

## 运行要求

主 GUI 会优先尝试调用：

```matlab
rtfg_solver_mex
```

如果 MEX 未编译或加载失败，会明确报错并提示先运行 `build_mex`，不会静默回退到旧的纯 MATLAB 主链。

## 说明

- `test/capsule-8515919` 已评估，不适合作为当前主链求解核心直接复用。
- 当前 MEX 实现面向 **UR10 专用** 工作流，不尝试抽象成通用机器人框架。
