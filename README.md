# UR10 机械臂铲泥轨迹规划与控制系统

> **完整技术手册**: [project.md](../project.md)（19 章，涵盖架构、算法、MEX 加速、运行指南）

基于 MATLAB R2025a + C++ MEX 的 UR10 六轴协作机械臂铲泥任务自动化控制系统。集成轨迹生成、逆运动学求解、碰撞检测、实时通信与 3D 可视化。

---

## 快速开始

```matlab
% 启动主 GUI（轨迹拟合系统）
cd /home/liuxiaopeng/公共的/MATLAB_ur10_control/UR10_control_Matlab
main_realtime_trajectory_fit_gui

% 启动实时关节控制 GUI（需连接 UR10 机械臂）
ur10_gui_control

% 查看 URDF 模型
show_assembly_urdf

% 编译 C++ MEX 加速模块
addpath(fullfile(pwd, 'realtime_trajectory_fit_gui', 'mex'));
build_mex
```

---

## 当前主线

```text
UR10_control_Matlab/realtime_trajectory_fit_gui
```

目标：
1. 在 MATLAB GUI 中统一显示完整场景、机械臂和铲泥轨迹
2. 让 `sensor_shovel_tcp` 按给定轨迹完成：切入段 → 圆弧段 → 垂直提升段
3. 支持环境位姿调整、轨迹参数调整、YAML 参数保存
4. 逐步把重计算核心从纯 MATLAB 迁到 C++/MEX，提高拟合求解速度

---

## 目录结构

```
UR10_control_Matlab/
├── realtime_trajectory_fit_gui/    ← ★ 当前主线
│   ├── main_realtime_trajectory_fit_gui.m  ← 主 GUI 入口
│   ├── ui/rtfg_ui.m                ← UI 构建与子窗口
│   ├── rendering/rtfg_render.m     ← 3D 场景渲染
│   ├── kinematics/rtfg_kinematics.m ← IK 求解与轨迹拟合核心
│   ├── collision/rtfg_collision.m  ← 碰撞检测（自碰撞/工具/盆体）
│   ├── trajectory/rtfg_quintic_joint_series.m ← 五次多项式插值
│   ├── io/rtfg_io.m                ← YAML/URDF 读写
│   ├── utils/rtfg_utils.m          ← 位姿/变换工具函数
│   ├── mex/                        ← C++ MEX 加速模块
│   │   ├── build_mex.m             ← MEX 编译脚本
│   │   ├── bin/rtfg_solver_mex.mexa64 ← 已编译二进制
│   │   └── src/                    ← C++ 源码 (6个 .cpp, 5个 .h)
│   └── environment_runtime_config.yaml ← 运行时配置
├── assembly/                       ← 机械臂装配模型
│   ├── assembly.urdf.xacro         ← UR10 + 铲子 + 基座
│   └── meshes/                     ← 3D 网格文件 (11个)
├── environmental_model/            ← 环境模型
│   ├── assembly_with_block_with_basin.urdf ← 完整场景
│   ├── ur10_shovel_only.urdf       ← 碰撞用机械臂模型
│   └── block_with_basin_pose.yaml  ← 环境位姿配置
├── ur10_gui_control.m              ← 实时关节控制 GUI
├── ur10_realtime_sync.m            ← 实时关节同步显示
├── show_assembly_urdf.m            ← URDF 预览
├── test/                           ← 测试与仿真
│   ├── tarjectory_plan/            ← 2D/3D 轨迹规划原型
│   └── virtual/                    ← Simulink 虚拟仿真
├── real_csv_fit_control/           ← CSV 数据拟合与回放
└── docs/                           ← 技术文档 (HTML)
```

---

## GUI 工作流

1. 打开主界面：`main_realtime_trajectory_fit_gui`
2. 调环境位姿 → 通过"底部盒子移动"子窗口
3. 调轨迹参数 → 通过"轨迹调整"子窗口（切入角、深度、轨迹截面等）
4. 点击 **"开始尖端轨迹拟合"** → 计算整条轨迹的关节解（优先 MEX，失败回退 MATLAB）
5. 点击 **"开始运行"** → 播放已拟合的关节轨迹
6. 保存 YAML / 回写 URDF → 持久化配置

---

## 当前拟合算法

面向铲泥任务的分段轨迹：

| 段 | 说明 | 姿态规则 |
|---|------|---------|
| 切入段 | TCP 沿轨迹切线方向推进 | 局部 -X 沿切线，Z 尽量朝上 |
| 圆弧段 | TCP 沿弧线运动 | 同切入段姿态 |
| 垂直提升段 | 从端点沿世界 Z 轴上升 | 固定垂直姿态（SLERP 过渡） |

---

## 碰撞检测

三类碰撞检测（间隙阈值 2mm）：

- **机械臂自碰撞**：排除相邻连杆对
- **末端工具与机械臂本体碰撞**：铲子/铲尖 vs 机械臂连杆
- **末端工具与 basin 碰撞**：解析盒体近似（5 个壁板）

---

## C++/MEX 加速状态

| 项目 | 状态 |
|------|------|
| MEX 框架与编译链 | ✅ 已完成 |
| 接口与 GUI 连接 | ✅ 已接入 `trackTrajectory` |
| MEX 编译成功 | ✅ `rtfg_solver_mex.mexa64` 已生成 |
| 整条轨迹稳定求解 | ⚠️ 优化中（首段/切入段 IK 鲁棒性待提升） |
| 回退机制 | ✅ MEX 失败 → 自动回退 MATLAB 主链 |

**技术栈**: Eigen 3 · FCL · orocos-kdl · urdfdom · ROS 2 Humble (kdl_parser/urdf)

---

## 性能数据

| 指标 | MATLAB 原生 | C++ MEX | 加速比 |
|------|------------|---------|--------|
| 单点 IK 求解 | ~8 ms | ~1.5 ms | ~5.3× |
| 单点碰撞检测 | ~1.2 ms | ~0.02 ms | ~60× |
| 完整轨迹求解 (258点) | ~8 s | ~6.9 s | ~1.2× |

---

## 环境要求

- **OS**: Ubuntu 22.04 LTS (x86_64)
- **MATLAB**: R2025a + Robotics System Toolbox
- **ROS 2**: Humble（MEX 编译需要）
- **编译器**: GCC 11+ (C++17)
- **依赖库**: Eigen 3, FCL, orocos-kdl, urdfdom, kdl_parser
- **硬件**: UR10 机械臂（可选，虚拟仿真不需要）

---

## 详细文档

完整的技术手册请参阅 **[project.md](../project.md)**，包含：

- 完整文件结构与功能说明
- UR10 运动学建模（DH 参数、正/逆运动学、雅可比）
- 逆运动学求解算法详解（DLS、KDL LMA、多级权重、多种子策略）
- 轨迹规划算法详解（三段轨迹生成、姿态规划、SLERP、自适应锚点采样）
- 碰撞检测系统（FCL、自碰撞/工具-盆体碰撞）
- MATLAB MEX C++ 实现详解（架构、源码分析、编译系统、回退机制）
- GUI 系统设计
- 实时机械臂通信（UR 协议解析、URScript 指令）
- 环境建模与 URDF
- 配置系统与 YAML
- 虚拟仿真系统（Simulink）
- 性能分析与优化
- 如何运行（含故障排除）
- 依赖项与编译
- 开发路线图

**在线文档**: [GitHub Pages](https://cagedcarrow.github.io/UR10_control_Matlab/realtime_trajectory_fit_gui_技术文档/)

---

## 推荐接手顺序

1. 先读 [project.md](../project.md) 了解整体架构
2. 从 `realtime_trajectory_fit_gui/mex/src/trajectory_solver.cpp` 入手
3. 优先解决首段和切入段 IK 稳定性
4. 跑通整条轨迹的 full-track MEX 求解
5. 再考虑进一步性能优化

---

## 关键文件索引

| 功能 | 文件 |
|------|------|
| 主 GUI | `realtime_trajectory_fit_gui/main_realtime_trajectory_fit_gui.m` |
| 运动学主链 | `realtime_trajectory_fit_gui/kinematics/rtfg_kinematics.m` |
| 碰撞逻辑 | `realtime_trajectory_fit_gui/collision/rtfg_collision.m` |
| 渲染 | `realtime_trajectory_fit_gui/rendering/rtfg_render.m` |
| IO / YAML | `realtime_trajectory_fit_gui/io/rtfg_io.m` |
| MEX 主实现 | `realtime_trajectory_fit_gui/mex/src/trajectory_solver.cpp` |
| MEX 构建 | `realtime_trajectory_fit_gui/mex/build_mex.m` |
| 环境主模型 | `environmental_model/assembly_with_block_with_basin.urdf` |
| 碰撞模型 | `environmental_model/ur10_shovel_only.urdf` |

---

> **当前状态**: MEX 框架、编译链和主入口接线已完成；整条轨迹的稳定 C++ 求解仍在继续攻关。GUI 采用 MEX 优先 + MATLAB 回退的容错策略。
