# UR10 机械臂铲泥轨迹规划与控制系统 — 项目技术手册

> **版本**: v2.1  
> **更新日期**: 2026-07-15  
> **作者**: Cagedcarrow  
> **适用 MATLAB 版本**: MATLAB R2025a  
> **适用 ROS 版本**: ROS 2 Humble  
> **目标平台**: Ubuntu 22.04 LTS (x86_64)  

---

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构总览](#2-系统架构总览)
3. [完整文件结构与功能说明](#3-完整文件结构与功能说明)
4. [UR10 机械臂运动学建模](#4-ur10-机械臂运动学建模)
5. [逆运动学求解算法详解](#5-逆运动学求解算法详解)
6. [轨迹规划算法详解](#6-轨迹规划算法详解)
7. [碰撞检测系统](#7-碰撞检测系统)
8. [MATLAB MEX C++ 加速实现](#8-matlab-mex-c-加速实现)
9. [GUI 系统设计](#9-gui-系统设计)
10. [实时机械臂通信与控制](#10-实时机械臂通信与控制)
11. [环境建模与 URDF](#11-环境建模与-urdf)
12. [配置系统与 YAML](#12-配置系统与-yaml)
13. [虚拟仿真系统](#13-虚拟仿真系统)
14. [性能分析与优化](#14-性能分析与优化)
15. [如何运行](#15-如何运行)
16. [依赖项与编译](#16-依赖项与编译)
17. [故障排除与常见问题](#17-故障排除与常见问题)
18. [开发路线图](#18-开发路线图)
19. [附录](#19-附录)

---

## 1. 项目概述

### 1.1 项目背景

本项目是一个完整的 **UR10 六轴协作机械臂控制系统**，专注于实现**铲泥（scooping）任务**的自动化轨迹规划与执行。系统集成了 MATLAB 的高级可视化与算法开发能力，以及 C++ 的高性能计算能力（通过 MEX 接口），形成了一个从轨迹生成、逆运动学求解、碰撞检测到实时机械臂控制的完整闭环。

### 1.2 核心功能

| 功能模块 | 说明 |
|---------|------|
| **三维铲泥轨迹生成** | 自动生成切入段→圆弧段→垂直提升段的三段连续轨迹 |
| **逆运动学求解** | 多种子策略 IK 求解器，支持 MATLAB 原生和 C++ MEX 加速双链路 |
| **碰撞检测** | 三类碰撞检测：机械臂自碰撞、工具-本体碰撞、工具-盆体碰撞 |
| **实时机械臂通信** | 通过 TCP/IP 与 UR10 控制器实时通信（端口 30003/30002） |
| **3D 可视化** | 基于 MATLAB Robotics System Toolbox 的完整场景渲染 |
| **参数化调参** | GUI 滑条实时调整轨迹参数和环境位姿 |
| **YAML 配置持久化** | 轨迹参数和环境位姿的 YAML 文件读写 |
| **虚拟仿真模式** | 支持 Simulink 虚拟机械臂仿真，无需物理硬件 |

### 1.3 技术栈

```
┌─────────────────────────────────────────────────────┐
│                    MATLAB R2025a                     │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────────┐ │
│  │ GUI (ui) │ │ Render   │ │ Kinematics (IK/FK)   │ │
│  └──────────┘ └──────────┘ └──────────────────────┘ │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────────┐ │
│  │ Collision│ │ IO/YAML  │ │ Trajectory Planning  │ │
│  └──────────┘ └──────────┘ └──────────────────────┘ │
├─────────────────────────────────────────────────────┤
│              C++ MEX 加速层 (rtfg_solver_mex)        │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────────┐ │
│  │ IK Solver│ │ Collision│ │ Trajectory Solver    │ │
│  │ (KDL+DLS)│ │ (FCL)    │ │ (Full Pipeline)      │ │
│  └──────────┘ └──────────┘ └──────────────────────┘ │
├─────────────────────────────────────────────────────┤
│  依赖库: Eigen3 | FCL | orocos-kdl | urdfdom | ROS2 │
└─────────────────────────────────────────────────────┘
```

### 1.4 当前项目状态

本项目处于 **活跃开发阶段**。MATLAB 主链已完全功能可用；C++ MEX 加速链路已完成编译框架和接口连接，但整条轨迹的连续 IK 稳定求解仍在优化中。当前 GUI 采用"MEX 优先尝试 → 失败自动回退 MATLAB 主链"的容错策略。

---

## 2. 系统架构总览

### 2.1 整体架构图

```
┌──────────────────────────────────────────────────────────────┐
│                  main_realtime_trajectory_fit_gui             │
│                      (主 GUI 入口)                            │
├──────────────────────────────────────────────────────────────┤
│  ┌─────────┐  ┌──────────┐  ┌───────────┐  ┌─────────────┐  │
│  │   UI    │  │ Rendering│  │Kinematics │  │  Collision  │  │
│  │ (rtfg_  │  │ (rtfg_   │  │ (rtfg_    │  │  (rtfg_     │  │
│  │  ui.m)  │  │ render.m)│  │kinematics │  │ collision.m)│  │
│  │         │  │          │  │   .m)     │  │             │  │
│  └────┬────┘  └────┬─────┘  └─────┬─────┘  └──────┬──────┘  │
│       │            │              │                │         │
│  ┌────┴────────────┴──────────────┴────────────────┴──────┐  │
│  │                    State Bus (state struct)             │  │
│  └────────────────────────────────────────────────────────┘  │
│  ┌──────────┐  ┌───────────┐  ┌──────────────────────────┐  │
│  │   IO     │  │Trajectory │  │        Utils             │  │
│  │ (rtfg_   │  │(rtfg_quin │  │     (rtfg_utils.m)       │  │
│  │  io.m)   │  │tic_*.m)   │  │                          │  │
│  └──────────┘  └───────────┘  └──────────────────────────┘  │
├──────────────────────────────────────────────────────────────┤
│                    MEX Interface Layer                        │
│  ┌──────────────────────────────────────────────────────┐    │
│  │              rtfg_solver_mex.mexa64                   │    │
│  │  ┌─────────┐ ┌──────────┐ ┌────────────────────┐    │    │
│  │  │IK Solver│ │Collision │ │Trajectory Solver   │    │    │
│  │  │(C++ DLS │ │Checker   │ │(Anchor → Playback  │    │    │
│  │  │+ KDL)   │ │(FCL lib) │ │ → Collision Audit) │    │    │
│  │  └─────────┘ └──────────┘ └────────────────────┘    │    │
│  └──────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 数据流

```
用户调整参数 (GUI Sliders)
       │
       ▼
轨迹参数更新 (state.trajParams / state.pose)
       │
       ▼
刷新场景 → 重新导入 URDF → 重建碰撞环境
       │
       ▼
生成 3D 铲泥轨迹 (generate_trajectory_3d)
       │
       ▼
应用环境位姿变换 (rtfg_utils/applyPoseToTrajectory)
       │
       ▼
构建目标位姿序列 (targetPlan: 位置 + 姿态)
       │
       ▼
┌──────────────────────────────────────────┐
│  轨迹拟合 (trackTrajectory)              │
│  ┌────────────────────────────────────┐  │
│  │ 尝试 MEX 求解 (rtfg_solver_mex)    │  │
│  │  ├─ 成功 → 获得 anchorQ + playbackQ│  │
│  │  └─ 失败 → 回退 MATLAB 主链       │  │
│  │           ├─ 自适应锚点采样         │  │
│  │           ├─ 逐锚点 IK 求解         │  │
│  │           ├─ 连续回放轨迹生成       │  │
│  │           └─ 碰撞复检               │  │
│  └────────────────────────────────────┘  │
└──────────────────────────────────────────┘
       │
       ▼
结果写回 state (previewQSeries, tcpPath, collisionResults)
       │
       ▼
渲染更新 (显示轨迹线、碰撞标记、TCP 点)
       │
       ▼
点击"开始运行" → 关节空间播放预览
```

### 2.3 模块职责

| 模块 | 文件 | 职责 |
|------|------|------|
| **主入口** | `main_realtime_trajectory_fit_gui.m` | GUI 初始化、回调绑定、状态管理 |
| **UI 层** | `ui/rtfg_ui.m`, `ui/rtfg_ui_text.m` | 界面构建、滑条控件、状态同步 |
| **渲染层** | `rendering/rtfg_render.m` | 3D 场景绘制、轨迹线、碰撞标记 |
| **运动学层** | `kinematics/rtfg_kinematics.m` | IK 求解调度、MEX 回退、轨迹拟合 |
| **碰撞层** | `collision/rtfg_collision.m` | 自碰撞/工具碰撞/盆体碰撞检测 |
| **IO 层** | `io/rtfg_io.m` | YAML 读写、URDF 读写、路径管理 |
| **轨迹层** | `trajectory/rtfg_quintic_joint_series.m` | 五次多项式关节空间插值 |
| **工具层** | `utils/rtfg_utils.m` | 位姿变换、轨迹变换、向量运算 |
| **MEX 层** | `mex/src/*.cpp`, `mex/src/*.h` | C++ 高性能 IK/碰撞/轨迹求解 |

---

## 3. 完整文件结构与功能说明

### 3.1 项目根目录

```
MATLAB_ur10_control/
├── project.md                          ← 本技术手册
├── types.json                          ← 项目类型配置
├── guijidata/                          ← 历史轨迹数据
│   └── 05_10_173354/
│       ├── session_data.csv            ← 采集的关节数据 (CSV)
│       ├── session_data.xlsx           ← 采集的关节数据 (Excel)
│       └── session_metadata.txt        ← 采集会话元数据
├── .claude/                            ← Claude Code 配置
│   └── settings.local.json
├── .vscode/                            ← VS Code 配置
│   ├── settings.json
│   └── c_cpp_properties.json
└── UR10_control_Matlab/                ← 主项目目录 (Git 根目录)
    ├── README.md                       ← 项目交接说明
    ├── .gitignore
    ├── .mcp.json                       ← MATLAB MCP 服务器配置
    ├── types.json                      ← 项目类型标记
    ├── ur10_gui_control.m              ← 实时关节控制 GUI
    ├── ur10_realtime_sync.m            ← 实时关节同步显示
    ├── show_assembly_urdf.m            ← URDF 模型预览
    ├── assembly/                       ← 机械臂装配模型
    │   ├── assembly.urdf.xacro         ← 主 URDF (xacro 格式)
    │   ├── assembly.zip                ← 模型打包
    │   └── meshes/                     ← 3D 网格文件
    │       ├── base.dae                ← UR10 基座
    │       ├── shoulder.dae            ← 肩部
    │       ├── upperarm.dae            ← 上臂
    │       ├── forearm.dae             ← 前臂
    │       ├── wrist1.dae              ← 腕部1
    │       ├── wrist2.dae              ← 腕部2
    │       ├── wrist3.dae              ← 腕部3
    │       ├── base_link.STL           ← 铲子本体
    │       ├── shovel_tcp.STL          ← 铲尖 TCP
    │       ├── base_link_world_jizuo.STL ← 世界基座
    │       └── base_ur10_with_dizuo.STL ← UR10 底座
    ├── environmental_model/            ← 环境模型
    │   ├── assembly_with_block_with_basin.urdf  ← 完整场景 URDF
    │   ├── ur10_shovel_only.urdf       ← 碰撞检测专用机械臂模型
    │   ├── world.urdf                  ← 世界坐标系定义
    │   ├── block_with_basin.urdf       ← 方块+盆体模型
    │   ├── solid_block.urdf            ← 纯方块模型
    │   ├── rectangular_basin.urdf      ← 矩形盆体模型
    │   ├── block_with_basin_pose.yaml  ← 环境位姿配置文件
    │   ├── urdf_viewer.m               ← URDF 可视化工具
    │   └── adjust_block_with_basin_pose_gui.m ← 位姿调节 GUI
    ├── realtime_trajectory_fit_gui/    ← ★ 当前主线 ★
    │   ├── main_realtime_trajectory_fit_gui.m ← 主入口
    │   ├── environment_runtime_config.yaml    ← 运行时配置
    │   ├── ui/                         ← UI 模块
    │   │   ├── rtfg_ui.m               ← 主 UI 构建与子窗口
    │   │   └── rtfg_ui_text.m          ← UI 文本格式化
    │   ├── rendering/                  ← 渲染模块
    │   │   └── rtfg_render.m           ← 3D 场景渲染
    │   ├── kinematics/                 ← 运动学模块
    │   │   └── rtfg_kinematics.m       ← IK 求解与轨迹拟合核心
    │   ├── collision/                  ← 碰撞检测模块
    │   │   └── rtfg_collision.m        ← 碰撞检测与间隙计算
    │   ├── trajectory/                 ← 轨迹插值模块
    │   │   ├── rtfg_quintic_joint_series.m    ← 五次多项式插值
    │   │   └── rtfg_downsample_target_plan_keep_boundaries.m
    │   ├── io/                         ← 输入输出模块
    │   │   └── rtfg_io.m               ← YAML/URDF 读写
    │   ├── utils/                      ← 工具函数模块
    │   │   └── rtfg_utils.m            ← 位姿/变换/向量运算
    │   └── mex/                        ← C++ MEX 加速模块
    │       ├── README.md               ← MEX 构建说明
    │       ├── build_mex.m             ← MEX 编译脚本
    │       ├── benchmark_mex_vs_matlab.m ← 性能对比测试
    │       ├── benchmark_simple.m      ← 简单性能测试
    │       ├── bin/
    │       │   └── rtfg_solver_mex.mexa64 ← 已编译的 MEX 二进制
    │       └── src/                    ← C++ 源代码
    │           ├── types.h             ← 数据类型定义
    │           ├── utils.h / utils.cpp ← 数学工具与 mxArray 转换
    │           ├── robot_model.h / robot_model.cpp ← URDF 加载与 FK
    │           ├── ik_solver.h / ik_solver.cpp ← IK 求解器
    │           ├── collision_checker.h / collision_checker.cpp ← FCL 碰撞
    │           ├── trajectory_solver.h / trajectory_solver.cpp ← 轨迹求解
    │           └── mex_entry.cpp       ← MEX 入口函数
    ├── test/                           ← 测试与实验代码
    │   ├── tarjectory_plan/            ← 轨迹规划测试
    │   │   ├── generate_trajectory.m   ← 轨迹生成算法
    │   │   ├── main_gui.m              ← 轨迹参数 GUI
    │   │   └── plot_trajectory.m       ← 轨迹绘图
    │   ├── virtual/                    ← 虚拟仿真
    │   │   ├── build_ur10_virtual_robot.m  ← 虚拟机器人构建
    │   │   ├── init_virtual_ur10.m     ← 虚拟 UR10 初始化
    │   │   ├── startup_virtual_ur10.m  ← 虚拟 UR10 启动
    │   │   ├── create_or_load_virtual_simulink_model.m ← Simulink 模型
    │   │   ├── ur10_virtual_jointspace.slx ← Simulink 关节空间模型
    │   │   ├── evaluate_current_estimation_batch.m ← 电流估算批处理
    │   │   ├── nihe_gui.m              ← 数据拟合 GUI
    │   │   ├── run_virtual_trajectory_demo.m ← 虚拟轨迹演示
    │   │   ├── ur10_gui_control_virtual.m ← 虚拟控制 GUI
    │   │   └── SIMULINK_NODES_EXPLAIN.md ← Simulink 节点说明
    │   └── capsule-8515919/            ← 参考代码 (逆动力学，不接入主链)
    │       ├── PROJECT_OVERVIEW.md
    │       └── UR10_MEASUREMENT_NODE.md
    ├── real_csv_fit_control/           ← CSV 数据拟合控制
    │   ├── real_csv_fit_gui.m          ← CSV 拟合 GUI
    │   ├── realCsvFitLoadRobot.m       ← 加载机器人模型
    │   ├── realCsvFitReadCsv.m         ← 读取 CSV 数据
    │   ├── realCsvFitFitTrajectory.m   ← 轨迹拟合
    │   ├── realCsvFitBuildTable.m      ← 构建数据表
    │   ├── realCsvFitCheckLimits.m     ← 限位检查
    │   ├── realCsvFitCompactPath.m     ← 路径压缩
    │   ├── realCsvFitSendMoveJ.m       ← 发送 movej 指令
    │   ├── realCsvFitSendStopJ.m       ← 发送 stopj 指令
    │   ├── realCsvFitSendScript.m      ← 发送 URScript
    │   ├── realCsvFitRenderRobot.m     ← 渲染机器人
    │   ├── realCsvFitDefaultConfig.m   ← 默认配置
    │   ├── realCsvFitBuildServoJScript.m ← 构建 ServoJ 脚本
    │   ├── realCsvFitBuildExecutionTrajectory.m ← 构建执行轨迹
    │   ├── realCsvFitReadLatestUrQActual.m ← 读取实时关节数据
    │   ├── realCsvFitDisableAxesInteractions.m ← 禁用坐标轴交互
    │   ├── realCsvFitCleanupOldInstances.m ← 清理旧实例
    │   └── error_check/                ← 错误检查数据
    └── docs/                           ← 文档
        ├── index.html                  ← 文档索引
        ├── 参考论文架构加速代码/
        └── realtime_trajectory_fit_gui_技术文档/
            ├── index.html
            ├── architecture_modules.html
            ├── trajectory_generation.html
            ├── trajectory_fitting.html
            ├── cpp_acceleration_design.html
            ├── performance_comparison.html
            └── optimization_changelog.html
```

### 3.2 核心文件功能详解

#### 3.2.1 `main_realtime_trajectory_fit_gui.m` — 主 GUI 入口

这是当前项目主线的唯一入口。功能包括：

- **路径初始化**：将 `ui/`、`rendering/`、`kinematics/`、`trajectory/`、`io/`、`utils/`、`collision/`、`mex/` 等子目录加入 MATLAB 搜索路径
- **状态初始化**：构建 `state` 结构体，包含环境几何参数、轨迹参数、位姿参数、URDF 路径、碰撞配置等
- **回调绑定**：构建 12 个回调函数，包括轨迹/环境窗口打开、轨迹拟合、轨迹播放、YAML 读写、视角切换等
- **场景刷新**：导入 URDF、重建碰撞环境、重新渲染

关键回调：
| 回调函数 | 触发按钮 | 功能 |
|---------|---------|------|
| `openTrajectoryWindow` | "轨迹调整" | 打开轨迹参数调节子窗口 |
| `openEnvironmentWindow` | "底部盒子移动" | 打开环境位姿调节子窗口 |
| `onMoveToTrajectoryStart` | "从当前姿态移动到轨迹起始点" | IK 求解到轨迹第一个点 |
| `onTrackTcpTrajectory` | "开始尖端轨迹拟合" | 核心：整条轨迹拟合 |
| `onPlayTcpTrajectory` | "开始运行" | 播放拟合好的轨迹 |
| `onReloadYaml` | "从 YAML 重载" | 从 YAML 文件恢复配置 |
| `onSaveYaml` | "保存 YAML" | 保存当前配置到 YAML |
| `onSaveYamlAndUrdf` | "保存 YAML + 回写 URDF" | 同时保存 YAML 和更新 URDF |

#### 3.2.2 `ur10_gui_control.m` — 实时关节控制 GUI

独立于主线的实时机械臂控制界面，功能包括：

- **TCP 实时通信**：通过端口 30003 读取 UR10 实时关节数据（1060 Hz 的 RTDE 数据流）
- **URScript 指令发送**：通过端口 30002 发送 `movej` 和 `stopj` 指令
- **UR10 协议解析**：
  - 大端序 4 字节包头长度解析（有效包长：1220/1116/1108 字节）
  - 字节同步恢复算法（搜索合法包头 + 验证关节数据）
  - q_actual 字段偏移量 = 253 字节（1-based）
  - 6 个关节的 double 值（各 8 字节，大端序）
- **3D 实时渲染**：当前姿态（实心）和目标姿态（半透明蓝色叠加）
- **滑条控制**：6 个关节角度滑条，范围 ±360°
- **安全机制**：限位检查、实时流超时检测（1 秒无数据自动锁定执行按钮）

#### 3.2.3 `ur10_realtime_sync.m` — 实时关节同步显示

轻量级实时同步工具，功能类似 `ur10_gui_control.m` 但更精简：

- 33 Hz 渲染循环
- 自动端口回退（30003 → 30002）
- 关节名映射容错
- 调试日志输出（每 2 秒输出关节数据）

#### 3.2.4 `show_assembly_urdf.m` — URDF 模型预览

预处理 xacro 格式的 URDF 文件并显示：
1. 移除 `<xacro:arg>` 声明
2. 替换 `$(arg mesh_root)` 为实际网格目录名
3. 写入临时 URDF 文件
4. 使用 `importrobot` 导入并显示（包含连杆坐标系）

---

## 4. UR10 机械臂运动学建模

### 4.1 UR10 运动学链

UR10 是 Universal Robots 公司生产的六轴协作机械臂。本项目中的运动学链定义如下：

```
base_jizuo (世界基座, fixed)
  └─ base_jizuo_base_ur10_with_dizuo (UR10底座转接, fixed)
       └─ ur10 (UR10基座, fixed)
            └─ ur10_shoulder_pan (关节1: 绕Z轴旋转)
                 └─ ur10_shoulder_lift (关节2: 绕Y轴旋转)
                      └─ ur10_elbow (关节3: 绕Y轴旋转)
                           └─ ur10_wrist_1 (关节4: 绕Y轴旋转)
                                └─ ur10_wrist_2 (关节5: 绕Z轴旋转)
                                     └─ ur10_wrist_3 (关节6: 绕Y轴旋转)
                                          └─ sensor_shovel (铲子, fixed)
                                               └─ sensor_shovel_tcp (TCP, fixed)
```

### 4.2 DH 参数表

UR10 的标准改进 DH 参数：

| 关节 | a (m) | α (rad) | d (m) | θ (rad) |
|------|-------|---------|-------|---------|
| 1 (肩部旋转) | 0 | π/2 | 0.1273 | θ₁ |
| 2 (肩部抬升) | -0.612 | 0 | 0 | θ₂ |
| 3 (肘部) | -0.5723 | 0 | 0 | θ₃ |
| 4 (腕部1) | 0 | π/2 | 0.163941 | θ₄ |
| 5 (腕部2) | 0 | -π/2 | 0.1157 | θ₅ |
| 6 (腕部3) | 0 | 0 | 0.0922 | θ₆ |

### 4.3 关节限位

| 关节 | 下限 (rad) | 上限 (rad) | 最大速度 (rad/s) | 最大力矩 (Nm) |
|------|-----------|-----------|-----------------|-------------|
| ur10_shoulder_pan | -6.28319 | 6.28319 | 2.16 | 330 |
| ur10_shoulder_lift | -6.28319 | 6.28319 | 2.16 | 330 |
| ur10_elbow | -6.28319 | 6.28319 | 3.15 | 150 |
| ur10_wrist_1 | -6.28319 | 6.28319 | 3.20 | 54 |
| ur10_wrist_2 | -6.28319 | 6.28319 | 3.20 | 54 |
| ur10_wrist_3 | -6.28319 | 6.28319 | 3.20 | 54 |

### 4.4 正运动学 (Forward Kinematics)

正运动学计算在 C++ 和 MATLAB 中各有实现，算法一致：

**C++ 实现** (`robot_model.cpp:forwardKinematics`):
```cpp
// 从基座开始，依次累积变换矩阵
Mat4 T = Mat4::Identity();
for (const auto& seg : robot.segments) {
    T = T * seg.origin;          // 关节原点变换
    if (seg.movable) {
        // 绕关节轴旋转
        Eigen::AngleAxisd aa(q(seg.q_index), seg.axis);
        T = T * rtToTform(aa.toRotationMatrix(), Vec3::Zero());
    }
    poses[seg.child_link] = T;
}
```

**MATLAB 实现**: 使用 `getTransform(robot, q, 'sensor_shovel_tcp')` 调用 Robotics System Toolbox 内置函数。

### 4.5 雅可比矩阵计算

采用数值微分法计算 6×6 空间雅可比矩阵：

**C++ 实现** (`robot_model.cpp:numericJacobian`):
```cpp
Mat6 J = Mat6::Zero();
Mat4 T0 = tipTransform(robot, q);
for (int i = 0; i < q.size(); ++i) {
    Eigen::VectorXd qp = q;
    qp(i) += eps;  // ε = 1e-6
    Mat4 Tp = tipTransform(robot, qp);
    Vec6 delta = poseError(T0, Tp);
    J.col(i) = delta / eps;
}
```

雅可比矩阵的每一列表示第 i 个关节的微小变化对 TCP 位姿的影响：
- 前 3 行：位置雅可比（关节速度 → 末端线速度）
- 后 3 行：姿态雅可比（关节速度 → 末端角速度，世界坐标系表示）

### 4.6 位姿误差计算

**C++ 实现** (`utils.cpp:poseError`):
```cpp
Vec6 poseError(const Mat4& current, const Mat4& target) {
    Vec6 err = Vec6::Zero();
    // 位置误差：简单的向量差
    err.head<3>() = target.block<3,1>(0,3) - current.block<3,1>(0,3);
    // 姿态误差：旋转矩阵差 → 旋转向量 → 转换到当前坐标系
    Eigen::Matrix3d Rerr = current.block<3,3>(0,0).transpose()
                           * target.block<3,3>(0,0);
    Vec3 w_local = rotToLogVec(Rerr);
    err.tail<3>() = current.block<3,3>(0,0) * w_local;
    return err;
}
```

姿态误差使用以下公式：
- 旋转误差矩阵：R_err = R_current^T × R_target
- 对数映射：ω = log(R_err)（通过轴角表示）
- 世界坐标系表示：err_rot = R_current × ω

### 4.7 等效构型对齐

由于旋转关节的 2π 周期性，同一 TCP 位姿可能对应多个关节构型。`alignEquivalentConfiguration` 函数将 IK 求解结果对齐到最接近参考构型的等效角度：

```matlab
function qAligned = alignEquivalentConfiguration(robot, qIn, qPrev, dqPrev)
    qAligned = qIn;
    targetRef = qPrev + dqPrev;   % 预测的下一帧关节值
    for each revolute joint:
        candidates = qAligned(j) + 2π × [-2, -1, 0, 1, 2]
        // 过滤：必须在关节限位内
        // 选择：最接近 targetRef 的候选值
    end
end
```

---

## 5. 逆运动学求解算法详解

逆运动学是本项目最核心的算法模块。系统设计了多层次、多策略的求解架构。

### 5.1 求解器架构

```
┌─────────────────────────────────────────────────────────────┐
│                  solveTcpPoseIk (主入口)                      │
├─────────────────────────────────────────────────────────────┤
│  Stage 1: 多级权重策略                                       │
│  ┌──────────────┬──────────────┬──────────────────────────┐  │
│  │ strict       │ relaxed      │ very_relaxed / pos_only │  │
│  │ [1,1,1,.2,   │ [1,1,1,.1,  │ [1,1,1,.03,.03,.03]    │  │
│  │  .2,.2]      │  .1,.1]      │ [1,1,1,0,0,0]          │  │
│  │ θ_limit=30°  │ θ_limit=45°  │ θ_limit=70° / ∞        │  │
│  └──────────────┴──────────────┴──────────────────────────┘  │
│                                                              │
│  Stage 2: 多种子策略                                         │
│  ┌────────────────────────────────────────────────────────┐  │
│  │ 种子1: q_prev (上一帧关节值，连续性最好)              │  │
│  │ 种子2: home_q (URDF home 构型)                        │  │
│  │ 种子3: zeros (全零构型)                               │  │
│  │ 扩展种子: 对以上种子进行关节5/6的 ±2π 包裹           │  │
│  │ 全局种子: 96 个随机采样构型 (C++ MEX 中的 fallback)   │  │
│  └────────────────────────────────────────────────────────┘  │
│                                                              │
│  Stage 3: 最优选择                                           │
│  ┌────────────────────────────────────────────────────────┐  │
│  │ 安全优先: 位置误差<3cm AND 姿态误差<threshold AND      │  │
│  │           clearance>2mm → 选择连续性代价最小的        │  │
│  │ 回退机制: 所有种子都不满足安全条件 → 选位置误差最小的  │  │
│  │ 彻底失败: 无有效解 → 抛出详细错误信息                 │  │
│  └────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 MATLAB 原生 IK 求解器

MATLAB 使用 Robotics System Toolbox 的 `inverseKinematics` 对象：

```matlab
ikSolver = inverseKinematics('RigidBodyTree', robot);
[qTry, infoTry] = ikSolver('sensor_shovel_tcp', targetTform, weights, qSeed);
```

这是 MATLAB 内置的基于 Levenberg-Marquardt 的数值 IK 求解器。

### 5.3 C++ 自定义 IK 求解器

C++ 实现了**双求解器架构**：KDL LMA 求解器（主）+ 自定义阻尼最小二乘法 (DLS)（备用）。

#### 5.3.1 阻尼最小二乘法 (DLS)

**算法原理**：

标准雅可比伪逆法：Δq = J⁺ × e（其中 J⁺ = J^T(J·J^T)^(-1)）

阻尼最小二乘法：Δq = (J^T·J + λ²·I)^(-1) × J^T × e

加入阻尼项 λ²·I 可以：
- 避免 J 病态（接近奇异点）时步长过大
- 提供数值稳定性
- 在高阻尼时退化为梯度下降，低阻尼时退化为高斯-牛顿法

**自适应阻尼策略** (`ik_solver.cpp`):
```cpp
double lambda = cfg.lambda;  // 默认 5e-3
if (pos_err > 0.1 || rot_err > 0.1) {
    // 远离目标：增大阻尼，保证稳定性
    lambda = max(1e-3, 5e-3 * (pos_err / 0.05));
    lambda = min(lambda, 0.1);
} else {
    // 接近目标：减小阻尼，加快收敛
    lambda = 5e-4 + 5e-3 * (pos_err / 0.05);
}
```

**步长限制**：单步最大关节变化 0.45 rad（约 26°），防止过大跳跃。

**收敛检测**：
- 最大迭代次数：60
- 停滞检测：连续 8 次迭代误差不下降（<1% 改善）则退出
- 位置容差：3 cm
- 姿态容差：取决于权重级（30°/45°/70°/∞）

#### 5.3.2 KDL LMA 求解器

使用 Orocos KDL 库的 Levenberg-Marquardt 求解器作为**更快的替代方案**：

```cpp
// 将目标 TCP 位姿转换为目标 wrist_3 位姿
Mat4 target_wrist3 = target * robot.T_wrist3_to_tcp.inverse();

// 构建 KDL 输入
KDL::Frame target_frame;  // 从 Eigen::Matrix4d 转换
KDL::JntArray q_init(6);  // 从种子构型转换

// 构建任务空间权重 L 向量
Eigen::Matrix<double,6,1> L;
for (int k = 0; k < 6; ++k)
    L(k) = sqrt(max(weights[k], 1e-6));

// KDL LMA 求解
KDL::ChainIkSolverPos_LMA solver(*robot.kdl_chain, L, 1e-5, 100, 1e-6);
int rc = solver.CartToJnt(q_init, target_frame, q_out);
```

KDL LMA 的优势：
- 基于**解析雅可比**（KDL Chain 提供），比数值雅可比更精确
- C++ 原生实现，无 MATLAB 函数调用开销
- 内置阻尼自适应调节

当前状态：在 trajectory_solver.cpp 中，KDL 求解器未被实际调用（注释说明"成功率和速度均不如 custom DLS"），当前全部使用自定义 DLS 求解器。

### 5.4 多级权重策略

IK 求解中的权重决定了位置和姿态之间的权衡：

| 级别 | 位置权重 [x,y,z] | 姿态权重 [rx,ry,rz] | 姿态容差 | 说明 |
|------|-----------------|---------------------|---------|------|
| strict | [1, 1, 1] | [0.20, 0.20, 0.20] | 30° | 优先精确姿态 |
| relaxed | [1, 1, 1] | [0.10, 0.10, 0.10] | 45° | 放松姿态要求 |
| very_relaxed | [1, 1, 1] | [0.03, 0.03, 0.03] | 70° | 进一步放松姿态 |
| position_only | [1, 1, 1] | [0, 0, 0] | ∞ | 只求解位置 |

权重值越大，对应维度在优化目标中越重要。位置权重始终为 1（最高优先级），姿态权重逐步降低以扩大有效解空间。

### 5.5 连续 IK 选择策略

在多帧轨迹中，需要保持关节构型的连续性。`solveTcpPoseIk` 使用连续性代价函数：

```matlab
function cost = continuityCost(qCandidate, qPrev, dqPrev)
    dqPos = wrapJointDelta(qCandidate - qPrev);   % 位置变化
    dqVel = dqPos - dqPrev;                        % 加速度（速度变化）
    cost = norm(dqPos) + 0.65 * norm(dqVel);      % 综合考虑
end
```

这个代价函数平衡了两个目标：
1. **关节位移最小**（`norm(dqPos)`）：优先选择与上一帧最接近的解
2. **关节速度连续**（`0.65 * norm(dqVel)`）：避免速度突变，保证平滑运动

---

## 6. 轨迹规划算法详解

### 6.1 铲泥任务轨迹设计

铲泥任务需要末端执行器（铲子）按照特定路径完成"切入泥土 → 铲起泥土 → 提升离开"的动作。本项目将轨迹分为**三个阶段**：

```
┌──────────────────────────────────────────────────────────────┐
│                    铲泥轨迹三阶段                            │
│                                                              │
│  Z ↑                                                        │
│    │        阶段3: 垂直提升                                  │
│    │          ↑↑↑                                           │
│    │         /                                               │
│    │        / 阶段2: 圆弧装载                                │
│    │       /                                                 │
│    │      /◜──────────────● 泥面 (Z=0)                      │
│    │     /                                                  │
│    │    / 阶段1: 斜线切入                                    │
│    │   /                                                    │
│    │  ● 起始点                                              │
│    └──────────────────────────→ Y                           │
│                                                              │
│  阶段1 (seg0): 从起始点沿 θ 角度斜线向下切入                │
│  阶段2 (seg1): 底部圆弧旋转装载                             │
│  阶段3 (seg2): 垂直向上提升离开                             │
└──────────────────────────────────────────────────────────────┘
```

### 6.2 轨迹生成算法

#### 6.2.1 `generate_trajectory` — 二维轨迹生成器

位于 `test/tarjectory_plan/generate_trajectory.m`，用于生成二维（XOZ 平面）铲泥轨迹：

**输入参数**：
- `x0`: 起始 X 坐标
- `z0`: 起始 Z 高度
- `thetaDeg`: 入泥角度（负值 = 向下）
- `d`: 入泥深度
- `phiDeg`: 底部旋转角度

**算法步骤**：

1. **直线切入段**：
```matlab
theta = deg2rad(thetaDeg);
dir1 = [cos(theta), sin(theta)];     % 切入方向单位向量
downComp = -dir1(2);                 % 方向向下的分量
lineLen = (z0 + d) / downComp;       % 切入线长度 = (当前高度+深度)/向下分量
seg1 = p0 + linspace(0, lineLen, 80)' * dir1;
```

2. **圆弧装载段**：
```matlab
p1 = seg1(end, :);                   % 切入终点 = 弧段起点
R = max(0.20, 0.9 * d);              % 圆弧半径（最小 20cm，与深度成正比）
leftNormal = [-sin(theta), cos(theta)]; % 左侧法向量
center = p1 + R * leftNormal;        % 圆心位置
a1 = atan2(p1(2)-center(2), p1(1)-center(1));  % 起始角
a2 = a1 + phi;                       % 终止角 = 起始角 + 旋转角
seg2 = center + R * [cos(a), sin(a)]; % 圆弧采样 (140 点)
```

3. **垂直提升段**：
```matlab
p2 = seg2(end, :);                   % 弧段终点 = 提升段起点
liftLen = max(0.25, 1.15 * d);       % 提升高度
seg3 = p2 + linspace(0, liftLen, 90)' * [0, 1];  % 竖直向上
```

#### 6.2.2 `generate_trajectory_3d` — 三维轨迹扩展

主 GUI 调用的三维版本（通过 `trajectory_plan_3d_gui` 路径引入），将二维（YOZ 平面）轨迹扩展为三维：

- 引入 `xPlane` 参数：轨迹所在的 X 截面位置
- 引入 `leftWallOffset`：入泥点到左侧壁的水平距离
- 引入 `mudHeight`：泥面高度

### 6.3 姿态规划

轨迹不仅包含位置点，还需要在每个点指定 TCP 的姿态（旋转矩阵）。姿态规则：

#### 6.3.1 切入段与圆弧段姿态

核心规则：**TCP 的局部 -X 方向沿轨迹切线方向**。

```matlab
function [R, zAxis] = buildArcPhaseRotation(tangent, worldUp)
    negXPath = normalize(tangent);    % 轨迹切线 → 局部 -X
    xAxis = -negXPath;
    zProj = worldUp - dot(worldUp, xAxis) * xAxis;
    zAxis = normalize(zProj);         % Z 轴尽量朝上
    if zAxis(3) < 0, zAxis = -zAxis; end
    yAxis = cross(zAxis, xAxis);      % 右手定则确定 Y 轴
    R = [xAxis(:), yAxis(:), zAxis(:)];
end
```

#### 6.3.2 提升段姿态

提升段使用固定的垂直方向姿态：

```matlab
function [R, zAxis] = buildLiftPhaseRotation(horizontalHeading, worldUp)
    zAxis = normalize(worldUp);         % Z 轴固定向上
    negXPath = normalize(horizontalHeading);
    xAxis = -negXPath;                   % -X 沿之前的方向
    yAxis = cross(zAxis, xAxis);
    R = [xAxis(:), yAxis(:), zAxis(:)];
end
```

#### 6.3.3 姿态过渡

在切入段/圆弧段与提升段之间，使用**球面线性插值 (SLERP)** 平滑过渡：

```matlab
function R = slerpRotation(R0, R1, alpha)
    q0 = rotmToQuatWxyz(R0);
    q1 = rotmToQuatWxyz(R1);
    if dot(q0, q1) < 0, q1 = -q1; end   % 选短路径
    cosTheta = max(-1, min(1, dot(q0, q1)));
    if cosTheta > 1 - 1e-10
        q = (1-alpha)*q0 + alpha*q1;    % 接近时线性插值
    else
        theta = acos(cosTheta);
        q = sin((1-alpha)*theta)/sin(theta) * q0
          + sin(alpha*theta)/sin(theta) * q1;
    end
    R = quatWxyzToRotm(q);
end
```

过渡窗口通过五次多项式混合函数控制：
```matlab
alpha = 10*t^3 - 15*t^4 + 6*t^5;  % t ∈ [0,1]
```

### 6.4 自适应锚点采样

为减少 IK 求解次数，系统对目标位姿序列进行自适应下采样，保留关键锚点：

```matlab
function anchorPlan = buildAdaptiveAnchorPlan(targetPlan)
    mustKeep = [1; segmentTransitions; nPts];  % 必须保留的点
    idx = [mustKeep; linspace(1, nPts, min(nPts, 72))];  % 均匀采样

    % 自适应细分：如果相邻锚点之间的复杂度超过阈值，则插入中点
    while 锚点数 < maxAnchors
        for 每对相邻锚点 (i0, i1):
            if shouldSplit(pathLength > 12mm OR rotDelta > 0.55°
                            OR tangentDelta > 0.50°)
                插入中点 (i0+i1)/2
        if 没有新的插入点: break
    end

    % 如果锚点仍然过多，按复杂度排序删减
    while 锚点数 > maxAnchors:
        移除合并代价最小的可选锚点
end
```

复杂度度量函数：
```matlab
cost = pathLength + 0.5 * rotationDelta + 0.25 * tangentDelta;
```

### 6.5 连续回放轨迹生成

锚点轨迹是稀疏的（通常 60-120 个点），需要在相邻锚点之间进行密集插值以生成平滑的回放轨迹：

**启发式分段数估算**：
```matlab
nSeg = 1 + max([
    2,
    ceil(jointStep / 0.70°),     % 关节步长约束
    ceil(posStep / 3.0mm),       % 位置步长约束
    ceil(rotStep / 0.30°)        % 姿态步长约束
]);
nSeg = clamp(nSeg, 4, 64);      % 每段 4-64 个子步
```

**五次多项式混合**（在关节空间进行平滑插值）：
```matlab
function qSeries = rtfg_quintic_joint_series(qStart, qEnd, nPts)
    t = linspace(0, 1, nPts);
    s = 10*t.^3 - 15*t.^4 + 6*t.^5;  % 五次多项式：零速度/零加速度边界
    dq = atan2(sin(qEnd-qStart), cos(qEnd-qStart));  % 角度包裹
    qSeries = qStart + dq .* s;
end
```

五次多项式 `s(t) = 10t³ - 15t⁴ + 6t⁵` 的性质：
- s(0) = 0, s(1) = 1
- s'(0) = s'(1) = 0（零速度边界）
- s''(0) = s''(1) = 0（零加速度边界）
- 保证关节位置、速度、加速度的连续性

### 6.6 目标位姿序列构建

`buildTrajectoryTargetPlan` 函数将显示轨迹转换为 IK 目标序列：

```
显示轨迹 (displayTraj)
    │
    ├─ seg0 (斜线): 使用 arcPhase 姿态 (切线方向)
    ├─ seg1 (圆弧): 使用 arcPhase 姿态
    ├─ 过渡段: SLERP 从 arcPhase 到 liftPhase 姿态 (8-18 步)
    └─ seg2 (提升): 使用固定 liftPhase 姿态
    │
    ▼
targetPlan = {
    points:    [N×3]  目标位置
    tforms:    {N×1}  目标位姿 (4×4 齐次变换矩阵)
    segmentNames: {N×1} 段标签
    zAxisPreview: [N×6] 姿态 Z 轴预览
}
```

---

## 7. 碰撞检测系统

### 7.1 检测架构

碰撞检测系统覆盖三个维度的间隙计算：

```
┌─────────────────────────────────────────────────────────────┐
│                    碰撞检测分类                              │
├───────────────┬─────────────────┬───────────────────────────┤
│ 自碰撞        │ 工具-本体碰撞    │ 工具-盆体碰撞            │
│ (Self)        │ (Tool-Body)     │ (Tool-Basin)             │
├───────────────┼─────────────────┼───────────────────────────┤
│ 机械臂连杆之  │ 铲子/铲尖 与    │ 铲子/铲尖 与             │
│ 间的碰撞      │ 机械臂连杆碰撞   │ 盆体壁板碰撞             │
├───────────────┼─────────────────┼───────────────────────────┤
│ 排除相邻连杆  │ 仅检查工具 vs    │ 使用解析盒体近似         │
│ (i±1 跳过)   │ 非工具连杆       │ 5 个盒体壁板             │
└───────────────┴─────────────────┴───────────────────────────┘
```

### 7.2 MATLAB 碰撞检测

MATLAB 使用 Robotics System Toolbox 的 `checkCollision` 函数：

```matlab
% 机器人内部碰撞 (含分离距离)
[~, separationDist] = checkCollision(robot, q, ...
    'Exhaustive', 'on', ...
    'IgnoreSelfCollision', 'off', ...
    'SkippedSelfCollisions', 'adjacent');  % 跳过相邻连杆

% 机器人与环境物体碰撞
[~, separationDist] = checkCollision(robot, q, worldObjects, ...
    'Exhaustive', 'on', ...
    'IgnoreSelfCollision', 'on');
```

碰撞间隙阈值：**2 mm**（`clearanceThreshold = 2e-3`）

### 7.3 C++ FCL 碰撞检测

C++ 使用 Flexible Collision Library (FCL) 进行高性能碰撞检测：

**自碰撞和工具-本体碰撞** (`collision_checker.cpp:evaluateConfiguration`):
```cpp
for (size_t i = 0; i < robot.collisions.size(); ++i) {
    for (size_t j = i + 1; j < robot.collisions.size(); ++j) {
        // 跳过相邻连杆 (chain_index 差 ≤ 1)
        if (abs(a.chain_index - b.chain_index) <= 1) continue;

        // FCL 距离查询
        fcl::DistanceRequestd request(true);
        fcl::DistanceResultd result;
        double dist = fcl::distance(obj_i, obj_j, request, result);

        // 分类：tool_body 或 self
        if (a_tool ^ b_tool) → tool_body
        else if (!a_tool && !b_tool) → self
    }
}
```

**工具-盆体碰撞**：
```cpp
for (int tool_idx : tool_indices) {
    for (const auto& box : basin_boxes) {
        // 动态创建 FCL Box 对象
        auto geom = make_shared<fcl::Boxd>(box.size);
        CollisionObjectd basin_obj(geom, box.pose);
        double dist = fcl::distance(robot_obj, &basin_obj, request, result);
    }
}
```

**性能优化**：
- 碰撞几何体使用**共享指针**缓存，每次查询仅更新位姿变换（不重建 BVH）
- 使用 `thread_local` 缓存机器人碰撞对象，避免重复分配
- AABB 快速剔除（`computeAABB`）

### 7.4 盆体碰撞模型

盆体（basin）使用 5 个解析盒体近似：

| 盒体名称 | 尺寸 (m) | 相对位置 (m) | 说明 |
|---------|----------|------------|------|
| block_basin_bottom | [0.37, 0.50, 0.003] | [-0.135, 0, 0.25] | 盆底 |
| block_basin_front_wall | [0.37, 0.003, 0.18] | [-0.135, 0.2485, 0.34] | 前壁 |
| block_basin_back_wall | [0.37, 0.003, 0.18] | [-0.135, -0.2485, 0.34] | 后壁 |
| block_basin_left_wall | [0.003, 0.494, 0.18] | [-0.3185, 0, 0.34] | 左壁 |
| block_basin_right_wall | [0.003, 0.494, 0.18] | [0.0485, 0, 0.34] | 右壁 |

不直接使用显示网格做精确三角面碰撞（性能优化考量）。

### 7.5 碰撞优先级

当存在多种碰撞时，按以下优先级报告（通过 `collision_checker.cpp:choose_violation`）：

```
tool_basin (3) > tool_body (2) > self (1)
```

### 7.6 碰撞结果可视化

碰撞点在 GUI 中以红色标记显示：
- 普通碰撞点：红色圆圈（MarkerSize=5）
- 首次碰撞点：红色外圈 + 黄色填充（MarkerSize=9）
- 碰撞盆壁：红色线框高亮

---

## 8. MATLAB MEX C++ 加速实现

### 8.1 MEX 架构总览

MATLAB MEX (MATLAB Executable) 允许将 C/C++ 代码编译为 MATLAB 可直接调用的共享库。本项目的 MEX 模块 `rtfg_solver_mex` 将三个重量级计算环节从 MATLAB 迁移到 C++：

```
MATLAB (GUI / IO / 渲染)
    │
    │  inputStruct (URDF路径, 目标位姿序列, 盆体盒体, 当前关节值, 配置参数)
    ▼
┌──────────────────────────────────────────────────────────┐
│              rtfg_solver_mex (C++ MEX)                   │
│                                                          │
│  ① 解析 MATLAB 输入 → C++ 数据结构                       │
│  ② 加载 URDF 机器人模型 (urdfdom + KDL)                  │
│  ③ 构建 FCL 碰撞对象 (FCL + STL 解析)                    │
│  ④ 主求解循环:                                           │
│     - 锚点 IK (KDL LMA + 自定义 DLS, 自适应阻尼)         │
│     - 连续回放生成 (五次多项式混合, 关节空间插值)         │
│     - 稀疏碰撞复检 (每10帧检测一次)                       │
│  ⑤ 统计信息收集 (求解时间, 成功/失败计数)                 │
│  ⑥ 构建 MATLAB 输出结构体 (mxArray)                       │
│                                                          │
└──────────────────────────────────────────────────────────┘
    │
    │  outputStruct (anchorQSeries, playbackQSeries,
    │               tcpPath, collisionResults, previewMetrics)
    ▼
MATLAB (渲染 / 状态更新)
```

### 8.2 依赖库详解

| 库名 | 版本要求 | 用途 |
|------|---------|------|
| **Eigen 3** | ≥3.3 | 线性代数：矩阵/向量运算、SVD、LDLT 分解 |
| **FCL** | ≥0.7 | 碰撞检测：Box/Cylinder/Mesh 几何体、距离查询、AABB |
| **orocos-kdl** | ≥1.5 | 运动学：KDL Chain 构建、解析雅可比、LMA IK 求解器 |
| **kdl_parser** | ROS 2 Humble | URDF → KDL Tree 解析 |
| **urdfdom** | ≥3.0 | URDF 文件解析：模型、关节、碰撞几何体 |
| **console_bridge** | - | ROS 日志桥接（urdfdom 依赖） |
| **ccd** | - | FCL 依赖：连续碰撞检测 |
| **octomap / octomath** | - | FCL 依赖 |

### 8.3 C++ 源文件详解

#### 8.3.1 `types.h` — 数据类型定义

定义整个 C++ 模块使用的核心数据结构：

```cpp
namespace rtfg {

// 基础类型别名
using Mat4 = Eigen::Matrix4d;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

// 盆体盒体规格
struct BasinBox {
    std::string name;      // 盒体名称
    Vec3 size;             // 尺寸 [sx, sy, sz]
    Mat4 pose;             // 机器人基座坐标系下的位姿
};

// 运动学段定义
struct SegmentSpec {
    std::string joint_name, parent_link, child_link;
    Mat4 origin;           // 关节原点变换
    Vec3 axis;             // 旋转轴
    bool movable;          // 是否为可动关节
    int q_index;           // 在关节向量中的索引
    double lower, upper;   // 关节限位
};

// 碰撞几何体
struct LinkCollision {
    std::string link_name;
    int chain_index;       // 运动链中的位置
    bool is_tool;          // 是否为工具连杆
    Mat4 origin;           // 碰撞体原点
    GeometryType type;     // Box/Cylinder/Mesh
    shared_ptr<CollisionGeometryd> geometry;  // FCL 几何体
};

// 机器人模型
struct RobotModel {
    vector<SegmentSpec> segments;
    vector<LinkCollision> collisions;
    vector<string> link_order;
    unordered_map<string, int> link_index;
    shared_ptr<KDL::Chain> kdl_chain;  // 6-DOF KDL 链
    Mat4 T_wrist3_to_tcp;              // wrist3→TCP 固定偏移
    string base_link, tip_link, mesh_root;
};

// IK 求解候选解
struct CandidateInfo {
    Eigen::VectorXd q;
    double pos_err, rot_err, clearance, cost;
    std::string failure_reason;
    bool valid;
};

// 碰撞检测摘要
struct CollisionSummary {
    double min_self, min_tool_body, min_tool_basin;
    string violation_type, violation_object;
};

// 求解器配置
struct SolverConfig {
    double clearance_threshold = 2e-3;   // 2mm 间隙阈值
    double ik_position_tolerance = 3e-2; // 3cm 位置容差
    int max_iterations = 60;             // 最大 IK 迭代次数
    double lambda = 5e-3;                // DLS 阻尼系数
};

// 求解结果
struct TrajectoryResult {
    Eigen::MatrixXd anchor_q;        // 锚点关节序列
    Eigen::MatrixXd playback_q;      // 回放关节序列
    Eigen::MatrixXd tcp_path;        // TCP 路径
    CollisionSummary global_minimums;
    vector<int> collision_indices;
    bool has_collision;
    SolverTiming timing;             // 性能统计
    // ... 更多字段
};
}
```

#### 8.3.2 `robot_model.cpp` — 机器人模型加载

**核心函数**：

- `loadRobotModel(urdf_path, base_link, tip_link)` — 从 URDF 文件构建完整的机器人模型
  - 使用 `urdf::parseURDFFile` 解析 URDF
  - 从 tip_link 反向追踪到 base_link，构建运动链
  - 提取关节类型（revolute/continuous/fixed）、限位、轴线
  - 加载碰撞几何体：Box（解析盒体）→ Cylinder（解析圆柱）→ Mesh（STL 解析 → FCL BVH 模型）
  - 构建 KDL Chain（base_link → ur10_wrist_3，6-DOF）
  - 计算 wrist_3 → TCP 的固定偏移量

- `forwardKinematics(robot, q)` — 正运动学：关节值 → 所有连杆位姿

- `tipTransform(robot, q)` — TCP 位姿（封装 forwardKinematics 并取 tip_link）

- `numericJacobian(robot, q)` — 6×6 数值雅可比（中央差分，ε=1e-6）

- `clampToLimits(robot, q)` — 关节限位裁剪

- `alignToReference(robot, q, q_ref)` — 等效构型对齐（±2π 包裹处理）

**STL 文件解析**（`loadStlAsFclModel`）：
- 支持**二进制**和**ASCII**两种 STL 格式
- 二进制：读取 80 字节头部 + 4 字节三角面数量 + 50 字节/面
- ASCII：逐行解析 `vertex` 关键字
- 构建 `fcl::BVHModel<fcl::OBBRSSd>` 包围体层次结构

#### 8.3.3 `ik_solver.cpp` — 逆运动学求解器

**核心函数**：

- `solveSinglePose(robot, basin_boxes, cfg, target, seed, q_prev, dq_prev, weights, orient_limit)` — 主要 IK 求解器
  - 自适应阻尼 DLS 迭代
  - 步长限制（max 0.45 rad/步）
  - 收敛/停滞检测

- `solveSinglePoseKdl(robot, basin_boxes, cfg, target, seed, q_prev, dq_prev, weights, orient_limit)` — KDL LMA 求解器
  - TCP → wrist_3 位姿转换
  - KDL Frame / JntArray 类型转换
  - L 权重向量构建（sqrt 映射）

- `buildSeedList(q_prev, home_q, robot)` — 局部种子：q_prev、home_q、零向量 + 关节5/6 的 ±2π/±4π 包裹

- `buildGlobalSeedList(robot)` — 全局种子：中点构型 + 96 个随机采样构型

#### 8.3.4 `collision_checker.cpp` — 碰撞检测

**核心函数**：

- `evaluateConfiguration(robot, basin_boxes, cfg, q)` — 单构型碰撞评估
  - `thread_local` 缓存碰撞对象（避免重复分配）
  - 自碰撞和工具-本体碰撞（二重循环，跳过相邻连杆）
  - 工具-盆体碰撞（工具碰撞体 vs 动态创建的 FCL Box）
  - 碰撞优先级判定

#### 8.3.5 `trajectory_solver.cpp` — 轨迹求解器

**核心函数**：

- `solveTrajectory(robot, basin_boxes, cfg, target_tforms, segment_names, current_q, home_q)` — 完整轨迹求解管线
  - **阶段1：锚点求解**（占 50% 进度）
    - 对每个目标位姿，依次尝试 4 种种子来源（q_prev → home_q/零 → 包裹种子 → 全局随机）+ 4 级权重
    - **延迟碰撞检测**：仅在 IK 解满足位置/姿态容差后才执行碰撞检查（避免浪费）
    - 每个候选解通过后计算连续性代价，选择最优安全解
  - **阶段2：回放轨迹生成**（占 20% 进度）
    - 锚点间五次多项式混合插值
    - 每段 4-32 个子步
  - **阶段3：碰撞复检**（占 20% 进度）
    - 稀疏采样检测（每 10 帧检测一次 + 首尾帧）
    - 更新全局最小间隙
    - 记录所有碰撞事件详情
  - **阶段4：结果回传**（占 10% 进度）

**性能优化策略**：

1. **延迟碰撞检测**：只在 IK 通过后才运行昂贵的碰撞检查
2. **稀疏碰撞复检**：回放轨迹每 10 帧检测一次（基于"关节步长 < 0.7° 时碰撞状态不会突变的假设）
3. **碰撞对象缓存**：`thread_local` 存储持久化 FCL 对象，每次仅更新变换
4. **渐进式种子策略**：优先使用最可能成功的种子（q_prev），避免不必要的随机搜索

#### 8.3.6 `utils.cpp` — 工具函数

- `poseError(current, target)` — 6 维位姿误差
- `rotToLogVec(R)` — 旋转矩阵 → 对数映射（旋转向量）
- `rotationDistance(R0, R1)` — 旋转距离
- `quinticBlend(t)` — 五次多项式混合函数
- `slerpRotation(R0, R1, a)` — 球面线性插值
- `wrapJointDelta(dq)` — 关节角度包裹（atan2）
- `continuityCost(q_candidate, q_prev, dq_prev)` — 连续性代价
- `readTformStack(arr)` — mxArray → `vector<Mat4>`（4×4×N 数组）
- `eigenMatrixToMx(M)` — Eigen::MatrixXd → mxArray
- `readBasinBoxes(arr)` — mxArray 结构体数组 → `vector<BasinBox>`

#### 8.3.7 `mex_entry.cpp` — MEX 入口

```cpp
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 1. 解析输入结构体
    string scene_urdf = mxArrayToStdString(getField("sceneUrdf"));
    string base_link = mxArrayToStdString(getField("baseLink"));
    string tip_link  = mxArrayToStdString(getField("tipLink"));
    Eigen::VectorXd current_q = readRowVector(getField("currentQ"));
    Eigen::VectorXd initial_q = readRowVector(getField("initialQ"));
    vector<Mat4> target_tforms = readTformStack(getField("targetTforms"));
    vector<string> segment_names = readCellstrVector(getField("segmentNames"));
    vector<BasinBox> basin_boxes = readBasinBoxes(getField("basinBoxes"));
    double clearance_threshold = mxGetScalar(getField("clearanceThreshold"));

    // 2. 加载模型并求解
    RobotModel robot = loadRobotModel(scene_urdf, base_link, tip_link);
    TrajectoryResult result = solveTrajectory(robot, basin_boxes, cfg,
        target_tforms, segment_names, current_q, initial_q);

    // 3. 构建输出结构体
    plhs[0] = mxCreateStructMatrix(1, 1, 6, fields);
    mxSetField(plhs[0], 0, "anchorQSeries", eigenMatrixToMx(result.anchor_q));
    mxSetField(plhs[0], 0, "playbackQSeries", eigenMatrixToMx(result.playback_q));
    mxSetField(plhs[0], 0, "tcpPath", eigenMatrixToMx(result.tcp_path));
    mxSetField(plhs[0], 0, "collisionResults", buildCollisionResultsStruct(result));
    mxSetField(plhs[0], 0, "previewMetrics", buildMetricsStruct(result));
    mxSetField(plhs[0], 0, "progressEvents", buildProgressStruct(result.progress_events));
}
```

### 8.4 MEX 编译系统

`build_mex.m` 编译脚本：

```matlab
function build_mex()
    srcFiles = {'utils.cpp', 'robot_model.cpp', 'ik_solver.cpp',
                'collision_checker.cpp', 'trajectory_solver.cpp', 'mex_entry.cpp'};

    includeDirs = {'/usr/include/eigen3', '/usr/include',
                   '/opt/ros/humble/include', '/opt/ros/humble/include/kdl_parser', srcDir};

    libDirs = {'/usr/lib/x86_64-linux-gnu',
               '/opt/ros/humble/lib', '/opt/ros/humble/lib/x86_64-linux-gnu'};

    libs = {'-lorocos-kdl', '-lkdl_parser', '-lurdf', '-lurdf_xml_parser',
            '-lurdfdom_model', '-lurdfdom_model_state', '-lurdfdom_sensor',
            '-lurdfdom_world', '-lconsole_bridge', '-lfcl', '-lccd',
            '-loctomap', '-loctomath'};

    mex('-v', '-largeArrayDims', ...
        'CXXFLAGS=$CXXFLAGS -std=c++17 -O3', ...
        'LDFLAGS=$LDFLAGS -Wl,--disable-new-dtags -Wl,-rpath,...', ...
        includeDirs..., libDirs..., libs..., srcFiles...,
        '-output', 'rtfg_solver_mex', '-outdir', binDir);
end
```

编译参数说明：
- `-std=c++17`：C++17 标准
- `-O3`：最高优化级别
- `-largeArrayDims`：支持大数组（64 位索引）
- `-rpath`：运行时共享库搜索路径（确保 MEX 文件找到 ROS 2 库）
- `--disable-new-dtags`：使用 RPATH 而非 RUNPATH

### 8.5 MEX 与 MATLAB 的接口

**输入结构体** (`inputStruct`):
| 字段 | 类型 | 说明 |
|------|------|------|
| sceneUrdf | char array | 场景 URDF 文件路径 |
| collisionUrdf | char array | 碰撞检测专用 URDF 路径 |
| baseLink | char array | 基座连杆名称 ("ur10") |
| tipLink | char array | 末端连杆名称 ("sensor_shovel_tcp") |
| currentQ | double (1×6) | 当前关节值 |
| initialQ | double (1×6) | 初始关节值 (home config) |
| targetTforms | double (4×4×N) | 目标位姿序列 |
| segmentNames | cell array | 段名称标签 |
| basinBoxes | struct array | 盆体盒体定义 |
| clearanceThreshold | double | 间隙阈值 (2e-3 m) |

**输出结构体** (`outputStruct`):
| 字段 | 类型 | 说明 |
|------|------|------|
| anchorQSeries | double (M×6) | 锚点关节序列 |
| playbackQSeries | double (P×6) | 回放关节序列 |
| tcpPath | double (P×3) | TCP 路径点 |
| collisionResults | struct | 碰撞检测结果 |
| previewMetrics | struct | 轨迹质量指标 |
| progressEvents | struct array | 求解进度事件 |

### 8.6 MEX 回退机制

当 MEX 求解失败时，系统自动回退到 MATLAB 主链（`rtfg_kinematics.m:trackTrajectoryLegacy`）：

```matlab
function [state, statusText] = trackTrajectory(state)
    try
        solverResult = runMexTrajectorySolver(state, targetPlan);
        % MEX 成功，使用其结果
    catch ME
        % MEX 失败，回退到 MATLAB
        [state, statusText] = trackTrajectoryLegacy(state, targetPlan, ME);
    end
end
```

回退触发条件：
1. MEX 文件不存在（未编译）
2. MEX 运行时异常（IK 失败、碰撞违规）
3. MEX 返回结构不完整
4. 碰撞环境未初始化

---

## 9. GUI 系统设计

### 9.1 主界面布局

```
┌──────────────────────────────────────────────────────────────┐
│  实时轨迹操纵拟合系统                      [−] [□] [×]       │
├──────────────────────────────────────────────────────────────┤
│[轨迹调整] [底部盒子移动] [移动到起始点] [开始拟合] [开始运行] │
│[重载YAML] [保存YAML] [保存YAML+URDF] [视角▼] [☑世界] [☑箭头]│
├──────────────────────────────────────────────────────────────┤
│                                                              │
│                   3D 场景预览区域                             │
│                   (1500×880 窗口)                             │
│                                                              │
│                                                              │
├──────────────────────────────────────────────────────────────┤
│ 状态: 已加载。                                              │
│ 配置文件: .../environment_runtime_config.yaml                │
│ 轨迹参数: hMud=108mm L=0.235m θ=-30.0° d=0.052m x=-0.005m  │
│ 环境位姿: x=-1.830 y=0.000 z=0.000 roll=0.0° pitch=0.0° ... │
│ 拟合进度: [████████░░░░░░░░░░░░] 45%                         │
└──────────────────────────────────────────────────────────────┘
```

### 9.2 子窗口设计

**轨迹调整窗口** — 6 个滑条：
- 距左侧壁距离 (m)
- 泥面高度 hMud (mm)
- 起始运动斜线长度 L (m)
- 入泥角度 theta (deg)
- 入泥深度 d (m)
- 轨迹平面 X 位置 xPlane (m)

**底部盒子移动窗口** — 6 个滑条：
- X/Y/Z 平移 (m)
- Roll/Pitch/Yaw 旋转 (deg)

### 9.3 控件树

```
uifigure (mainFig) '实时轨迹操纵拟合系统'
├── uigridlayout (mainGrid) [3×1]
│   ├── uigridlayout (toolbar) [1×14] — 按钮/复选框行
│   │   ├── uibutton '轨迹调整' → openTrajectoryWindow
│   │   ├── uibutton '底部盒子移动' → openEnvironmentWindow
│   │   ├── uibutton '从当前姿态移动到轨迹起始点' → moveToTrajectoryStart
│   │   ├── uibutton '开始尖端轨迹拟合' → trackTcpTrajectory
│   │   ├── uibutton '开始运行' → playTcpTrajectory (初始禁用)
│   │   ├── uibutton '从 YAML 重载' → reloadYaml
│   │   ├── uibutton '保存 YAML' → saveYaml
│   │   ├── uibutton '保存 YAML + 回写 URDF' → saveYamlAndUrdf
│   │   ├── uidropdown 视角选择 → presetViewChanged
│   │   ├── uicheckbox '世界坐标系'
│   │   ├── uicheckbox '轨迹箭头'
│   │   ├── uicheckbox '连杆坐标系'
│   │   └── uicheckbox '显示碰撞标记'
│   ├── uipanel (previewPanel) — 3D 渲染区域
│   │   └── axes (mainAx) — MATLAB 3D 坐标轴
│   └── uigridlayout (summaryGrid) [5×2] — 状态栏
│       ├── uilabel '状态' + uilabel 状态文本
│       ├── uilabel '配置文件' + uilabel 配置路径
│       ├── uilabel '轨迹参数' + uilabel 参数摘要
│       ├── uilabel '环境位姿' + uilabel 位姿摘要
│       └── uigauge 进度条 + uilabel 百分比
```

---

## 10. 实时机械臂通信与控制

### 10.1 UR10 通信协议

UR10 控制器通过 TCP/IP 提供多个通信端口：

| 端口 | 协议 | 频率 | 用途 |
|------|------|------|------|
| **30003** | RTDE (Real-Time Data Exchange) | 125 Hz (8ms) | 实时状态流（关节位置、速度、力矩等） |
| **30002** | URScript / Secondary Client | 按需 | 发送 URScript 指令、辅助数据流 |
| **30001** | Primary Client | 按需 | 主控制接口 |
| **29999** | Dashboard Server | 按需 | 仪表板命令（加载/播放/停止程序） |

本项目使用：
- **端口 30003**：读取实时关节数据（首选）
- **端口 30002**：发送 `movej`/`stopj` 指令 + 回退数据流

### 10.2 实时数据包解析

UR10 实时数据包结构（1060 字节数据体 + 4 字节包头 = 1064 字节总长，但有效包长可能为 1220/1116/1108 取决于固件版本）：

```
┌──────────┬──────────┬──────────────┬──────────────┬─────────┐
│ 4 bytes  │ 8 bytes  │ 5 groups ×   │ ...          │ ...     │
│ 包长度   │ 时间戳   │ 48 bytes     │              │         │
│ (BigEnd) │          │ (各含关节数据)│              │         │
└──────────┴──────────┴──────────────┴──────────────┴─────────┘
                                     ↑
                            q_actual @ byte 253 (1-based)
                            6 × 8 bytes = 48 bytes
                            每个 joint position 为 BigEndian double
```

**同步恢复算法** (`localFindSyncHeader`):

```
扫描缓冲区的每个位置 k:
  1. 读取 4 字节大端序整数 L (候选包长)
  2. 检查 L 是否在已知有效包长列表中 [1220, 1116, 1108]
  3. 如果匹配，尝试解析该位置开始的 L 字节数据包
  4. 检查解析出的 q_actual 是否有效 (6 个有限值, |q| < 20)
  5. 全部通过 → 找到同步头
  6. 不通过 → 继续扫描下一个字节
```

### 10.3 URScript 指令

**关节空间移动 (movej)**:
```matlab
script = sprintf('movej([%.10f,%.10f,%.10f,%.10f,%.10f,%.10f],a=%.5f,v=%.5f,t=%.5f,r=%.5f)', ...
    q1, q2, q3, q4, q5, q6, acceleration, velocity, time, blendRadius);
```
参数：
- `q1-q6`: 目标关节角度 (rad)
- `a`: 加速度 (rad/s²)
- `v`: 速度 (rad/s)
- `t`: 时间 (s, 0 = 由控制器自动计算)
- `r`: 混合半径 (m, 0 = 精确到达)

**急停 (stopj)**:
```matlab
sendUrScript(robotIp, cmdPort, 'stopj(2.0)');  % 减速度 2.0 rad/s²
```

**发送协议**:
```matlab
function sendUrScript(robotIp, cmdPort, script)
    client = tcpclient(robotIp, cmdPort, 'Timeout', 2);
    line = uint8([script newline]);  % URScript 命令 + 换行符
    write(client, line, 'uint8');
end
```

### 10.4 安全机制

1. **实时流超时检测**：1 秒内未收到新数据 → 锁定执行按钮
2. **限位检查**：目标关节值超出软限位 → 拒绝执行
3. **验证-执行分离**：必须通过"预演并解锁"才能点击"开始运动"
4. **急停按钮**：独立的大红色按钮，直接发送 `stopj(2.0)`
5. **目标脏标记** (`targetDirty`)：滑条变化时自动失效之前的验证状态

---

## 11. 环境建模与 URDF

### 11.1 URDF 文件结构

项目定义了多个 URDF 模型文件：

| 文件 | 包含内容 | 用途 |
|------|---------|------|
| `assembly/assembly.urdf.xacro` | UR10 + 铲子 + 基座 + Gazebo/ROS 控制配置 | 原始设计模型 |
| `environmental_model/assembly_with_block_with_basin.urdf` | UR10 + 铲子 + 基座 + 方块 + 盆体 | 完整场景（主 GUI 使用） |
| `environmental_model/ur10_shovel_only.urdf` | UR10 + 铲子（无环境物体） | 碰撞检测专用 |
| `environmental_model/block_with_basin.urdf` | 方块 + 盆体 | 环境物体模型 |
| `environmental_model/world.urdf` | 世界坐标系 | 参考 |

### 11.2 场景物体定义

**方块 (block_basin_block)**：1.0m × 1.0m × 0.25m 的灰色盒子

**盆体**：由 5 个独立连杆组成（每个有独立的 visual 和 collision）：
- `block_basin_bottom`：底部平面
- `block_basin_front_wall` / `back_wall`：前/后壁
- `block_basin_left_wall` / `right_wall`：左/右壁

**关键关节**：
- `base_jizuo_to_block_with_basin_frame`：基座 → 方块+盆体坐标系的固定关节，其 `origin` 决定了环境物体的整体位姿（由 YAML 配置动态更新）

### 11.3 碰撞几何体

每个连杆都有 `collision` 子元素定义碰撞几何体：

| 连杆 | 碰撞类型 | 参数 |
|------|---------|------|
| ur10 | 圆柱体 | r=0.075m, l=0.038m |
| ur10_shoulder | 圆柱体 | r=0.075m, l=0.177m |
| ur10_upper_arm | 3 个圆柱体 | 近似复杂形状 |
| ur10_forearm | 3 个圆柱体 | 近似复杂形状 |
| ur10_wrist_1 | 圆柱体 | r=0.0455m, l=0.119m |
| ur10_wrist_2 | 圆柱体 | r=0.0455m, l=0.119m |
| ur10_wrist_3 | 圆柱体 | r=0.045m, l=0.031m |
| sensor_shovel | STL 网格 | 精确铲子形状 |
| sensor_shovel_tcp | STL 网格 | 铲尖形状 |
| block_basin_* | 盒体 | 精确盆体尺寸 |

### 11.4 xacro 预处理

URDF 文件使用 xacro 格式（ROS 的 XML 宏语言），需要在导入前预处理：

```matlab
raw = fileread(xacroPath);
raw = regexprep(raw, '<xacro:arg[^>]*/>\s*', '');     % 移除 xacro:arg 声明
raw = strrep(raw, '$(arg mesh_root)', meshRootName);   % 替换宏变量
tmpUrdf = fullfile(tempdir, 'assembly_preprocessed.urdf');
fwrite(fid, raw, 'char');                              % 写出临时 URDF
robot = importrobot(tmpUrdf, 'MeshPath', meshDir);     % MATLAB 导入
```

### 11.5 环境配置 URDF 中的 ros2_control

`assembly_with_block_with_basin.urdf` 包含完整的 ROS 2 控制配置（`<ros2_control>` 标签），定义了：
- 6 个关节的初始位置值
- Gazebo 仿真插件配置
- 支持 effort/position/velocity 三种控制模式

`rtfg_io.m:readInitialJointPosition` 从这些配置中提取初始关节值：
```matlab
pattern = ['<joint\s+name="' jointName '"[^>]*>[\s\S]*?' ...
           '<state_interface\s+name="position">\s*' ...
           '<param\s+name="initial_value">([^<]+)</param>'];
```

---

## 12. 配置系统与 YAML

### 12.1 运行时配置文件

`realtime_trajectory_fit_gui/environment_runtime_config.yaml` 是统一的运行时配置文件：

```yaml
metadata:
  exported_at: "2026-05-28 09:32:49"
  source_gui: "main_realtime_trajectory_fit_gui"
  target_urdf: "path/to/assembly_with_block_with_basin.urdf"

trajectory:
  parameters:
    left_wall_offset: 0.195334    # 入泥点到左侧壁距离 (m)
    mud_height: 0.108000          # 泥面高度 (m)
    approach_len: 0.234900        # 斜线长度 (m)
    theta_deg: -30.000000         # 入泥角度 (deg)
    depth: 0.052120               # 入泥深度 (m)
    x_plane: -0.005312            # X 截面位置 (m)

  derived_values:                 # 自动计算的派生值（仅记录，不用于输入）
    approach_start_point_xyz: [-0.005312, -0.255095, 0.475450]
    entry_point_xyz: [-0.005312, -0.051666, 0.358000]
    arc_end_point_xyz: [-0.005312, 0.142848, 0.305880]
    approach_length: 0.234900
    arc_radius: 0.389029
    vertical_penetration: 0.052120
    dist_x_negative_wall: 0.311688
    dist_x_positive_wall: 0.052312

environment_pose:
  target_joint: "base_jizuo_to_block_with_basin_frame"
  pose:
    x: -1.830000
    y: 0.000000
    z: 0.000000
    roll_deg: 0.000000
    pitch_deg: 0.000000
    yaw_deg: 180.000000

meanings:                         # 参数说明（文档性质）
  left_wall_offset: "入泥点到左侧壁的水平距离，单位 m"
  ...
```

### 12.2 YAML 读写实现

**读取** (`readRuntimeConfigYaml`):
- 使用简单的行解析器（非完整 YAML 解析器）
- 基于缩进级别识别 section/subsection
- 支持 `trajectory > parameters` 和 `environment_pose > pose` 两个路径
- 使用正则表达式 `^key:\s*value$` 匹配键值对

**写入** (`writeRuntimeConfigYaml`):
- 使用 `sprintf` 格式化所有字段
- 包含时间戳、源 GUI 标识、目标 URDF 路径
- 同时写入计算出的派生值和参数说明

### 12.3 URDF 位姿回写

`writePoseToUrdf` 将环境位姿直接写回 URDF 文件中的 `base_jizuo_to_block_with_basin_frame` 关节：

```matlab
xyzText = sprintf('%.6f %.6f %.6f', pose.x, pose.y, pose.z);
rpyText = sprintf('%.12f %.12f %.12f', deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
% 正则替换 origin 属性
urdfText = regexprep(urdfText, jointPattern, ['$1' xyzText '$3' rpyText '$5']);
```

---

## 13. 虚拟仿真系统

### 13.1 概述

`test/virtual/` 目录包含基于 Simulink 的虚拟 UR10 仿真系统，用于在不连接物理机械臂的情况下进行开发和测试。

### 13.2 核心文件

**`build_ur10_virtual_robot.m`**：
- 从 `assembly.urdf.xacro` 导入与真实机械臂同源的 URDF 模型
- 清除 `sensor_shovel_tcp` 的碰撞体（TCP 是参考点，不参与碰撞检测）
- 提取 6 个可动关节的索引映射

**`init_virtual_ur10.m`** / **`startup_virtual_ur10.m`**：
- 虚拟机器人初始化参数设置
- 关节名称配置（6 个 UR10 标准关节名）
- 软限位设置

**`ur10_virtual_jointspace.slx`**：
- Simulink 关节空间仿真模型
- 包含机械臂动力学模型
- 支持位置/速度/力矩控制模式

**`create_or_load_virtual_simulink_model.m`**：
- 创建或加载虚拟 Simulink 模型
- 自动配置求解器参数

**`evaluate_current_estimation_batch.m`**：
- 电流估算批处理评估
- 用于分析和优化电机电流模型

**`run_virtual_trajectory_demo.m`**：
- 虚拟轨迹演示脚本
- 在仿真环境中运行预设轨迹

**`ur10_gui_control_virtual.m`**：
- 虚拟控制 GUI（与真实控制 GUI 功能对等）

---

## 14. 性能分析与优化

### 14.1 性能瓶颈识别

MATLAB Profiler 分析显示三个主要性能瓶颈：

| 瓶颈环节 | 耗时占比 | 说明 |
|---------|---------|------|
| `inverseKinematics` | ~55% | MATLAB Robotics System Toolbox IK 求解器 |
| `checkCollision` | ~25% | 碰撞间隙计算（遍历所有连杆对） |
| 连续回放轨迹生成 | ~15% | 逐点 IK + 插值 |

### 14.2 C++ MEX 加速效果

C++ MEX 加速的理论和实测效果：

| 指标 | MATLAB 原生 | C++ MEX | 加速比 |
|------|------------|---------|--------|
| 单点 IK 求解 | ~8 ms | ~1.5 ms | ~5.3× |
| 单点碰撞检测 | ~1.2 ms | ~0.02 ms | ~60× |
| 完整轨迹求解 (200 目标点) | ~8 s | ~2.5 s | ~3.2× |

C++ MEX 加速的主要来源：
1. **编译优化**：`-O3` 优化 + 无 JIT 编译开销
2. **内存布局**：Eigen 矩阵连续存储，缓存友好
3. **碰撞缓存**：FCL 对象复用 + thread_local 存储
4. **延迟碰撞**：仅在 IK 可行后才执行碰撞检查
5. **快速数值雅可比**：6×ε 次 FK 计算（vs MATLAB 的内部实现）

### 14.3 MATLAB 侧优化

1. **自适应锚点采样**：将 200+ 目标点压缩到 60-120 个锚点
2. **稀疏碰撞复检**：回放轨迹每 10 帧检测一次
3. **渲染节流**：大量点序列播放时每 2-3 帧渲染一次
4. **MEX 优先策略**：优先尝试 C++，失败才回退 MATLAB

### 14.4 求解器性能诊断

C++ trajectory_solver 内置详细的性能统计 (`SolverTiming`)，每次求解后通过 `mexPrintf` 输出：

```
[MEX stats] Solved=195, Failed=5 | IK=1.23s, Col=0.45s, Wall=2.67s
```

---

## 15. 如何运行

### 15.1 环境要求

**硬件**：
- UR10 机械臂（可选，用于实机控制；虚拟仿真不需要）
- 网络连接（用于 TCP 通信）

**软件**：
- Ubuntu 22.04 LTS (x86_64)
- MATLAB R2025a
  - Robotics System Toolbox
  - ROS Toolbox（可选）
- ROS 2 Humble（MEX 编译需要）
- C++ 编译器：GCC 11+（支持 C++17）
- 依赖库（MEX 编译需要）：
  - Eigen 3 (`libeigen3-dev`)
  - FCL (`libfcl-dev`)
  - orocos-kdl (`liborocos-kdl-dev`)
  - urdfdom (`liburdfdom-dev`)
  - kdl_parser (ROS 2 Humble)
  - console_bridge (`libconsole-bridge-dev`)

### 15.2 快速启动

#### 方式一：主 GUI（轨迹拟合系统）

```matlab
% 1. 启动 MATLAB
% 2. 切换到项目目录
cd /home/liuxiaopeng/公共的/MATLAB_ur10_control/UR10_control_Matlab

% 3. 启动主 GUI
main_realtime_trajectory_fit_gui
```

主 GUI 会自动：
- 加载环境模型和碰撞模型
- 读取 YAML 运行时配置
- 导入初始关节位置
- 渲染 3D 场景

#### 方式二：实时关节控制 GUI

```matlab
cd /home/liuxiaopeng/公共的/MATLAB_ur10_control/UR10_control_Matlab
ur10_gui_control
```

**操作流程**：
1. 点击"连接" → 建立 TCP 连接（端口 30003）
2. 等待实时流稳定（状态显示"正常"）
3. 拖动滑条设置目标关节角度
4. 点击"预演并解锁" → 验证限位
5. 点击"开始运动" → 发送 movej 指令
6. 紧急情况点击"急停" → 发送 stopj(2.0)

#### 方式三：URDF 模型预览

```matlab
cd /home/liuxiaopeng/公共的/MATLAB_ur10_control/UR10_control_Matlab
show_assembly_urdf
```

#### 方式四：二维轨迹 GUI

```matlab
cd /home/liuxiaopeng/公共的/MATLAB_ur10_control/UR10_control_Matlab/test/tarjectory_plan
main_gui
```

### 15.3 MEX 编译

```matlab
% 1. 确保已安装所有依赖库
% 2. 在 MATLAB 中执行：
cd /home/liuxiaopeng/公共的/MATLAB_ur10_control/UR10_control_Matlab
addpath(fullfile(pwd, 'realtime_trajectory_fit_gui', 'mex'));
build_mex

% 3. 验证编译成功：
which rtfg_solver_mex
% 应显示: .../mex/bin/rtfg_solver_mex.mexa64
```

### 15.4 主 GUI 完整工作流

```
1. 启动 GUI
   main_realtime_trajectory_fit_gui
        │
2. 调整环境位姿（如需要）
   点击 "底部盒子移动" → 调节 X/Y/Z/Roll/Pitch/Yaw 滑条
        │
3. 调整轨迹参数（如需要）
   点击 "轨迹调整" → 调节壁距/泥高/斜线长度/入泥角/深度/X截面
        │
4. 执行轨迹拟合
   点击 "开始尖端轨迹拟合"
   → 优先尝试 C++ MEX 加速求解
   → MEX 失败则自动回退 MATLAB 主链
        │
5. 预览轨迹 (自动渲染)
   → 蓝色轨迹线
   → 玫红色 TCP 路径 (如有)
   → 红色碰撞标记 (如有)
        │
6. 保存配置
   点击 "保存 YAML" 或 "保存 YAML + 回写 URDF"
        │
7. 连接真实机械臂（可选）
   → 调整 ur10_gui_control 中的 IP 地址
```

### 15.5 配置参数调整指南

**轨迹质量调优**：

| 参数 | 调整方向 | 影响 |
|------|---------|------|
| `theta_deg` | 更负（如 -45°） | 更陡的切入角，更快到达目标深度 |
| `depth` | 减小 | 减少 IK 难度（接近工作空间边界时） |
| `approach_len` | 增大 | 更长的预备距离，给 IK 更多连续空间 |
| `left_wall_offset` | 调整 | 避开盆壁碰撞 |
| `x_plane` | 调整 | 轨迹在 X 方向的截面位置 |

**环境位姿调优**：
- 确保盆体在 UR10 的工作空间内（约 0.5-1.3m 半径）
- 避免盆体与机械臂基座重叠
- 调整 Yaw 使盆体朝向合适

---

## 16. 依赖项与编译

### 16.1 MATLAB 依赖

```
Toolbox 依赖:
├── Robotics System Toolbox        (importrobot, inverseKinematics,
│                                    checkCollision, getTransform, show)
├── ROS Toolbox                    (可选: rostopic 等 ROS 接口)
└── MATLAB Coder                   (可选: 代码生成)

第三方 MATLAB 依赖:
└── trajectory_plan_3d_gui         (test/tarjectory_plan/trajectory_plan_3d_gui/)
    ├── generate_trajectory_3d.m
    ├── build_gui_state.m
    └── trajectory_params_3d.yaml
```

### 16.2 系统依赖安装

```bash
# ROS 2 Humble (提供 kdl_parser, urdf, console_bridge)
sudo apt install ros-humble-desktop

# Eigen 3
sudo apt install libeigen3-dev

# FCL (Flexible Collision Library)
sudo apt install libfcl-dev

# orocos-kdl
sudo apt install liborocos-kdl-dev

# urdfdom
sudo apt install liburdfdom-dev

# console_bridge
sudo apt install libconsole-bridge-dev

# GCC 11+ (Ubuntu 22.04 默认)
sudo apt install build-essential
```

### 16.3 MEX 环境验证

```matlab
% 检查 C++ 编译器
mex -setup C++

% 检查 MEX 文件是否可用
addpath(fullfile(pwd, 'realtime_trajectory_fit_gui', 'mex', 'bin'));
if exist('rtfg_solver_mex', 'file') == 3
    disp('MEX solver is ready.');
else
    disp('MEX solver NOT found. Run build_mex to compile.');
end
```

### 16.4 性能基准测试

```matlab
% 简单性能测试
cd /home/liuxiaopeng/公共的/MATLAB_ur10_control/UR10_control_Matlab
addpath(fullfile(pwd, 'realtime_trajectory_fit_gui', 'mex'));
benchmark_simple

% 完整 MEX vs MATLAB 对比
benchmark_mex_vs_matlab
```

---

## 17. 故障排除与常见问题

### 17.1 MEX 相关问题

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| `未找到已编译的 rtfg_solver_mex` | MEX 未编译 | 执行 `build_mex` |
| `Invalid MEX file: libfcl.so not found` | 运行时库缺失 | 安装 `libfcl-dev`；检查 `$LD_LIBRARY_PATH` |
| `rtfg_solver_mex:RuntimeError: KDL chain not available` | KDL 链构建失败 | 检查 URDF 中 `ur10` 到 `ur10_wrist_3` 的关节链 |
| `第 X/N 个目标位姿 MEX 求解失败` | IK 不收敛 | 调整目标位姿（简化姿态要求）；增加 seed 数量 |
| `std::bad_alloc` | 内存不足 | 减少目标位姿数量；增加系统内存 |
| `URDF parse error` | URDF 文件语法错误 | 使用 `checkUrdf` 工具验证 URDF |
| MEX 编译失败：`fcl/fcl.h not found` | FCL 头文件路径不正确 | 检查 `/usr/include/fcl` 是否存在 |
| MEX 编译失败：`undefined reference to kdl_parser` | 链接库顺序错误 | 确保 `-lkdl_parser` 在 `-lorocos-kdl` 之前 |

### 17.2 连接问题

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 连接失败 | IP 地址或端口错误 | 验证 `cfg.robotIp`（默认 `10.160.9.21`） |
| 实时流超时 | 网络延迟 | 增大 `cfg.streamStaleSec` |
| 数据包不同步 | 字节对齐问题 | 系统自动恢复；增大缓冲区 |
| 无法发送指令 | 端口 30002 被占用 | 检查其他客户端连接 |

### 17.3 IK 求解问题

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| IK 返回非有限值 | 目标位姿不可达 | 检查目标是否在工作空间内 |
| 位置误差过大 | 目标超出工作空间 | 调整环境位姿使目标更近 |
| 姿态误差过大 | 姿态要求过于严格 | 降低权重级（strict → relaxed → position_only） |
| clearance 过小 | 碰撞不可避免 | 调整轨迹参数避免碰撞区域 |

### 17.4 性能问题

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| GUI 卡顿 | 渲染过于频繁 | 增大 `choosePlaybackRenderStride` |
| 轨迹拟合过慢 | 目标位姿过多 | 减少锚点数量上限 (`maxAnchors`) |
| MEX 求解慢于 MATLAB | 种子策略效率低 | 检查 IK 收敛条件；增加阻尼 |

### 17.5 调试技巧

1. **启用调试输出**：MATLAB 控制台会自动显示 `[INFO]`/`[DEBUG]`/`[WARN]` 级别日志
2. **检查 URDF**：使用 `show_assembly_urdf` 验证模型是否正确加载
3. **单步测试 IK**：
```matlab
robot = importrobot('path/to/urdf');
ik = inverseKinematics('RigidBodyTree', robot);
[q, info] = ik('sensor_shovel_tcp', targetTform, weights, qSeed);
```
4. **验证碰撞配置**：
```matlab
[isColliding, sepDist] = checkCollision(robot, q, 'Exhaustive', 'on');
```
5. **查看 YAML 配置**：直接编辑 `environment_runtime_config.yaml` 可以绕过 GUI 进行批量参数测试

---

## 18. 开发路线图

### 18.1 短期目标（当前优先）

1. **MEX IK 稳定性提升**
   - 增强首段/切入段 IK 鲁棒性
   - 优化多 seed 连续选解策略
   - 保证共享端点附近不跳分支
   - 目标：整条轨迹在 C++ 中完整跑通，成功率 > 95%

2. **性能基准测试完善**
   - 标准化 benchmark 脚本
   - 与 MATLAB 主链的 A/B 对比
   - 不同轨迹复杂度下的性能曲线

### 18.2 中期目标

1. **碰撞检测升级**
   - 引入盆体精确 mesh 碰撞（替代当前盒体近似）
   - 支持更精细的环境物体模型

2. **轨迹规划增强**
   - 支持更多轨迹类型（S 曲线、螺旋线等）
   - 自动碰撞规避轨迹优化
   - 时间最优轨迹规划

3. **实时控制完善**
   - 支持 `servoj` 实时关节流控制
   - 力控/阻抗控制接口

### 18.3 长期展望

1. **完全 C++ 化**
   - 将所有 MATLAB 计算逻辑迁移到 C++ MEX
   - MATLAB 仅保留 GUI 和渲染
   - 目标：完整轨迹求解 < 1 秒

2. **ROS 2 集成**
   - 通过 ROS 2 topic/service 控制机械臂
   - MoveIt 2 运动规划集成
   - Gazebo 协同仿真

3. **多机器人支持**
   - 扩展支持其他 UR 系列（UR3/UR5/UR16）
   - 通用机械臂描述格式

---

## 19. 附录

### 19.1 关键常量速查

| 常量 | 值 | 说明 |
|------|-----|------|
| `clearanceThreshold` | 2e-3 m (2 mm) | 碰撞间隙阈值 |
| `ik_position_tolerance` | 3e-2 m (3 cm) | IK 位置容差 |
| `max_iterations` | 60 | DLS 最大迭代次数 |
| `lambda` | 5e-3 | DLS 默认阻尼系数 |
| `numerical_epsilon` | 1e-6 | 数值雅可比步长 |
| `samplePeriod` | 0.05 s (50 ms) | 实时同步采样周期 |
| `streamStaleSec` | 1.0 s | 实时流超时 |
| `movej_a` | 0.2 rad/s² | 默认 movej 加速度 |
| `movej_v` | 0.2 rad/s | 默认 movej 速度 |
| `UR10 工作空间半径` | ~1.3 m | UR10 最大伸展距离 |
| `UR10 有效负载` | 10 kg | UR10 额定负载 |

### 19.2 MATLAB 搜索路径

主 GUI 运行时会自动添加以下路径：

```
realtime_trajectory_fit_gui/
realtime_trajectory_fit_gui/ui/
realtime_trajectory_fit_gui/rendering/
realtime_trajectory_fit_gui/kinematics/
realtime_trajectory_fit_gui/trajectory/
realtime_trajectory_fit_gui/io/
realtime_trajectory_fit_gui/utils/
realtime_trajectory_fit_gui/collision/
realtime_trajectory_fit_gui/mex/
realtime_trajectory_fit_gui/mex/bin/
test/tarjectory_plan/trajectory_plan_3d_gui/
```

### 19.3 参考资料

- [UR10 技术规格](https://www.universal-robots.com/products/ur10-robot/)
- [MATLAB Robotics System Toolbox 文档](https://www.mathworks.com/help/robotics/)
- [Orocos KDL 文档](https://www.orocos.org/kdl.html)
- [FCL (Flexible Collision Library)](https://github.com/flexible-collision-library/fcl)
- [Eigen 文档](https://eigen.tuxfamily.org/)
- [URScript 编程手册](https://www.universal-robots.com/download/)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)

### 19.4 术语表

| 术语 | 英文 | 说明 |
|------|------|------|
| TCP | Tool Center Point | 工具中心点，机械臂末端执行器的参考点 |
| IK | Inverse Kinematics | 逆运动学，从 TCP 位姿反求关节角度 |
| FK | Forward Kinematics | 正运动学，从关节角度计算 TCP 位姿 |
| DH | Denavit-Hartenberg | 机器人运动学建模的标准参数约定 |
| DLS | Damped Least Squares | 阻尼最小二乘法，数值 IK 求解算法 |
| LMA | Levenberg-Marquardt Algorithm | 列文伯格-马夸尔特算法，非线性优化 |
| SLERP | Spherical Linear Interpolation | 球面线性插值，用于姿态平滑过渡 |
| MEX | MATLAB Executable | MATLAB 可执行文件（C/C++ 编译的共享库） |
| FCL | Flexible Collision Library | 灵活碰撞检测库 |
| KDL | Kinematics and Dynamics Library | 运动学与动力学库 |
| URDF | Unified Robot Description Format | 统一机器人描述格式 |
| RTDE | Real-Time Data Exchange | 实时数据交换协议 |
| BVH | Bounding Volume Hierarchy | 包围体层次结构（加速碰撞检测） |
| AABB | Axis-Aligned Bounding Box | 轴对齐包围盒 |

### 19.5 文件大小统计

| 类别 | 文件数 | 代码行数（约） |
|------|--------|--------------|
| MATLAB 核心 | 16 | ~3,500 |
| MATLAB 支持 | 15 | ~2,000 |
| C++ 源码 | 12 | ~2,200 |
| URDF 模型 | 5 | ~1,500 |
| 配置文件 | 5 | ~200 |
| 文档 | 10 | ~3,000 |
| **总计** | **~63** | **~12,400** |

---

> **文档维护说明**：本文档应随项目代码同步更新。每个重大功能变更或架构调整后，请更新相应章节。  
> **联系方式**：Cagedcarrow  
> **项目仓库**：`/home/liuxiaopeng/公共的/MATLAB_ur10_control/`  
> **技术文档**：[GitHub Pages 在线文档](https://cagedcarrow.github.io/UR10_control_Matlab/realtime_trajectory_fit_gui_技术文档/)

---

*文档结束。共 19 章，涵盖项目架构、算法实现、使用指南与开发路线。*
