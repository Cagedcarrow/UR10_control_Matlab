# UR10 机器人逆动力学求解器 — ROS2 集成项目

## 项目概述

本项目是一个 **ROS2 (Humble Hawksbill) 软件模块**，用于求解**串联机械臂的逆动力学问题**，并具体针对 **6 自由度 UR10 工业机器人** 进行了专门实现。项目由意大利萨莱诺大学（Università degli Studi di Salerno）自动控制组的 Vincenzo Petrone 和 Enrico Ferrentino 等人开发，基于 BSD 3-Clause 许可证开源。

项目核心功能是：**给定机器人关节位置、速度和加速度，计算维持该运动所需的关节力矩（torques），或输出惯性矩阵、科里奥利向量、重力向量等动力学矩阵**。这在机器人控制（如前馈补偿、力矩控制、动力学参数辨识）中至关重要。

---

## 项目结构树

```
capsule-8515919/
├── metadata/
│   └── metadata.yml              # Code Ocean 元数据：项目名称、描述、作者信息
├── environment/
│   ├── Dockerfile                 # Docker 构建文件，基于 ubuntu-22.04 桌面版
│   └── postInstall                # 安装后脚本：配置 ROS2 Humble 环境
├── code/
│   ├── run.sh                     # CI 构建/测试入口脚本
│   ├── README.md                  # 项目根 README
│   ├── LICENSE                    # BSD 3-Clause 许可证
│   ├── inverse_dynamics_solver/   # [核心抽象层] 逆动力学求解器抽象基类
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── config/
│   │   │   └── payload_parameters.yaml    # 负载惯性参数模板（默认全零）
│   │   ├── doc/media/
│   │   │   └── inverse_dynamics_solver.png
│   │   └── include/inverse_dynamics_solver/
│   │       └── inverse_dynamics_solver.h  # ★ 核心抽象基类定义
│   ├── kdl_inverse_dynamics_solver/       # [KDL 实现] 基于 KDL 的通用求解器（仿真）
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── kdl_inverse_dynamics_solver.xml # pluginlib 插件注册
│   │   ├── launch/
│   │   │   └── test_kdl_inverse_dynamics_solver.py
│   │   ├── include/kdl_inverse_dynamics_solver/
│   │   │   ├── kdl_inverse_dynamics_solver.h  # ★ KDL 求解器头文件
│   │   │   └── exceptions.h                    # 异常定义
│   │   ├── src/
│   │   │   └── kdl_inverse_dynamics_solver.cpp # ★ KDL 求解器实现
│   │   └── test/
│   │       └── test_kdl_inverse_dynamics_solver.cpp # 单元测试
│   ├── ur10_inverse_dynamics_solver/            # [UR10 实现] 真实 UR10 机器人专属求解器
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── ur10_inverse_dynamics_solver.xml     # pluginlib 插件注册
│   │   ├── config/
│   │   │   └── franka_hand_payload_parameters.yaml # Franka Hand 夹爪负载参数
│   │   ├── launch/
│   │   │   └── test_ur10_inverse_dynamics_solver.py
│   │   ├── include/ur10_inverse_dynamics_solver/
│   │   │   ├── ur10_inverse_dynamics_solver.h      # ★ UR10 求解器头文件
│   │   │   ├── getInertiaCurrents.h                 # MATLAB 自动生成：惯性电流计算
│   │   │   ├── getCoriolisCurrents.h                # MATLAB 自动生成：科里奥利电流计算
│   │   │   ├── getGravityCurrents.h                 # MATLAB 自动生成：重力电流计算
│   │   │   ├── getCurrents.h                        # MATLAB 自动生成：综合电流计算
│   │   │   ├── getPayloadRegressionMatrix.h         # 负载回归矩阵（综合）
│   │   │   ├── getPayloadInertiaRegressionMatrix.h  # 负载惯性回归矩阵
│   │   │   ├── getPayloadCoriolisRegressionMatrix.h # 负载科里奥利回归矩阵
│   │   │   └── getPayloadGravityRegressionMatrix.h  # 负载重力回归矩阵
│   │   ├── src/
│   │   │   ├── ur10_inverse_dynamics_solver.cpp     # ★ UR10 求解器主实现
│   │   │   ├── getInertiaCurrents.cpp
│   │   │   ├── getCoriolisCurrents.cpp
│   │   │   ├── getGravityCurrents.cpp
│   │   │   ├── getCurrents.cpp
│   │   │   ├── getPayloadRegressionMatrix.cpp
│   │   │   ├── getPayloadInertiaRegressionMatrix.cpp
│   │   │   ├── getPayloadCoriolisRegressionMatrix.cpp
│   │   │   └── getPayloadGravityRegressionMatrix.cpp
│   │   └── test/
│   │       ├── test_ur10_inverse_dynamics_solver.cpp # 单元测试
│   │       └── bagfiles/
│   │           ├── metadata.yaml
│   │           └── validation_trajectory.db3         # ROS2 bag 验证轨迹数据
│   └── ros_extensions_utilities/                    # [工具库] ROS 扩展工具函数
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── README.md
│       └── include/ros_extensions_utilities/
│           ├── ros_extensions_utils.h     # ★ 工具函数：参数读取、时间戳、链名称提取
│           └── exceptions.h               # 自定义异常
```

---

## 项目架构与模块详解

本项目的设计采用了**分层架构**，核心是通过 ROS2 的 **pluginlib** 机制实现多态插件化求解器：

### 1. [inverse_dynamics_solver](code/inverse_dynamics_solver/) — 抽象基类层

这是整个项目的**核心抽象层**，定义了一个纯虚基类 `InverseDynamicsSolver`，提供了逆动力学求解的标准接口：

| 方法 | 功能 |
|------|------|
| `getInertiaMatrix(q)` | 返回惯性矩阵 H(q) |
| `getCoriolisVector(q, dq)` | 返回科里奥利/离心力向量 C(q,dq)·dq |
| `getGravityVector(q)` | 返回重力向量 g(q) |
| `getFrictionVector(dq)` | 返回摩擦力向量 f(dq) |
| `getDynamicParameters(q, dq)` | 一次性返回 (H, C, g) 三元组 |
| `getTorques(q, dq, ddq)` | 返回总力矩 H·ddq + C·dq + g |

**关键设计点**：
- 默认构造函数为无参（满足 pluginlib 要求），通过 `initialize()` 方法传入 ROS2 节点进行初始化
- 内置**负载惯性参数管理**（质量、质心 COM、惯性张量），每个维度 10 个参数
- 负载参数从 ROS2 参数系统读取，支持运行时动态配置
- 参数校验：质量必须非负，COM 长度为 3，惯性张量长度为 6

### 2. [kdl_inverse_dynamics_solver](code/kdl_inverse_dynamics_solver/) — KDL 通用求解器（仿真用）

基于 **[Orocos KDL](https://www.orocos.org/kdl.html)** 库的通用逆动力学求解器实现。它通过 URDF（统一机器人描述格式）解析机器人运动学/动力学参数，适用于**仿真环境**中的任意机器人。

**关键特性**：
- 使用 `kdl_parser` 从 ROS 参数服务器加载 URDF 模型
- 通过 `KDL::ChainDynParam` 进行动力学计算
- 依赖 `ur_description` 和 `ur_moveit_config` 获取 UR10 的 URDF/SRDF 描述
- **不包含摩擦力模型**（KDL 不提供摩擦力计算），`getFrictionVector` 返回零向量
- 使用 MoveIt 的 `JointModelGroup` 提取运动链的连杆名称

**实现流程**：
```
ROS Node → URDF Parameter → KDL Tree → KDL Chain → ChainDynParam → 动力学计算
```

### 3. [ur10_inverse_dynamics_solver](code/ur10_inverse_dynamics_solver/) — UR10 真实机器人求解器

这是针对真实 UR10 机械臂的**高精度专用求解器**。其核心模型基于 **MATLAB 系统辨识**获得，以**电流级**（current-level）回归形式表达动力学模型。

**核心创新点**：
- **电流级建模**：模型在电机电流层面辨识，而非传统的力矩层面
- **6 个关节的电机增益 K1~K6**：将电流转换为力矩（τ = K·i）
- **19 维负载动力学参数向量**：比基类的 10 个惯性参数更丰富，包含了质量和质心的耦合项
- **Sigmoid 摩擦力模型**：采用非线性 S 形曲线（sigmoid）更精确地建模关节摩擦

**MATLAB 自动生成的函数**（共 8 个头文件 + 8 个 cpp 文件）：
| 函数 | 输入 | 输出 |
|------|------|------|
| `getInertiaCurrents` | 关节位置 q | 惯性电流矩阵 (6×6) |
| `getCoriolisCurrents` | q, dq | 科里奥利电流向量 (6×1) |
| `getGravityCurrents` | q | 重力电流向量 (6×1) |
| `getCurrents` | q, dq, ddq | 总电流向量 (6×1) |
| `getPayloadRegressionMatrix` | q, dq, ddq | 负载回归矩阵 (6×19) |
| `getPayloadInertiaRegressionMatrix` | q | 负载惯性回归矩阵 |
| `getPayloadCoriolisRegressionMatrix` | q, dq | 负载科里奥利回归矩阵 |
| `getPayloadGravityRegressionMatrix` | q | 负载重力回归矩阵 |

**完整力矩计算公式**：
```
τ = K · [i_arm(q, dq, ddq) + i_friction(dq)] + Y(q, dq, ddq) · π_load
  = K · i_arm + Y · π_load + K · i_friction
```
其中 `Y` 是负载回归矩阵，`π_load` 是 19 维负载动力学系数。

### 4. [ros_extensions_utilities](code/ros_extensions_utilities/) — 工具库

提供 ROS2 相关的辅助功能：
- `getParameterFromNode()`：模板函数，从 ROS2 节点安全读取参数（支持带默认值和不带默认值两种重载）
- `nowStr()`：返回当前时间戳字符串
- `extractLinkNamesFromJointModelGroup()`：从 MoveIt 规划组提取完整的运动链连杆名称
- `UnavailableParameterException`：自定义参数缺失异常

---

## 关键文件清单

### 架构核心

| 文件 | 说明 |
|------|------|
| [code/inverse_dynamics_solver/include/inverse_dynamics_solver/inverse_dynamics_solver.h](code/inverse_dynamics_solver/include/inverse_dynamics_solver/inverse_dynamics_solver.h) | ★ 抽象基类，定义逆动力学求解器统一接口 |
| [code/kdl_inverse_dynamics_solver/include/kdl_inverse_dynamics_solver/kdl_inverse_dynamics_solver.h](code/kdl_inverse_dynamics_solver/include/kdl_inverse_dynamics_solver/kdl_inverse_dynamics_solver.h) | KDL 求解器头文件 |
| [code/kdl_inverse_dynamics_solver/src/kdl_inverse_dynamics_solver.cpp](code/kdl_inverse_dynamics_solver/src/kdl_inverse_dynamics_solver.cpp) | KDL 求解器实现（调用 KDL::ChainDynParam） |
| [code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/ur10_inverse_dynamics_solver.h](code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/ur10_inverse_dynamics_solver.h) | UR10 求解器头文件（含电机增益常量） |
| [code/ur10_inverse_dynamics_solver/src/ur10_inverse_dynamics_solver.cpp](code/ur10_inverse_dynamics_solver/src/ur10_inverse_dynamics_solver.cpp) | UR10 求解器实现（电流→力矩转换 + 负载补偿） |

### MATLAB 生成的动力学模型

| 文件 | 说明 |
|------|------|
| [code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/getInertiaCurrents.h](code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/getInertiaCurrents.h) | 惯性电流计算（MATLAB 生成） |
| [code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/getCoriolisCurrents.h](code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/getCoriolisCurrents.h) | 科里奥利电流计算（MATLAB 生成） |
| [code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/getGravityCurrents.h](code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/getGravityCurrents.h) | 重力电流计算（MATLAB 生成） |
| [code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/getCurrents.h](code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/getCurrents.h) | 综合电流计算（MATLAB 生成） |
| [code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/getPayloadRegressionMatrix.h](code/ur10_inverse_dynamics_solver/include/ur10_inverse_dynamics_solver/getPayloadRegressionMatrix.h) | 负载回归矩阵（MATLAB 生成） |

### ROS2 插件注册

| 文件 | 说明 |
|------|------|
| [code/kdl_inverse_dynamics_solver/kdl_inverse_dynamics_solver.xml](code/kdl_inverse_dynamics_solver/kdl_inverse_dynamics_solver.xml) | KDL 求解器的 pluginlib 插件描述文件 |
| [code/ur10_inverse_dynamics_solver/ur10_inverse_dynamics_solver.xml](code/ur10_inverse_dynamics_solver/ur10_inverse_dynamics_solver.xml) | UR10 求解器的 pluginlib 插件描述文件 |

### 配置文件

| 文件 | 说明 |
|------|------|
| [code/inverse_dynamics_solver/config/payload_parameters.yaml](code/inverse_dynamics_solver/config/payload_parameters.yaml) | 负载参数模板（默认全零，即无负载） |
| [code/ur10_inverse_dynamics_solver/config/franka_hand_payload_parameters.yaml](code/ur10_inverse_dynamics_solver/config/franka_hand_payload_parameters.yaml) | Franka Hand 夹爪负载参数（质量 0.73 kg） |

### 测试与验证

| 文件 | 说明 |
|------|------|
| [code/kdl_inverse_dynamics_solver/test/test_kdl_inverse_dynamics_solver.cpp](code/kdl_inverse_dynamics_solver/test/test_kdl_inverse_dynamics_solver.cpp) | KDL 求解器 GTest 单元测试（含固定状态参考值对比） |
| [code/kdl_inverse_dynamics_solver/launch/test_kdl_inverse_dynamics_solver.py](code/kdl_inverse_dynamics_solver/launch/test_kdl_inverse_dynamics_solver.py) | KDL 测试启动文件 |
| [code/ur10_inverse_dynamics_solver/test/test_ur10_inverse_dynamics_solver.cpp](code/ur10_inverse_dynamics_solver/test/test_ur10_inverse_dynamics_solver.cpp) | UR10 求解器 GTest 单元测试（含 bag 轨迹验证 + 负载参数验证） |
| [code/ur10_inverse_dynamics_solver/launch/test_ur10_inverse_dynamics_solver.py](code/ur10_inverse_dynamics_solver/launch/test_ur10_inverse_dynamics_solver.py) | UR10 测试启动文件 |
| [code/ur10_inverse_dynamics_solver/test/bagfiles/validation_trajectory.db3](code/ur10_inverse_dynamics_solver/test/bagfiles/validation_trajectory.db3) | ROS2 bag 验证轨迹（从 ROS1 Noetic 转换而来） |

### 构建与环境

| 文件 | 说明 |
|------|------|
| [environment/Dockerfile](environment/Dockerfile) | 基于 Ubuntu 22.04 的 Docker 构建镜像 |
| [environment/postInstall](environment/postInstall) | 后安装脚本（ROS2 Humble 完整桌面版安装） |
| [code/run.sh](code/run.sh) | CI 构建与测试入口（colcon build → colcon test） |
| [metadata/metadata.yml](metadata/metadata.yml) | Code Ocean 胶囊元数据 |

---

## 技术栈

| 技术 | 用途 |
|------|------|
| **ROS2 Humble** | 机器人操作系统框架 |
| **C++17 / Eigen3** | 数值计算与线性代数 |
| **Orocos KDL** | 通用运动学/动力学库（用于仿真求解器） |
| **MoveIt** | 运动规划框架（用于提取运动链信息） |
| **pluginlib** | ROS2 插件加载机制（多态求解器） |
| **GTest / launch_testing** | 单元测试框架 |
| **MATLAB** | UR10 动力学模型系统辨识与代码自动生成 |
| **colcon** | ROS2 构建系统 |
| **rosbag2** | 数据记录与回放（用于测试验证） |

---

## 应用场景

1. **前馈力矩控制**：在机器人轨迹跟踪控制器中，使用计算出的力矩作为前馈项，提高跟踪精度
2. **动力学参数辨识**：通过回归矩阵形式，可在线辨识负载参数
3. **仿真验证**：KDL 求解器可在 Gazebo 等仿真环境中使用
4. **拖动示教/力控制**：准确的动力学模型是零力/重力补偿控制的基础
5. **碰撞检测**：基于力矩残差的碰撞检测需要精确的逆动力学模型
