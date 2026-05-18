# 基于逆动力学求解器的 UR10 实时测量节点设计方案

## 现有代码的限制

当前项目**是一个 C++ 库（pluginlib 插件）**，而非一个可直接运行的 ROS2 节点。它提供逆动力学计算能力，但不包含订阅 `/joint_states` 话题、发布测量数据或与真实 UR10 机器人通信的节点层。

**已有能力**：
- `ur10::InverseDynamicsSolverUR10` 插件已完整实现，可计算 UR10 的所有动力学量
- MATALB 辨识的电流级动力学模型已嵌入（8 个自动生成函数 + sigmoid 摩擦模型）
- 负载参数支持运行时配置（YAML → ROS2 参数）

**缺失部分**（需要构建的节点框架）：
- 订阅 UR10 驱动发布的实时关节状态
- 加速度的估算（`/joint_states` 不直接提供加速度）
- 将计算结果发布为 ROS2 话题
- 数据记录与可视化

---

## 架构设计方案

```
┌─────────────────────────────────────────────────────────────────┐
│                     Measurement Node                            │
│                                                                 │
│  ┌──────────────┐    ┌────────────────────┐    ┌─────────────┐ │
│  │ /joint_states│───▶│ Joint State Buffer  │───▶│ Accelerator │ │
│  │  (subscribe) │    │  (滑动窗口，缓存    │    │ (数值微分    │ │
│  └──────────────┘    │   q, dq, effort)    │    │  求 qdd)    │ │
│                      └────────────────────┘    └──────┬──────┘ │
│                                                        │        │
│  ┌──────────────┐                                      │        │
│  │ UR10 Solver  │◀─────────────────────────────────────┘        │
│  │  (pluginlib) │── q, dq, ddq                                  │
│  └──────┬───────┘                                               │
│         │ 输出: H, C, g, τ_computed, τ_friction                 │
│         ▼                                                       │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                  Publishers                              │    │
│  │  /diagnostics    /torques_components    /model_quality   │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                  Service / Action                        │    │
│  │  ~/compute_dynamics (srv)    ~/identify_payload (action)│    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 可测量的数据类型

### 1. 关节状态测量（来自 /joint_states 话题）

| 符号 | 名称 | 维度 | 来源 | 说明 |
|------|------|------|------|------|
| q | 关节位置 | 6×1 | UR10 驱动编码器 | 单位 rad |
| dq | 关节速度 | 6×1 | UR10 驱动编码器微分 | 单位 rad/s |
| τ_measured | 实测关节力矩 | 6×1 | UR10 驱动电流估算 | 单位 Nm（来自 motor currents） |

> **注意**：`/joint_states` **不提供关节加速度** qdd，需要通过数值微分估算（见下文）。

### 2. 逆动力学计算量（求解器直接输出）

| 符号 | 名称 | 维度 | 公式 | 说明 |
|------|------|------|------|------|
| H(q) | **惯性矩阵** | 6×6 | `getInertiaMatrix(q)` | 对称正定矩阵，与关节位置相关 |
| C(q,dq)·dq | **科里奥利/离心向量** | 6×1 | `getCoriolisVector(q, dq)` | 速度相关的非线性力 |
| g(q) | **重力向量** | 6×1 | `getGravityVector(q)` | 仅与位置相关的重力补偿 |
| f(dq) | **摩擦力向量** | 6×1 | `getFrictionVector(dq)` | 含 sigmoid 非线性摩擦模型 |
| τ_computed | **计算力矩** | 6×1 | `getTorques(q, dq, ddq)` | 总力矩 = H·ddq + C·dq + g + f |

### 3. 实测 vs 计算对比（核心测量指标）

| 指标 | 公式 | 含义 |
|------|------|------|
| 力矩残差 Δτ | τ_measured - τ_computed | 模型与真实偏差，可用于碰撞检测 |
| RMS 力矩误差 | sqrt(mean(Δτ²)) | 模型总体精度 |
| 各关节误差 | Δτ₁, Δτ₂, ..., Δτ₆ | 逐关节评估 |
| 摩擦占比 | \|f(dq)\| / \|τ_computed\| | 摩擦在总力矩中的比例 |
| 重力占比 | \|g(q)\| / \|τ_computed\| | 重力在总力矩中的比例 |
| 惯性占比 | \|H·ddq\| / \|τ_computed\| | 惯性在总力矩中的比例 |

### 4. 内部电流级测量

求解器内部使用电流级模型计算，这些中间量也可以暴露：

| 符号 | 名称 | 维度 | 单位 |
|------|------|------|------|
| i_inertia | 惯性电流 | 6×1 | A（安培） |
| i_coriolis | 科里奥利电流 | 6×1 | A |
| i_gravity | 重力电流 | 6×1 | A |
| i_total | 总电流（无摩擦） | 6×1 | A |
| i_friction | 摩擦电流 | 6×1 | A |
| K | 电机增益矩阵 | 6×6 (对角) | Nm/A |

**电流与力矩关系**：
```
τ_joint = K · i_total + K · i_friction + τ_payload
```

### 5. 负载辨识与测量

求解器支持通过回归矩阵计算负载对力矩的贡献：

| 符号 | 名称 | 维度 | 说明 |
|------|------|------|------|
| YL | 负载回归矩阵 | 6×19 | 由 q, dq, ddq 决定 |
| π_load | 负载动力学系数 | 19×1 | 包含质量、质心、惯性张量的组合 |
| τ_payload | 负载力矩 | 6×1 | YL · π_load |

可辨识的**负载参数**（从 π_load 解算）：

| 参数 | 含义 | 典型值（Franka Hand） |
|------|------|----------------------|
| m | 负载质量 | 0.73 kg |
| COM_x, COM_y, COM_z | 质心坐标 | [0.0, 0.01, 0.03] m |
| Ixx, Ixy, Ixz, Iyy, Iyz, Izz | 惯性张量分量 | 见配置文件 |

---

## 节点接口设计

### 订阅话题

```
/joint_states                          sensor_msgs/JointState   UR10 驱动发布的关节状态
```

### 发布话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `~/torques_total` | `std_msgs/Float64MultiArray` | 与 joint_states 同步 | 6 维总力矩 |
| `~/torques_components` | `ur10_msgs/DynamicsComponents` (自定义) | 同上 | 惯性/科里奥利/重力/摩擦分量 |
| `~/inertia_matrix` | `std_msgs/Float64MultiArray` | 同上 | 6×6 惯性矩阵（展平） |
| `~/model_residual` | `std_msgs/Float64MultiArray` | 同上 | τ_measured - τ_computed |
| `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | 模型健康状态 |
| `~/motor_currents` | `std_msgs/Float64MultiArray` | 与 joint_states 同步 | 6 维电流值 |

### 服务

```
~/compute_dynamics (ur10_msgs/ComputeDynamics)
    请求:  joint_positions[6], joint_velocities[6], joint_accelerations[6]
    响应:  torques[6], inertia_matrix[36], coriolis[6], gravity[6], friction[6]

~/set_payload (ur10_msgs/SetPayload)
    请求:  mass, center_of_mass[3], inertia_tensor[6]
    响应:  success, message

~/identify_payload (ur10_msgs/IdentifyPayload) — Action
    目标:  采集 N 秒数据，在线辨识负载参数
    反馈:  采集进度 %
    结果:  辨识出的 mass, COM, 惯性张量
```

---

## 关键实现细节

### 加速度估算

由于 `/joint_states` 不提供加速度，需对速度进行数值微分：

```cpp
// 滑动窗口法：中心差分 + 低通滤波
// 窗口大小 N = 5（可配置）
qdd(t) = (dq(t+Δt) - dq(t-Δt)) / (2Δt)
```

> **替代方案**：UR10 的 RTDE 接口可以输出 `actual_qdd`（关节加速度），若使用 `ur_robot_driver` 的 RTDE 模式，可绕过数值微分直接获得加速度。

### 轨迹数据采集

```cpp
// 缓存结构
struct JointSample {
    rclcpp::Time stamp;
    Vector6d position;
    Vector6d velocity;
    Vector6d effort;      // 实测力矩
};

// 滑动窗口（保存最后 N 个样本用于加速度估算）
std::deque<JointSample> buffer_;
```

### 导出 Rosbag 记录

节点应内置 bag 写入功能，将原始关节状态与所有计算结果同步记录为 ROS2 bag，用于离线分析：

```
recorded_topics:
  - /joint_states
  - ~/torques_components
  - ~/model_residual
  - ~/motor_currents
```

---

## 快速开始（节点构建指南）

### 1. 新增 ROS2 包

创建 `ur10_dynamics_measurement` 包，依赖现有的 `ur10_inverse_dynamics_solver`：

```xml
<depend>ur10_inverse_dynamics_solver</depend>
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>diagnostic_msgs</depend>
```

### 2. 核心循环（伪代码）

```cpp
class DynamicsMeasurementNode : public rclcpp::Node {
public:
    DynamicsMeasurementNode() : Node("ur10_dynamics_measurement") {
        // 1. 加载 solver plugin
        solver_ = loader_->createUniqueInstance("ur10/InverseDynamicsSolverUR10");
        solver_->initialize(shared_from_this(), {});

        // 2. 订阅 joint_states
        sub_ = create_subscription<JointState>(
            "/joint_states", 10, std::bind(&onJointState, this, _1));

        // 3. 创建 publishers
        pub_torques_ = create_publisher<Float64MultiArray>("~/torques_total", 10);
        pub_residual_ = create_publisher<Float64MultiArray>("~/model_residual", 10);
        // ...
    }

    void onJointState(const JointState::SharedPtr msg) {
        // 缓存当前样本
        buffer_.push_back({now(), msg->position, msg->velocity, msg->effort});

        // 估算加速度
        Vector6d qdd = estimateAcceleration();

        // 计算逆动力学
        Vector6d torques = solver_->getTorques(q, dq, qdd);
        Vector6d friction = solver_->getFrictionVector(dq);
        // ...

        // 计算残差
        Vector6d residual = measured_effort - torques;

        // 发布结果
        publishAll(torques, friction, residual, ...);
    }
};
```

### 3. 启动

```bash
# 启动 UR10 驱动
ros2 launch ur_robot_driver ur10_launch.py

# 启动测量节点（含 Franka Hand 负载）
ros2 run ur10_dynamics_measurement measurement_node \
    --ros-args -p mass:=0.73 \
    -p center_of_mass:="[0.0, 0.01, 0.03]" \
    -p inertia_tensor:="[0.001, 0.0, 0.0, 0.0025, 0.0, 0.0017]"
```

---

## 可回答的关键问题

部署本节点后，可以解答以下问题：

1. **动力学模型有多精确？** — τ_computed vs τ_measured 的残差分析
2. **摩擦模型是否准确？** — 低速段 sigmoid 模型 vs 实测数据的拟合度
3. **负载参数是否正确？** — 辨识出的负载参数与标称值对比
4. **机器人在空载/带载下动力学表现如何？** — 不同负载配置下的残差统计
5. **动力学模型能否用于碰撞检测？** — 力矩残差阈值 vs 外部碰撞灵敏度
6. **哪个关节的非线性最强？** — 各关节摩擦参数和力矩残差对比
