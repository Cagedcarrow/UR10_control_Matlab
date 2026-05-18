# UR10 Virtual Simulink 节点说明

本文档说明 `test/virtual` 工程中由 `create_or_load_virtual_simulink_model.m` 自动搭建的 Simulink 模型节点含义、连接关系与数据流。

## 1. 模型入口与用途

- 模型名：`ur10_virtual_jointspace`
- 主要用途：
  - 接收关节命令轨迹，生成虚拟关节响应
  - 计算 `dq/ddq` 及电流估计 `i_est`
  - 估算力矩 `tau_est`
  - 与实验 CSV 参考信号进行可视化对比

## 2. 输入节点（From Workspace）

### `q_cmd` (`q_cmd_ts`)
- 含义：仿真执行的关节命令轨迹（6轴）
- 来源：GUI 在预执行/正式执行时写入

### `q_exp_ref` (`q_ref_ts`)
- 含义：实验关节角参考（6轴）
- 用途：`scope_q` 对比显示

### `dq_exp_ref` (`dq_ref_ts`)
- 含义：实验关节速度参考（6轴）
- 用途：`scope_dq` 对比显示

### `tau_exp_csv_ref` (`tau_csv_ref_ts`)
- 含义：CSV 中已有 `tau_estimated_*` 参考（6轴）
- 用途：`scope_tau` 对比显示

### `tau_exp_from_current_ref` (`tau_from_i_ref_ts`)
- 含义：由实验电流按电机增益重算得到的力矩参考（6轴）
- 用途：`scope_tau` 对比显示

## 3. 核心动力学节点

### `JointSpaceMotionModel`（Discrete State-Space）
- 输入：`q_cmd`
- 输出：`q_actual`
- 含义：虚拟关节响应模型（离散一阶平滑）
- 关键参数：
  - `A=(1-alpha)I`
  - `B=alpha I`
  - `C=I`
  - `D=0`

### `dq_actual`（Discrete Derivative）
- 输入：`q_actual`
- 输出：`dq_actual`
- 含义：关节速度数值求导

### `ddq_est`（Discrete Derivative）
- 输入：`dq_actual`
- 输出：`ddq_est`
- 含义：关节加速度数值求导

### `mux_current_input`
- 输入：`q_actual`, `dq_actual`, `ddq_est`
- 输出：拼接后的 18 维向量
- 含义：给电流估计函数统一喂入输入

### `estimate_current`（Interpreted MATLAB Function）
- 输入：18 维向量（q/dq/ddq）
- 输出：`i_est`（6轴）
- 调用函数：
  - `estimate_current_simple(q,dq,ddq,kq,kdq,kddq,b)`
- 含义：当前工程的电流估计模型

### `motor_gain`（Constant）
- 输出：`virtual_ur10_motor_gains(:)`
- 含义：6轴电机增益常量

### `tau_est`（Product, Element-wise）
- 输入：`i_est` 与 `motor_gain`
- 输出：`tau_est`
- 公式：`tau_est = motor_gain .* i_est`

## 4. 输出节点（Outport）

### `q_actual_out` (Port1)
- 输出：`q_actual`（6轴）

### `dq_actual_out` (Port2)
- 输出：`dq_actual`（6轴）

### `tau_est_out` (Port3)
- 输出：`tau_est`（6轴）

### `i_est_out` (Port4)
- 输出：`i_est`（6轴）

## 5. 对比显示节点（Scope）

### `mux_q_scope` -> `scope_q`
- 通道1：`q_actual`
- 通道2：`q_ref_ts`

### `mux_dq_scope` -> `scope_dq`
- 通道1：`dq_actual`
- 通道2：`dq_ref_ts`

### `mux_tau_scope` -> `scope_tau`
- 通道1：`tau_est`
- 通道2：`tau_csv_ref_ts`
- 通道3：`tau_from_i_ref_ts`

## 6. 与 GUI 的关系

- GUI 文件：`ur10_gui_control_virtual.m`
- 点击“可达性检查”后会生成路径点 `state.rrtPath`
- 点击“预执行/正式执行”时：
  1. 生成 `q_cmd_ts`
  2. 读取 CSV 对齐参考（`q_ref_ts/dq_ref_ts/tau_*_ref_ts`）
  3. 调 `sim()` 运行模型
  4. 读取 Outport 回写 `virtual_ur10_sim_data`
  5. 打开 Scope 查看对比

## 7. 常见问题

### 7.1 From Workspace 变量丢失
- 现已通过 `utils/ensure_virtual_from_workspace_signals.m` 自动补占位变量，避免模型直接报缺变量。

### 7.2 动画期间旋转视角
- 现已支持在预执行/正式执行动画中实时旋转观察，动画期间不再强制锁定相机。
