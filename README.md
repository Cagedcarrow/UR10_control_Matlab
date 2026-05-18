# UR10 Virtual Simulink 与电流模型拟合说明

本仓库当前重点是 `E:\UR10_control\test\virtual` 的纯 MATLAB/Simulink 虚拟链路：
1. 在 GUI 内完成轨迹规划与动画执行。
2. 在 Simulink 内输出 `q/dq/tau/i_est`。
3. 用 `data` 目录实测 CSV 做“估算电流 vs 实测电流”批量评估。

---

## 1. 目录与入口

核心目录：`E:\UR10_control\test\virtual`

关键入口：
- `startup_virtual_ur10.m`：一键启动（建模 + GUI）
- `ur10_gui_control_virtual.m`：虚拟交互控制主界面
- `create_or_load_virtual_simulink_model.m`：程序化搭建 Simulink 模型
- `evaluate_current_estimation_batch.m`：批量电流评估入口

辅助文件：
- `init_virtual_ur10.m`：统一参数配置（采样周期、增益、模型参数）
- `utils/estimate_current_simple.m`：当前电流估算函数
- `utils/ensure_virtual_from_workspace_signals.m`：From Workspace 占位变量自愈

### 1.1 机器人模型来源（已切换）

`virtual` 侧现在不再使用裁剪后的简化链模型，而是改为与 `E:\UR10_control\ur10_gui_control.m` 同源的真实机械臂模型导入方式：
- 数据源：`assembly/assembly.urdf.xacro`
- 导入方式：`importrobot(..., 'DataFormat','row')`
- 网格路径：`assembly/meshes`

也就是说，`ur10_gui_control_virtual.m` 在未传入 robot 时，通过 `build_ur10_virtual_robot.m` 构建的是与真机控制脚本一致的 URDF/Xacro 模型源，只是在 `virtual` 场景下用于仿真与数据评估。

---

## 2. Simulink 节点搭建（`create_or_load_virtual_simulink_model`）

模型名默认：`ur10_virtual_jointspace`

### 2.1 输入层（From Workspace）

使用 5 路时序输入：
- `q_cmd_ts`：仿真关节命令
- `q_ref_ts`：实验关节角参考
- `dq_ref_ts`：实验关节速度参考
- `tau_csv_ref_ts`：CSV 中 `tau_estimated_*` 参考
- `tau_from_i_ref_ts`：由实测电流重算的力矩参考

> 若 base workspace 被清空，`ensure_virtual_from_workspace_signals` 会自动补 5 路占位 timeseries，避免模型直接报错。

### 2.2 运动与动力学链

模型主链如下：

1. `JointSpaceMotionModel`（离散状态空间）
- 用 `q_cmd_ts` 生成 `q_actual`
- 平滑系数：`virtual_ur10_alpha`

2. `dq_actual`、`ddq_est`
- 对 `q_actual` 做离散求导，得到速度和加速度估计

3. `estimate_current`（Interpreted MATLAB Function）
- 调用 `estimate_current_simple(q,dq,ddq,kq,kdq,kddq,b)`
- 输出 `i_est`

4. `tau_est`
- `tau_est = motor_gain .* i_est`

### 2.3 输出层

Outport 共 4 路：
- Port1: `q_actual_out`
- Port2: `dq_actual_out`
- Port3: `tau_est_out`
- Port4: `i_est_out`

### 2.4 Scope 对比

默认三个 Scope：
- `scope_q`：`q_actual` vs `q_ref`
- `scope_dq`：`dq_actual` vs `dq_ref`
- `scope_tau`：`tau_est` vs `tau_csv_ref` vs `tau_from_i_ref`

---

## 3. GUI 工作流（`ur10_gui_control_virtual`）

GUI 流程：
1. 调滑轨设置目标关节角。
2. `检查可达性`：RRT 规划。
3. `点击预执行`：仿真并回到起点。
4. `点击正式执行`：仿真 + 动画播放。
5. 动画结束后自动弹 Scope，并写出 `virtual_ur10_sim_data`。

`virtual_ur10_sim_data` 当前字段：
- `t`, `q`, `dq`, `tau`, `iEst`, `tcpPos`, `ref`

---

## 4. 论文电流模型在本工程中的落地方式

### 4.1 当前实现定位

当前不是完整论文辨识器复刻，而是“可运行的首版近似链路”：

- 估算电流：
  `i_est = b + kq.*q + kdq.*dq + kddq.*ddq`

- 力矩估算：
  `tau_est = K .* i_est`

参数来源统一在 `init_virtual_ur10.m`：
- `cfg.motorGains`
- `cfg.currentModel.bias / kq / kdq / kddq`

### 4.2 与实测数据对比来源

CSV（如 `data/05_10_173354/session_data.csv`）主要字段：
- `Act_q0~Act_q5`
- `Act_qd0~Act_qd5`
- `Act_I0~Act_I5`
- `tau_estimated_0~tau_estimated_5`

GUI 在线模式会对齐这些字段做参考曲线显示。

---

## 5. 批量拟合评估（`evaluate_current_estimation_batch`）

目标：在 `data` 目录批量计算“估算电流 vs 实测电流”误差。

### 5.1 默认行为

- 递归扫描：`data/**/session_data.csv`
- 每文件执行：
  1. 读取 `Act_q*`, `Act_qd*`, `Act_I*`
  2. 驱动虚拟模型仿真
  3. 取 `i_est` 与 `Act_I*` 对比
  4. 统计每轴 `RMSE/MAE/MaxAbs`

### 5.2 输出物

默认输出目录：`test/virtual/outputs/current_eval/`

包含：
- `summary.csv`：每个 session 的统计
- `overall_report.md`：总体统计、最差样本
- `*_current_compare.png`：每个 session 的 6 轴对比图（按 `maxPlots` 控制）

### 5.3 调用示例

```matlab
addpath('E:/UR10_control/test/virtual');
cfg = init_virtual_ur10('E:/UR10_control/test/virtual');

% 全量评估
opts = struct('outputDir','E:/UR10_control/test/virtual/outputs/current_eval', ...
              'maxPlots',20);
r = evaluate_current_estimation_batch('E:/UR10_control/test/data', cfg, opts);

% 单文件评估
opts = struct('singleFile','E:/UR10_control/test/data/05_10_173354/session_data.csv', ...
              'outputDir','E:/UR10_control/test/virtual/outputs/current_eval_single', ...
              'maxPlots',1);
r1 = evaluate_current_estimation_batch('E:/UR10_control/test/data', cfg, opts);
```

---

## 6. 已知现状与下一步

当前链路已打通：
- Simulink 节点可生成 `q/dq/tau/i_est`
- 可批量产出 `i_est vs Act_I` 统计与图表

但首版参数仍为固定经验值，误差可能较大。下一步建议：
1. 以 `summary.csv` 为目标函数，做 `kq/kdq/kddq/b` 参数拟合。
2. 分工况拟合（静止段/动态段），避免单一参数覆盖全部动作。
3. 拟合后复跑批量评估，比较 RMSE 降幅。

---

## 7. 快速启动

```matlab
cd('E:/UR10_control/test/virtual');
startup_virtual_ur10;
```

启动后按 GUI 顺序执行：
- 检查可达性 -> 预执行 -> 正式执行

正式执行动画结束后会弹出 Simulink Scope 并更新 `virtual_ur10_sim_data`。
