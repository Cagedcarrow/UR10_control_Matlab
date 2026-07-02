# 抹泥轨迹拟合模块说明

本文档说明 `shovel_with_tcp_rot/mud_smearing_trajectory_fitting` 中抹泥轨迹拟合程序的运动自由度、拟合方法、目录结构和主要函数职责。

## 1. 参与的运动自由度

本模块使用 `claying_shovel_env.urdf` 导入完整场景模型，轨迹求解时参与运动的是 7 个转动自由度：

| 序号 | 关节名 | 含义 |
|---:|---|---|
| 1 | `ur10_shoulder_pan` | UR10 第 1 轴，肩部水平旋转 |
| 2 | `ur10_shoulder_lift` | UR10 第 2 轴，肩部俯仰 |
| 3 | `ur10_elbow` | UR10 第 3 轴，肘部旋转 |
| 4 | `ur10_wrist_1` | UR10 第 4 轴，腕部 1 |
| 5 | `ur10_wrist_2` | UR10 第 5 轴，腕部 2 |
| 6 | `ur10_wrist_3` | UR10 第 6 轴，腕部 3 |
| 7 | `base_tail_to_shovel_base` | 铲子末端电机，控制 `shovel_base_link` 绕自身旋转 |

铲面由以下 4 个坐标系描述：

| 坐标系 | 作用 |
|---|---|
| `shovel_forward_link` | 铲面前方参考点 |
| `shovel_left_link` | 铲面左侧参考点 |
| `shovel_right_link` | 铲面右侧参考点 |
| `shovel_surface_center_link` | 三个参考点构成三角形的形心，是 IK 控制的末端坐标系 |

其中 `shovel_surface_center_link` 是轨迹拟合的控制末端。程序控制 6 个 UR10 关节和 1 个铲子末端电机，使该形心坐标系沿目标墙面轨迹运动。

## 2. 拟合方式

### 2.1 目标墙面与轨迹点

目标接触对象是 `block_basin_back_wall`。程序读取其当前 URDF 位姿，并使用其坐标系生成拟合轨迹：

- 轨迹位于 `block_basin_back_wall` 的中心线上。
- 轨迹方向沿 `block_basin_back_wall` 的 local Z 轴，从下向上。
- 轨迹强制经过 `block_basin_back_wall` 坐标系原点。
- 起点高度为 `block_basin_block` 上表面再向上 `0.20 m`。
- 终点为 `block_basin_back_wall` 顶部边界，即脱离接触的位置。

当前默认配置在 `config/default_mud_smearing_config.m` 中：

```matlab
cfg.trajectory.startClearanceAboveBlock = 0.20;
cfg.trajectory.endDetachMargin = 0.00;
cfg.trajectory.numWaypoints = 25;
cfg.trajectory.approachSteps = 35;
```

`build_wall_contact_trajectory` 会根据当前 URDF 自动计算起止点，而不是写死世界坐标。因此只要 URDF 中 `block_basin_back_wall` 和 `block_basin_block` 的位置改变，轨迹会跟随模型更新。

### 2.2 铲面姿态约束

当前拟合姿态使用 `shovel_surface_center_link` 的坐标轴作为约束：

| 形心坐标轴 | 目标方向 |
|---|---|
| center X 轴 | 世界坐标系 +Z 方向，即沿轨迹向上 |
| center Y 轴 | 世界坐标系 -X 方向 |
| center Z 轴 | 世界坐标系 -Y 方向 |

默认配置为：

```matlab
cfg.trajectory.desiredCenterXAxis = [0; 0; 1];
cfg.trajectory.desiredCenterYAxis = [-1; 0; 0];
cfg.trajectory.desiredCenterZAxis = [0; -1; 0];
```

这三个方向构成右手坐标系。`make_frame_from_x_y_axes` 根据 X/Y 目标方向构造完整目标姿态矩阵，Z 轴由右手系自动确定。

### 2.3 IK 求解流程

主程序的计算流程如下：

1. `load_mud_smearing_robot` 读取 `claying_shovel_env.urdf` 和 `meshes/`，生成 MATLAB `rigidBodyTree`。
2. `validate_mud_smearing_model` 检查关键 link、7 个可动关节和 mesh 文件是否存在。
3. `initial_mud_smearing_config` 设置当前初始姿态。
4. `build_wall_contact_trajectory` 生成目标墙面中心线轨迹和每个 waypoint 的目标位姿。
5. `solve_mud_smearing_ik` 使用 `inverseKinematics` 顺序求解每个 waypoint。
6. 每个 waypoint 使用上一个 waypoint 的解作为初值，使关节运动连续。
7. `interpolate_joint_path` 从当前姿态插值到拟合起点，然后拼接完整拟合轨迹。
8. `animate_mud_smearing_result` 播放拟合动画。

IK 的主控末端是：

```matlab
cfg.links.center = "shovel_surface_center_link";
```

IK 权重默认位置优先，姿态次之：

```matlab
cfg.ik.weightsList = [
    1, 1, 1, 0.25, 0.25, 0.25
    1, 1, 1, 0.10, 0.10, 0.10
    1, 1, 1, 0.03, 0.03, 0.03
];
```

程序会记录每个 waypoint 的误差：

- `positionError`：形心位置与目标点的距离误差。
- `planeDistance`：形心到目标墙面中心平面的距离。
- `centerXAxisAngleDeg`：形心 X 轴与目标 X 方向的夹角。
- `centerYAxisAngleDeg`：形心 Y 轴与目标 Y 方向的夹角。
- `centerZAxisAngleDeg`：形心 Z 轴与目标 Z 方向的夹角。
- `maxCenterAxisAngleDeg`：三个轴角度误差中的最大值。

当前验收容差在配置中定义：

```matlab
cfg.ik.positionTolerance = 0.02;
cfg.ik.centerAxisToleranceDeg = 25;
```

### 2.4 GUI 与动画播放

运行主入口：

```matlab
addpath(genpath('shovel_with_tcp_rot/mud_smearing_trajectory_fitting'))
main_mud_smearing_trajectory_fitting
```

GUI 有两个按钮：

- `开始计算`：计算轨迹和 IK，成功后显示“轨迹计算成功”。
- `开始拟合`：播放拟合动画。

动画播放时会保留用户点击 `开始拟合` 时的相机视角。也就是说，用户可以先手动旋转到合适视角，再点击播放，播放过程中不会因每帧刷新而重置视角。

如果需要无头运行计算：

```matlab
result = main_mud_smearing_trajectory_fitting( ...
    'ShowUi', false, ...
    'ShowAnimation', false);
```

## 3. 文件夹作用与函数功能

### 根目录

| 文件 | 作用 |
|---|---|
| `main_mud_smearing_trajectory_fitting.m` | 主入口。负责加载配置、导入模型、打开 GUI、计算 IK、播放动画。 |
| `claying_shovel_env.urdf` | 本模块独立使用的场景 URDF，包含 UR10、铲子、`block_basin_block` 和 `block_basin_back_wall`。 |
| `plan.md` | 初始任务计划说明。 |

`main_mud_smearing_trajectory_fitting.m` 内部主要函数：

| 函数 | 作用 |
|---|---|
| `main_mud_smearing_trajectory_fitting` | 外部调用入口，支持 GUI 模式和无头模式。 |
| `openMudSmearingGui` | 创建“开始计算 / 开始拟合”界面。 |
| `computeMudSmearingResult` | 完成轨迹生成、IK 求解和起始段插值。 |
| `printResultSummary` | 在命令行输出 waypoint 数量、最大位置误差和最大姿态误差。 |
| `captureCameraState` | 记录当前 MATLAB axes 的相机视角，用于动画播放时保持视角。 |
| `applyNameValueOptions` | 解析 `ShowUi`、`ShowAnimation`、`NumWaypoints` 等参数。 |

### `config/`

| 文件 | 作用 |
|---|---|
| `default_mud_smearing_config.m` | 所有默认参数集中配置，包括路径、link 名称、墙面尺寸、block 尺寸、轨迹参数、IK 权重、初始关节角和可视化参数。 |

关键配置内容：

- `cfg.links.*`：定义参与控制和显示的关键 link。
- `cfg.wall.size`：`block_basin_back_wall` 几何尺寸。
- `cfg.block.size`：`block_basin_block` 几何尺寸。
- `cfg.trajectory.*`：轨迹起点、终点、waypoint 数量和目标姿态。
- `cfg.ik.*`：IK 权重和误差容差。
- `cfg.initialJointValues`：当前 UR10 和铲子电机初始姿态。

### `kinematics/`

| 文件 | 作用 |
|---|---|
| `load_mud_smearing_robot.m` | 使用 `importrobot` 加载 URDF，并设置 `DataFormat="row"` 和 mesh 路径。 |
| `validate_mud_smearing_model.m` | 检查关键 link、可动关节数量和 mesh 文件完整性。 |
| `initial_mud_smearing_config.m` | 按关节名写入初始关节角，生成起始 configuration。 |
| `solve_mud_smearing_ik.m` | 顺序求解所有 waypoint 的 IK，使用前一帧作为下一帧初值。 |
| `evaluate_mud_smearing_solution.m` | 计算位置误差、平面距离误差和 center 坐标轴姿态误差。 |

### `trajectory/`

| 文件 | 作用 |
|---|---|
| `build_wall_contact_trajectory.m` | 根据当前 URDF 中 wall 和 block 位姿生成墙面中心线拟合轨迹。 |
| `interpolate_joint_path.m` | 在关节空间中从当前姿态插值到拟合起点。 |

`build_wall_contact_trajectory.m` 的关键行为：

- 读取 `block_basin_back_wall` 的位姿。
- 读取 `block_basin_block` 的上表面位置。
- 计算起点：block 上表面 + `0.20 m`。
- 计算终点：wall 顶部。
- 生成沿 wall local Z 的中心线 waypoint。
- 目标姿态由 center X/Y 轴指定，center Z 由右手系确定。

### `visualization/`

| 文件 | 作用 |
|---|---|
| `animate_mud_smearing_result.m` | 播放完整拟合动画，显示 URDF、墙面平面、目标轨迹线、红色铲面三角形和 center 坐标轴。 |
| `plot_mud_smearing_joint_angles.m` | 将 UR10 六轴和铲子末端电机角度画在同一张图中，并保存到 `figures/`。 |

`animate_mud_smearing_result.m` 的主要显示内容：

- 完整机器人和环境模型。
- `block_basin_back_wall` 的蓝色半透明目标平面。
- 拟合目标轨迹线。
- 由 `shovel_forward_link`、`shovel_left_link`、`shovel_right_link` 构成的红色铲面三角形。
- `shovel_surface_center_link` 的红绿蓝三轴。

`plot_mud_smearing_joint_angles.m` 会输出：

- `figures/mud_smearing_joint_angles.png`
- `figures/mud_smearing_joint_angles.fig`

曲线包含：

- UR10 六个关节角度。
- 铲子末端电机 `base_tail_to_shovel_base` 角度。

### `utils/`

| 文件 | 作用 |
|---|---|
| `list_movable_joint_names.m` | 按 MATLAB configuration 向量顺序列出所有非 fixed joint 名称。 |
| `normalize_vector.m` | 向量归一化，并对零向量报错。 |
| `clamp.m` | 将数值限制在指定区间，用于角度计算前的点积截断。 |
| `make_frame_from_x_y_axes.m` | 根据目标 X/Y 轴构造右手旋转矩阵，是当前姿态拟合使用的主要函数。 |
| `make_frame_from_x_z_axes.m` | 根据目标 X/Z 轴构造右手旋转矩阵，保留为备用工具。 |
| `make_frame_from_z_axis.m` | 根据目标 Z 轴和优先 X 方向构造右手旋转矩阵，保留为备用工具。 |

### `meshes/`

保存本模块 URDF 引用的 mesh 文件，使 `mud_smearing_trajectory_fitting` 可以独立加载场景模型，不依赖其他目录的 mesh 路径。

### `figures/`

保存轨迹分析图，例如关节角度曲线：

- `mud_smearing_joint_angles.png`
- `mud_smearing_joint_angles.fig`

### `video/`

保留用户手动录屏或实验视频文件。当前程序已经去掉自动保存视频功能，动画播放只负责 MATLAB 窗口显示。

## 4. 常用命令

### GUI 运行

```matlab
addpath(genpath('shovel_with_tcp_rot/mud_smearing_trajectory_fitting'))
main_mud_smearing_trajectory_fitting
```

### 无头计算

```matlab
addpath(genpath('shovel_with_tcp_rot/mud_smearing_trajectory_fitting'))
result = main_mud_smearing_trajectory_fitting( ...
    'ShowUi', false, ...
    'ShowAnimation', false);
```

### 生成关节角度图

```matlab
addpath(genpath('shovel_with_tcp_rot/mud_smearing_trajectory_fitting'))
result = main_mud_smearing_trajectory_fitting( ...
    'ShowUi', false, ...
    'ShowAnimation', false);
plot_mud_smearing_joint_angles(result);
```
