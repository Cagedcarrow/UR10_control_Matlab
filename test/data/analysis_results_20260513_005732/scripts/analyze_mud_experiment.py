#!/usr/bin/env python3
"""
UR10 机械臂铲泥实验数据分析脚本
==================================
功能：对 27 组铲泥实验进行自动分析、绘图、响应面建模和多目标优化。
"""

import os, sys, glob, json, math, textwrap, traceback
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import warnings
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd

# Matplotlib - 强制 Agg 后端，清理缓存
import matplotlib
matplotlib.use('Agg')
_cache_dir = matplotlib.get_cachedir()
for _f in glob.glob(os.path.join(_cache_dir, 'fontlist*')):
    try: os.remove(_f)
    except: pass
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
fm._load_fontmanager(try_read_cache=False)

# 中文字体设置
_ZH_FONTS = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Noto Sans CJK SC',
             'Noto Sans CJK', 'Source Han Sans CN', 'SimHei',
             'AR PL UMing CN', 'AR PL SungtiL GB', 'Droid Sans Fallback',
             'DejaVu Sans']
plt.rcParams['font.sans-serif'] = _ZH_FONTS
plt.rcParams['axes.unicode_minus'] = False

# scikit-learn
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score, mean_squared_error, mean_absolute_error

# scipy
from scipy.signal import savgol_filter

# ============================================================
# 一、实验元数据（硬编码，27 组）
# ============================================================

EXPERIMENTS = [
    # (folder, test_no, repeat_no, angle_deg, speed_percent, depth_mm, weight_g_ref)
    ("05_10_173354", 1, 1, 20, 35, 20, 80),
    ("05_10_173540", 1, 2, 20, 70, 20, 60),
    ("05_10_173703", 1, 3, 20, 100, 20, 50),
    ("05_10_174040", 2, 1, 20, 35, 40, 190),
    ("05_10_174602", 2, 2, 20, 70, 40, 130),
    ("05_10_174829", 2, 3, 20, 100, 40, 140),
    ("05_10_175513", 3, 1, 20, 35, 60, 240),
    ("05_10_175643", 3, 2, 20, 70, 60, 170),
    ("05_10_175826", 3, 3, 20, 100, 60, 140),
    ("05_10_180549", 4, 1, 35, 35, 40, 90),
    ("05_10_181037", 4, 2, 35, 70, 40, 140),
    ("05_10_181252", 4, 3, 35, 100, 40, 100),
    ("05_10_181630", 5, 1, 35, 35, 60, 80),
    ("05_10_181739", 5, 2, 35, 70, 60, 90),
    ("05_10_181858", 5, 3, 35, 100, 60, 130),
    ("05_10_182626", 6, 1, 35, 35, 20, 150),
    ("05_10_182736", 6, 2, 35, 70, 20, 60),
    ("05_10_182925", 6, 3, 35, 100, 20, 70),
    ("05_10_183213", 7, 1, 50, 35, 60, 120),
    ("05_10_183339", 7, 2, 50, 70, 60, 100),
    ("05_10_183438", 7, 3, 50, 100, 60, 90),
    ("05_10_183705", 8, 1, 50, 35, 20, 110),
    ("05_10_183817", 8, 2, 50, 70, 20, 120),
    ("05_10_183920", 8, 3, 50, 100, 20, 110),
    ("05_10_184140", 9, 1, 50, 35, 40, 210),
    ("05_10_184245", 9, 2, 50, 70, 40, 190),
    ("05_10_184357", 9, 3, 50, 100, 40, 150),
]

# 因素水平
ANGLE_LEVELS = [20, 35, 50]
SPEED_LEVELS = [35, 70, 100]
DEPTH_LEVELS = [20, 40, 60]

# ============================================================
# 二、输出目录
# ============================================================

DATA_ROOT = Path(__file__).resolve().parent / "data"
OUTPUT_BASE = DATA_ROOT / f"analysis_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
SCRIPT_DIR = OUTPUT_BASE / "scripts"
TABLE_DIR  = OUTPUT_BASE / "tables"
FIGURE_DIR = OUTPUT_BASE / "figures"
REPORT_DIR = OUTPUT_BASE / "report"

# ============================================================
# 三、工具函数
# ============================================================

def log(msg: str):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")

def safe_numeric(df: pd.DataFrame, cols: List[str]) -> pd.DataFrame:
    """将指定列转为数值，非数值变为 NaN。"""
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce')
    return df

def find_csv_file(folder: Path) -> Optional[Path]:
    """在文件夹中查找 CSV 文件。"""
    csvs = list(folder.glob("*.csv"))
    # 排除临时文件
    csvs = [f for f in csvs if not f.name.startswith('~')]
    if not csvs:
        return None
    if len(csvs) == 1:
        return csvs[0]
    # 多个 csv，选择 session_data.csv
    for f in csvs:
        if f.name == "session_data.csv":
            return f
    return csvs[0]

def resolve_force_cols(df: pd.DataFrame) -> Tuple[List[str], str]:
    """解析力列，优先 force_base_x/y/z。"""
    preferred = ["force_base_x", "force_base_y", "force_base_z"]
    fallback  = ["Fx", "Fy", "Fz"]
    if all(c in df.columns for c in preferred):
        return preferred, "base_frame"
    elif all(c in df.columns for c in fallback):
        return fallback, "sensor_frame"
    else:
        avail = [c for c in preferred + fallback if c in df.columns]
        if len(avail) >= 2:
            return avail[:3], "partial"
        return [], "none"

def resolve_tau_cols(df: pd.DataFrame) -> Tuple[List[str], str]:
    """解析扭矩列。"""
    preferred = [f"tau_estimated_{i}" for i in range(6)]
    fallback  = [f"tau_ft_{i}" for i in range(6)]
    if all(c in df.columns for c in preferred):
        return preferred, "estimated"
    elif all(c in df.columns for c in fallback):
        return fallback, "ft"
    else:
        return [], "none"

def compute_F_res(df: pd.DataFrame, force_cols: List[str]) -> np.ndarray:
    """计算合力。"""
    vals = df[force_cols].values.astype(float)
    return np.sqrt(np.nansum(vals**2, axis=1))

def detect_shoveling_phase(df: pd.DataFrame, force_cols: List[str],
                           t_col: str = "t") -> dict:
    """
    有效铲泥阶段识别。
    前 10% 作为基线，mu + 3*sigma 作为阈值。
    """
    result = {
        "stage_detected": False,
        "mud_start_time": np.nan,
        "mud_end_time": np.nan,
        "mud_duration": np.nan,
        "mud_mask": np.ones(len(df), dtype=bool),  # 默认全量
    }
    if len(df) < 20 or not force_cols:
        return result

    f_res = compute_F_res(df, force_cols)

    # 基线
    n_base = max(5, len(df) // 10)
    baseline = f_res[:n_base]
    mu0 = np.nanmean(baseline)
    sigma0 = np.nanstd(baseline)
    threshold = mu0 + 3 * sigma0

    # 超阈值区域
    above = f_res > threshold
    if not np.any(above):
        return result

    # 找起始和结束
    indices = np.where(above)[0]
    # 找第一个连续 N 个点以上的区域
    min_run = max(3, len(df) // 20)
    start_idx = indices[0]
    # 从第一个超过阈值的点开始，但要确保后面有连续点
    for i in range(len(indices) - min_run + 1):
        if indices[i + min_run - 1] - indices[i] == min_run - 1:
            start_idx = indices[i]
            break
    else:
        start_idx = indices[0]

    end_idx = indices[-1]

    # 向前向后扩展一些
    start_idx = max(0, start_idx - 5)
    end_idx = min(len(df) - 1, end_idx + 5)

    mask = np.zeros(len(df), dtype=bool)
    mask[start_idx:end_idx + 1] = True

    t = df[t_col].values
    result["stage_detected"] = True
    result["mud_start_time"] = float(t[start_idx])
    result["mud_end_time"] = float(t[end_idx])
    result["mud_duration"] = float(t[end_idx] - t[start_idx])
    result["mud_mask"] = mask
    return result


# ============================================================
# 四、特征提取
# ============================================================

def extract_features(df_raw: pd.DataFrame, meta: dict,
                     force_cols: List[str], force_frame: str,
                     tau_cols: List[str], tau_source: str) -> dict:
    """从一次实验数据中提取所有特征。"""
    feat = {}
    df = df_raw.copy()

    # 时间
    t = df["epoch_time"].values
    t0 = t[0]
    df["t"] = t - t0
    dt = np.diff(t)
    dt_mean = np.mean(dt) if len(dt) > 0 else np.nan
    dt_std  = np.std(dt) if len(dt) > 0 else np.nan

    feat["n_samples"] = len(df)
    feat["duration"]  = float(t[-1] - t[0]) if len(t) > 1 else np.nan
    feat["dt_mean"]   = float(dt_mean)
    feat["dt_std"]    = float(dt_std)

    # fresh 比例
    if "ur_fresh" in df.columns:
        ur_valid = df["ur_fresh"].dropna()
        feat["ur_valid_ratio"] = float(ur_valid.mean()) if len(ur_valid) > 0 else np.nan
    else:
        feat["ur_valid_ratio"] = np.nan
    if "ft_fresh" in df.columns:
        ft_valid = df["ft_fresh"].dropna()
        feat["ft_valid_ratio"] = float(ft_valid.mean()) if len(ft_valid) > 0 else np.nan
    else:
        feat["ft_valid_ratio"] = np.nan

    # 有效铲泥阶段
    phase = detect_shoveling_phase(df, force_cols, "t")
    feat["stage_detected"]  = phase["stage_detected"]
    feat["mud_start_time"]  = phase["mud_start_time"]
    feat["mud_end_time"]    = phase["mud_end_time"]
    feat["mud_duration"]    = phase["mud_duration"]
    mud_mask = phase["mud_mask"]

    # ---- 铲泥质量 ----
    w = df["weight_g"].values
    # 只取有效铲泥阶段
    w_mud = w[mud_mask]
    w_clean = w_mud[~np.isnan(w_mud)]
    if len(w_clean) > 0:
        feat["mass_mean_g"] = float(np.mean(w_clean))
        feat["mass_max_g"]  = float(np.max(w_clean))
        feat["mass_min_g"]  = float(np.min(w_clean))
        feat["mass_std_g"]  = float(np.std(w_clean))
    else:
        w_all = w[~np.isnan(w)]
        feat["mass_mean_g"] = float(np.mean(w_all))
        feat["mass_max_g"]  = float(np.max(w_all))
        feat["mass_min_g"]  = float(np.min(w_all))
        feat["mass_std_g"]  = float(np.std(w_all))

    # 使用人工称重作为质量
    feat["mass_g"] = meta["weight_g_ref"]

    # ---- 力学特征 (铲泥阶段内) ----
    df_mud = df.iloc[mud_mask].copy()
    if force_cols:
        f_res = compute_F_res(df_mud, force_cols)
        f_res = f_res[~np.isnan(f_res)]
        if len(f_res) > 0:
            feat["F_mean"]   = float(np.mean(f_res))
            feat["F_max"]    = float(np.max(f_res))
            feat["F_min"]    = float(np.min(f_res))
            feat["F_std"]    = float(np.std(f_res))
            feat["F_rms"]    = float(np.sqrt(np.mean(f_res**2)))
            # F_impulse = integral(F dt)
            t_mud = df_mud["t"].values[:len(f_res)]
            feat["F_impulse"] = float(np.trapz(f_res, t_mud))
        else:
            for k in ["F_mean","F_max","F_min","F_std","F_rms","F_impulse"]:
                feat[k] = np.nan

        # 各轴分量
        for ax, col in zip(["x","y","z"], force_cols[:3]):
            vals = df_mud[col].values.astype(float)
            vals = vals[~np.isnan(vals)]
            if len(vals) > 0:
                feat[f"force_base_{ax}_mean"]   = float(np.mean(vals))
                feat[f"force_base_{ax}_max_abs"] = float(np.max(np.abs(vals)))
            else:
                feat[f"force_base_{ax}_mean"]   = np.nan
                feat[f"force_base_{ax}_max_abs"] = np.nan
    else:
        for k in ["F_mean","F_max","F_min","F_std","F_rms","F_impulse",
                  "force_base_x_mean","force_base_y_mean","force_base_z_mean",
                  "force_base_x_max_abs","force_base_y_max_abs","force_base_z_max_abs"]:
            feat[k] = np.nan

    # ---- TCP 轨迹特征 ----
    df_ur = df_mud.copy()
    # TCP 位置
    pos_cols = ["Act_X", "Act_Y", "Act_Z"]
    if all(c in df_ur.columns for c in pos_cols):
        X = df_ur["Act_X"].values.astype(float)
        Y = df_ur["Act_Y"].values.astype(float)
        Z = df_ur["Act_Z"].values.astype(float)
        dX = np.diff(X); dY = np.diff(Y); dZ = np.diff(Z)
        ds = np.sqrt(dX**2 + dY**2 + dZ**2)
        feat["tcp_path_length"] = float(np.nansum(ds))
        feat["Z_min"] = float(np.nanmin(Z))
        feat["Z_max"] = float(np.nanmax(Z))
        feat["actual_depth_estimated"] = float(np.nanmax(Z) - np.nanmin(Z))

    # TCP 速度
    vel_cols = ["Act_dX", "Act_dY", "Act_dZ"]
    if all(c in df_ur.columns for c in vel_cols):
        vx = df_ur["Act_dX"].values.astype(float)
        vy = df_ur["Act_dY"].values.astype(float)
        vz = df_ur["Act_dZ"].values.astype(float)
        speed = np.sqrt(vx**2 + vy**2 + vz**2)
        speed = speed[~np.isnan(speed)]
        if len(speed) > 0:
            feat["tcp_speed_mean"] = float(np.mean(speed))
            feat["tcp_speed_max"]  = float(np.max(speed))
            feat["tcp_speed_std"]  = float(np.std(speed))
        else:
            feat["tcp_speed_mean"] = feat["tcp_speed_max"] = feat["tcp_speed_std"] = np.nan
    else:
        feat["tcp_speed_mean"] = feat["tcp_speed_max"] = feat["tcp_speed_std"] = np.nan

    # ---- 轨迹平滑性：acc + jerk ----
    if vel_cols and all(c in df_ur.columns for c in vel_cols):
        vx = df_ur["Act_dX"].values.astype(float)
        vy = df_ur["Act_dY"].values.astype(float)
        vz = df_ur["Act_dZ"].values.astype(float)
        speed = np.sqrt(vx**2 + vy**2 + vz**2)
        t_arr  = df_ur["t"].values
        # 加速度
        acc = np.gradient(speed, t_arr)
        acc = acc[~np.isnan(acc)]
        if len(acc) > 0:
            feat["tcp_acc_mean"] = float(np.mean(acc))
            feat["tcp_acc_max"]  = float(np.max(np.abs(acc)))
            feat["tcp_acc_rms"]  = float(np.sqrt(np.mean(acc**2)))
        else:
            feat["tcp_acc_mean"] = feat["tcp_acc_max"] = feat["tcp_acc_rms"] = np.nan
        # Jerk
        jerk = np.gradient(acc, t_arr[:len(acc)]) if len(acc) > 1 else np.array([np.nan])
        jerk = jerk[~np.isnan(jerk)]
        if len(jerk) > 0:
            feat["tcp_jerk_mean"] = float(np.mean(jerk))
            feat["tcp_jerk_max"]  = float(np.max(np.abs(jerk)))
            feat["tcp_jerk_rms"]  = float(np.sqrt(np.mean(jerk**2)))
        else:
            feat["tcp_jerk_mean"] = feat["tcp_jerk_max"] = feat["tcp_jerk_rms"] = np.nan
    else:
        for k in ["tcp_acc_mean","tcp_acc_max","tcp_acc_rms",
                  "tcp_jerk_mean","tcp_jerk_max","tcp_jerk_rms"]:
            feat[k] = np.nan

    # ---- 关节跟踪误差 ----
    joint_errors = []
    for i in range(6):
        tgt = f"Tgt_q{i}"
        act = f"Act_q{i}"
        if tgt in df_ur.columns and act in df_ur.columns:
            err = np.abs(df_ur[tgt].values.astype(float) - df_ur[act].values.astype(float))
            err = err[~np.isnan(err)]
            if len(err) > 0:
                joint_errors.extend(err.tolist())
    if joint_errors:
        arr = np.array(joint_errors)
        feat["joint_rmse"]      = float(np.sqrt(np.mean(arr**2)))
        feat["joint_mae"]       = float(np.mean(arr))
        feat["joint_max_error"] = float(np.max(arr))
    else:
        feat["joint_rmse"] = feat["joint_mae"] = feat["joint_max_error"] = np.nan

    # ---- 能耗特征 ----
    if tau_cols:
        # 机械能耗
        t_mud = df_mud["t"].values
        P_total_list = []
        for i in range(6):
            tc = tau_cols[i]
            qd = f"Act_qd{i}"
            if tc in df_mud.columns and qd in df_mud.columns:
                tau = df_mud[tc].values.astype(float)
                qd_v = df_mud[qd].values.astype(float)
                Pi = np.abs(tau * qd_v)
                Pi = np.nan_to_num(Pi, nan=0.0)
                P_total_list.append(Pi)
        if P_total_list:
            P_total = np.sum(P_total_list, axis=0)
            # E_mech = integral(P_total dt)
            n = min(len(P_total), len(t_mud))
            feat["E_mech"]  = float(np.trapz(P_total[:n], t_mud[:n]))
            feat["P_mean"]  = float(np.mean(P_total))
            feat["P_max"]   = float(np.max(P_total))
        else:
            feat["E_mech"] = feat["P_mean"] = feat["P_max"] = np.nan

        # tau 统计
        tau_all = []
        for i in range(6):
            tc = tau_cols[i]
            if tc in df_mud.columns:
                tv = df_mud[tc].values.astype(float)
                tv = tv[~np.isnan(tv)]
                if len(tv) > 0:
                    tau_all.extend(tv.tolist())
        if tau_all:
            arr_tau = np.array(tau_all)
            feat["tau_rms"] = float(np.sqrt(np.mean(arr_tau**2)))
            feat["tau_max"] = float(np.max(np.abs(arr_tau)))
        else:
            feat["tau_rms"] = feat["tau_max"] = np.nan
    else:
        feat["E_mech"] = feat["P_mean"] = feat["P_max"] = np.nan
        feat["tau_rms"] = feat["tau_max"] = np.nan

    # ---- 电能辅助特征 ----
    if "Robot_Current" in df_mud.columns and "Robot_Voltage" in df_mud.columns:
        rc = df_mud["Robot_Current"].values.astype(float)
        rv = df_mud["Robot_Voltage"].values.astype(float)
        rc = rc[~np.isnan(rc)]
        rv = rv[~np.isnan(rv)]
        if len(rc) > 0 and len(rv) > 0:
            P_elec = np.abs(rc * rv)
            feat["Robot_Current_mean"] = float(np.mean(rc))
            feat["Robot_Current_max"]  = float(np.max(rc))
            feat["Robot_Voltage_mean"] = float(np.mean(rv))
            t_mud_short = t_mud[:min(len(P_elec), len(t_mud))]
            feat["E_elec"] = float(np.trapz(P_elec, t_mud_short))
        else:
            for k in ["Robot_Current_mean","Robot_Current_max","Robot_Voltage_mean","E_elec"]:
                feat[k] = np.nan
    else:
        for k in ["Robot_Current_mean","Robot_Current_max","Robot_Voltage_mean","E_elec"]:
            feat[k] = np.nan
        feat["E_elec"] = np.nan

    # ---- 效率指标 ----
    mass_g = meta["weight_g_ref"]
    def safe_div(a, b):
        if b is None or (isinstance(b, float) and (np.isnan(b) or b == 0)):
            return np.nan
        if isinstance(a, float) and np.isnan(a):
            return np.nan
        return a / b

    feat["mass_per_energy"]       = safe_div(mass_g, feat.get("E_mech"))
    feat["mass_per_force_impulse"] = safe_div(mass_g, feat.get("F_impulse"))
    feat["mass_per_time"]         = safe_div(mass_g, feat.get("mud_duration"))

    # ---- 元数据 ----
    for k in ["folder","test_no","repeat_no","angle_deg","speed_percent","depth_mm","weight_g_ref"]:
        feat[k] = meta[k]

    return feat


# ============================================================
# 五、单实验处理
# ============================================================

def process_single_session(meta: dict) -> Optional[dict]:
    """处理单个实验文件夹。"""
    folder_path = DATA_ROOT / meta["folder"]
    if not folder_path.exists():
        log(f"  WARNING: 文件夹不存在 {folder_path}")
        return None

    csv_path = find_csv_file(folder_path)
    if csv_path is None:
        log(f"  WARNING: 在 {folder_path} 中未找到 CSV 文件")
        return None

    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        log(f"  ERROR: 读取 {csv_path} 失败: {e}")
        return None

    if df.empty:
        log(f"  WARNING: {csv_path} 为空")
        return None

    # strip 列名
    df.columns = df.columns.str.strip()

    # 转为数值
    num_cols = [c for c in df.columns if c not in ("Label",)]
    df = safe_numeric(df, num_cols)

    # 按 epoch_time 排序
    if "epoch_time" in df.columns:
        df = df.sort_values("epoch_time").reset_index(drop=True)
    else:
        log(f"  WARNING: {csv_path} 缺少 epoch_time 列")

    # 解析力列
    force_cols, force_frame = resolve_force_cols(df)
    if not force_cols:
        log(f"  WARNING: {csv_path} 缺少力列")

    # 解析扭矩列
    tau_cols, tau_source = resolve_tau_cols(df)

    # 提取特征
    feat = extract_features(df, meta, force_cols, force_frame, tau_cols, tau_source)
    feat["data_file"] = csv_path.name
    feat["force_source"] = force_frame
    feat["tau_source"]   = tau_source

    log(f"  OK {meta['folder']}: {feat['n_samples']} samples, "
        f"mass={feat['mass_g']}g, Fmax={feat.get('F_max', np.nan):.1f}N, "
        f"E_mech={feat.get('E_mech', np.nan):.1f}J, "
        f"stage={feat['stage_detected']}")
    return feat


# ============================================================
# 六、因素分析
# ============================================================

def factor_analysis(df: pd.DataFrame, factor_col: str, response_cols: List[str]) -> pd.DataFrame:
    """对单个因素进行主效应分析。"""
    rows = []
    for level in sorted(df[factor_col].unique()):
        sub = df[df[factor_col] == level]
        row = {"factor": factor_col, "level": level}
        for rc in response_cols:
            vals = sub[rc].dropna()
            if len(vals) > 0:
                row[f"{rc}_mean"] = float(vals.mean())
                row[f"{rc}_std"]  = float(vals.std())
            else:
                row[f"{rc}_mean"] = np.nan
                row[f"{rc}_std"]  = np.nan
        rows.append(row)
    result = pd.DataFrame(rows)
    # 极差
    for rc in response_cols:
        means = result[f"{rc}_mean"].dropna()
        if len(means) > 0:
            result[f"{rc}_range"] = float(means.max() - means.min())
        else:
            result[f"{rc}_range"] = np.nan
    return result


# ============================================================
# 七、二次响应面模型
# ============================================================

def build_response_surface(df: pd.DataFrame, response: str,
                            factors: List[str]) -> dict:
    """构建二次响应面模型。"""
    # 移除 NaN
    valid = df[factors + [response]].dropna()
    if len(valid) < len(factors) + 2:
        return {
            "response": response,
            "r2": np.nan, "rmse": np.nan, "mae": np.nan,
            "coefs": {}, "intercept": np.nan,
            "n_samples": len(valid),
            "success": False,
        }

    X = valid[factors].values
    y = valid[response].values.astype(float)

    poly = PolynomialFeatures(degree=2, include_bias=False)
    X_poly = poly.fit_transform(X)
    feature_names = poly.get_feature_names_out(factors)

    reg = LinearRegression()
    reg.fit(X_poly, y)
    y_pred = reg.predict(X_poly)

    # 计算 R2 和 RMSE
    r2 = r2_score(y, y_pred)
    n, p = len(y), X_poly.shape[1]
    adj_r2 = 1 - (1 - r2) * (n - 1) / (n - p - 1) if n > p + 1 else np.nan
    rmse = np.sqrt(mean_squared_error(y, y_pred))
    mae  = mean_absolute_error(y, y_pred)

    coefs = dict(zip(feature_names, reg.coef_))

    return {
        "response": response,
        "r2": r2, "adj_r2": adj_r2, "rmse": rmse, "mae": mae,
        "coefs": coefs, "intercept": reg.intercept_,
        "n_samples": len(valid),
        "success": True,
        "poly": poly, "reg": reg,
        "y_true": y, "y_pred": y_pred,
    }


# ============================================================
# 八、多目标优化
# ============================================================

def multi_objective_optimize(df_grid: pd.DataFrame,
                              objectives: Dict[str, float]) -> pd.DataFrame:
    """
    对网格点进行加权多目标综合评分。
    objectives: {col_name: weight}，权重正表示最大化，负表示最小化。
    """
    scores = np.zeros(len(df_grid))
    weight_sum = 0.0

    for col, w in objectives.items():
        if col not in df_grid.columns:
            continue
        vals = df_grid[col].values.astype(float)
        valid_mask = ~np.isnan(vals)
        if not np.any(valid_mask):
            continue
        v = vals.copy()
        v_min, v_max = np.nanmin(v), np.nanmax(v)
        if v_max - v_min < 1e-9:
            continue
        if w > 0:  # 正向指标，最大化
            norm = (v - v_min) / (v_max - v_min)
        else:      # 负向指标，最小化
            norm = (v_max - v) / (v_max - v_min)
        norm[~valid_mask] = np.nan
        scores += abs(w) * norm
        weight_sum += abs(w)

    if weight_sum > 0:
        scores = scores / weight_sum
    df_grid = df_grid.copy()
    df_grid["score"] = scores
    return df_grid.sort_values("score", ascending=False).reset_index(drop=True)


# ============================================================
# 九、绘图函数
# ============================================================

def plot_weight_bar(df_feat: pd.DataFrame, path: Path):
    """01: 铲泥质量柱状图"""
    fig, ax = plt.subplots(figsize=(16, 6))
    names = [f"{r['folder']}\n{r['angle_deg']}°/{r['speed_percent']}%/{r['depth_mm']}mm"
             for _, r in df_feat.iterrows()]
    colors = plt.cm.viridis(np.linspace(0.2, 0.8, 9))
    bar_colors = [colors[r['test_no'] - 1] for _, r in df_feat.iterrows()]
    bars = ax.bar(range(len(df_feat)), df_feat["mass_g"].values, color=bar_colors, edgecolor='k', linewidth=0.5)
    ax.set_xticks(range(len(df_feat)))
    ax.set_xticklabels(names, fontsize=6, rotation=45, ha='right')
    ax.set_ylabel("Weight (g)")
    ax.set_title("Scooped Mud Weight per Experiment")
    # 标注最大值
    idx_max = df_feat["mass_g"].idxmax()
    ax.text(idx_max, df_feat.loc[idx_max, "mass_g"] + 5,
            f"Max: {df_feat.loc[idx_max, 'mass_g']:.0f}g",
            ha='center', fontsize=10, fontweight='bold', color='red')
    fig.tight_layout()
    fig.savefig(str(path), dpi=300)
    plt.close(fig)

def plot_weight_by_factors(df_feat: pd.DataFrame, path: Path):
    """02: 三因素铲泥质量分布"""
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    for idx, (factor, label, levels) in enumerate([
        ("angle_deg", "Angle (deg)", ANGLE_LEVELS),
        ("speed_percent", "Speed (%)", SPEED_LEVELS),
        ("depth_mm", "Depth (mm)", DEPTH_LEVELS),
    ]):
        ax = axes[idx]
        data = []
        labels = []
        for lv in levels:
            sub = df_feat[df_feat[factor] == lv]["mass_g"].dropna()
            if len(sub) > 0:
                data.append(sub.values)
                labels.append(str(lv))
        bp = ax.boxplot(data, labels=labels, patch_artist=True)
        for patch in bp['boxes']:
            patch.set_facecolor(plt.cm.viridis(0.5))
        ax.set_xlabel(label)
        ax.set_ylabel("Weight (g)")
        ax.set_title(f"Weight vs {label}")
        # Overlay scatter
        for i, d in enumerate(data):
            ax.scatter(np.ones(len(d)) * (i + 1) + np.random.uniform(-0.1, 0.1, len(d)),
                       d, alpha=0.6, color='red', s=30, zorder=3)

    fig.tight_layout()
    fig.savefig(str(path), dpi=300)
    plt.close(fig)

def plot_force_curves(df_feat: pd.DataFrame, all_data: dict, path: Path):
    """03: 典型力曲线（最大质量、最高评分、最低质量）"""
    # 找出三个典型实验
    idx_max_w = df_feat["mass_g"].idxmax()
    idx_min_w = df_feat["mass_g"].idxmin()

    # 综合评分最高的（需要从特征中计算个简化评分）
    df_temp = df_feat.copy()
    for c in ["mass_g", "E_mech", "F_max"]:
        if c in df_temp.columns:
            v = df_temp[c].values.astype(float)
            vm = np.nanmin(v); vM = np.nanmax(v)
            if vM - vm > 1e-9:
                if c == "mass_g":
                    df_temp["_score"] = (v - vm) / (vM - vm)
                else:
                    df_temp["_score"] = df_temp.get("_score", 0) - (v - vm) / (vM - vm)
    df_temp["_score"] = df_temp.get("_score", 0)
    idx_best = df_temp["_score"].idxmax()

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    titles = [
        f"Best Quality: {df_feat.loc[idx_max_w, 'folder']} ({df_feat.loc[idx_max_w, 'mass_g']:.0f}g)",
        f"Best Composite: {df_feat.loc[idx_best, 'folder']}",
        f"Worst Quality: {df_feat.loc[idx_min_w, 'folder']} ({df_feat.loc[idx_min_w, 'mass_g']:.0f}g)",
    ]
    for idx_plot, (ax, idx_exp) in enumerate(zip(axes, [idx_max_w, idx_best, idx_min_w])):
        folder = df_feat.loc[idx_exp, "folder"]
        if folder in all_data:
            d = all_data[folder]
            t = d["t"]
            f_res = d["f_res"]
            ax.plot(t, f_res, 'b-', linewidth=0.8, label="F_res (N)")
            if d["mud_start"] is not None:
                ax.axvline(d["mud_start"], color='g', linestyle='--', alpha=0.7, label="mud_start")
                ax.axvline(d["mud_end"], color='r', linestyle='--', alpha=0.7, label="mud_end")
                ax.axvspan(d["mud_start"], d["mud_end"], alpha=0.1, color='green')
        ax.set_title(titles[idx_plot])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Force (N)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(str(path), dpi=300)
    plt.close(fig)

def plot_force_features_bar(df_feat: pd.DataFrame, path: Path):
    """04: 受力特征柱状图"""
    fig, ax = plt.subplots(figsize=(16, 6))
    x = np.arange(len(df_feat))
    w = 0.25
    ax.bar(x - w, df_feat["F_mean"].values, w, label="F_mean", alpha=0.8)
    ax.bar(x,     df_feat["F_max"].values,  w, label="F_max", alpha=0.8)
    ax.bar(x + w, df_feat["F_impulse"].values / 10, w, label="F_impulse/10", alpha=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{r['folder']}\n{r['angle_deg']}°/ {r['speed_percent']}%/ {r['depth_mm']}mm"
                        for _, r in df_feat.iterrows()], fontsize=5, rotation=45, ha='right')
    ax.set_ylabel("Force (N)")
    ax.set_title("Force Features per Experiment")
    ax.legend()
    fig.tight_layout()
    fig.savefig(str(path), dpi=300)
    plt.close(fig)

def plot_energy_features_bar(df_feat: pd.DataFrame, path: Path):
    """05: 能耗特征"""
    fig, ax1 = plt.subplots(figsize=(16, 5))
    x = np.arange(len(df_feat))
    ax1.bar(x - 0.15, df_feat["E_mech"].values / 100, w=0.3, label="E_mech/100", alpha=0.8, color='orange')
    ax1.set_ylabel("E_mech/100 (J)", color='orange')
    ax1.tick_params(axis='y', labelcolor='orange')

    ax2 = ax1.twinx()
    ax2.bar(x + 0.15, df_feat["mass_per_energy"].values, w=0.3, label="mass/energy", alpha=0.8, color='green')
    ax2.set_ylabel("mass_per_energy (g/J)", color='green')
    ax2.tick_params(axis='y', labelcolor='green')

    ax1.set_xticks(x)
    ax1.set_xticklabels([r['folder'] for _, r in df_feat.iterrows()], fontsize=6, rotation=45, ha='right')
    ax1.set_title("Energy Features per Experiment")
    fig.tight_layout()
    fig.savefig(str(path), dpi=300)
    plt.close(fig)

def plot_smoothness_bar(df_feat: pd.DataFrame, path: Path):
    """06: 轨迹平滑性"""
    fig, ax = plt.subplots(figsize=(16, 5))
    x = np.arange(len(df_feat))
    w = 0.25
    ax.bar(x - w, df_feat["tcp_acc_rms"].values,  w, label="tcp_acc_rms", alpha=0.8)
    ax.bar(x,     df_feat["tcp_jerk_rms"].values, w, label="tcp_jerk_rms", alpha=0.8)
    ax.bar(x + w, df_feat["joint_rmse"].values * 100, w, label="joint_rmse*100", alpha=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([r['folder'] for _, r in df_feat.iterrows()], fontsize=6, rotation=45, ha='right')
    ax.set_ylabel("Value")
    ax.set_title("Trajectory Smoothness & Tracking Error")
    ax.legend()
    fig.tight_layout()
    fig.savefig(str(path), dpi=300)
    plt.close(fig)

def plot_response_surface_3d(model: dict, x1: str, x2: str, fixed: dict,
                              xlabel: str, ylabel: str, zlabel: str,
                              title: str, path: Path,
                              actual_points: Optional[pd.DataFrame] = None):
    """绘制 3D 响应面图。"""
    if not model.get("success"):
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.text(0.5, 0.5, "Model not available", ha='center', va='center', fontsize=14)
        fig.savefig(str(path), dpi=300)
        plt.close(fig)
        return

    poly = model["poly"]
    reg  = model["reg"]

    # 确定范围
    ranges = {"angle_deg": (20, 50), "speed_percent": (35, 100), "depth_mm": (20, 60)}
    x1r = ranges[x1]; x2r = ranges[x2]

    n = 30
    x1v = np.linspace(x1r[0], x1r[1], n)
    x2v = np.linspace(x2r[0], x2r[1], n)
    X1g, X2g = np.meshgrid(x1v, x2v)

    # 构建输入
    points = []
    for i in range(n):
        for j in range(n):
            p = {}
            p[x1] = X1g[i, j]
            p[x2] = X2g[i, j]
            for k, v in fixed.items():
                p[k] = v
            points.append(p)
    X_pred = pd.DataFrame(points)
    X_poly = poly.transform(X_pred.values)
    Z = reg.predict(X_poly).reshape(n, n)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(X1g, X2g, Z, cmap='viridis', alpha=0.8, linewidth=0, antialiased=True)
    fig.colorbar(surf, ax=ax, shrink=0.5, label=zlabel)

    # 实际数据点
    if actual_points is not None and not actual_points.empty:
        ax.scatter(actual_points[x1], actual_points[x2], actual_points[model["response"]],
                   c='red', s=40, marker='o', label='Actual')

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel(zlabel)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(str(path), dpi=300)
    plt.close(fig)

def plot_pareto(df_feat: pd.DataFrame, df_candidates: pd.DataFrame, path: Path):
    """10: Pareto 多目标散点图"""
    fig, ax = plt.subplots(figsize=(10, 8))

    # 实际实验点
    sc = ax.scatter(df_feat["E_mech"].values, df_feat["mass_g"].values,
                    c=df_feat["mass_per_energy"].values, cmap='viridis',
                    s=80, alpha=0.8, edgecolors='k', linewidth=0.5)
    cbar = plt.colorbar(sc, ax=ax, label="mass_per_energy (g/J)")

    # 标注最佳
    idx_best_mass = df_feat["mass_g"].idxmax()
    ax.annotate(f"Best mass\n{df_feat.loc[idx_best_mass, 'folder']}",
                (df_feat.loc[idx_best_mass, "E_mech"], df_feat.loc[idx_best_mass, "mass_g"]),
                fontsize=8, xytext=(5, 5), textcoords='offset points')

    ax.set_xlabel("E_mech (J)")
    ax.set_ylabel("Weight (g)")
    ax.set_title("Pareto: Energy vs Mass (color: mass_per_energy)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(str(path), dpi=300)
    plt.close(fig)

def plot_main_effect(df_feat: pd.DataFrame, response: str, path: Path):
    """11-13: 主效应图"""
    factors = ["angle_deg", "speed_percent", "depth_mm"]
    labels  = ["Angle (deg)", "Speed (%)", "Depth (mm)"]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for idx, (fac, lab) in enumerate(zip(factors, labels)):
        ax = axes[idx]
        levels = sorted(df_feat[fac].unique())
        means = [df_feat[df_feat[fac] == l][response].mean() for l in levels]
        stds  = [df_feat[df_feat[fac] == l][response].std() for l in levels]
        ax.errorbar(levels, means, yerr=stds, marker='o', capsize=5, linewidth=2)
        ax.set_xlabel(lab)
        ax.set_ylabel(response)
        ax.set_title(f"Main Effect: {fac} on {response}")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(str(path), dpi=300)
    plt.close(fig)

def plot_correlation_heatmap(df_feat: pd.DataFrame, path: Path):
    """14: 相关性热力图"""
    cols = ["mass_g", "F_mean", "F_max", "F_impulse", "E_mech",
            "mass_per_energy", "tcp_jerk_rms", "joint_rmse",
            "P_mean", "tau_rms"]
    avail = [c for c in cols if c in df_feat.columns]
    if len(avail) < 2:
        fig, ax = plt.subplots()
        ax.text(0.5, 0.5, "Not enough data", ha='center', va='center')
        fig.savefig(str(path), dpi=300)
        plt.close(fig)
        return

    corr = df_feat[avail].corr()
    fig, ax = plt.subplots(figsize=(10, 8))
    im = ax.imshow(corr.values, cmap='RdBu_r', vmin=-1, vmax=1, aspect='auto')
    cbar = fig.colorbar(im, ax=ax, shrink=0.8)
    ax.set_xticks(range(len(avail)))
    ax.set_yticks(range(len(avail)))
    ax.set_xticklabels(avail, fontsize=8, rotation=45, ha='right')
    ax.set_yticklabels(avail, fontsize=8)
    for i in range(len(avail)):
        for j in range(len(avail)):
            ax.text(j, i, f"{corr.values[i, j]:.2f}", ha='center', va='center', fontsize=7)
    ax.set_title("Correlation Heatmap of Key Metrics")
    fig.tight_layout()
    fig.savefig(str(path), dpi=300)
    plt.close(fig)


# ============================================================
# 十、生成报告
# ============================================================

def generate_report(df_feat: pd.DataFrame, models: dict,
                    df_candidates: pd.DataFrame,
                    errors: List[dict], path: Path):
    """生成 Markdown 分析报告。"""
    lines = []
    lines.append("# 机械臂铲泥实验数据分析报告\n")
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines.append(f"*生成时间: {now_str}*\n")

    # 1. 实验数据说明
    lines.append("## 1. 实验数据说明\n")
    n_ok = len(df_feat)
    lines.append(f"本次分析共包含 **{n_ok}** 组有效实验数据。实验因素包括：\n")
    lines.append("- **入泥角度 (angle_deg)**: 20°, 35°, 50°")
    lines.append("- **铲泥速度 (speed_percent)**: 35%, 70%, 100%")
    lines.append("- **入泥深度 (depth_mm)**: 20 mm, 40 mm, 60 mm")
    lines.append("- **响应值 (weight_g)**: 人工称重的铲泥质量")
    lines.append("")
    lines.append(f"数据来源目录: `{DATA_ROOT}`")
    lines.append(f"结果输出目录: `{OUTPUT_BASE}`\n")

    # 2. 数据列说明
    lines.append("## 2. 数据列说明与分析指标\n")
    lines.append("| 类别 | 列名 | 用途 |")
    lines.append("| --- | --- | --- |")
    lines.append("| 力 | force_base_x/y/z, Fx/Fy/Fz | 铲泥阻力计算 |")
    lines.append("| 扭矩 | tau_estimated_0~5, tau_ft_0~5 | 关节能耗计算 |")
    lines.append("| 关节速度 | Act_qd0~5 | 机械功率计算 |")
    lines.append("| TCP 位置 | Act_X/Y/Z | 轨迹分析 |")
    lines.append("| TCP 速度 | Act_dX/dY/dZ | 运动平滑性 |")
    lines.append("| 目标/实际关节 | Tgt_q0~5, Act_q0~5 | 跟踪误差 |")
    lines.append("| 时间 | epoch_time, Time | 时域分析 |")
    lines.append("| 质量 | weight_g | 铲泥质量主响应 |\n")

    # 3. 数据有效性
    lines.append("## 3. 数据有效性检查\n")
    lines.append(f"### 采样频率\n")
    dt_mean_mean = df_feat["dt_mean"].mean()
    dt_mean_std  = df_feat["dt_mean"].std()
    lines.append(f"- 平均采样周期: {dt_mean_mean:.4f} s ({1/dt_mean_mean:.0f} Hz)")
    lines.append(f"- 采样周期标准差: {dt_mean_std:.4f} s\n")

    ur_ratios = df_feat["ur_valid_ratio"].dropna()
    ft_ratios = df_feat["ft_valid_ratio"].dropna()
    lines.append("### 数据新鲜度\n")
    if len(ur_ratios) > 0:
        lines.append(f"- ur_fresh 有效比例: 均值 {ur_ratios.mean():.1%}, 范围 [{ur_ratios.min():.1%}, {ur_ratios.max():.1%}]")
    if len(ft_ratios) > 0:
        lines.append(f"- ft_fresh 有效比例: 均值 {ft_ratios.mean():.1%}, 范围 [{ft_ratios.min():.1%}, {ft_ratios.max():.1%}]\n")

    # 铲泥阶段检测
    n_stage_detected = df_feat["stage_detected"].sum()
    lines.append(f"### 有效铲泥阶段识别\n")
    lines.append(f"- 成功检测有效阶段: {int(n_stage_detected)}/{n_ok} 组")
    if n_stage_detected < n_ok:
        failed_stage = df_feat[~df_feat["stage_detected"]]["folder"].tolist()
        lines.append(f"- 未检测到有效阶段: {', '.join(failed_stage)}（已使用全程数据）\n")
    else:
        lines.append("")

    # 平均铲泥时长
    mud_durs = df_feat["mud_duration"].dropna()
    if len(mud_durs) > 0:
        lines.append(f"- 平均铲泥阶段时长: {mud_durs.mean():.1f} s\n")

    # 异常
    if errors:
        lines.append("### 异常记录\n")
        for e in errors:
            lines.append(f"- {e.get('folder', '?')}: {e.get('error', '?')}")
        lines.append("")

    # 4. 铲泥质量分析
    lines.append("## 4. 铲泥质量分析\n")
    idx_best = df_feat["mass_g"].idxmax()
    idx_worst = df_feat["mass_g"].idxmin()
    best = df_feat.loc[idx_best]
    worst = df_feat.loc[idx_worst]
    lines.append(f"### 最大铲泥质量\n")
    lines.append(f"- **文件夹**: {best['folder']}")
    lines.append(f"- **参数**: 角度={best['angle_deg']}°, 速度={best['speed_percent']}%, 深度={best['depth_mm']}mm")
    lines.append(f"- **铲泥质量**: {best['mass_g']:.0f} g\n")
    lines.append(f"### 最小铲泥质量\n")
    lines.append(f"- **文件夹**: {worst['folder']}")
    lines.append(f"- **参数**: 角度={worst['angle_deg']}°, 速度={worst['speed_percent']}%, 深度={worst['depth_mm']}mm")
    lines.append(f"- **铲泥质量**: {worst['mass_g']:.0f} g\n")

    # 因素趋势
    lines.append("### 因素影响趋势\n")
    for fac, lab, levels in [("angle_deg", "角度", ANGLE_LEVELS),
                              ("speed_percent", "速度", SPEED_LEVELS),
                              ("depth_mm", "深度", DEPTH_LEVELS)]:
        means = [df_feat[df_feat[fac] == l]["mass_g"].mean() for l in levels]
        stds  = [df_feat[df_feat[fac] == l]["mass_g"].std() for l in levels]
        trend = " → ".join([f"{m:.0f}±{s:.0f}g" for m, s in zip(means, stds)])
        lines.append(f"- **{lab}**: {trend}")
    lines.append("")

    # 回答关键问题
    lines.append("### 关键结论\n")
    lines.append(f"> **什么角度、什么速度、什么深度下铲泥质量最大？**")
    lines.append(f"> 角度 {best['angle_deg']}°、速度 {best['speed_percent']}%、深度 {best['depth_mm']}mm 时铲泥质量最大，")
    lines.append(f"> 最大数值为 **{best['mass_g']:.0f} g**。\n")

    # 5. 阻力特征
    lines.append("## 5. 阻力特征分析\n")
    for metric, label in [("F_mean", "F_mean"), ("F_max", "F_max"), ("F_impulse", "F_impulse")]:
        if metric in df_feat.columns:
            vals = df_feat[metric].dropna()
            if len(vals) > 0:
                idx_m = vals.idxmax()
                lines.append(f"- **{label}** 最大: {df_feat.loc[idx_m, 'folder']} ({vals.max():.1f} N), "
                             f"对应质量 {df_feat.loc[idx_m, 'mass_g']:.0f}g")
    lines.append("")

    # 阻力-质量相关性
    if "F_mean" in df_feat.columns and "mass_g" in df_feat.columns:
        corr_fm = df_feat["F_mean"].corr(df_feat["mass_g"])
        corr_fx = df_feat["F_max"].corr(df_feat["mass_g"])
        lines.append(f"- 铲泥质量与 F_mean 相关系数: {corr_fm:.3f}")
        lines.append(f"- 铲泥质量与 F_max 相关系数: {corr_fx:.3f}")
        lines.append(f"- {'铲泥量大的实验通常伴随较大阻力。' if corr_fm > 0.3 else '铲泥量与阻力之间相关性不强。'}\n")

    # 6. 能耗
    lines.append("## 6. 能量消耗分析\n")
    if "E_mech" in df_feat.columns:
        vals = df_feat["E_mech"].dropna()
        if len(vals) > 0:
            idx_me = vals.idxmax()
            idx_me_min = vals.idxmin()
            lines.append(f"- **E_mech 最大**: {df_feat.loc[idx_me, 'folder']} ({vals.max():.0f} J), "
                         f"对应质量 {df_feat.loc[idx_me, 'mass_g']:.0f}g")
            lines.append(f"- **E_mech 最小**: {df_feat.loc[idx_me_min, 'folder']} ({vals.min():.0f} J), "
                         f"对应质量 {df_feat.loc[idx_me_min, 'mass_g']:.0f}g")
    if "mass_per_energy" in df_feat.columns:
        vals = df_feat["mass_per_energy"].dropna()
        if len(vals) > 0:
            idx_meff = vals.idxmax()
            lines.append(f"- **mass_per_energy 最大**: {df_feat.loc[idx_meff, 'folder']} "
                         f"({vals.max():.3f} g/J), 铲泥质量 {df_feat.loc[idx_meff, 'mass_g']:.0f}g")
    lines.append("")

    # 7. 轨迹平滑性
    lines.append("## 7. 机械臂轨迹平滑性分析\n")
    for metric, label in [("tcp_acc_rms", "tcp_acc_rms"), ("tcp_jerk_rms", "tcp_jerk_rms"),
                           ("joint_rmse", "joint_rmse")]:
        if metric in df_feat.columns:
            vals = df_feat[metric].dropna()
            if len(vals) > 0:
                lines.append(f"- **{label}**: 均值 {vals.mean():.4f}, 范围 [{vals.min():.4f}, {vals.max():.4f}]")
    lines.append("")

    # 8. 响应面
    lines.append("## 8. 二次响应面分析\n")
    lines.append("### 模型形式\n")
    lines.append("$$ y = \\beta_0 + \\beta_1 x_1 + \\beta_2 x_2 + \\beta_3 x_3 + \\beta_{11} x_1^2 + \\beta_{22} x_2^2 + \\beta_{33} x_3^2 + \\beta_{12} x_1 x_2 + \\beta_{13} x_1 x_3 + \\beta_{23} x_2 x_3 $$\n")
    lines.append("其中 $x_1$ = angle_deg, $x_2$ = speed_percent, $x_3$ = depth_mm\n")
    lines.append("### 模型质量\n")
    lines.append("| 响应变量 | R² | Adj-R² | RMSE | MAE | 样本数 |")
    lines.append("| --- | --- | --- | --- | --- | --- |")
    for resp in ["mass_g", "E_mech", "F_max", "F_impulse", "tcp_jerk_rms", "joint_rmse", "mass_per_energy"]:
        if resp in models:
            m = models[resp]
            r2s = f"{m['r2']:.3f}" if not np.isnan(m['r2']) else "N/A"
            a2s = f"{m['adj_r2']:.3f}" if not np.isnan(m.get('adj_r2', np.nan)) else "N/A"
            rms = f"{m['rmse']:.2f}" if not np.isnan(m['rmse']) else "N/A"
            maes = f"{m['mae']:.2f}" if not np.isnan(m['mae']) else "N/A"
            ns = str(m.get('n_samples', ''))
            lines.append(f"| {resp} | {r2s} | {a2s} | {rms} | {maes} | {ns} |")
    lines.append("")
    lines.append(f"> **注意**：由于样本数仅 {n_ok} 组，二次响应面模型主要用于趋势分析与参数寻优参考，")
    lines.append("> 不能过度解释为严格泛化模型。\n")

    # 9. 多目标优化
    lines.append("## 9. 多目标优化结果\n")
    lines.append("### 优化目标与权重\n")
    lines.append("| 目标 | 方向 | 权重 |")
    lines.append("| --- | --- | --- |")
    lines.append("| weight_g | 最大化 | 0.30 |")
    lines.append("| mass_per_energy | 最大化 | 0.20 |")
    lines.append("| mass_per_force_impulse | 最大化 | 0.15 |")
    lines.append("| E_mech | 最小化 | 0.10 |")
    lines.append("| F_max | 最小化 | 0.10 |")
    lines.append("| F_impulse | 最小化 | 0.05 |")
    lines.append("| tcp_jerk_rms | 最小化 | 0.05 |")
    lines.append("| joint_rmse | 最小化 | 0.05 |\n")

    # Top 10 候选
    if not df_candidates.empty:
        lines.append("### Top 10 多目标候选方案\n")
        lines.append("| 排名 | 角度(°) | 速度(%) | 深度(mm) | 综合评分 | 预测质量(g) | 预测能耗(J) | 预测F_max(N) | 预测mass/energy(g/J) |")
        lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        for i in range(min(10, len(df_candidates))):
            r = df_candidates.iloc[i]
            lines.append(f"| {i+1} | {r.get('angle_deg', '?'):.0f} | {r.get('speed_percent', '?'):.0f} | "
                         f"{r.get('depth_mm', '?'):.0f} | {r.get('score', 0):.3f} | "
                         f"{r.get('pred_weight_g', 0):.0f} | {r.get('pred_E_mech', 0):.0f} | "
                         f"{r.get('pred_F_max', 0):.1f} | {r.get('pred_mass_per_energy', 0):.3f} |")
        lines.append("")

        # 最优方案
        best_cand = df_candidates.iloc[0]
        lines.append(f"### 单目标最优\n")
        lines.append(f"- **铲泥质量最大预测组合**: 角度 {best_cand['angle_deg']:.0f}°, 速度 {best_cand['speed_percent']:.0f}%, "
                     f"深度 {best_cand['depth_mm']:.0f}mm, 预测质量 {best_cand['pred_weight_g']:.0f}g")
        # 找 mass_per_energy 最高
        if "pred_mass_per_energy" in df_candidates.columns:
            idx_meff = df_candidates["pred_mass_per_energy"].idxmax()
            meff_cand = df_candidates.loc[idx_meff]
            lines.append(f"- **单位能耗效率最高预测组合**: 角度 {meff_cand['angle_deg']:.0f}°, 速度 {meff_cand['speed_percent']:.0f}%, "
                         f"深度 {meff_cand['depth_mm']:.0f}mm, 预测效率 {meff_cand['pred_mass_per_energy']:.3f} g/J")
        lines.append(f"- **综合多目标评分最高**: 角度 {best_cand['angle_deg']:.0f}°, 速度 {best_cand['speed_percent']:.0f}%, "
                     f"深度 {best_cand['depth_mm']:.0f}mm, 评分 {best_cand['score']:.3f}\n")
    else:
        lines.append("多目标优化候选为空，可能是因为响应面模型未成功。\n")

    # 对比实际
    lines.append("### 与实际实验对比\n")
    lines.append(f"- 实际最大铲泥质量: {best['mass_g']:.0f}g（角度 {best['angle_deg']}°, 速度 {best['speed_percent']}%, 深度 {best['depth_mm']}mm）")
    if not df_candidates.empty:
        pred_w = df_candidates.iloc[0].get("pred_weight_g", 0)
        lines.append(f"- 响应面预测最大铲泥质量: {pred_w:.0f}g")
        lines.append(f"- 推荐的最优组合是否在实际实验中验证过？")
        # 检查是否匹配
        matched = False
        for _, r in df_feat.iterrows():
            if (abs(r["angle_deg"] - best_cand["angle_deg"]) < 1 and
                abs(r["speed_percent"] - best_cand["speed_percent"]) < 1 and
                abs(r["depth_mm"] - best_cand["depth_mm"]) < 1):
                matched = True
                lines.append(f"  - 实际实验 {r['folder']} 使用了该参数组合，铲泥质量 {r['mass_g']:.0f}g")
                break
        if not matched:
            lines.append("  - 该最优组合在现有 27 组实验中未直接测试，建议作为后续实验参考。")
    lines.append("")

    # 10. 结论
    lines.append("## 10. 结论\n")
    lines.append("### 主要发现\n")
    lines.append(f"1. **实际实验中铲泥质量最大的组**: {best['folder']}，角度 {best['angle_deg']}°、速度 {best['speed_percent']}%、"
                 f"深度 {best['depth_mm']}mm，铲泥质量 {best['mass_g']:.0f}g。")
    if not df_candidates.empty:
        lines.append(f"2. **响应面预测中铲泥质量最大组合**: 角度 {best_cand['angle_deg']:.0f}°、速度 {best_cand['speed_percent']:.0f}%、"
                     f"深度 {best_cand['depth_mm']:.0f}mm，预测质量 {best_cand['pred_weight_g']:.0f}g。")
        lines.append(f"3. **多目标优化推荐组合**: 角度 {best_cand['angle_deg']:.0f}°、速度 {best_cand['speed_percent']:.0f}%、"
                     f"深度 {best_cand['depth_mm']:.0f}mm，综合评分 {best_cand['score']:.3f}。")

    # 因素排序
    lines.append("")
    lines.append("### 因素影响排序\n")
    for resp, label in [("mass_g", "铲泥质量"), ("E_mech", "能耗"), ("F_max", "阻力峰值")]:
        if resp in df_feat.columns:
            ranges = {}
            for fac in ["angle_deg", "speed_percent", "depth_mm"]:
                ms = [df_feat[df_feat[fac] == l][resp].mean() for l in sorted(df_feat[fac].unique())]
                ranges[fac] = max(ms) - min(ms)
            sorted_factors = sorted(ranges, key=ranges.get, reverse=True)
            lines.append(f"- **{label}** 影响因素排序: {' > '.join(sorted_factors)}")
    lines.append("")

    lines.append("### 后续实验建议\n")
    lines.append("1. **响应面验证**: 在多目标优化推荐参数组合附近进行重复实验验证。")
    lines.append("2. **增加重复实验**: 当前每个参数组合只有一次实验，无法评估实验误差。")
    lines.append("3. **控制泥浆状态**: 泥浆含水量、粘度等可能对铲泥质量有显著干扰。")
    lines.append("4. **扩展参数范围**: 考虑更大角度（如 60°）、更高速、更深度的组合。")
    lines.append("5. **动态力矩监测**: 考虑实时监测关节力矩，预防超载。\n")

    lines.append("### 局限性说明\n")
    lines.append(f"- 当前样本量有限（{n_ok} 组），每个参数组合缺少真正重复实验。")
    lines.append("- 泥浆状态（含水量、粘度、密度）未作为控制变量，可能对 weight_g 有干扰。")
    lines.append("- 响应面模型基于二次多项式假设，实际物理过程可能更复杂。")
    lines.append("- 响应面模型需要后续重复实验验证其预测能力。")
    lines.append("- 能耗计算基于 tau_estimated 和 Act_qd，实际电气损耗未纳入。\n")

    # --- 附录 ---
    lines.append("---\n")
    lines.append("## 附录: 图表索引\n")
    lines.append("| 图号 | 文件名 | 说明 |")
    lines.append("| --- | --- | --- |")
    lines.append("| 1 | 01_weight_bar_all_runs.png | 27 次实验铲泥质量柱状图 |")
    lines.append("| 2 | 02_weight_by_angle_speed_depth.png | 三因素铲泥质量分布图 |")
    lines.append("| 3 | 03_force_time_examples.png | 典型力曲线（最大/最优/最小质量） |")
    lines.append("| 4 | 04_force_features_bar.png | 受力特征对比柱状图 |")
    lines.append("| 5 | 05_energy_features_bar.png | 能耗特征对比柱状图 |")
    lines.append("| 6 | 06_trajectory_smoothness_bar.png | 轨迹平滑性对比柱状图 |")
    lines.append("| 7 | 07_response_surface_weight_angle_speed.png | 质量响应面(角度×速度) |")
    lines.append("| 8 | 08_response_surface_weight_angle_depth.png | 质量响应面(角度×深度) |")
    lines.append("| 9 | 09_response_surface_weight_speed_depth.png | 质量响应面(速度×深度) |")
    lines.append("| 10 | 10_pareto_mass_energy_force.png | Pareto 多目标散点图 |")
    lines.append("| 11 | 11_factor_main_effect_weight.png | 铲泥质量主效应图 |")
    lines.append("| 12 | 12_factor_main_effect_energy.png | 能耗主效应图 |")
    lines.append("| 13 | 13_factor_main_effect_force.png | 阻力主效应图 |")
    lines.append("| 14 | 14_correlation_heatmap.png | 指标相关性热力图 |\n")

    lines.append("### 输出文件列表\n")
    lines.append(f"- 脚本: `scripts/analyze_mud_experiment.py`")
    lines.append(f"- 表格: `tables/` 目录下 6 个 CSV 文件")
    lines.append(f"- 图片: `figures/` 目录下 14 个 PNG 文件")
    lines.append(f"- 报告: `report/mud_experiment_analysis_report.md`\n")

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(str(path), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    log(f"报告已保存: {path}")


# ============================================================
# 十一、主函数
# ============================================================

def main():
    log("=" * 60)
    log("UR10 机械臂铲泥实验数据分析")
    log("=" * 60)

    # 创建输出目录
    for d in [SCRIPT_DIR, TABLE_DIR, FIGURE_DIR, REPORT_DIR]:
        d.mkdir(parents=True, exist_ok=True)
    log(f"输出目录: {OUTPUT_BASE}")

    # 复制脚本自身到 scripts 目录
    script_src = Path(__file__).resolve()
    script_dst = SCRIPT_DIR / "analyze_mud_experiment.py"
    try:
        with open(script_src, "r", encoding="utf-8") as f_src:
            with open(script_dst, "w", encoding="utf-8") as f_dst:
                f_dst.write(f_src.read())
        log(f"脚本已复制: {script_dst}")
    except Exception as e:
        log(f"复制脚本失败: {e}")

    # ---- 实验元数据 ----
    meta_list = []
    for (folder, test_no, repeat_no, angle_deg, speed_percent, depth_mm, weight_g_ref) in EXPERIMENTS:
        meta_list.append({
            "folder": folder,
            "test_no": test_no,
            "repeat_no": repeat_no,
            "angle_deg": angle_deg,
            "speed_percent": speed_percent,
            "depth_mm": depth_mm,
            "weight_g_ref": weight_g_ref,
        })
    log(f"共 {len(meta_list)} 组实验元数据")

    # 保存实验计划
    df_plan = pd.DataFrame(meta_list)
    df_plan.to_csv(TABLE_DIR / "experiment_plan.csv", index=False, encoding="utf-8")

    # ---- 处理每个实验 ----
    results = []
    errors  = []
    all_force_data = {}  # 用于画力曲线
    warnings_list = []

    for meta in meta_list:
        log(f"处理 {meta['folder']} ...")
        try:
            feat = process_single_session(meta)
            if feat is not None:
                results.append(feat)
                # 保存力数据用于绘图
                try:
                    csv_path = find_csv_file(DATA_ROOT / meta["folder"])
                    if csv_path:
                        df = pd.read_csv(csv_path)
                        df.columns = df.columns.str.strip()
                        df = safe_numeric(df, [c for c in df.columns])
                        if "epoch_time" in df.columns:
                            df = df.sort_values("epoch_time")
                        df["t"] = df["epoch_time"] - df["epoch_time"].iloc[0]
                        force_cols, _ = resolve_force_cols(df)
                        if force_cols:
                            f_res = compute_F_res(df, force_cols)
                            phase = detect_shoveling_phase(df, force_cols, "t")
                            all_force_data[meta["folder"]] = {
                                "t": df["t"].values,
                                "f_res": f_res,
                                "mud_start": phase["mud_start_time"],
                                "mud_end": phase["mud_end_time"],
                            }
                except Exception as e:
                    pass
        except Exception as e:
            err_msg = f"{meta['folder']}: {str(e)}"
            log(f"  ERROR: {err_msg}")
            errors.append({"folder": meta["folder"], "error": str(e), "traceback": traceback.format_exc()})

    if not results:
        log("ERROR: 无有效实验结果，退出。")
        return

    df_feat = pd.DataFrame(results)
    n_ok = len(df_feat)
    n_total = len(meta_list)
    log(f"成功处理 {n_ok}/{n_total} 组实验")

    # ---- 保存特征 CSV ----
    # 选择基本列
    base_cols = ["folder","test_no","repeat_no","angle_deg","speed_percent","depth_mm","mass_g"]
    feature_cols = [
        "n_samples","duration","dt_mean","dt_std",
        "ur_valid_ratio","ft_valid_ratio",
        "stage_detected","mud_start_time","mud_end_time","mud_duration",
        "F_mean","F_max","F_min","F_std","F_rms","F_impulse",
        "force_base_x_mean","force_base_y_mean","force_base_z_mean",
        "force_base_x_max_abs","force_base_y_max_abs","force_base_z_max_abs",
        "tcp_path_length","tcp_speed_mean","tcp_speed_max","tcp_speed_std",
        "Z_min","Z_max","actual_depth_estimated",
        "tcp_acc_mean","tcp_acc_max","tcp_acc_rms",
        "tcp_jerk_mean","tcp_jerk_max","tcp_jerk_rms",
        "joint_rmse","joint_mae","joint_max_error",
        "E_mech","P_mean","P_max","tau_rms","tau_max",
        "Robot_Current_mean","Robot_Current_max","Robot_Voltage_mean","E_elec",
        "mass_per_energy","mass_per_force_impulse","mass_per_time",
        "data_file","force_source","tau_source",
    ]
    avail_cols = base_cols + [c for c in feature_cols if c in df_feat.columns]
    df_feat_out = df_feat[avail_cols].copy()
    df_feat_out.to_csv(TABLE_DIR / "single_run_features.csv", index=False, encoding="utf-8")
    log(f"single_run_features.csv 已保存 ({len(df_feat_out)} 行)")

    # ---- 分组统计（按 test_no） ----
    group_cols = ["test_no", "angle_deg", "depth_mm"]
    agg_dict = {}
    for c in ["mass_g", "F_mean", "F_max", "F_impulse", "E_mech",
              "mass_per_energy", "mass_per_force_impulse",
              "tcp_acc_rms", "tcp_jerk_rms", "joint_rmse"]:
        if c in df_feat.columns:
            agg_dict[c] = ["mean", "std", "max", "min"] if c == "mass_g" else ["mean"]
    df_grouped = df_feat.groupby(group_cols).agg(agg_dict).round(3)
    df_grouped.columns = [f"{a}_{b}".strip("_") for a, b in df_grouped.columns]
    df_grouped = df_grouped.reset_index()
    df_grouped.to_csv(TABLE_DIR / "grouped_summary_by_test.csv", index=False, encoding="utf-8")
    log(f"grouped_summary_by_test.csv 已保存")

    # ---- 因素分析 ----
    factor_responses = ["mass_g", "E_mech", "F_max", "F_impulse", "mass_per_energy", "tcp_jerk_rms", "joint_rmse"]
    factor_responses = [c for c in factor_responses if c in df_feat.columns]
    dfs_factors = []
    for fac in ["angle_deg", "speed_percent", "depth_mm"]:
        df_f = factor_analysis(df_feat, fac, factor_responses)
        dfs_factors.append(df_f)
    if dfs_factors:
        df_factor_level = pd.concat(dfs_factors, ignore_index=True)
        df_factor_level.to_csv(TABLE_DIR / "factor_level_summary.csv", index=False, encoding="utf-8")
        log(f"factor_level_summary.csv 已保存")

    # ---- 响应面建模 ----
    factors = ["angle_deg", "speed_percent", "depth_mm"]
    responses = ["mass_g", "E_mech", "F_max", "F_impulse", "tcp_jerk_rms", "joint_rmse", "mass_per_energy"]
    responses = [c for c in responses if c in df_feat.columns]

    models = {}
    for resp in responses:
        model = build_response_surface(df_feat, resp, factors)
        models[resp] = model
        if model["success"]:
            log(f"  响应面 {resp}: R2={model['r2']:.3f}, RMSE={model['rmse']:.2f}")
        else:
            log(f"  响应面 {resp}: 建模失败（仅 {model['n_samples']} 个有效样本）")

    # ---- 网格预测 ----
    n_grid = 25
    grid = {}
    grid["angle_deg"]    = np.linspace(20, 50, n_grid)
    grid["speed_percent"] = np.linspace(35, 100, n_grid)
    grid["depth_mm"]     = np.linspace(20, 60, n_grid)

    # 全组合网格
    ag, sp, dp = np.meshgrid(grid["angle_deg"], grid["speed_percent"], grid["depth_mm"])
    df_grid = pd.DataFrame({
        "angle_deg": ag.ravel(),
        "speed_percent": sp.ravel(),
        "depth_mm": dp.ravel(),
    })
    log(f"响应面网格: {len(df_grid)} 个预测点")

    for resp in responses:
        model = models.get(resp)
        if model and model["success"]:
            try:
                X_grid = df_grid[factors].values
                X_poly = model["poly"].transform(X_grid)
                pred = model["reg"].predict(X_poly)
                df_grid[f"pred_{resp}"] = pred
            except Exception as e:
                log(f"  WARNING: 预测 {resp} 失败: {e}")
                df_grid[f"pred_{resp}"] = np.nan
        else:
            df_grid[f"pred_{resp}"] = np.nan

    # 保存网格
    df_grid.to_csv(TABLE_DIR / "response_surface_grid.csv", index=False, encoding="utf-8")
    log(f"response_surface_grid.csv 已保存 ({len(df_grid)} 行)")

    # ---- 多目标优化 ----
    # 只在网格点上进行优化
    objectives = {
        "pred_mass_g": 0.30,
        "pred_mass_per_energy": 0.20,
        "pred_E_mech": -0.10,
        "pred_F_max": -0.10,
        "pred_F_impulse": -0.05,
        "pred_tcp_jerk_rms": -0.05,
        "pred_joint_rmse": -0.05,
    }
    # 检查哪些 pred 列存在
    avail_pred = {k: v for k, v in objectives.items() if k in df_grid.columns}

    # mass_per_force_impulse 是可选目标
    if "pred_mass_per_force_impulse" in df_grid.columns:
        avail_pred["pred_mass_per_force_impulse"] = 0.15
    else:
        # 重新归一化权重
        extra = 0.15
        for k in avail_pred:
            avail_pred[k] = avail_pred.get(k, 0) * (1 + extra / sum(abs(v) for v in avail_pred.values()))

    df_candidates = multi_objective_optimize(df_grid, avail_pred)

    # 取前 20
    df_top20 = df_candidates.head(20).reset_index(drop=True)
    df_top20.insert(0, "rank", range(1, 21))
    df_top20.to_csv(TABLE_DIR / "multi_objective_candidates.csv", index=False, encoding="utf-8")
    log(f"multi_objective_candidates.csv 已保存 ({len(df_top20)} 个候选)")

    # ---- 生成图表 ----
    log("生成图表 ...")

    try:
        plot_weight_bar(df_feat, FIGURE_DIR / "01_weight_bar_all_runs.png")
        log("  01 OK")
    except Exception as e:
        log(f"  WARNING: 01 图失败: {e}")

    try:
        plot_weight_by_factors(df_feat, FIGURE_DIR / "02_weight_by_angle_speed_depth.png")
        log("  02 OK")
    except Exception as e:
        log(f"  WARNING: 02 图失败: {e}")

    try:
        plot_force_curves(df_feat, all_force_data, FIGURE_DIR / "03_force_time_examples.png")
        log("  03 OK")
    except Exception as e:
        log(f"  WARNING: 03 图失败: {e}")

    try:
        plot_force_features_bar(df_feat, FIGURE_DIR / "04_force_features_bar.png")
        log("  04 OK")
    except Exception as e:
        log(f"  WARNING: 04 图失败: {e}")

    try:
        plot_energy_features_bar(df_feat, FIGURE_DIR / "05_energy_features_bar.png")
        log("  05 OK")
    except Exception as e:
        log(f"  WARNING: 05 图失败: {e}")

    try:
        plot_smoothness_bar(df_feat, FIGURE_DIR / "06_trajectory_smoothness_bar.png")
        log("  06 OK")
    except Exception as e:
        log(f"  WARNING: 06 图失败: {e}")

    # 响应面 3D 图
    fixed_combos = [
        ("angle_deg", "speed_percent", {"depth_mm": 40},
         "07_response_surface_weight_angle_speed",
         "Angle (deg)", "Speed (%)", "Weight (g)",
         "RSM: Weight vs Angle & Speed (Depth=40mm)"),
        ("angle_deg", "depth_mm", {"speed_percent": 70},
         "08_response_surface_weight_angle_depth",
         "Angle (deg)", "Depth (mm)", "Weight (g)",
         "RSM: Weight vs Angle & Depth (Speed=70%)"),
        ("speed_percent", "depth_mm", {"angle_deg": 35},
         "09_response_surface_weight_speed_depth",
         "Speed (%)", "Depth (mm)", "Weight (g)",
         "RSM: Weight vs Speed & Depth (Angle=35°)"),
    ]

    for x1, x2, fixed, fname, xl, yl, zl, title in fixed_combos:
        try:
            model = models.get("mass_g", {})
            plot_response_surface_3d(model, x1, x2, fixed, xl, yl, zl, title,
                                     FIGURE_DIR / f"{fname}.png", df_feat)
            log(f"  {fname} OK")
        except Exception as e:
            log(f"  WARNING: {fname} 失败: {e}")

    try:
        plot_pareto(df_feat, df_candidates, FIGURE_DIR / "10_pareto_mass_energy_force.png")
        log("  10 OK")
    except Exception as e:
        log(f"  WARNING: 10 图失败: {e}")

    try:
        plot_main_effect(df_feat, "mass_g", FIGURE_DIR / "11_factor_main_effect_weight.png")
        log("  11 OK")
    except Exception as e:
        log(f"  WARNING: 11 图失败: {e}")

    try:
        plot_main_effect(df_feat, "E_mech", FIGURE_DIR / "12_factor_main_effect_energy.png")
        log("  12 OK")
    except Exception as e:
        log(f"  WARNING: 12 图失败: {e}")

    try:
        plot_main_effect(df_feat, "F_max", FIGURE_DIR / "13_factor_main_effect_force.png")
        log("  13 OK")
    except Exception as e:
        log(f"  WARNING: 13 图失败: {e}")

    try:
        plot_correlation_heatmap(df_feat, FIGURE_DIR / "14_correlation_heatmap.png")
        log("  14 OK")
    except Exception as e:
        log(f"  WARNING: 14 图失败: {e}")

    # ---- 生成报告 ----
    log("生成报告 ...")
    try:
        generate_report(df_feat, models, df_candidates, errors,
                        REPORT_DIR / "mud_experiment_analysis_report.md")
    except Exception as e:
        log(f"  WARNING: 报告生成失败: {e}")

    # ---- 最终总结 ----
    log("=" * 60)
    log("分析完成！")
    log(f"输出目录: {OUTPUT_BASE}")
    if n_ok > 0:
        idx_best_mass = df_feat["mass_g"].idxmax()
        bm = df_feat.loc[idx_best_mass]
        log(f"实际最大铲泥质量: {bm['folder']}  {bm['mass_g']:.0f}g  "
            f"(角度={bm['angle_deg']}°, 速度={bm['speed_percent']}%, 深度={bm['depth_mm']}mm)")
    if not df_candidates.empty:
        bc = df_candidates.iloc[0]
        log(f"多目标最优预测: 角度={bc['angle_deg']:.0f}°, 速度={bc['speed_percent']:.0f}%, "
            f"深度={bc['depth_mm']:.0f}mm, 评分={bc['score']:.3f}")
    log(f"报告路径: {REPORT_DIR / 'mud_experiment_analysis_report.md'}")
    log("=" * 60)


if __name__ == "__main__":
    main()
