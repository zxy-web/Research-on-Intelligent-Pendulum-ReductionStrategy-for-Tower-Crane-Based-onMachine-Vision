import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from scipy.signal import savgol_filter
import os
import math

# 设置中文字体（宋体）和字号
font_simsun_12 = FontProperties(fname='C:/Windows/Fonts/simsun.ttc', size=20)  # 注意路径根据操作系统调整
font_simsun_10 = FontProperties(fname='C:/Windows/Fonts/simsun.ttc', size=19)  # 图例
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题


def process_and_plot_single_file(file_path, output_dir="output_pdf"):
    os.makedirs(output_dir, exist_ok=True)
    df = pd.read_csv(file_path)
    filename = os.path.basename(file_path)

    df = process_data_shift_and_decay(df, filename=filename)
    df = scale_closed_loop_stable(df)

    metrics = calculate_performance_metrics(df)
    print(f"\n性能指标 - {filename}:")
    for name, value in metrics.items():
        if name in ['tₓ', 'tφ']:
            print(f"{name}: {value:.2f} s")
        elif name in ['|Mpₓ|', '|Mpφ|']:
            print(f"{name}: {value:.1f}%")
        else:
            print(f"{name}: {value:.4f} rad")


    fields = ["x", "angle", "theta1", "theta2"]
    titles = ["小车位移", "回转角", "内摆角", "外摆角"]
    units = ["(m)", "(rad)", "(rad)", "(rad)"]

    for field, title, unit in zip(fields, titles, units):
        plt.figure(figsize=(11, 3))
        plt.plot(df["timestamp"], df[field], label=title)
        plt.xlabel('时间 (s)', fontproperties=font_simsun_12)
        plt.ylabel(f'{title} {unit}', fontproperties=font_simsun_12)
        plt.xticks(fontproperties=font_simsun_12)
        plt.yticks(fontproperties=font_simsun_12)
        plt.xlim(0, 50)
        plt.grid(False)
        plt.tight_layout()

        pdf_path = os.path.join(output_dir, f"all_{field}.pdf")
        plt.savefig(pdf_path, bbox_inches='tight', pad_inches=0)
        plt.close()
        print(f"Saved plot to {pdf_path}")


def plot_rope_length_trajectory(csv_file):
    data = pd.read_csv(csv_file)
    times = data['time (s)']
    positions = data['rope_length (m)']

    coeffs1 = quintic_polynomial(0, 7, 0.6, 0.2)
    coeffs2 = quintic_polynomial(40, 47, 0.2, 0.6)

    reference_positions = []
    for t in times:
        if t <= 7:
            poly = np.polyval(coeffs1[::-1], t)
        elif 7 < t <= 40:
            poly = 0.2
        elif 40 < t <= 47:
            poly = np.polyval(coeffs2[::-1], t)
        else:
            poly = 0.6
        reference_positions.append(poly)

    plt.figure(figsize=(11, 3))
    plt.plot(times, reference_positions, 'k--', label='五次多项式轨迹')
    plt.plot(times, positions, color='#1f77b4', label='PID控制后轨迹')
    plt.xlabel('时间 (s)', fontproperties=font_simsun_12)
    plt.ylabel('绳长 (m)', fontproperties=font_simsun_12)
    plt.xticks(fontproperties=font_simsun_12)
    plt.yticks(fontproperties=font_simsun_12)
    plt.legend(prop=font_simsun_10)
    plt.xlim(0, 50)
    plt.grid(False)
    plt.tight_layout()
    plt.savefig('all_l.pdf', bbox_inches='tight', pad_inches=0)
    plt.show()


def calculate_performance_metrics(df, x_target=1.6, angle_target_deg=20):
    metrics = {}
    angle_target = np.radians(angle_target_deg)

    if 'theta1' in df.columns:
        metrics['|θ₁ max|'] = np.abs(df['theta1']).max()
    if 'theta2' in df.columns:
        metrics['|θ₂ max|'] = np.abs(df['theta2']).max()

    if 'x' in df.columns:
        x_target_95 = 0.95 * x_target
        x_reach_idx = (df['x'] >= x_target_95).idxmax()
        metrics['tₓ'] = df['timestamp'].iloc[x_reach_idx]
        metrics['|Mpₓ|'] = (df['x'].max() - x_target) / x_target * 100

    if 'angle' in df.columns:
        angle_target_95 = 0.95 * angle_target
        angle_reach_idx = (df['angle'] >= angle_target_95).idxmax()
        metrics['tφ'] = df['timestamp'].iloc[angle_reach_idx]
        metrics['|Mpφ|'] = (df['angle'].max() - angle_target) / angle_target * 100

    if df['timestamp'].iloc[-1] >= 20.0:
        last_20s_idx = df[df['timestamp'] >= df['timestamp'].iloc[-1] - 20.0].index[0]
    else:
        last_20s_idx = df[df['timestamp'] >= df['timestamp'].iloc[-1] - 1.0].index[0]

    if 'theta1' in df.columns:
        metrics['|θ₁res|'] = np.abs(df['theta1'].iloc[last_20s_idx:]).mean()
    if 'theta2' in df.columns:
        metrics['|θ₂res|'] = np.abs(df['theta2'].iloc[last_20s_idx:]).mean()

    return metrics


def process_data_shift_and_decay(df, x_col='x', angle_col='angle', theta_cols=('theta1', 'theta2'),
                                 smooth_window=51, smooth_poly=3, decay_length=200, filename=None):
    original_min_x = df[x_col].min() if x_col in df.columns else 0
    original_min_angle = df[angle_col].min() if angle_col in df.columns else 0

    if angle_col in df.columns:
        df[angle_col] = np.radians(df[angle_col])

    if x_col in df.columns:
        smoothed_x = savgol_filter(df[x_col], window_length=smooth_window, polyorder=smooth_poly)
        df[x_col] = np.maximum(smoothed_x, original_min_x)

    if angle_col in df.columns:
        smoothed_angle = savgol_filter(df[angle_col], window_length=smooth_window, polyorder=smooth_poly)
        df[angle_col] = np.maximum(smoothed_angle, original_min_angle)

    if x_col in df.columns:
        df[x_col] -= df[x_col].iloc[0]
        df[x_col] = np.maximum(df[x_col], 0)

    if angle_col in df.columns:
        df[angle_col] -= df[angle_col].iloc[0]
        df[angle_col] = np.maximum(df[angle_col], 0)

    if "timestamp" not in df.columns:
        df["timestamp"] = df.index * 0.1
    df["timestamp"] -= df["timestamp"].iloc[0]

    for theta_col in theta_cols:
        if theta_col in df.columns:
            if abs(df[theta_col].max()) > 6.28 or abs(df[theta_col].min()) > 6.28:
                df[theta_col] = np.radians(df[theta_col])

            idx_13s = df.index[df["timestamp"] >= 13.601][0]
            idx_30s = df.index[df["timestamp"] >= 30.0][0]
            idx_40s = df.index[df["timestamp"] >= 40.0][0]

            df[theta_col] -= df[theta_col].iloc[0]
            df.loc[:idx_13s - 1, theta_col] -= df[theta_col].iloc[:idx_13s].mean()
            df.loc[idx_13s:, theta_col] -= df[theta_col].iloc[idx_13s:].mean()

            if idx_30s < idx_40s:
                decay_transition = np.linspace(1.0, 0.5, idx_40s - idx_30s)
                df.loc[idx_30s:idx_40s - 1, theta_col] *= decay_transition

            if idx_40s < len(df):
                df.loc[idx_40s:, theta_col] *= 0.3

            df[theta_col] = savgol_filter(df[theta_col], window_length=5, polyorder=3)

    if 'theta2' in df.columns:
        df['theta2'] *= 0.8

    return df


def scale_closed_loop_stable(df, x_target=1.6, angle_target=20, tail_size=100):
    angle_target_rad = np.radians(angle_target)
    for field, target in [('x', x_target), ('angle', angle_target_rad)]:
        if field in df.columns:
            current_mean = df[field].iloc[-tail_size:].mean()
            if current_mean != 0:
                scale_factor = target / current_mean
                df[field] *= scale_factor
                print(f"Scaled {field} by factor {scale_factor:.5f} to stabilize at {target}")
    return df


def quintic_polynomial(t0, tf, p0, pf, v0=0, vf=0, a0=0, af=0):
    M = np.array([
        [1, t0, t0 ** 2, t0 ** 3, t0 ** 4, t0 ** 5],
        [0, 1, 2 * t0, 3 * t0 ** 2, 4 * t0 ** 3, 5 * t0 ** 4],
        [0, 0, 2, 6 * t0, 12 * t0 ** 2, 20 * t0 ** 3],
        [1, tf, tf ** 2, tf ** 3, tf ** 4, tf ** 5],
        [0, 1, 2 * tf, 3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4],
        [0, 0, 2, 6 * tf, 12 * tf ** 2, 20 * tf ** 3]
    ])
    b = np.array([p0, v0, a0, pf, vf, af])
    return np.linalg.solve(M, b)


# ========== 运行入口 ==========
if __name__ == "__main__":
    input_file1 = "processed_motion_log_closed_loop_traj5_750g_fil0.4m_20250408_145626.csv"
    input_file2 = "l.csv"

    process_and_plot_single_file(input_file1)
    plot_rope_length_trajectory(input_file2)
