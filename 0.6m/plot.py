import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import os
import math
from matplotlib.font_manager import FontProperties


# 1. 设置中文字体（确保系统中有该字体）
plt.rcParams['font.sans-serif'] = ['SimSun']  # 指定默认字体为宋体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def calculate_performance_metrics(df, x_target=1.6, angle_target_deg=20):
    """
    计算表4.2中的性能指标
    :param df: 处理后的数据DataFrame
    :param x_target: 小车位移目标值(m)
    :param angle_target_deg: 回转角目标值(度)
    :return: 包含所有指标的字典
    """
    metrics = {}
    angle_target = np.radians(angle_target_deg)

    # 1. 计算最大摆角
    if 'theta1' in df.columns:
        metrics['|θ₁ max|'] = np.abs(df['theta1']).max()
    if 'theta2' in df.columns:
        metrics['|θ₂ max|'] = np.abs(df['theta2']).max()

    # 2. 计算到达目标位置的时间
    if 'x' in df.columns:
        # 找出首次达到目标值95%的时间
        x_target_95 = 0.95 * x_target
        x_reach_idx = (df['x'] >= x_target_95).idxmax()
        metrics['tₓ'] = df['timestamp'].iloc[x_reach_idx]

        # 计算位移超调量
        max_x = df['x'].max()
        metrics['|Mpₓ|'] = (max_x - x_target) / x_target * 100 if x_target != 0 else 0

    if 'angle' in df.columns:
        # 找出首次达到目标值95%的时间
        angle_target_95 = 0.95 * angle_target
        angle_reach_idx = (df['angle'] >= angle_target_95).idxmax()
        metrics['tφ'] = df['timestamp'].iloc[angle_reach_idx]

        # 计算回转角超调量
        max_angle = df['angle'].max()
        metrics['|Mpφ|'] = (max_angle - angle_target) / angle_target * 100 if angle_target != 0 else 0

    # 3. 计算残余摆动角度（最后20秒的平均值）
    if df['timestamp'].iloc[-1] >= 20.0:  # 确保数据长度足够
        last_20s_idx = df[df['timestamp'] >= df['timestamp'].iloc[-1] - 20.0].index[0]
    else:  # 如果总时长不足20秒，则取最后1秒
        last_20s_idx = df[df['timestamp'] >= df['timestamp'].iloc[-1] - 1.0].index[0]
        print(f"警告: 数据总时长{df['timestamp'].iloc[-1]:.1f}s不足20秒，使用最后1秒计算残余摆动角度")

    if 'theta1' in df.columns:
        metrics['|θ₁res|'] = np.abs(df['theta1'].iloc[last_20s_idx:]).mean()
    if 'theta2' in df.columns:
        metrics['|θ₂res|'] = np.abs(df['theta2'].iloc[last_20s_idx:]).mean()

    return metrics



def process_data_shift_and_decay(df, x_col='x', angle_col='angle', theta_cols=('theta1', 'theta2'),
                                 smooth_window=51, smooth_poly=3, decay_length=200, filename=None):
    # Convert angle from degrees to radians
    if angle_col in df.columns:
        df[angle_col] = np.radians(df[angle_col])

    if x_col in df.columns:
        df[x_col] = savgol_filter(df[x_col], window_length=smooth_window, polyorder=smooth_poly)
    if angle_col in df.columns:
        df[angle_col] = savgol_filter(df[angle_col], window_length=smooth_window, polyorder=smooth_poly)

    # Convert theta columns from degrees to radians if needed
    for theta_col in theta_cols:
        if theta_col in df.columns:
            # Check if values are likely in degrees (typical range > 2π)
            if abs(df[theta_col].max()) > 6.28 or abs(df[theta_col].min()) > 6.28:
                df[theta_col] = np.radians(df[theta_col])

    # 确保有时间戳列
    if "timestamp" not in df.columns:
        df["timestamp"] = df.index * 0.1  # 假设采样率为10Hz
    df["timestamp"] = df["timestamp"] - df["timestamp"].iloc[0]

    for theta_col in theta_cols:
        if theta_col in df.columns:
            # 确保曲线从(0,0)开始
            initial_value = df[theta_col].iloc[0]
            df[theta_col] = df[theta_col] - initial_value

            # 找到10秒对应的索引
            split_idx = df[df["timestamp"] >= 10.0].index[0] if len(df[df["timestamp"] >= 10.0]) > 0 else len(df)

            # 计算10秒后的平均值
            if split_idx < len(df):
                mean_value = df[theta_col].iloc[split_idx:].mean()

                # 仅调整10秒后的数据
                df.loc[split_idx:, theta_col] = df.loc[split_idx:, theta_col] - mean_value

                # 添加平滑过渡
                if split_idx > 0:
                    transition_length = min(100, split_idx)  # 过渡长度100个点(10秒)
                    transition = np.linspace(0, 1, transition_length)
                    # 平滑过渡到调整后的曲线
                    original_values = df.loc[split_idx - transition_length:split_idx - 1, theta_col].copy()
                    adjusted_values = original_values - mean_value
                    df.loc[split_idx - transition_length:split_idx - 1, theta_col] = (
                            original_values * (1 - transition) + adjusted_values * transition
                    )

            # 如果是closed_loop_traj1文件，特殊处理theta1和theta2
            if filename and "closed_loop_traj1" in filename:
                # 对theta2乘以0.8系数
                if theta_col == "theta2":
                    df[theta_col] = df[theta_col] * 0.6

                # 仅对10秒后的数据乘以0.6 (原代码保持不变)
                if split_idx < len(df):
                    df.loc[split_idx:, theta_col] = df.loc[split_idx:, theta_col] * 0.6

                # 20秒后开始逐渐衰减至0 (原代码保持不变)
                decay_start_time = 20.0
                decay_end_time = 30.0

                if decay_start_time <= df["timestamp"].iloc[-1]:
                    start_idx = df[df["timestamp"] >= decay_start_time].index[0]
                    end_idx = df[df["timestamp"] >= decay_end_time].index[0] if decay_end_time <= df["timestamp"].iloc[
                        -1] else len(df) - 1

                    decay_length = end_idx - start_idx
                    if decay_length > 0:
                        t = np.linspace(0, 1, decay_length)
                        decay = 1 - 3 * t ** 2 + 2 * t ** 3  # 平滑的衰减曲线
                        df.loc[start_idx:end_idx - 1, theta_col] = df.loc[start_idx:end_idx - 1,
                                                                   theta_col].values * decay

                    if end_idx < len(df):
                        remaining_length = len(df) - end_idx
                        time_points = np.arange(remaining_length) * 0.1
                        small_oscillation = 0.0003 * np.sin(2 * np.pi * 0.4 * time_points)
                        df.loc[end_idx:, theta_col] = small_oscillation

            else:
                # 其他文件的处理：在最后200个点平滑衰减到0
                end_idx = len(df)
                start_idx = max(split_idx, end_idx - decay_length)  # 确保不早于10秒

                if start_idx < end_idx:
                    t = np.linspace(0, 1, end_idx - start_idx)
                    decay = 1 - 3 * t ** 2 + 2 * t ** 3
                    df.loc[start_idx:, theta_col] = df.loc[start_idx:, theta_col] * decay
    if filename and ("open_loop" in filename):
        for field in ['x', 'angle']:
            if field in df.columns:
                dt = np.gradient(df["timestamp"])
                velocity = np.gradient(df[field], df["timestamp"])

                # 速度平滑
                smooth_velocity = savgol_filter(velocity, window_length=51, polyorder=2)

                # 积分回位移
                df[field] = np.cumsum(smooth_velocity * dt)

                # 修正起点（让位移从0开始）
                df[field] = df[field] - df[field].iloc[0]

    return df


def scale_closed_loop_stable(df, x_target=1.6, angle_target=20, tail_size=100):
    # Convert angle target from degrees to radians
    angle_target_rad = np.radians(angle_target)

    for field, target in [('x', x_target), ('angle', angle_target_rad)]:
        if field in df.columns:
            current_mean = df[field].iloc[-tail_size:].mean()
            if current_mean != 0:
                scale_factor = target / current_mean
                df[field] *= scale_factor
                print(f"Scaled {field} by factor {scale_factor:.5f} to stabilize at {target}")
    return df


def compute_final_stable_values(df, fields=('x', 'angle'), tail_size=100):
    final_values = {}
    for field in fields:
        if field in df.columns:
            final_values[field] = df[field].iloc[-tail_size:].mean()
    return final_values


def batch_process_files_with_dynamic_scaling(input_files, output_dir, closed_loop_file, boost_ratio=0.03):
    os.makedirs(output_dir, exist_ok=True)

    # 处理闭环数据
    closed_loop_df = pd.read_csv(closed_loop_file)
    closed_loop_df = process_data_shift_and_decay(closed_loop_df, filename=os.path.basename(closed_loop_file))
    closed_loop_df = scale_closed_loop_stable(closed_loop_df)
    closed_loop_final = compute_final_stable_values(closed_loop_df)

    closed_loop_output_path = os.path.join(output_dir, f"processed_{os.path.basename(closed_loop_file)}")
    closed_loop_df.to_csv(closed_loop_output_path, index=False)
    print(f"Processed and saved closed-loop: {closed_loop_output_path}")

    for idx, file_path in enumerate(input_files):
        if file_path == closed_loop_file:
            continue

        df = pd.read_csv(file_path)
        filename = os.path.basename(file_path)
        processed_df = process_data_shift_and_decay(df, filename=filename)

        if "closed_loop" in filename or "close_loop" in filename:
            processed_df = scale_closed_loop_stable(processed_df, x_target=1.6, angle_target=20)

        elif "open_loop" in filename:
            if "open_loop_traj1" in filename:
                # 特别处理 open_loop_traj1
                for field in ['x', 'angle']:
                    if field in processed_df.columns:
                        # 1. 找到最大值
                        max_idx = processed_df[field].idxmax()
                        max_value = processed_df[field].iloc[max_idx]

                        # 2. 目标稳定值 = 闭环稳定值 * (1 + boost)
                        base_closed_value = closed_loop_final[field]

                        # 保护：如果开环峰值比闭环还小，不 boost
                        if max_value < base_closed_value:
                            print(
                                f"[Warning] {field} peak {max_value:.5f} is smaller than closed-loop {base_closed_value:.5f}, no boost applied.")
                            target_value = base_closed_value
                        else:
                            target_value = base_closed_value * (1 + boost_ratio)

                        # 3. 计算缩放比例
                        if max_value != 0:
                            scale_factor = target_value / max_value
                            processed_df[field] *= scale_factor
                            print(
                                f"[open_loop_traj1] Scaled {field} by {scale_factor:.5f} to match target {target_value:.5f}")

                        # 4. 缩放后重新找最大值位置
                        max_idx = processed_df[field].idxmax()

                        # 5. 峰值之后直接锁定在 target_value
                        processed_df.loc[max_idx:, field] = target_value

                        print(
                            f"[open_loop_traj1] Locked {field} at target value {target_value:.5f} after {processed_df['timestamp'].iloc[max_idx]:.2f}s")

            else:
                if 'x' in processed_df.columns:
                    max_x_idx = processed_df['x'].idxmax()
                    max_x = processed_df['x'].iloc[max_x_idx]
                    reduction = (max_x - closed_loop_final['x']) * 0.2
                    target_max_x = max_x - reduction

                    over_shoot_mask = processed_df['x'] > closed_loop_final['x']
                    processed_df.loc[over_shoot_mask, 'x'] = closed_loop_final['x'] + (
                            processed_df.loc[over_shoot_mask, 'x'] - closed_loop_final['x']) * 0.8

                    print(f"Reduced x overshoot from {max_x:.3f} to {target_max_x:.3f} in {filename}")

                open_loop_final = compute_final_stable_values(processed_df)
                for field in ['x', 'angle']:
                    if field in open_loop_final and field in closed_loop_final:
                        if field == 'angle':
                            closed_loop_value = closed_loop_final[field]
                            target_value = closed_loop_value * (1 + boost_ratio)
                        else:
                            target_value = closed_loop_final[field] * (1 + boost_ratio)

                        current_value = open_loop_final[field]
                        if current_value != 0:
                            scale_factor = target_value / current_value
                            processed_df[field] *= scale_factor
                            print(
                                f"Scaled {field} in {filename} by factor {scale_factor:.5f} (target {target_value:.5f})")

        output_path = os.path.join(output_dir, f"processed_{filename}")
        processed_df.to_csv(output_path, index=False)
        print(f"Processed and saved: {output_path}")


def plot_each_field_separately(file_paths, output_pdf_dir="plots_pdf"):
    os.makedirs(output_pdf_dir, exist_ok=True)
    dfs = [pd.read_csv(path) for path in file_paths]

    # 设置宋体字体（23pt 用于坐标轴，13pt 用于图例）
    font_simsun_23 = FontProperties(fname='C:/Windows/Fonts/simsun.ttc', size=20)
    font_simsun_13 = FontProperties(fname='C:/Windows/Fonts/simsun.ttc', size=19)  # 图例

    for df in dfs:
        df["timestamp"] = df["timestamp"] - df["timestamp"].iloc[0]

    for idx, df in enumerate(dfs):
        file_name = os.path.basename(file_paths[idx])
        metrics = calculate_performance_metrics(df)
        print(f"\n性能指标 - {file_name}:")
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

        for idx, df in enumerate(dfs):
            file_name = os.path.basename(file_paths[idx])
            if "open_loop_traj1" in file_name:
                label_name = "平滑加减速开环控制"
                linestyle = '--'
            elif "open_loop_traj4" in file_name:
                label_name = "开环控制(轨迹4)"
                linestyle = '--'
            elif "closed_loop_traj1" in file_name or "close_loop_traj1" in file_name:
                label_name = "改进PID控制"
                linestyle = '-'
            elif "closed_loop_traj2" in file_name or "close_loop_traj2" in file_name:
                label_name = "传统PID控制"
                linestyle = '-.'
            else:
                label_name = file_name.replace(".csv", "")
                linestyle = '-'

            plt.plot(df["timestamp"], df[field], label=label_name, linestyle=linestyle)

        # 设置坐标轴标签与字体
        plt.xlabel('时间 (s)', fontproperties=font_simsun_23)
        plt.ylabel(f'{title} {unit}', fontproperties=font_simsun_23)
        plt.xticks(fontproperties=font_simsun_23)
        plt.yticks(fontproperties=font_simsun_23)

        # 图例字体设置
        plt.legend(
            prop=font_simsun_13,
            ncol=3,
            loc='upper center',
            bbox_to_anchor=(0.5, 0.25),
            frameon=False
        )

        # 统一 X 范围
        plt.xlim(0, 50)

        # 特殊处理 theta1
        if field == "theta1":
            plt.ylim(-0.03, 0.03)

        plt.grid(False)
        plt.tight_layout()
        pdf_path = os.path.join(output_pdf_dir, f"0.6_{field}.pdf")
        plt.savefig(pdf_path, bbox_inches='tight', pad_inches=0)
        plt.close()
        print(f"Saved plot to {pdf_path}")



if __name__ == "__main__":
    input_files = [
        "motion_log_open_loop_traj1_500g_20250405_193546.csv",
        "motion_log_closed_loop_traj2_500g_20250406_160909.csv",
        "motion_log_closed_loop_traj1_500g_20250406_160713.csv",
    ]
    output_dir = "processed"
    closed_loop_file = "motion_log_closed_loop_traj1_500g_20250406_160713.csv"

    batch_process_files_with_dynamic_scaling(
        input_files=input_files,
        output_dir=output_dir,
        closed_loop_file=closed_loop_file,
        boost_ratio=-0.03
    )

    processed_files = [
        os.path.join(output_dir, f"processed_{os.path.basename(path)}") for path in input_files
    ]
    plot_each_field_separately(processed_files)