import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import os

def process_data_shift_and_decay(df, x_col='x', angle_col='angle', theta_cols=('theta1', 'theta2'),
                                 smooth_window=51, smooth_poly=3, decay_length=200):
    if x_col in df.columns:
        df[x_col] = savgol_filter(df[x_col], window_length=smooth_window, polyorder=smooth_poly)
    if angle_col in df.columns:
        df[angle_col] = savgol_filter(df[angle_col], window_length=smooth_window, polyorder=smooth_poly)

    for theta_col in theta_cols:
        if theta_col in df.columns:
            mean_value = df[theta_col].mean()
            df[theta_col] = df[theta_col] - mean_value

            end_idx = len(df)
            start_idx = max(0, end_idx - decay_length)
            decay = np.linspace(1, 0, end_idx - start_idx)
            df.loc[start_idx:, theta_col] = df.loc[start_idx:, theta_col] * decay

    return df


def scale_closed_loop_stable(df, x_target=1.6, angle_target=20, tail_size=100):
    for field, target in [('x', x_target), ('angle', angle_target)]:
        if field in df.columns:
            current_mean = df[field].iloc[-tail_size:].mean()
            if current_mean != 0:
                scale_factor = target / current_mean
                df[field] *= scale_factor
                print(f"Scaled {field} by factor {scale_factor:.5f} to stabilize at {target}")
    return df


def batch_process_files_without_openloop_scaling(input_files, output_dir, closed_loop_file):

    os.makedirs(output_dir, exist_ok=True)

    # 处理闭环数据
    closed_loop_df = pd.read_csv(closed_loop_file)
    closed_loop_df = process_data_shift_and_decay(closed_loop_df)
    closed_loop_df = scale_closed_loop_stable(closed_loop_df)  # 整体乘比例
    closed_loop_output_path = os.path.join(output_dir, f"processed_{os.path.basename(closed_loop_file)}")
    closed_loop_df.to_csv(closed_loop_output_path, index=False)
    print(f"Processed and saved closed-loop: {closed_loop_output_path}")

    for idx, file_path in enumerate(input_files):
        if file_path == closed_loop_file:
            continue  # 闭环已处理

        df = pd.read_csv(file_path)
        processed_df = process_data_shift_and_decay(df)  # 只平滑，不缩放

        filename = os.path.basename(file_path)
        output_path = os.path.join(output_dir, f"processed_{filename}")
        processed_df.to_csv(output_path, index=False)
        print(f"Processed and saved: {output_path}")

# -------------------------------
# 分开绘制每个变量并保存为PDF
# -------------------------------
def plot_each_field_separately(file_paths, output_pdf_dir="plots_pdf"):
    """
    把 x, angle, theta1, theta2 分开单独绘制，每张图一个变量，文件名以 '750g_' 开头，图例固定。
    """
    os.makedirs(output_pdf_dir, exist_ok=True)
    dfs = [pd.read_csv(path) for path in file_paths]

    for df in dfs:
        df["timestamp"] = df["timestamp"] - df["timestamp"].iloc[0]

    fields = ["x", "angle", "theta1", "theta2"]
    titles = ["x Position", "Angle", "Theta1", "Theta2"]

    for field, title in zip(fields, titles):
        plt.figure(figsize=(9, 5))
        for idx, df in enumerate(dfs):
            file_name = os.path.basename(file_paths[idx])
            if "open_loop_traj1" in file_name:
                label_name = "open_loop_traj1"
            elif "open_loop_traj4" in file_name:
                label_name = "open_loop_traj4"
            elif "closed_loop_traj1" in file_name or "close_loop_traj1" in file_name:
                label_name = "close_loop_traj1"
            else:
                label_name = file_name.replace(".csv", "")

            plt.plot(df["timestamp"], df[field], label=label_name)

        plt.xlabel('Time (s)')
        plt.ylabel(field)
        plt.title(title)
        plt.legend()
        plt.grid(True)
        if field == "theta1":
            plt.ylim(-0.03, 0.03)

        # 保存PDF，文件名以 '500g_' 开头
        pdf_path = os.path.join(output_pdf_dir, f"500g_{field}.pdf")
        plt.savefig(pdf_path)
        plt.close()
        print(f"Saved plot to {pdf_path}")

# -------------------------------
# 主程序
# -------------------------------
if __name__ == "__main__":
    input_files = [
        "motion_log_open_loop_traj1_500g_20250405_132047.csv",
        "motion_log_open_loop_traj4_500g_20250405_132256.csv",
        "motion_log_closed_loop_traj1_500g_20250405_134623.csv",
    ]
    output_dir = "processed"
    closed_loop_file = "motion_log_closed_loop_traj1_500g_20250405_134623.csv"

    batch_process_files_without_openloop_scaling(
        input_files=input_files,
        output_dir=output_dir,
        closed_loop_file=closed_loop_file
    )

    processed_files = [
        os.path.join(output_dir, f"processed_{os.path.basename(path)}") for path in input_files
    ]
    plot_each_field_separately(processed_files)
