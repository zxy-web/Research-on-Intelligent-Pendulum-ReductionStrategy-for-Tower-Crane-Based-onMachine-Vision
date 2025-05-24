import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimSun']  # 指定默认字体为宋体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 1. 画你的实际记录数据
file_path_real = "output_csv/all_20250414_100436.csv"
df_real = pd.read_csv(file_path_real)

columns_to_plot = ['x', 'angle', 'theta1', 'theta2']

for col in columns_to_plot:
    plt.figure(figsize=(10, 4))
    plt.plot(df_real["timestamp"], df_real[col], label=f"实际 {col}", linewidth=2)
    plt.xlabel("时间 (s)")
    plt.ylabel(col)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

# 2. 单独画 custom_traj_trajectory.csv
file_path_traj = "custom_traj_trajectory.csv"
df_traj = pd.read_csv(file_path_traj)

# 检查一下列名
print("custom_traj_trajectory.csv 列名：", df_traj.columns.tolist())

# 确保列名是 reference_position (m) 和 actual_position (m)
if "reference_position (m)" in df_traj.columns and "actual_position (m)" in df_traj.columns:
    plt.figure(figsize=(10, 4))
    plt.plot(df_traj["time (s)"], df_traj["reference_position (m)"], 'k--', label="参考轨迹 (reference)", linewidth=2)
    plt.plot(df_traj["time (s)"], df_traj["actual_position (m)"], color='#1f77b4', label="实际轨迹 (actual)", linewidth=2)
    plt.xlabel("时间 (s)")
    plt.ylabel("绳长 (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
else:
    print("错误：找不到 reference_position (m) 或 actual_position (m) 这两列，请检查文件！")
