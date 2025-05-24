import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
# 1. 设置中文字体（确保系统中有该字体）
plt.rcParams['font.sans-serif'] = ['SimSun']  # 指定默认字体为宋体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
# --- 五次多项式轨迹生成函数 ---
def quintic_polynomial(t0, tf, p0, pf, v0=0, vf=0, a0=0, af=0):
    M = np.array([
        [1, t0, t0**2, t0**3, t0**4, t0**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
    ])
    b = np.array([p0, v0, a0, pf, vf, af])
    coeffs = np.linalg.solve(M, b)
    return coeffs

# --- 给定轨迹系数和时间，生成轨迹位置 ---
def generate_trajectory(coeffs, time_steps):
    trajectory = []
    for t in time_steps:
        t_pow = np.array([1, t, t**2, t**3, t**4, t**5])
        position = np.dot(coeffs, t_pow)
        trajectory.append(position)
    return trajectory

# --- PID 控制器 ---
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

# --- 封装好的总函数 ---
def generate_and_save_trajectory(down_start=0.6, down_end=0.2, down_time=10,
                                 hold_time=30,
                                 up_start=0.2, up_end=0.6, up_time=10,
                                 filename_prefix='trajectory',
                                 add_sparse_noise=False, noise_std=0.002, noise_rate=0.03):
    # 时间段定义
    time_steps1 = np.linspace(0, down_time, 200)
    time_steps_hold = np.linspace(down_time, down_time + hold_time, 300)
    time_steps2 = np.linspace(down_time + hold_time, down_time + hold_time + up_time, 200)

    coeffs1 = quintic_polynomial(0, down_time, down_start, down_end)
    coeffs2 = quintic_polynomial(down_time + hold_time, down_time + hold_time + up_time, up_start, up_end)

    reference_positions1 = generate_trajectory(coeffs1, time_steps1)
    reference_positions_hold = np.ones_like(time_steps_hold) * down_end
    reference_positions2 = generate_trajectory(coeffs2, time_steps2)

    # 合并时间与参考轨迹
    time_steps = np.concatenate((time_steps1, time_steps_hold, time_steps2))
    reference_positions = np.concatenate((reference_positions1, reference_positions_hold, reference_positions2))

    # PID 控制器
    pid1 = PIDController(Kp=2.0, Ki=0.1, Kd=0.3)
    pid2 = PIDController(Kp=2.0, Ki=0.1, Kd=0.3)

    # 初始实际状态
    actual_positions = []
    actual_position = down_start
    actual_velocity = 0

    # 控制过程
    for i in range(1, len(time_steps)):
        t = time_steps[i]

        if 0 <= t <= down_time:
            error = reference_positions[i] - actual_position
            velocity_command = pid1.update(error)
        elif down_time < t < down_time + hold_time:
            velocity_command = 0
            actual_velocity = 0
        elif down_time + hold_time <= t <= down_time + hold_time + up_time:
            error = reference_positions[i] - actual_position
            velocity_command = pid2.update(error)
        else:
            velocity_command = 0

        dt = time_steps[i] - time_steps[i-1]
        actual_velocity += velocity_command * dt
        actual_position += actual_velocity * dt

        actual_positions.append(actual_position)

    # 添加稀疏噪声
    if add_sparse_noise:
        for i in range(len(actual_positions)):
            if np.random.rand() < noise_rate:
                actual_positions[i] += np.random.normal(0, noise_std)

    # 保存为 CSV 文件
    df = pd.DataFrame({
        'time (s)': time_steps[1:],
        'reference_position (m)': reference_positions[1:],
        'actual_position (m)': actual_positions
    })
    df.to_csv(f'{filename_prefix}_trajectory.csv', index=False)

    # 绘制图
    plt.figure(figsize=(10, 4))
    plt.plot(time_steps, reference_positions, label='参考轨迹', linestyle='--', color='orange')
    plt.plot(time_steps[1:], actual_positions, label='实际轨迹', color='blue')
    plt.xlabel("时间 (s)")
    plt.ylabel("绳长 (m)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    #plt.show()

    #print(f"CSV文件已保存: {filename_prefix}_trajectory.csv")

# --- 调用例子 ---
generate_and_save_trajectory(down_start=0.4, down_end=0.2, down_time=10, hold_time=30, up_start=0.2, up_end=0.4, up_time=10, filename_prefix='custom_traj', add_sparse_noise=True, noise_std=0.002, noise_rate=0.1)
