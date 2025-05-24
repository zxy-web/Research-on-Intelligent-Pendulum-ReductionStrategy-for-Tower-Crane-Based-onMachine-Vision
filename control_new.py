import sys
import cv2
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLabel, QPushButton, QGroupBox,
                             QDoubleSpinBox, QTextEdit, QGridLayout, QComboBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QImage, QPixmap
from collections import deque, defaultdict
import math
import serial
import threading
import time
from typing import Optional, Tuple
from ultralytics import YOLO
from trajectory_generator import TrajectoryGenerator
import os
import glob

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"  # 临时绕过
import pandas as pd
from plot_1000 import process_data_shift_and_decay
from scipy.signal import savgol_filter
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import (QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLabel, QPushButton)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class DataCurveWindow(QMainWindow):
    def __init__(self, traj_index: int, mode_prefix: str, processed_file_path: str, logging_start_time: float):
        super().__init__()
        self.traj_index = traj_index
        self.mode_prefix = mode_prefix
        self.processed_file_path = processed_file_path
        self.logging_start_time = logging_start_time

        self.setWindowTitle("数据曲线显示")
        self.setGeometry(100, 100, 1200, 800)

        # 创建主布局
        main_layout = QHBoxLayout()

        # 左侧布局：图片和按钮
        left_layout = QVBoxLayout()
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)

        # 加载图片
        pixmap = QPixmap("实验平台.png")
        scaled_pixmap = pixmap.scaled(200, 200, Qt.KeepAspectRatio)
        self.image_label.setPixmap(scaled_pixmap)
        left_layout.addWidget(self.image_label)

        # 退出按钮
        self.exit_button = QPushButton("退出")
        self.exit_button.clicked.connect(self.close)
        left_layout.addWidget(self.exit_button)

        # 右侧布局：数据图表（4个子图）
        right_layout = QVBoxLayout()

        # Matplotlib绘图设置（4个子图）
        self.fig, self.axs = plt.subplots(4, 1, figsize=(6, 10))
        self.canvas = FigureCanvas(self.fig)
        right_layout.addWidget(self.canvas)

        # 布局整合
        main_layout.addLayout(left_layout, 1)
        main_layout.addLayout(right_layout, 3)

        # 设置窗口的中央部件
        central_widget = QWidget(self)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # 启动定时器，每0.5秒自动更新一次
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.auto_update_data)
        self.timer.start(500)

    def auto_update_data(self):
        current_duration = time.time() - self.logging_start_time

        if not os.path.exists(self.processed_file_path):
            print(f"文件不存在: {self.processed_file_path}")
            return

        try:
            # 加载数据
            df = pd.read_csv(self.processed_file_path)

            # 截取数据到当前记录时间
            df_segment = df[df["timestamp"] <= current_duration]

            if df_segment.empty:
                print("当前时间段内无数据")
                return

            time_data = df_segment["timestamp"]
            x = df_segment["x"]
            angle = df_segment["angle"]
            theta1 = df_segment["theta1"]
            theta2 = df_segment["theta2"]

            # 清除旧图
            self.fig.clf()
            self.axs = self.fig.subplots(4, 1)

            # 绘图数据准备
            data_list = [
                (x, "小车位移", "位移 (m)"),
                (angle, "控制器角度", "角度 (rad)"),
                (theta1, "视觉方向角度 θ1", "角度 (rad)"),
                (theta2, "视觉方向角度 θ2", "角度 (rad)")
            ]

            for ax, (data, title, ylabel) in zip(self.axs, data_list):
                ax.plot(time_data, data, label=title, color='tab:blue')
                ax.set_title(title)
                ax.set_xlabel("时间 (s)")
                ax.set_ylabel(ylabel)
                ax.grid(True)
                ax.legend()

            self.fig.tight_layout()
            self.canvas.draw()

        except Exception as e:
            print(f"数据更新失败: {e}")


class DataProcessor:
    def __init__(self):
        self.angle_value = None
        self.x_value = None
        self.phi_value = None
        self.lock = threading.Lock()
        self.angle_history = deque(maxlen=10)
        self.initial_angle = 0.0
        self.x_initial = 0.0

    def process_data(self, data: str):
        with self.lock:
            if data.startswith("Angle:"):
                angle_str = data.split(":")[1]
                current_angle = float(angle_str)

                if not hasattr(self, 'initial_angle_calibrated'):
                    self.angle_history.append(current_angle)

                    if len(self.angle_history) == 5:
                        if all(math.isclose(x, current_angle, abs_tol=0.5) for x in self.angle_history):
                            self.initial_angle = current_angle
                            self.initial_angle_calibrated = True
                            print(f"初始角度校准完成: {self.initial_angle:.2f}°")
                        else:
                            self.angle_history.clear()

                if hasattr(self, 'initial_angle_calibrated'):
                    self.angle_value = current_angle - self.initial_angle
                    print(f"相对角度: {self.angle_value:.2f}° (原始: {current_angle:.2f}°)")
                else:
                    self.angle_value = current_angle
                    print(f"校准中... 当前原始角度: {current_angle:.2f}°")

            elif data.startswith("x:"):
                x_str = data.split(":")[1]
                self.x_value = float(x_str) / 100
                print(f"X位置: {self.x_value:.2f}m")

    def get_values(self) -> Tuple[Optional[float], Optional[float], float]:
        with self.lock:
            x_relative = self.x_value - self.x_initial if self.x_value is not None else None
            return (
                self.angle_value if hasattr(self, 'initial_angle_calibrated') else None,
                x_relative,
                0.00
            )

    def recalibrate(self):
        with self.lock:
            if hasattr(self, 'initial_angle_calibrated'):
                del self.initial_angle_calibrated

            self.angle_history.clear()

            if self.angle_value is not None:
                self.initial_angle += self.angle_value
            if self.x_value is not None:
                self.x_initial = self.x_value

            print(f"重新校准完成：当前角度设置为0°, X位置设置为0m")

class VisionSystem:
    def __init__(self, model_path):
        self.model = YOLO(model_path)
        self.history = deque(maxlen=10)
        self.balance_pos = None
        self.stabilized = False
        self.current_offset = (0, 0)
        self.current_angles = (0, 0)
        self.lock = threading.Lock()

    def process_frame(self, frame):
        results = self.model(frame, verbose=False)
        boxes = results[0].boxes.xyxy.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy()
        confidences = results[0].boxes.conf.cpu().numpy()

        annotated_frame = results[0].plot()

        for box, cls, conf in zip(boxes, classes, confidences):
            x_center = int((box[0] + box[2]) / 2)
            y_center = int((box[1] + box[3]) / 2)
            current_pos = (x_center, y_center)

            self.history.append(current_pos)
            pos_count = defaultdict(int)
            for pos in self.history:
                pos_count[pos] += 1

            for pos, count in pos_count.items():
                if count >= 6 and not self.stabilized:
                    with self.lock:
                        self.balance_pos = pos
                        self.stabilized = True
                    print(f"平衡位置设置: {self.balance_pos}")
                    break

            cross_size = 10
            cv2.line(annotated_frame, (x_center - cross_size, y_center),
                     (x_center + cross_size, y_center), (0, 0, 255), 2)
            cv2.line(annotated_frame, (x_center, y_center - cross_size),
                     (x_center, y_center + cross_size), (0, 0, 255), 2)

            if self.stabilized:
                with self.lock:
                    dx = x_center - self.balance_pos[0]
                    dy = y_center - self.balance_pos[1]
                    self.current_offset = (dx, dy)

                    right_side = dx * 0.0680 / 44
                    n_rad = math.atan(right_side)
                    left_side = dy * 0.0680 / 44
                    w_rad = math.atan(left_side)
                    self.current_angles = (n_rad, w_rad)

                cv2.putText(annotated_frame, f"偏移: ({dx}, {dy})", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(annotated_frame, f"角度: ({n_rad:.4f}, {w_rad:.4f})", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return annotated_frame

    def get_vision_data(self):
        with self.lock:
            return {
                'offset': self.current_offset,
                'angles': self.current_angles,
                'stabilized': self.stabilized
            }

class SerialThread(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, port, baudrate, processor):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.processor = processor
        self.serial = None
        self.running = True

    def run(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"已连接到串口设备 {self.port}")

            while self.running:
                if self.serial.in_waiting > 0:
                    data = self.serial.readline().decode('utf-8').strip()
                    self.data_received.emit(data)
                    self.processor.process_data(data)
                time.sleep(0.01)

        except Exception as e:
            self.data_received.emit(f"串口错误: {str(e)}")
        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()

    def stop(self):
        self.running = False
        self.wait()

    def send_data(self, data):
        if self.serial and self.serial.is_open:
            try:
                if isinstance(data, str):
                    self.serial.write(data.encode())
                else:
                    self.serial.write(data)
            except Exception as e:
                self.data_received.emit(f"发送失败: {str(e)}")



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("视觉平衡控制系统")
        self.setGeometry(100, 100, 1200, 800)

        # Initialize system components
        self.data_processor = DataProcessor()
        self.vision = VisionSystem(r"runs/train/exp4/weights/best.pt")
        self.serial_thread = None

        # Target values
        self.x_target = 1.6
        self.phi_target = 20.0

        # PID parameters (now as floats)
        self.Kp_x = 450.0
        self.Ki_x = 5.0
        self.Kd_x = 0.0
        self.Kp_phi = 35.0
        self.Ki_phi = 3.0
        self.Kd_phi = 0.0
        self.K_theta1 = 200.0  # 摆角反馈增益
        self.K_theta2 = 50.0  # 摆角反馈增益

        # Integral terms
        self.integral_x = 0.0
        self.integral_phi = 0.0
        self.last_time = time.time()

        self.logging_timer = QTimer(self)
        self.logging_timer.timeout.connect(self.auto_stop_logging)

        # Manual control mode
        self.manual_mode = True
        self.manual_commands = {
            'forward': 0x61,
            'backward': 0x62,
            'left': 0x63,
            'right': 0x64,
            'up': 0x65,
            'down': 0x66,
            'stop': 0x67
        }

        # Data logging
        self.logging_active = False
        self.log_file = None
        self.log_start_time = 0
        self.log_headers_written = False

        # Initialize UI
        self.init_ui()

        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.log_message("无法打开摄像头")

        # Timer for video frame updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def init_ui(self):
        # Main layout
        main_layout = QHBoxLayout()

        # Left side - video display and controls
        left_layout = QVBoxLayout()

        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        left_layout.addWidget(self.video_label)

        # Control buttons
        control_group = QGroupBox("系统控制")
        control_layout = QHBoxLayout()

        self.connect_btn = QPushButton("连接串口")
        self.connect_btn.clicked.connect(self.toggle_serial_connection)
        control_layout.addWidget(self.connect_btn)

        self.start_btn = QPushButton("开始自动控制")
        self.start_btn.clicked.connect(self.toggle_control)
        self.start_btn.setEnabled(False)
        control_layout.addWidget(self.start_btn)

        self.mode_switch_btn = QPushButton("切换到闭环控制")
        self.mode_switch_btn.clicked.connect(self.toggle_control_mode)
        control_layout.addWidget(self.mode_switch_btn)

        self.open_loop_mode = True

        self.recalibrate_btn = QPushButton("重新校准&清除轨迹")
        self.recalibrate_btn.clicked.connect(self.recalibrate_system)
        control_layout.addWidget(self.recalibrate_btn)

        self.data_curve_btn = QPushButton("数据曲线显示")
        self.data_curve_btn.clicked.connect(self.open_data_curve_window)
        control_layout.addWidget(self.data_curve_btn)

        control_group.setLayout(control_layout)
        left_layout.addWidget(control_group)

        # Manual control group
        manual_group = QGroupBox("手动控制")
        manual_layout = QGridLayout()

        self.forward_btn = QPushButton("前进 (A)")
        self.backward_btn = QPushButton("后退 (B)")
        self.left_btn = QPushButton("左转 (C)")
        self.right_btn = QPushButton("右转 (D)")
        self.up_btn = QPushButton("上升 (E)")
        self.down_btn = QPushButton("下降 (F)")
        self.stop_btn = QPushButton("停止 (G)")

        self.forward_btn.setShortcut("A")
        self.backward_btn.setShortcut("B")
        self.left_btn.setShortcut("C")
        self.right_btn.setShortcut("D")
        self.up_btn.setShortcut("E")
        self.down_btn.setShortcut("F")
        self.stop_btn.setShortcut("G")

        self.forward_btn.clicked.connect(lambda: self.send_manual_command('forward'))
        self.backward_btn.clicked.connect(lambda: self.send_manual_command('backward'))
        self.left_btn.clicked.connect(lambda: self.send_manual_command('left'))
        self.right_btn.clicked.connect(lambda: self.send_manual_command('right'))
        self.up_btn.clicked.connect(lambda: self.send_manual_command('up'))
        self.down_btn.clicked.connect(lambda: self.send_manual_command('down'))
        self.stop_btn.clicked.connect(lambda: self.send_manual_command('stop'))

        manual_layout.addWidget(self.forward_btn, 0, 1)
        manual_layout.addWidget(self.left_btn, 1, 0)
        manual_layout.addWidget(self.stop_btn, 1, 1)
        manual_layout.addWidget(self.right_btn, 1, 2)
        manual_layout.addWidget(self.backward_btn, 2, 1)
        manual_layout.addWidget(self.up_btn, 0, 3)
        manual_layout.addWidget(self.down_btn, 2, 3)

        manual_group.setLayout(manual_layout)
        left_layout.addWidget(manual_group)

        # Right side - parameters and status
        right_layout = QVBoxLayout()

        # Target settings
        target_group = QGroupBox("目标设置")
        target_layout = QVBoxLayout()

        self.x_target_spin = QDoubleSpinBox()
        self.x_target_spin.setRange(-10.0, 10.0)
        self.x_target_spin.setSingleStep(0.1)
        self.x_target_spin.setValue(self.x_target)
        self.x_target_spin.setPrefix("X目标: ")
        self.x_target_spin.valueChanged.connect(self.update_x_target)
        target_layout.addWidget(self.x_target_spin)

        self.trajectory_combo = QComboBox()
        self.trajectory_combo.addItems([
            "Trajectory 1 (Smooth Cosine)",
            "Trajectory 2 (Bang-Bang)",
            "Trajectory 3 (Linear Ramp)",
            "Trajectory 4 (S-curve)",
            "Trajectory 5 (S-curve with v0)"
        ])
        self.trajectory_combo.setCurrentIndex(0)
        self.trajectory_combo.currentIndexChanged.connect(self.update_trajectory_choice)
        target_layout.addWidget(self.trajectory_combo)

        self.selected_trajectory = 0

        self.phi_target_spin = QDoubleSpinBox()
        self.phi_target_spin.setRange(-180.0, 180.0)
        self.phi_target_spin.setSingleStep(1.0)
        self.phi_target_spin.setValue(self.phi_target)
        self.phi_target_spin.setPrefix("角度目标: ")
        self.phi_target_spin.valueChanged.connect(self.update_phi_target)
        target_layout.addWidget(self.phi_target_spin)

        target_group.setLayout(target_layout)
        right_layout.addWidget(target_group)

        # PID parameters (now using QDoubleSpinBox)
        pid_group = QGroupBox("PID参数")
        pid_layout = QVBoxLayout()

        self.kp_x_spin = QDoubleSpinBox()
        self.kp_x_spin.setRange(0, 1000)
        self.kp_x_spin.setSingleStep(0.1)
        self.kp_x_spin.setValue(self.Kp_x)
        self.kp_x_spin.setPrefix("Kp_x: ")
        self.kp_x_spin.valueChanged.connect(self.update_kp_x)
        pid_layout.addWidget(self.kp_x_spin)

        self.ki_x_spin = QDoubleSpinBox()
        self.ki_x_spin.setRange(0, 1000)
        self.ki_x_spin.setSingleStep(0.01)
        self.ki_x_spin.setValue(self.Ki_x)
        self.ki_x_spin.setPrefix("Ki_x: ")
        self.ki_x_spin.valueChanged.connect(self.update_ki_x)
        pid_layout.addWidget(self.ki_x_spin)

        self.kd_x_spin = QDoubleSpinBox()
        self.kd_x_spin.setRange(0, 1000)
        self.kd_x_spin.setSingleStep(0.1)
        self.kd_x_spin.setValue(self.Kd_x)
        self.kd_x_spin.setPrefix("Kd_x: ")
        self.kd_x_spin.valueChanged.connect(self.update_kd_x)
        pid_layout.addWidget(self.kd_x_spin)

        self.kp_phi_spin = QDoubleSpinBox()
        self.kp_phi_spin.setRange(0, 1000)
        self.kp_phi_spin.setSingleStep(0.1)
        self.kp_phi_spin.setValue(self.Kp_phi)
        self.kp_phi_spin.setPrefix("Kp_phi: ")
        self.kp_phi_spin.valueChanged.connect(self.update_kp_phi)
        pid_layout.addWidget(self.kp_phi_spin)

        self.ki_phi_spin = QDoubleSpinBox()
        self.ki_phi_spin.setRange(0, 1000)
        self.ki_phi_spin.setSingleStep(0.01)
        self.ki_phi_spin.setValue(self.Ki_phi)
        self.ki_phi_spin.setPrefix("Ki_phi: ")
        self.ki_phi_spin.valueChanged.connect(self.update_ki_phi)
        pid_layout.addWidget(self.ki_phi_spin)

        self.kd_phi_spin = QDoubleSpinBox()
        self.kd_phi_spin.setRange(0, 1000)
        self.kd_phi_spin.setSingleStep(0.1)
        self.kd_phi_spin.setValue(self.Kd_phi)
        self.kd_phi_spin.setPrefix("Kd_phi: ")
        self.kd_phi_spin.valueChanged.connect(self.update_kd_phi)
        pid_layout.addWidget(self.kd_phi_spin)

        self.k_theta1_spin = QDoubleSpinBox()
        self.k_theta1_spin.setRange(0, 1000)
        self.k_theta1_spin.setSingleStep(10)
        self.k_theta1_spin.setValue(self.K_theta1)
        self.k_theta1_spin.setPrefix("Kθ1: ")
        self.k_theta1_spin.valueChanged.connect(self.update_k_theta1)
        pid_layout.addWidget(self.k_theta1_spin)

        self.k_theta2_spin = QDoubleSpinBox()
        self.k_theta2_spin.setRange(0, 1000)
        self.k_theta2_spin.setSingleStep(10)
        self.k_theta2_spin.setValue(self.K_theta2)
        self.k_theta2_spin.setPrefix("Kθ2: ")
        self.k_theta2_spin.valueChanged.connect(self.update_k_theta2)
        pid_layout.addWidget(self.k_theta2_spin)

        pid_group.setLayout(pid_layout)
        right_layout.addWidget(pid_group)

        # Data logging controls
        logging_group = QGroupBox("数据记录")
        logging_layout = QVBoxLayout()

        self.logging_btn = QPushButton("开始记录")
        self.logging_btn.clicked.connect(self.toggle_logging)
        logging_layout.addWidget(self.logging_btn)

        self.logging_status_label = QLabel("记录状态: 未激活")
        logging_layout.addWidget(self.logging_status_label)

        logging_group.setLayout(logging_layout)
        right_layout.addWidget(logging_group)

        # Status display
        status_group = QGroupBox("系统状态")
        status_layout = QVBoxLayout()

        self.serial_status_label = QLabel("串口状态: 未连接")
        status_layout.addWidget(self.serial_status_label)

        self.mode_status_label = QLabel("当前模式: 手动")
        status_layout.addWidget(self.mode_status_label)

        self.vision_status_label = QLabel("视觉状态: 不稳定")
        status_layout.addWidget(self.vision_status_label)

        self.angle_label = QLabel("当前角度: -")
        status_layout.addWidget(self.angle_label)

        self.initial_angle_label = QLabel("初始角度: -")
        status_layout.addWidget(self.initial_angle_label)

        self.x_label = QLabel("当前X位置: -")
        status_layout.addWidget(self.x_label)

        self.offset_label = QLabel("视觉偏移: -")
        status_layout.addWidget(self.offset_label)

        self.pwm_label = QLabel("控制输出: -")
        status_layout.addWidget(self.pwm_label)

        self.last_command_label = QLabel("最后指令: -")
        status_layout.addWidget(self.last_command_label)

        status_group.setLayout(status_layout)
        right_layout.addWidget(status_group)

        # Log area
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        right_layout.addWidget(self.log_text)

        # Combine layouts
        main_layout.addLayout(left_layout, 60)
        main_layout.addLayout(right_layout, 40)

        # Central widget
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def open_data_curve_window(self):
        traj_index = self.selected_trajectory + 1
        mode_prefix = "open_loop" if self.open_loop_mode else "closed_loop"

        processed_pattern = f"processed/processed_motion_log_{mode_prefix}_traj{traj_index}_*.csv"
        matched_files = glob.glob(processed_pattern)

        if not matched_files:
            self.log_message("未找到对应的已处理数据文件！")
            return

        # 选择最新的处理文件
        latest_processed_file = max(matched_files, key=os.path.getctime)

        if hasattr(self, 'log_start_time'):
            self.data_curve_window = DataCurveWindow(traj_index, mode_prefix, latest_processed_file,
                                                     self.log_start_time)
            self.data_curve_window.show()
        else:
            self.log_message("请先开始记录数据再显示曲线！")

    def update_k_theta1(self, value):
        self.K_theta1 = float(value)
        self.log_message(f"Kθ1更新为: {value:.2f}")

    def update_k_theta2(self, value):
        self.K_theta2 = float(value)
        self.log_message(f"Kθ2更新为: {value:.2f}")

    def toggle_serial_connection(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
            self.serial_thread = None
            self.connect_btn.setText("连接串口")
            self.start_btn.setEnabled(False)
            self.serial_status_label.setText("串口状态: 未连接")
            self.log_message("串口连接已关闭")
        else:
            self.serial_thread = SerialThread("COM9", 9600, self.data_processor)
            self.serial_thread.data_received.connect(self.update_serial_data)
            self.serial_thread.start()
            self.connect_btn.setText("断开串口")
            self.start_btn.setEnabled(True)
            self.serial_status_label.setText("串口状态: 已连接")
            self.log_message("正在连接串口...")

    def toggle_control(self):
        if self.start_btn.text() == "开始自动控制":
            self.start_btn.setText("停止自动控制")
            self.log_message("自动控制已启动")
            self.manual_mode = False
            self.mode_status_label.setText("当前模式: 自动")
            self.integral_x = 0.0
            self.integral_phi = 0.0
            self.last_time = time.time()
        else:
            self.start_btn.setText("开始自动控制")
            self.log_message("自动控制已停止")
            self.manual_mode = True
            self.mode_status_label.setText("当前模式: 手动")
            self.send_manual_command('stop')

    def toggle_control_mode(self):
        self.open_loop_mode = not self.open_loop_mode
        if self.open_loop_mode:
            self.mode_switch_btn.setText("切换到闭环控制")
            self.log_message("已切换到 开环控制模式")
            self.mode_status_label.setText("当前控制模式: 开环")
        else:
            self.mode_switch_btn.setText("切换到开环控制")
            self.log_message("已切换到 闭环控制模式")
            self.mode_status_label.setText("当前控制模式: 闭环")

    def recalibrate_system(self):
        if hasattr(self.data_processor, 'initial_angle_calibrated'):
            del self.data_processor.initial_angle_calibrated
        self.data_processor.angle_history.clear()

        angle, x_value, _ = self.data_processor.get_values()
        if angle is not None:
            self.data_processor.initial_angle = angle + self.data_processor.initial_angle
        if x_value is not None:
            self.data_processor.x_initial = x_value

        if hasattr(self, 'trajectory_generated'):
            del self.trajectory_generated

        self.log_message("✅ 已重新校准角度和位置，清除旧轨迹，请重新选择轨迹并生成")

    def update_trajectory_choice(self, index):
        self.selected_trajectory = index
        self.log_message(f"轨迹选择更新为: Trajectory {index + 1}")

    def send_manual_command(self, command):
        if self.serial_thread and self.serial_thread.isRunning():
            cmd_byte = self.manual_commands[command]
            try:
                self.serial_thread.send_data(bytes([cmd_byte]))
                self.log_message(f"发送手动指令: {command} (0x{cmd_byte:02X})")
                self.last_command_label.setText(f"最后指令: {command} (0x{cmd_byte:02X})")

                if command == 'stop':
                    self.manual_mode = True
                    self.mode_status_label.setText("当前模式: 手动")
                    self.start_btn.setText("开始自动控制")
                    self.log_message("已切换至手动模式")

            except Exception as e:
                self.log_message(f"手动指令失败: {str(e)}")
        else:
            self.log_message("串口未连接，无法发送手动指令")

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            annotated_frame = self.vision.process_frame(frame)

            rgb_image = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.video_label.setPixmap(pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio))

            vision_data = self.vision.get_vision_data()
            if vision_data['stabilized']:
                self.vision_status_label.setText("视觉状态: 稳定")
                self.offset_label.setText(f"视觉偏移: ({vision_data['offset'][0]:.1f}, {vision_data['offset'][1]:.1f})")
            else:
                self.vision_status_label.setText("视觉状态: 不稳定")
                self.offset_label.setText("视觉偏移: -")

            angle, x_value, initial_angle = self.data_processor.get_values()
            if angle is not None:
                self.angle_label.setText(f"当前角度: {angle:.2f}°")
            if x_value is not None:
                self.x_label.setText(f"当前X位置: {x_value:.2f}m")

            if not self.manual_mode and self.start_btn.text() == "停止自动控制" and angle is not None and x_value is not None:
                self.apply_control_logic(angle, x_value, vision_data)

    def apply_control_logic(self, angle, x_value, vision_data):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if not hasattr(self, 'trajectory_generated') or not self.trajectory_generated:
            T_total = 6.0
            dt_traj = 0.01
            distance = abs(self.x_target)
            angle_distance = abs(self.phi_target)

            if self.selected_trajectory == 0:
                self.t_traj, self.x_traj, self.v_traj, self.a_traj = TrajectoryGenerator.generate_trajectory_1(T_total,
                                                                                                               dt_traj,
                                                                                                               distance)
                self.t_traj_phi, self.phi_traj, self.v_phi_traj, self.a_phi_traj = TrajectoryGenerator.generate_trajectory_1(
                    T_total, dt_traj, angle_distance)
            elif self.selected_trajectory == 1:
                self.t_traj, self.x_traj, self.v_traj, self.a_traj = TrajectoryGenerator.generate_trajectory_2(T_total,
                                                                                                               dt_traj,
                                                                                                               distance)
                self.t_traj_phi, self.phi_traj, self.v_phi_traj, self.a_phi_traj = TrajectoryGenerator.generate_trajectory_2(
                    T_total, dt_traj, angle_distance)
            elif self.selected_trajectory == 2:
                self.t_traj, self.x_traj, self.v_traj, self.a_traj = TrajectoryGenerator.generate_trajectory_3(T_total,
                                                                                                               dt_traj,
                                                                                                               distance)
                self.t_traj_phi, self.phi_traj, self.v_phi_traj, self.a_phi_traj = TrajectoryGenerator.generate_trajectory_3(
                    T_total, dt_traj, angle_distance)
            elif self.selected_trajectory == 3:
                self.t_traj, self.x_traj, self.v_traj, self.a_traj = TrajectoryGenerator.generate_trajectory_4(T_total,
                                                                                                               dt_traj,
                                                                                                               distance)
                self.t_traj_phi, self.phi_traj, self.v_phi_traj, self.a_phi_traj = TrajectoryGenerator.generate_trajectory_4(
                    T_total, dt_traj, angle_distance)
            elif self.selected_trajectory == 4:
                self.t_traj, self.x_traj, self.v_traj, self.a_traj = TrajectoryGenerator.generate_trajectory_5(T_total,
                                                                                                               dt_traj,
                                                                                                               distance)
                self.t_traj_phi, self.phi_traj, self.v_phi_traj, self.a_phi_traj = TrajectoryGenerator.generate_trajectory_5(
                    T_total, dt_traj, angle_distance)

            self.control_start_time = time.time()
            self.trajectory_generated = True
            self.save_open_loop_trajectory()

        t_now = time.time() - self.control_start_time
        index = int(t_now / 0.01)
        if index >= len(self.x_traj):
            index = len(self.x_traj) - 1

        reference_x = self.x_traj[index]
        reference_a = self.a_traj[index]
        reference_phi = self.phi_traj[index]
        reference_phi_a = self.a_phi_traj[index]

        theta1 = vision_data['angles'][0] if vision_data['stabilized'] else 0.0
        theta2 = vision_data['angles'][1] if vision_data['stabilized'] else 0.0

        # ==== 角度滤波处理 ====
        alpha = 0.4  # 滤波系数

        if not hasattr(self, 'theta1_filtered'):
            self.theta1_filtered = theta1
        else:
            self.theta1_filtered = alpha * theta1 + (1 - alpha) * self.theta1_filtered

        if not hasattr(self, 'theta2_filtered'):
            self.theta2_filtered = theta2
        else:
            self.theta2_filtered = alpha * theta2 + (1 - alpha) * self.theta2_filtered

        # 用滤波后的theta
        theta1 = self.theta1_filtered
        theta2 = self.theta2_filtered

        if not self.open_loop_mode:
            theta1 *= 0.8
            theta2 *= 0.8

        if self.logging_active:
            self.log_motion_data(x_value, angle, theta1, theta2)

        if self.open_loop_mode:
            K_pwm_x = 1200.0
            K_pwm_phi = 2200.0

            pwm_x = reference_a * K_pwm_x
            pwm_phi = reference_phi_a * K_pwm_phi
        else:
            err_x = reference_x - x_value
            err_phi = reference_phi - angle

            self.integral_x += err_x * dt
            self.integral_phi += err_phi * dt

            self.integral_x = max(-100.0, min(100.0, self.integral_x))
            self.integral_phi = max(-100.0, min(100.0, self.integral_phi))

            if dt > 0:
                d_angle = theta1
                d_x = (x_value - getattr(self, 'last_x', x_value)) / dt
            else:
                d_angle = 0.0
                d_x = 0.0

            pwm_x = (self.Kp_x * err_x +
                     self.Ki_x * self.integral_x -
                     self.Kd_x * d_x +
                     self.K_theta1 * theta1)  # 摆角反馈！


            pwm_phi = (self.Kp_phi * err_phi +
                      self.Ki_phi * self.integral_phi -
                      self.Kd_phi * d_angle +
                       self.K_theta2 * theta2)

            self.last_x = x_value

        pwm_x_abs = max(0, min(100, int(round(abs(pwm_x)))))
        pwm_phi_abs = max(0, min(100, int(round(abs(pwm_phi)))))

        pwm_x = -pwm_x_abs if pwm_x < 0 else pwm_x_abs
        pwm_phi = -pwm_phi_abs if pwm_phi < 0 else pwm_phi_abs

        control_str = f"{pwm_x},{pwm_phi}\n"
        if self.serial_thread:
            self.serial_thread.send_data(control_str)
            self.pwm_label.setText(f"控制输出: {pwm_x}, {pwm_phi}")
            self.log_message(f"[{'开环' if self.open_loop_mode else '闭环'}] 发送PWM: {control_str.strip()}")

    def toggle_logging(self):
        if not self.logging_active:
            # 自动启动控制逻辑
            if self.start_btn.text() == "开始自动控制":
                self.toggle_control()  # 自动切换到自动控制模式

            timestamp = time.strftime("%Y%m%d_%H%M%S")
            mode = "open_loop" if self.open_loop_mode else "closed_loop"
            traj_num = self.selected_trajectory + 1

            # 保存当前记录文件路径（方便后续处理）
            self.log_file_path = f"motion_log_{mode}_traj{traj_num}_0.4mmmm_{timestamp}.csv"
            self.log_file = open(self.log_file_path, "w")
            self.log_start_time = time.time()
            self.log_headers_written = False
            self.logging_active = True

            self.logging_btn.setText("记录中...")
            self.logging_btn.setEnabled(False)
            self.logging_status_label.setText("记录状态: 激活")
            self.log_message(f"已开始数据记录 ({mode.replace('_', ' ')} Trajectory {traj_num}，50秒后自动停止)")
            self.logging_timer.start(50000)  # 50秒后自动停止
        else:
            # 停止数据记录
            if self.log_file:
                self.log_file.close()

            self.logging_active = False
            self.logging_btn.setText("开始记录")
            self.logging_btn.setEnabled(True)
            self.logging_status_label.setText("记录状态: 未激活")
            self.log_message("已停止数据记录")
            self.logging_timer.stop()

            # === 自动处理刚才保存的数据 ===
            try:
                if hasattr(self, 'log_file_path'):
                    self.process_and_save_logged_data(self.log_file_path)
            except Exception as e:
                self.log_message(f"数据处理失败: {e}")

    def write_log_headers(self):
        if self.log_file and not self.log_headers_written:
            self.log_file.write("timestamp,x,angle,theta1,theta2,x_target,phi_target,pwm_x,pwm_phi\n")
            self.log_headers_written = True

    def auto_stop_logging(self):
        if self.logging_active:
            # 如果当前是自动控制模式，则停止控制
            if self.start_btn.text() == "停止自动控制":
                self.toggle_control()
            self.toggle_logging()

    def log_motion_data(self, x, angle, theta1, theta2):
        if self.logging_active and self.log_file:
            self.write_log_headers()
            timestamp = time.time() - self.log_start_time
            pwm_info = self.pwm_label.text().replace("控制输出: ", "").split(", ")
            if len(pwm_info) == 2:
                pwm_x, pwm_phi = pwm_info
            else:
                pwm_x, pwm_phi = 0, 0
            self.log_file.write(
                f"{timestamp:.3f},{x:.4f},{angle:.4f},{theta1:.4f},{theta2:.4f},{self.x_target:.4f},{self.phi_target:.4f},{pwm_x},{pwm_phi}\n")
            self.log_file.flush()

    def update_serial_data(self, data):
        self.log_text.append(f"串口数据: {data}")

    def log_message(self, message):
        self.log_text.append(f"[{time.strftime('%H:%M:%S')}] {message}")

    def update_x_target(self, value):
        self.x_target = float(value)
        self.log_message(f"X目标更新为: {value:.2f}")

    def update_phi_target(self, value):
        self.phi_target = float(value)
        self.log_message(f"角度目标更新为: {value:.2f}")

    def update_kp_x(self, value):
        self.Kp_x = float(value)
        self.log_message(f"Kp_x更新为: {value:.2f}")

    def update_ki_x(self, value):
        self.Ki_x = float(value)
        self.log_message(f"Ki_x更新为: {value:.2f}")

    def update_kd_x(self, value):
        self.Kd_x = float(value)
        self.log_message(f"Kd_x更新为: {value:.2f}")

    def update_kp_phi(self, value):
        self.Kp_phi = float(value)
        self.log_message(f"Kp_phi更新为: {value:.2f}")

    def update_ki_phi(self, value):
        self.Ki_phi = float(value)
        self.log_message(f"Ki_phi更新为: {value:.2f}")

    def update_kd_phi(self, value):
        self.Kd_phi = float(value)
        self.log_message(f"Kd_phi更新为: {value:.2f}")

    def process_and_save_logged_data(self, file_path):
        if not file_path or not os.path.exists(file_path):
            self.log_message(f"找不到文件 {file_path}，无法处理")
            return

        df = pd.read_csv(file_path)
        filename = os.path.basename(file_path)

        processed_df = process_data_shift_and_decay(df, filename=filename)

        output_dir = "processed"
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, f"processed_{filename}")

        processed_df.to_csv(output_path, index=False)
        self.log_message(f"✅ 已保存处理后数据到 {output_path}")

    def closeEvent(self, event):
        if self.logging_active:
            self.toggle_logging()
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
        if self.cap.isOpened():
            self.cap.release()
        self.timer.stop()
        event.accept()

    def save_open_loop_trajectory(self):
        import csv
        import os

        os.makedirs("trajectories", exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        traj_num = self.selected_trajectory + 1
        filename = f"trajectories/open_loop_traj{traj_num}_new_real_{timestamp}.csv"
        min_length = min(len(self.t_traj), len(self.phi_traj))

        try:
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["t", "x", "v", "a", "phi", "phi_v", "phi_a"])

                for i in range(min_length):
                    writer.writerow([
                        self.t_traj[i],
                        self.x_traj[i],
                        self.v_traj[i],
                        self.a_traj[i],
                        self.phi_traj[i],
                        self.v_phi_traj[i],
                        self.a_phi_traj[i]
                    ])
            self.log_message(f"✅ 开环轨迹数据已保存到: {filename}")
        except Exception as e:
            self.log_message(f"❌ 保存轨迹数据失败: {e}")



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())