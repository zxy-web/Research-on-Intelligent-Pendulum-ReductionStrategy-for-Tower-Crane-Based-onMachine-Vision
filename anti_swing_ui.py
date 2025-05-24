import sys
import cv2
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLabel, QPushButton, QGroupBox,
                             QSpinBox, QDoubleSpinBox, QTextEdit, QGridLayout)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QImage, QPixmap
from collections import deque, defaultdict
import math
import serial
import threading
import time
from typing import Optional, Tuple
from ultralytics import YOLO


def sliding_mode_controller(x_value, theta_deg, theta1_deg, x_target, current_time, params):
    kp = params.get('kp', 360)
    kd = params.get('kd', 80)
    ks = params.get('ks', 60)
    gamma = params.get('gamma', 1.2)
    alpha = params.get('alpha', 2.0)
    lambda_rate = params.get('lambda_rate', 0.08)

    theta_rad = theta_deg * math.pi / 180
    theta1_rad = theta1_deg * math.pi / 180

    xi_x = x_value - gamma * theta_rad
    xi_x_dot = -gamma * theta1_rad

    lambda_d = x_target * (1 - math.exp(-lambda_rate * current_time))
    lambda_d_dot = lambda_rate * x_target * math.exp(-lambda_rate * current_time)

    e = xi_x - lambda_d
    e_dot = xi_x_dot - lambda_d_dot
    s = e_dot + alpha * e

    pwm_x = -kp * e - kd * e_dot - ks * math.tanh(10 * s)
    pwm_x = max(-100, min(100, int(pwm_x)))

    return pwm_x


class DataProcessor:
    def __init__(self):
        self.angle_value = None
        self.x_value = None
        self.phi_value = None
        self.lock = threading.Lock()
        self.angle_history = deque(maxlen=10)  # Store last 10 angle values
        self.initial_angle = 0  # Initial angle value

    def process_data(self, data):
        with self.lock:
            if data.startswith("Angle:"):
                angle_str = data.split(":")[1]
                current_angle = float(angle_str)
                self.angle_history.append(current_angle)

                # Check if all history values are same and we have 10 values
                if len(self.angle_history) == 10 and all(x == self.angle_history[0] for x in self.angle_history):
                    self.initial_angle = self.angle_history[0]
                    print(f"初始角度设置: {self.initial_angle}")

                self.angle_value = current_angle - self.initial_angle
                print(f"当前角度: {self.angle_value}")

            if data.startswith("x:"):
                x_str = data.split(":")[1]
                self.x_value = float(x_str)
                self.x_value = self.x_value / 100
                print(f"X值: {self.x_value}")

    def get_values(self) -> Tuple[Optional[float], Optional[float]]:
        with self.lock:
            return self.angle_value, self.x_value, 0.00


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

                    right_side = dx * 0.0446 / 28
                    n_rad = math.atan(right_side)
                    left_side = dy * 0.0446 / 28
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
                else:  # Assume bytes
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
        self.x_target = -1.6
        self.phi_target = -30.0

        # PID parameters
        self.Kp_x = 327
        self.Ki_x = 5
        self.Kd_x = 50
        self.Kp_phi = 20
        self.Ki_phi = 0
        self.Kd_phi = 60

        # Integral terms
        self.integral_x = 0
        self.integral_phi = 0
        self.last_time = time.time()

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

        control_group.setLayout(control_layout)
        left_layout.addWidget(control_group)

        # Manual control group
        manual_group = QGroupBox("手动控制")
        manual_layout = QGridLayout()

        # Manual control buttons
        self.forward_btn = QPushButton("前进 (A)")
        self.backward_btn = QPushButton("后退 (B)")
        self.left_btn = QPushButton("左转 (C)")
        self.right_btn = QPushButton("右转 (D)")
        self.up_btn = QPushButton("上升 (E)")
        self.down_btn = QPushButton("下降 (F)")
        self.stop_btn = QPushButton("停止 (G)")

        # Set shortcuts
        self.forward_btn.setShortcut("A")
        self.backward_btn.setShortcut("B")
        self.left_btn.setShortcut("C")
        self.right_btn.setShortcut("D")
        self.up_btn.setShortcut("E")
        self.down_btn.setShortcut("F")
        self.stop_btn.setShortcut("G")

        # Connect buttons
        self.forward_btn.clicked.connect(lambda: self.send_manual_command('forward'))
        self.backward_btn.clicked.connect(lambda: self.send_manual_command('backward'))
        self.left_btn.clicked.connect(lambda: self.send_manual_command('left'))
        self.right_btn.clicked.connect(lambda: self.send_manual_command('right'))
        self.up_btn.clicked.connect(lambda: self.send_manual_command('up'))
        self.down_btn.clicked.connect(lambda: self.send_manual_command('down'))
        self.stop_btn.clicked.connect(lambda: self.send_manual_command('stop'))

        # Layout buttons
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

        self.phi_target_spin = QDoubleSpinBox()
        self.phi_target_spin.setRange(-180.0, 180.0)
        self.phi_target_spin.setSingleStep(1.0)
        self.phi_target_spin.setValue(self.phi_target)
        self.phi_target_spin.setPrefix("角度目标: ")
        self.phi_target_spin.valueChanged.connect(self.update_phi_target)
        target_layout.addWidget(self.phi_target_spin)

        target_group.setLayout(target_layout)
        right_layout.addWidget(target_group)

        # PID parameters
        pid_group = QGroupBox("PID参数")
        pid_layout = QVBoxLayout()

        self.kp_x_spin = QSpinBox()
        self.kp_x_spin.setRange(0, 1000)
        self.kp_x_spin.setValue(self.Kp_x)
        self.kp_x_spin.setPrefix("Kp_x: ")
        self.kp_x_spin.valueChanged.connect(self.update_kp_x)
        pid_layout.addWidget(self.kp_x_spin)

        self.ki_x_spin = QSpinBox()
        self.ki_x_spin.setRange(0, 1000)
        self.ki_x_spin.setValue(self.Ki_x)
        self.ki_x_spin.setPrefix("Ks_x: ")
        self.ki_x_spin.valueChanged.connect(self.update_ki_x)
        pid_layout.addWidget(self.ki_x_spin)

        self.kd_x_spin = QSpinBox()
        self.kd_x_spin.setRange(0, 1000)
        self.kd_x_spin.setValue(self.Kd_x)
        self.kd_x_spin.setPrefix("Kd_x: ")
        self.kd_x_spin.valueChanged.connect(self.update_kd_x)
        pid_layout.addWidget(self.kd_x_spin)

        self.kp_phi_spin = QSpinBox()
        self.kp_phi_spin.setRange(0, 1000)
        self.kp_phi_spin.setValue(self.Kp_phi)
        self.kp_phi_spin.setPrefix("Kp_phi: ")
        self.kp_phi_spin.valueChanged.connect(self.update_kp_phi)
        pid_layout.addWidget(self.kp_phi_spin)

        self.ki_phi_spin = QSpinBox()
        self.ki_phi_spin.setRange(0, 1000)
        self.ki_phi_spin.setValue(self.Ki_phi)
        self.ki_phi_spin.setPrefix("Ki_phi: ")
        self.ki_phi_spin.valueChanged.connect(self.update_ki_phi)
        pid_layout.addWidget(self.ki_phi_spin)

        self.kd_phi_spin = QSpinBox()
        self.kd_phi_spin.setRange(0, 1000)
        self.kd_phi_spin.setValue(self.Kd_phi)
        self.kd_phi_spin.setPrefix("Kd_phi: ")
        self.kd_phi_spin.valueChanged.connect(self.update_kd_phi)
        pid_layout.addWidget(self.kd_phi_spin)

        pid_group.setLayout(pid_layout)
        right_layout.addWidget(pid_group)

        # Sliding mode parameters
        sliding_group = QGroupBox("滑模控制参数")
        sliding_layout = QVBoxLayout()

        self.gamma_spin = QDoubleSpinBox()
        self.gamma_spin.setRange(0.1, 5.0)
        self.gamma_spin.setSingleStep(0.1)
        self.gamma_spin.setValue(1.2)
        self.gamma_spin.setPrefix("γ: ")
        sliding_layout.addWidget(self.gamma_spin)

        self.alpha_spin = QDoubleSpinBox()
        self.alpha_spin.setRange(0.1, 10.0)
        self.alpha_spin.setSingleStep(0.1)
        self.alpha_spin.setValue(2.0)
        self.alpha_spin.setPrefix("α: ")
        sliding_layout.addWidget(self.alpha_spin)

        self.lambda_rate_spin = QDoubleSpinBox()
        self.lambda_rate_spin.setRange(0.01, 1.0)
        self.lambda_rate_spin.setSingleStep(0.01)
        self.lambda_rate_spin.setValue(0.08)
        self.lambda_rate_spin.setPrefix("λ 速率: ")
        sliding_layout.addWidget(self.lambda_rate_spin)

        sliding_group.setLayout(sliding_layout)
        right_layout.addWidget(sliding_group)

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
            # Reset integral terms
            self.integral_x = 0
            self.integral_phi = 0
            self.last_time = time.time()
        else:
            self.start_btn.setText("开始自动控制")
            self.log_message("自动控制已停止")
            self.manual_mode = True
            self.mode_status_label.setText("当前模式: 手动")
            # Send stop command
            self.send_manual_command('stop')

    def send_manual_command(self, command):
        if self.serial_thread and self.serial_thread.isRunning():
            cmd_byte = self.manual_commands[command]
            try:
                # Send single byte command
                self.serial_thread.send_data(bytes([cmd_byte]))
                self.log_message(f"发送手动指令: {command} (0x{cmd_byte:02X})")
                self.last_command_label.setText(f"最后指令: {command} (0x{cmd_byte:02X})")

                # If stop command, switch back to manual mode
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

            # Convert to Qt image format
            rgb_image = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.video_label.setPixmap(pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio))

            # Update vision status
            vision_data = self.vision.get_vision_data()
            if vision_data['stabilized']:
                self.vision_status_label.setText("视觉状态: 稳定")
                self.offset_label.setText(f"视觉偏移: ({vision_data['offset'][0]:.1f}, {vision_data['offset'][1]:.1f})")
            else:
                self.vision_status_label.setText("视觉状态: 不稳定")
                self.offset_label.setText("视觉偏移: -")

            # Get serial data
            angle, x_value, initial_angle = self.data_processor.get_values()
            if angle is not None:
                self.angle_label.setText(f"当前角度: {angle:.2f}°")
            if initial_angle is not None:
                self.initial_angle_label.setText(f"初始角度: {initial_angle:.2f}°")
                # Use initial angle as target angle
                self.phi_target = initial_angle
                self.phi_target_spin.setValue(self.phi_target)
            if x_value is not None:
                self.x_label.setText(f"当前X位置: {x_value:.2f}m")

            # Control logic (only in auto mode)
            if not self.manual_mode and self.start_btn.text() == "停止自动控制" and angle is not None and x_value is not None:
                self.apply_control_logic(angle, x_value, vision_data)

    def apply_control_logic(self, angle, x_value, vision_data, controller_params=None):
        # Calculate time difference
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        theta1 = vision_data['angles'][0] if vision_data['stabilized'] else 0
        theta2 = vision_data['angles'][1] if vision_data['stabilized'] else 0

        # Log the data if logging is active
        if self.logging_active:
            self.log_motion_data(x_value, angle, theta1, theta2)

        err_x = self.x_target - x_value
        err_phi = self.phi_target - angle

        # Update integral terms
        self.integral_x += err_x * dt
        self.integral_phi += err_phi * dt

        # Prevent integral windup
        self.integral_x = max(-100, min(100, self.integral_x))
        self.integral_phi = max(-100, min(100, self.integral_phi))

        if controller_params is None:
            # Sliding mode controller parameters
            controller_params = {
                'kp': self.Kp_x,
                'kd': self.Kd_x,
                'ks': self.Ki_x,
                'gamma': self.gamma_spin.value(),
                'alpha': self.alpha_spin.value(),
                'lambda_rate': self.lambda_rate_spin.value()
            }

        # Calculate sliding mode control output (cart direction)
        pwm_x = sliding_mode_controller(
            x_value=x_value,
            theta_deg=theta1,
            theta1_deg=theta1,
            x_target=self.x_target,
            current_time=current_time,
            params=controller_params
        )

        pwm_phi = (self.Kp_phi * err_phi +
                   self.Ki_phi * self.integral_phi -
                   self.Kd_phi * theta2)

        pwm_x_abs = abs(pwm_x)
        pwm_phi_abs = abs(pwm_phi)

        pwm_x_abs = max(0, min(100, int(pwm_x_abs)))
        pwm_phi_abs = max(0, min(100, int(pwm_phi_abs)))

        pwm_x = -pwm_x_abs if pwm_x < 0 else pwm_x_abs
        pwm_phi = -pwm_phi_abs if pwm_phi < 0 else pwm_phi_abs

        control_str = f"{pwm_x},{pwm_phi}\n"
        if self.serial_thread:
            self.serial_thread.send_data(control_str)
            self.pwm_label.setText(f"控制输出: {pwm_x}, {0} (I_x: {self.integral_x:.2f}, I_phi: {self.integral_phi:.2f})")
            self.log_message(f"发送自动控制: {control_str.strip()}")

    def toggle_logging(self):
        if not self.logging_active:
            # Start logging
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self.log_file = open(f"motion_log_{timestamp}.csv", "w")
            self.log_start_time = time.time()
            self.log_headers_written = False
            self.logging_active = True
            self.logging_btn.setText("停止记录")
            self.logging_status_label.setText("记录状态: 激活")
            self.log_message("已开始数据记录")
        else:
            # Stop logging
            if self.log_file:
                self.log_file.close()
            self.logging_active = False
            self.logging_btn.setText("开始记录")
            self.logging_status_label.setText("记录状态: 未激活")
            self.log_message("已停止数据记录")

    def write_log_headers(self):
        if self.log_file and not self.log_headers_written:
            self.log_file.write("timestamp,x,angle,theta1,theta2,x_target,phi_target\n")
            self.log_headers_written = True

    def log_motion_data(self, x, angle, theta1, theta2):
        if self.logging_active and self.log_file:
            self.write_log_headers()
            timestamp = time.time() - self.log_start_time
            self.log_file.write(f"{timestamp:.3f},{x:.4f},{angle:.4f},{theta1:.4f},{theta2:.4f},{self.x_target:.4f},{self.phi_target:.4f}\n")
            self.log_file.flush()  # Ensure data is written immediately

    def update_serial_data(self, data):
        self.log_text.append(f"串口数据: {data}")

    def log_message(self, message):
        self.log_text.append(f"[{time.strftime('%H:%M:%S')}] {message}")

    def update_x_target(self, value):
        self.x_target = value
        self.log_message(f"X目标更新为: {value}")

    def update_phi_target(self, value):
        self.phi_target = value
        self.log_message(f"角度目标更新为: {value}")

    def update_kp_x(self, value):
        self.Kp_x = value
        self.log_message(f"Kp_x更新为: {value}")

    def update_ki_x(self, value):
        self.Ki_x = value
        self.log_message(f"Ks_x更新为: {value}")

    def update_kd_x(self, value):
        self.Kd_x = value
        self.log_message(f"Kd_x更新为: {value}")

    def update_kp_phi(self, value):
        self.Kp_phi = value
        self.log_message(f"Kp_phi更新为: {value}")

    def update_ki_phi(self, value):
        self.Ki_phi = value
        self.log_message(f"Ki_phi更新为: {value}")

    def update_kd_phi(self, value):
        self.Kd_phi = value
        self.log_message(f"Kd_phi更新为: {value}")

    def closeEvent(self, event):
        if self.logging_active:
            self.toggle_logging()  # This will close the log file
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
        if self.cap.isOpened():
            self.cap.release()
        self.timer.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())