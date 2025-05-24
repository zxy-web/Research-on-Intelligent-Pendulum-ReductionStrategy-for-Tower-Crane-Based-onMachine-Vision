import cv2
from ultralytics import YOLO
from collections import deque, defaultdict
import math
import serial
import threading
import time
from typing import Optional, Tuple


class DataProcessor:
    def __init__(self):
        self.angle_value = None  # 存储angle的最后一个值
        self.x_value = None  # 存储x的值
        self.lock = threading.Lock()  # 线程锁保证数据安全

    def process_data(self, data):
        """处理接收到的串口数据"""
        with self.lock:
            if data.startswith("Angle:"):
                angle_str = data.split(":")[1]
                self.angle_value = float(angle_str)
                print(f"提取到angle值: {self.angle_value}")

            if data.startswith("x:"):
                x_str = data.split(":")[1]
                self.x_value = float(x_str)
                self.x_value = self.x_value/100;
                print(f"提取到X值: {self.x_value}")

    def get_values(self) -> Tuple[Optional[float], Optional[float]]:
        """获取当前存储的值"""
        with self.lock:
            return self.angle_value, self.x_value


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
        """处理摄像头帧"""
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
                    print(f"平衡位置已确定: {self.balance_pos}")
                    break

            # 绘制红十字
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

                cv2.putText(annotated_frame, f"Offset: ({dx}, {dy})", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(annotated_frame, f"Angles: ({n_rad:.4f}, {w_rad:.4f})", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return annotated_frame

    def get_vision_data(self):
        """获取当前视觉数据"""
        with self.lock:
            return {
                'offset': self.current_offset,
                'angles': self.current_angles,
                'stabilized': self.stabilized
            }


def serial_thread(processor, port="COM9", baudrate=9600):
    """串口数据读取线程"""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"已连接到串口蓝牙设备 {port}")

        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                print(f"接收到数据: {data}")
                processor.process_data(data)
            time.sleep(0.01)  # 防止CPU占用过高

    except Exception as e:
        print(f"串口线程错误: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


def main():
    # 初始化系统
    vision = VisionSystem(r"D:\桌面\ultralytics-main\ultralytics-main\ultralytics\runs\train\exp4\weights\best.pt")
    data_processor = DataProcessor()

    # 启动串口线程
    serial_t = threading.Thread(target=serial_thread, args=(data_processor,), daemon=True)
    serial_t.start()

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    try:
        while True:
            # 处理摄像头数据
            ret, frame = cap.read()
            if not ret:
                print("无法读取摄像头画面")
                break

            annotated_frame = vision.process_frame(frame)

            # 获取并显示串口数据
            angle, x_value = data_processor.get_values()
            if angle is not None and x_value is not None:
                cv2.putText(annotated_frame, f"Serial: Angle={angle}, X={x_value}", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            # 获取并显示视觉数据
            vision_data = vision.get_vision_data()
            if vision_data['stabilized']:
                cv2.putText(annotated_frame,
                            f"Vision Angles: {vision_data['angles'][0]:.2f}, {vision_data['angles'][1]:.2f}",
                            (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 显示画面
            cv2.imshow("Integrated System", annotated_frame)

            # 按'q'退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()