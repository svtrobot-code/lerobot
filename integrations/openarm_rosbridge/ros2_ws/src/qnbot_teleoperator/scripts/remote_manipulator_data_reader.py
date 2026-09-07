#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
远程操作器数据读取模块
通过串口读取远程操作器的数据，并提供接口给上层应用使用

使用示例：

1. 基本使用方法：
```python
# 导入模块
from remote_manipulator_data_reader import RemoteManipulatorReader

# 创建读取器实例
reader = RemoteManipulatorReader(port="/dev/ttyACM0", baudrate=2000000)

# 方法1：使用回调函数
def on_data_received(data):
    print(f"收到新数据:\n{data}")
    # 处理数据...
    
# 注册回调函数
reader.register_callback(on_data_received)

# 启动数据读取
reader.start()

# 方法2：主动获取数据

# 启动数据读取
reader.start()

# 读取数据
latest_data = reader.get_latest_data()
print(f"主动获取的最新数据:\n{latest_data}")

# 使用完成后停止读取
reader.stop()
```

2. 在ROS2中使用示例：
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import Joy

# 导入远程操作器数据读取模块
from remote_manipulator_data_reader import RemoteManipulatorReader

class RemoteManipulatorNode(Node):
    def __init__(self):
        super().__init__('remote_manipulator_node')
        
        # 创建发布者
        self.joy_left_pub = self.create_publisher(Joy, 'joy_left', 10)
        self.joy_right_pub = self.create_publisher(Joy, 'joy_right', 10)
        self.arm_joint_left_pub = self.create_publisher(
            Float32MultiArray, 'arm_joint_left', 10)
        self.arm_joint_right_pub = self.create_publisher(
            Float32MultiArray, 'arm_joint_right', 10)
        
        # 创建定时器，10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 创建读取器实例
        self.reader = RemoteManipulatorReader(
            port="/dev/ttyACM0", 
            baudrate=2000000
        )
        
        # 启动数据读取
        if not self.reader.start():
            self.get_logger().error('无法启动数据读取器')
            rclpy.shutdown()
            return
        
        self.get_logger().info('远程操作器节点已启动')
    
    def timer_callback(self):
        # 获取最新数据
        data = self.reader.get_latest_data()
        
        # 发布左摇杆数据
        joy_left_msg = Joy()
        joy_left_msg.header.stamp = self.get_clock().now().to_msg()
        joy_left_msg.axes = [data.joystick_left[0]/512.0 - 1.0, data.joystick_left[1]/512.0 - 1.0]  # 归一化到[-1,1]
        joy_left_msg.buttons = [1 if data.joystick_left[2] > 0 else 0]
        self.joy_left_pub.publish(joy_left_msg)
        
        # 发布右摇杆数据
        joy_right_msg = Joy()
        joy_right_msg.header.stamp = self.get_clock().now().to_msg()
        joy_right_msg.axes = [data.joystick_right[0]/512.0 - 1.0, data.joystick_right[1]/512.0 - 1.0]  # 归一化到[-1,1]
        joy_right_msg.buttons = [1 if data.joystick_right[2] > 0 else 0]
        self.joy_right_pub.publish(joy_right_msg)
        
        # 发布左臂关节数据 (使用弧度制)
        arm_left_msg = Float32MultiArray()
        arm_left_msg.layout.dim = [MultiArrayDimension(label="joints", size=len(data.arm_joint_left_rad), stride=1)]
        arm_left_msg.data = [float(val) for val in data.arm_joint_left_rad]
        self.arm_joint_left_pub.publish(arm_left_msg)
        
        # 发布右臂关节数据 (使用弧度制)
        arm_right_msg = Float32MultiArray()
        arm_right_msg.layout.dim = [MultiArrayDimension(label="joints", size=len(data.arm_joint_right_rad), stride=1)]
        arm_right_msg.data = [float(val) for val in data.arm_joint_right_rad]
        self.arm_joint_right_pub.publish(arm_right_msg)
    
    def destroy_node(self):
        # 停止数据读取
        if hasattr(self, 'reader'):
            self.reader.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RemoteManipulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. 使用回调方式在ROS2中处理数据：
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import threading

# 导入远程操作器数据读取模块
from remote_manipulator_data_reader import RemoteManipulatorReader, RemoteManipulatorData

class RemoteManipulatorCallbackNode(Node):
    def __init__(self):
        super().__init__('remote_manipulator_callback_node')
        
        # 创建发布者
        self.data_pub = self.create_publisher(
            Float32MultiArray, 'manipulator_data', 10)
        
        # 创建读取器实例
        self.reader = RemoteManipulatorReader(
            port="/dev/ttyACM0", 
            baudrate=2000000
        )
        
        # 注册回调函数
        self.reader.register_callback(self.data_callback)
        
        # 启动数据读取
        if not self.reader.start():
            self.get_logger().error('无法启动数据读取器')
            rclpy.shutdown()
            return
            
        self.get_logger().info('远程操作器回调节点已启动')
    
    def data_callback(self, data: RemoteManipulatorData):
        # 在回调函数中处理数据并发布
        # 注意：此回调函数在非ROS线程中执行，需要特别注意线程安全
        
        # 将所有数据合并为一个Float32MultiArray消息
        msg = Float32MultiArray()
        msg.layout.dim = [MultiArrayDimension(label="data", size=40, stride=1)]  # 更新数据大小：摇杆8 + 关节原始16 + 关节弧度16 = 40
        
        # 合并所有数据 (使用弧度制关节数据)
        all_data = []
        all_data.extend([float(val) for val in data.joystick_left])
        all_data.extend([float(val) for val in data.joystick_right])
        all_data.extend([float(val) for val in data.arm_joint_left_rad])
        all_data.extend([float(val) for val in data.arm_joint_right_rad])
        
        msg.data = all_data
        
        # 发布消息
        # 注意：由于这是在非ROS线程中调用，需要确保线程安全
        self.data_pub.publish(msg)
        
        # 记录日志
        # 注意：避免频繁记录日志，会影响性能
        # self.get_logger().info('收到新数据')
    
    def destroy_node(self):
        # 停止数据读取
        if hasattr(self, 'reader'):
            self.reader.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RemoteManipulatorCallbackNode()
    
    try:
        # 运行ROS2节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

import serial
import struct
import threading
import time
import queue
import logging
import math
from typing import Callable, Optional, List, Dict, Any

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("RemoteManipulatorReader")

# 常量定义
REPORT_FRAME_HEADER = 0xAA
REPORT_FRAME_TAIL = 0x55

# 支持的载荷长度（不含帧头/校验/帧尾）
# 旧版基础: 摇杆(8*int16) + 关节(16*int16) = 48B
# 新版(四元数): 基础48B + (fAcc[3]+fGyro[3]+fQuat[4]) = 48 + (10*4) = 88B
# 新版(四元数+附加IMU): 48 + 40(躯干) + 40(附加) = 128B
DATA_LEN_BASE = 48
DATA_LEN_TORSO_IMU_QUAT = 88
DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT = 128
SUPPORTED_DATA_LENGTHS = [DATA_LEN_BASE, DATA_LEN_TORSO_IMU_QUAT, DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT]

# 编码器转换常量：16384对应2π弧度
ENCODER_TO_RADIAN_RATIO = 2 * math.pi / 16384

# 远程操作器数据结构
class RemoteManipulatorData:
    def __init__(self):
        self.joystick_left = [0, 0, 0, 0]  # X轴, Y轴, 按键事件, 板机
        self.joystick_right = [0, 0, 0, 0]  # X轴, Y轴, 按键事件, 板机
        self.arm_joint_left = [0] * 8  # 原始编码器数据
        self.arm_joint_right = [0] * 8  # 原始编码器数据
        self.arm_joint_left_rad = [0.0] * 8  # 转换为弧度制的数据
        self.arm_joint_right_rad = [0.0] * 8  # 转换为弧度制的数据
        self.timestamp = 0  # 时间戳，记录数据接收时间
        # 躯干IMU（如果报文包含）
        self.torso_acc = [0.0, 0.0, 0.0]
        self.torso_gyro = [0.0, 0.0, 0.0]
        # 兼容旧字段（欧拉角），保持占位
        self.torso_angle = [0.0, 0.0, 0.0]
        # 新增：四元数
        self.torso_quat = [0.0, 0.0, 0.0, 0.0]  # q0, q1, q2, q3
        # 附加IMU（头部等，如报文包含）
        self.extra_acc = [0.0, 0.0, 0.0]
        self.extra_gyro = [0.0, 0.0, 0.0]
        # 兼容旧字段（欧拉角），保持占位
        self.extra_angle = [0.0, 0.0, 0.0]
        # 新增：四元数
        self.extra_quat = [0.0, 0.0, 0.0, 0.0]  # q0, q1, q2, q3
        # 实际解析到的格式版本: 1(旧), 2(躯干IMU), 3(躯干+附加IMU)
        self.format_version = 1

    def to_dict(self) -> Dict[str, Any]:
        """将数据转换为字典格式"""
        return {
            "timestamp": self.timestamp,
            "joystick_left": self.joystick_left,
            "joystick_right": self.joystick_right,
            "arm_joint_left": self.arm_joint_left,
            "arm_joint_right": self.arm_joint_right,
            "arm_joint_left_rad": self.arm_joint_left_rad,
            "arm_joint_right_rad": self.arm_joint_right_rad,
            "torso_acc": self.torso_acc,
            "torso_gyro": self.torso_gyro,
            "torso_angle": self.torso_angle,
            "torso_quat": self.torso_quat,
            "extra_acc": self.extra_acc,
            "extra_gyro": self.extra_gyro,
            "extra_angle": self.extra_angle,
            "extra_quat": self.extra_quat,
            "format_version": self.format_version,
        }

    def __str__(self) -> str:
        """友好的字符串表示"""
        return (f"时间戳: {self.timestamp}\n"
                f"左摇杆: X={self.joystick_left[0]}, Y={self.joystick_left[1]}, "
                f"按键={self.joystick_left[2]}, 板机={self.joystick_left[3]}\n"
                f"右摇杆: X={self.joystick_right[0]}, Y={self.joystick_right[1]}, "
                f"按键={self.joystick_right[2]}, 板机={self.joystick_right[3]}\n"
                f"左臂关节(原始): {self.arm_joint_left}\n"
                f"右臂关节(原始): {self.arm_joint_right}\n"
                f"左臂关节(弧度): {[f'{x:.4f}' for x in self.arm_joint_left_rad]}\n"
                f"右臂关节(弧度): {[f'{x:.4f}' for x in self.arm_joint_right_rad]}\n"
                f"躯干IMU Acc: {[f'{x:.3f}' for x in self.torso_acc]} Gyro: {[f'{x:.3f}' for x in self.torso_gyro]} Quat: {[f'{x:.3f}' for x in self.torso_quat]}\n"
                f"附加IMU Acc: {[f'{x:.3f}' for x in self.extra_acc]} Gyro: {[f'{x:.3f}' for x in self.extra_gyro]} Quat: {[f'{x:.3f}' for x in self.extra_quat]}")

    def _convert_encoder_to_radian(self, encoder_value: int) -> float:
        """
        将编码器原始数据转换为弧度制
        
        Args:
            encoder_value: 编码器原始值
            
        Returns:
            float: 转换后的弧度值
        """
        return encoder_value * ENCODER_TO_RADIAN_RATIO

class RemoteManipulatorReader:
    def __init__(self, port: str = "/dev/tty.usbmodem3960356230341", baudrate: int = 2000000,
                 queue_size: int = 10):
        """
        初始化远程操作器数据读取器
        
        Args:
            port: 串口设备名称
            baudrate: 波特率
            queue_size: 数据队列大小
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.read_thread = None
        
        # 使用固定大小的队列，当队列满时自动丢弃旧数据
        self.data_queue = queue.Queue(maxsize=queue_size)
        self.latest_data = RemoteManipulatorData()
        
        # 回调函数列表
        self.callbacks = []
        
        # 回调处理线程
        self.callback_thread = None
        self.callback_lock = threading.Lock()
        
    def start(self) -> bool:
        """
        启动数据读取
        
        Returns:
            bool: 是否成功启动
        """
        if self.running:
            logger.warning("数据读取器已经在运行")
            return True
            
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            
            self.running = True
            
            # 启动数据读取线程
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            # 启动回调处理线程
            self.callback_thread = threading.Thread(target=self._callback_loop, daemon=True)
            self.callback_thread.start()
            
            logger.info(f"成功启动数据读取，端口：{self.port}, 波特率：{self.baudrate}")
            return True
            
        except Exception as e:
            logger.error(f"启动数据读取失败: {str(e)}")
            self.running = False
            return False
    
    def stop(self) -> None:
        """停止数据读取"""
        self.running = False
        
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
            
        if self.callback_thread:
            self.callback_thread.join(timeout=1.0)
            
        if self.serial and self.serial.is_open:
            self.serial.close()
            
        logger.info("数据读取已停止")
    
    def _read_loop(self) -> None:
        """数据读取循环"""
        buffer = bytearray()
        
        while self.running:
            try:
                if self.serial and self.serial.is_open:
                    # 读取可用数据
                    if self.serial.in_waiting > 0:
                        data = self.serial.read(self.serial.in_waiting)
                        buffer.extend(data)
                        
                        # 处理缓冲区中的所有完整数据包
                        while len(buffer) >= 4:  # 至少包含帧头、帧尾和校验和
                            # 查找帧头
                            start_idx = buffer.find(bytes([REPORT_FRAME_HEADER]))
                            if start_idx == -1:
                                # 没有找到帧头，清空缓冲区
                                buffer.clear()
                                break
                                
                            # 如果帧头不在开始位置，丢弃之前的数据
                            if start_idx > 0:
                                buffer = buffer[start_idx:]
                                
                            # 尝试按支持的不同数据长度解析
                            parsed_any = False
                            # 首先确保至少有最小报文长度
                            min_packet_size = 1 + min(SUPPORTED_DATA_LENGTHS) + 2
                            if len(buffer) < min_packet_size:
                                break

                            for data_len in SUPPORTED_DATA_LENGTHS:
                                packet_size = 1 + data_len + 2  # header + payload + checksum + tail
                                if len(buffer) < packet_size:
                                    # 缓冲不足以组成此长度的完整包
                                    continue

                                candidate = buffer[:packet_size]
                                # 帧尾校验
                                if candidate[-1] != REPORT_FRAME_TAIL:
                                    continue

                                # 校验和（对数据字节异或），与现有实现保持一致：从索引1到 packet_size-3（含）
                                calculated_checksum = 0
                                for i in range(1, packet_size - 2):
                                    calculated_checksum ^= candidate[i]
                                if calculated_checksum != candidate[packet_size - 2]:
                                    continue

                                # 解析成对象
                                if self._parse_data_packet(candidate, data_len):
                                    try:
                                        if self.data_queue.full():
                                            self.data_queue.get_nowait()
                                        self.data_queue.put_nowait(self.latest_data)
                                    except queue.Full:
                                        pass
                                    # 消费该包
                                    buffer = buffer[packet_size:]
                                    parsed_any = True
                                    break

                            if not parsed_any:
                                # 未能以任何支持的长度解析，丢弃一个字节以重新同步
                                buffer = buffer[1:]
                
                # 防止CPU占用过高
                time.sleep(0.001)
                
            except Exception as e:
                logger.error(f"数据读取错误: {str(e)}")
                time.sleep(1)  # 发生错误时暂停一下
    
    def _parse_data_packet(self, packet: bytearray, data_len: int) -> bool:
        """
        解析数据包
        
        Args:
            packet: 完整的数据包
            data_len: 载荷长度（不含帧头/校验/帧尾）
            
        Returns:
            bool: 是否成功解析
        """
        try:
            # 跳过帧头，直接解析数据部分
            data_part = packet[1:-2]  # 去掉帧头、校验和和帧尾
            
            # 创建新的数据对象
            data = RemoteManipulatorData()
            data.timestamp = time.time()
            
            # 解析左摇杆数据 (4个int16_t = 8字节)
            offset = 0
            for i in range(4):
                data.joystick_left[i] = struct.unpack('<h', data_part[offset:offset+2])[0]
                offset += 2
                
            # 解析右摇杆数据 (4个int16_t = 8字节)
            for i in range(4):
                data.joystick_right[i] = struct.unpack('<h', data_part[offset:offset+2])[0]
                offset += 2
                
            # 解析左臂关节数据 (8个int16_t = 16字节)
            for i in range(8):
                raw_value = struct.unpack('<h', data_part[offset:offset+2])[0]
                data.arm_joint_left[i] = raw_value
                # 转换为弧度制
                data.arm_joint_left_rad[i] = data._convert_encoder_to_radian(raw_value)
                offset += 2
                
            # 解析右臂关节数据 (8个int16_t = 16字节)
            for i in range(8):
                raw_value = struct.unpack('<h', data_part[offset:offset+2])[0]
                data.arm_joint_right[i] = raw_value
                # 转换为弧度制
                data.arm_joint_right_rad[i] = data._convert_encoder_to_radian(raw_value)
                offset += 2

            # 旧版基础数据长度为48字节；新版四元数格式长度为88/128字节
            if data_len == DATA_LEN_BASE:
                data.format_version = 1
            elif data_len == DATA_LEN_TORSO_IMU_QUAT:
                # 躯干IMU(四元数): fAcc[3], fGyro[3], fQuat[4]
                data.format_version = 2
                for i in range(3):
                    data.torso_acc[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(3):
                    data.torso_gyro[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(4):
                    data.torso_quat[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
            elif data_len == DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT:
                # 躯干IMU + 附加IMU (四元数)
                data.format_version = 3
                for i in range(3):
                    data.torso_acc[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(3):
                    data.torso_gyro[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(4):
                    data.torso_quat[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(3):
                    data.extra_acc[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(3):
                    data.extra_gyro[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
                for i in range(4):
                    data.extra_quat[i] = struct.unpack('<f', data_part[offset:offset+4])[0]
                    offset += 4
            else:
                # 不支持的数据长度
                return False

            # 更新最新数据
            self.latest_data = data
            return True
            
        except Exception as e:
            logger.error(f"数据解析错误: {str(e)}")
            return False
    
    def _callback_loop(self) -> None:
        """回调处理循环"""
        while self.running:
            try:
                # 从队列获取最新数据
                if not self.data_queue.empty():
                    data = self.data_queue.get_nowait()
                    
                    # 调用所有注册的回调函数
                    with self.callback_lock:
                        for callback in self.callbacks:
                            try:
                                callback(data)
                            except Exception as e:
                                logger.error(f"回调函数执行错误: {str(e)}")
                
                # 防止CPU占用过高
                time.sleep(0.01)
                
            except queue.Empty:
                time.sleep(0.01)  # 队列为空时等待
            except Exception as e:
                logger.error(f"回调处理错误: {str(e)}")
                time.sleep(1)  # 发生错误时暂停一下
    
    def get_latest_data(self) -> RemoteManipulatorData:
        """
        获取最新的传感器数据
        
        Returns:
            RemoteManipulatorData: 最新的数据
        """
        return self.latest_data
    
    def register_callback(self, callback: Callable[[RemoteManipulatorData], None]) -> bool:
        """
        注册数据回调函数
        
        Args:
            callback: 回调函数，接收一个RemoteManipulatorData参数
            
        Returns:
            bool: 是否成功注册
        """
        with self.callback_lock:
            if callback not in self.callbacks:
                self.callbacks.append(callback)
                logger.info("已注册回调函数")
                return True
            else:
                logger.warning("回调函数已经注册")
                return False
    
    def unregister_callback(self, callback: Callable[[RemoteManipulatorData], None]) -> bool:
        """
        取消注册数据回调函数
        
        Args:
            callback: 之前注册的回调函数
            
        Returns:
            bool: 是否成功取消注册
        """
        with self.callback_lock:
            if callback in self.callbacks:
                self.callbacks.remove(callback)
                logger.info("已取消注册回调函数")
                return True
            else:
                logger.warning("回调函数未注册")
                return False


def run_gui_demo():
    """
    运行GUI演示程序
    
    注意：此函数需要安装以下依赖：
    - matplotlib
    - PyQt5
    - numpy
    
    可以通过以下命令安装：
    pip install matplotlib PyQt5 numpy
    """
    try:
        import sys
        import numpy as np
        import matplotlib
        matplotlib.use('Qt5Agg')
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
        from matplotlib.figure import Figure
        from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                                    QWidget, QTableWidget, QTableWidgetItem, QHeaderView, 
                                    QLabel, QSplitter)
        from PyQt5.QtCore import QTimer, Qt
    except ImportError as e:
        print(f"错误：缺少GUI依赖包，请安装：pip install matplotlib PyQt5 numpy")
        print(f"具体错误：{e}")
        return
    
    class RemoteManipulatorGUI(QMainWindow):
        def __init__(self):
            super().__init__()
            
            # 设置窗口标题和大小
            self.setWindowTitle("远程操作器右臂关节数据监视器")
            self.setGeometry(100, 100, 1200, 800)
            
            # 创建中央控件
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            main_layout = QHBoxLayout(central_widget)
            
            # 创建分割器
            splitter = QSplitter(Qt.Horizontal)
            main_layout.addWidget(splitter)
            
            # 创建左侧和右侧窗口部件
            left_widget = QWidget()
            right_widget = QWidget()
            left_layout = QVBoxLayout(left_widget)
            right_layout = QVBoxLayout(right_widget)
            
            # 添加到分割器
            splitter.addWidget(left_widget)
            splitter.addWidget(right_widget)
            splitter.setSizes([400, 800])  # 设置初始分割比例
            
            # 创建表格标题
            table_label = QLabel("右臂关节数据表格 (原始值 | 弧度值)")
            table_label.setAlignment(Qt.AlignCenter)
            left_layout.addWidget(table_label)
            
            # 创建数据表格
            self.table = QTableWidget()
            self.table.setColumnCount(17)  # 时间 + 8个关节原始值 + 8个关节弧度值
            self.table.setRowCount(20)    # 显示最近20条数据
            
            # 设置表格标题
            headers = ["时间(s)"]
            # 添加原始值列标题
            for i in range(8):
                headers.append(f"关节{i+1}(原始)")
            # 添加弧度值列标题
            for i in range(8):
                headers.append(f"关节{i+1}(弧度)")
            self.table.setHorizontalHeaderLabels(headers)
            
            # 设置表格列宽自动调整
            header = self.table.horizontalHeader()
            for i in range(17):
                header.setSectionResizeMode(i, QHeaderView.Stretch)
            
            left_layout.addWidget(self.table)
            
            # 创建图表标题
            chart_label = QLabel("右臂关节数据曲线")
            chart_label.setAlignment(Qt.AlignCenter)
            right_layout.addWidget(chart_label)
            
            # 创建图表
            self.figure = Figure(figsize=(8, 8), dpi=100)
            self.canvas = FigureCanvas(self.figure)
            right_layout.addWidget(self.canvas)
            
            # 创建8个子图用于显示每个关节的数据
            self.axes = []
            for i in range(8):
                ax = self.figure.add_subplot(4, 2, i+1)
                ax.set_title(f"Joint{i+1}")
                ax.set_xlabel("Time(s)")
                ax.set_ylabel("Angle(rad)")  # 更新Y轴标签为弧度
                ax.grid(True)
                self.axes.append(ax)
            
            self.figure.tight_layout()
            
            # 数据存储
            self.time_data = []  # 时间数据
            self.joint_data = [[] for _ in range(8)]  # 8个关节的数据
            self.max_data_points = 100  # 最多保存100个数据点
            self.display_data = []  # 用于表格显示的数据
            
            # 创建读取器实例
            self.reader = RemoteManipulatorReader(port="/dev/tty.usbmodem3960356230341", baudrate=2000000)
            
            # 启动数据读取
            if not self.reader.start():
                print("错误：无法启动数据读取器")
                sys.exit(1)
            
            # 记录起始时间
            self.start_time = time.time()
            
            # 创建定时器，50ms更新一次(20Hz)
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_data)
            self.timer.start(50)
            
            print("GUI已启动，开始监视右臂关节数据...")
        
        def update_data(self):
            """更新数据并刷新GUI"""
            # 获取最新数据
            data = self.reader.get_latest_data()
            
            # 获取当前时间（相对于起始时间）
            current_time = time.time() - self.start_time
            
            # 更新数据存储
            self.time_data.append(current_time)
            for i in range(8):
                self.joint_data[i].append(data.arm_joint_right_rad[i])  # 使用弧度制数据用于图表显示
            
            # 保存这一条完整数据用于表格显示
            self.display_data.append({
                'time': current_time,
                'joints_raw': data.arm_joint_right.copy(),  # 原始数据
                'joints_rad': data.arm_joint_right_rad.copy()  # 弧度数据
            })
            
            # 限制数据点数量
            if len(self.time_data) > self.max_data_points:
                self.time_data = self.time_data[-self.max_data_points:]
                for i in range(8):
                    self.joint_data[i] = self.joint_data[i][-self.max_data_points:]
                self.display_data = self.display_data[-20:]  # 只保留最近20条用于表格显示
            
            # 更新表格
            self.update_table()
            
            # 更新图表
            self.update_charts()
        
        def update_table(self):
            """更新表格数据"""
            # 清空表格
            self.table.clearContents()
            
            # 倒序显示数据（最新的在上面）
            for row, data_item in enumerate(reversed(self.display_data)):
                if row >= 20:  # 最多显示20行
                    break
                    
                # 设置时间
                self.table.setItem(row, 0, QTableWidgetItem(f"{data_item['time']:.2f}"))
                
                # 设置原始关节数据 (列1-8)
                for joint, value in enumerate(data_item['joints_raw']):
                    self.table.setItem(row, joint + 1, QTableWidgetItem(f"{value}"))
                
                # 设置弧度关节数据 (列9-16)
                for joint, value in enumerate(data_item['joints_rad']):
                    self.table.setItem(row, joint + 9, QTableWidgetItem(f"{value:.4f}"))
        
        def update_charts(self):
            """更新图表"""
            # 更新每个关节的图表
            for i in range(8):
                self.axes[i].clear()
                self.axes[i].plot(self.time_data, self.joint_data[i], 'b-')
                self.axes[i].set_title(f"Joint{i+1}")
                self.axes[i].set_xlabel("Time(s)")
                self.axes[i].set_ylabel("Angle(rad)")  # 更新Y轴标签为弧度
                self.axes[i].grid(True)
                
                # 设置Y轴范围，使曲线更加明显
                if len(self.joint_data[i]) > 1:
                    y_min = min(self.joint_data[i])
                    y_max = max(self.joint_data[i])
                    
                    # 处理所有值相同的情况
                    if y_min == y_max:
                        y_min -= 0.1  # 对于弧度值，使用更小的余量
                        y_max += 0.1
                    else:
                        # 添加一点余量
                        margin = (y_max - y_min) * 0.1
                        y_min -= margin
                        y_max += margin
                    
                    self.axes[i].set_ylim(y_min, y_max)
                
                # 设置X轴范围，只显示最近的数据
                if len(self.time_data) > 0:
                    x_min = max(0, self.time_data[-1] - 10)  # 显示最近10秒的数据
                    x_max = self.time_data[-1]
                    self.axes[i].set_xlim(x_min, x_max)
            
            # 调整布局并重绘
            self.figure.tight_layout()
            self.canvas.draw()
        
        def closeEvent(self, event):
            """窗口关闭事件处理"""
            # 停止数据读取器
            if hasattr(self, 'reader'):
                self.reader.stop()
            event.accept()
    
    # 创建QT应用
    app = QApplication(sys.argv)
    window = RemoteManipulatorGUI()
    window.show()
    sys.exit(app.exec_())


def simple_console_demo():
    """
    简单的控制台演示程序
    """
    print("启动远程操作器数据读取演示...")
    
    # 创建读取器实例
    reader = RemoteManipulatorReader(port="/dev/tty.usbmodem3960356230341", baudrate=2000000)
    
    # 定义回调函数
    def on_data_received(data):
        print(f"\n收到新数据 (时间戳: {data.timestamp:.3f}):")
        print(f"  左摇杆: X={data.joystick_left[0]:4d}, Y={data.joystick_left[1]:4d}, 按键={data.joystick_left[2]}, 板机={data.joystick_left[3]}")
        print(f"  右摇杆: X={data.joystick_right[0]:4d}, Y={data.joystick_right[1]:4d}, 按键={data.joystick_right[2]}, 板机={data.joystick_right[3]}")
        print(f"  左臂关节(原始): {[f'{x:4d}' for x in data.arm_joint_left]}")
        print(f"  右臂关节(原始): {[f'{x:4d}' for x in data.arm_joint_right]}")
        print(f"  左臂关节(弧度): {[f'{x:7.4f}' for x in data.arm_joint_left_rad]}")
        print(f"  右臂关节(弧度): {[f'{x:7.4f}' for x in data.arm_joint_right_rad]}")
    
    # 注册回调函数
    reader.register_callback(on_data_received)
    
    # 启动数据读取
    if not reader.start():
        print("错误：无法启动数据读取器")
        return
    
    print("数据读取已启动，按 Ctrl+C 退出...")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n正在停止数据读取...")
        reader.stop()
        print("演示程序已退出")


# 示例用法
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--gui":
        # 运行GUI演示
        run_gui_demo()
    else:
        # 运行简单的控制台演示
        simple_console_demo()
