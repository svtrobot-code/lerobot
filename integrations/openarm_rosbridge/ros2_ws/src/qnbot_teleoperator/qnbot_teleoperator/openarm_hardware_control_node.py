#!/usr/bin/env python3
"""
OpenArm硬件控制节点
订阅关节命令话题，调用openarm_single_control.py提供的控制接口
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import sys
import signal
import os

# 导入openarm控制器（所有功能都在openarm_single_control.py中实现）
from qnbot_teleoperator.openarm.openarm_single_control import OpenArmBimanualController


class OpenArmHardwareControlNode(Node):
    """OpenArm硬件控制节点 - 轻量级ROS2封装"""
    
    def __init__(self):
        super().__init__('openarm_hardware_control_node')
        
        # 声明ROS2参数
        self.declare_parameter('usb_serial_number', '1F1AD573CBBDF4AC08D448446A81E925')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('control_frequency', 100.0)  # Hz
        self.declare_parameter('gravity_compensation_factor', 0.8)
        self.declare_parameter('enable_gravity_compensation', True)
        
        # 获取参数
        self.usb_sn = self.get_parameter('usb_serial_number').value
        urdf_path = self.get_parameter('urdf_path').value
        self.control_freq = self.get_parameter('control_frequency').value
        gravity_factor = self.get_parameter('gravity_compensation_factor').value
        enable_gravity = self.get_parameter('enable_gravity_compensation').value
        
        # 如果URDF路径为空，使用默认路径
        if not urdf_path:
            current_file_dir = os.path.dirname(os.path.abspath(__file__))
            urdf_path = os.path.join(current_file_dir, 'openarm', 'mode', 'openarm_bimanual.urdf')
            self.get_logger().info(f'使用默认URDF路径: {urdf_path}')
        
        # 当前关节位置
        self.left_arm_positions = np.zeros(9)   # 7关节 + 2手指
        self.right_arm_positions = np.zeros(9)  # 7关节 + 2手指
        self.left_arm_received = False
        self.right_arm_received = False
        self.last_left_arm_time = self.get_clock().now()
        self.last_right_arm_time = self.get_clock().now()
        
        # 夹爪调试计数器
        self.gripper_debug_counter = 0
        
        # 订阅左右臂关节命令话题
        self.left_arm_sub = self.create_subscription(
            JointState,
            '/left_arm/joint_command',
            self.left_arm_callback,
            10
        )
        
        self.right_arm_sub = self.create_subscription(
            JointState,
            '/right_arm/joint_command',
            self.right_arm_callback,
            10
        )
        
        # OpenArm控制器（使用openarm_single_control.py的统一接口）
        self.controller = None
        self.control_timer = None
        self.init_timer = None
        self.consecutive_errors = 0  # 连续错误计数
        self.max_consecutive_errors = 5  # 最大连续错误次数
        
        self.get_logger().info('OpenArm硬件控制节点初始化完成')
        self.get_logger().info(f'USB序列号: {self.usb_sn}')
        self.get_logger().info(f'控制频率: {self.control_freq} Hz')
        self.get_logger().info(f'重力补偿: {"启用" if enable_gravity else "禁用"}')
        
        # 保存初始化参数，用于延迟初始化
        self.init_params = {
            'usb_serial_number': self.usb_sn,
            'urdf_path': urdf_path,
            'enable_gravity_compensation': enable_gravity,
            'gravity_compensation_factor': gravity_factor
        }
        
        # 延迟启动控制器（等待关节命令数据）
        self.init_timer = self.create_timer(2.0, self.initialize_controller)
        
    def left_arm_callback(self, msg):
        """左臂关节命令回调"""
        try:
            if len(msg.position) >= 7:
                # 复制前7个关节位置
                self.left_arm_positions[:7] = msg.position[:7]
                
                # 处理夹爪数据
                # retargeting节点发布的消息可能包含8个位置（7关节+1夹爪）
                # 或者通过joint_name查找夹爪数据
                gripper_position = 0.0  # 默认值
                
                # 方法1：检查是否有第8个位置（夹爪归一化位置0.0-1.0）
                if len(msg.position) >= 8:
                    gripper_position = msg.position[7]
                
                # 方法2：通过joint_name查找'left_gripper_joint'
                elif 'left_gripper_joint' in msg.name:
                    gripper_idx = msg.name.index('left_gripper_joint')
                    if gripper_idx < len(msg.position):
                        gripper_position = msg.position[gripper_idx]
                
                # 将归一化的夹爪位置(0.0-1.0)转换为OpenArm手指位置(0.0-0.044m)
                # OpenArm的两个手指是镜像控制的，所以使用相同的位置
                finger_position = gripper_position * 0.67
                self.left_arm_positions[7] = finger_position  # finger1
                self.left_arm_positions[8] = finger_position  # finger2
                
                # 每1000次输出一次夹爪调试信息
                self.gripper_debug_counter += 1
                if self.gripper_debug_counter % 100 == 1:
                    self.get_logger().info(
                        f'左臂夹爪控制: 归一化位置={gripper_position:.3f} -> 手指位置={finger_position:.5f}m'
                    )
                
                self.left_arm_received = True
                self.last_left_arm_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f'处理左臂关节命令时出错: {e}')
    
    def right_arm_callback(self, msg):
        """右臂关节命令回调"""
        try:
            if len(msg.position) >= 7:
                # 复制前7个关节位置
                self.right_arm_positions[:7] = msg.position[:7]
                
                # 处理夹爪数据
                # retargeting节点发布的消息可能包含8个位置（7关节+1夹爪）
                # 或者通过joint_name查找夹爪数据
                gripper_position = 0.0  # 默认值
                
                # 方法1：检查是否有第8个位置（夹爪归一化位置0.0-1.0）
                if len(msg.position) >= 8:
                    gripper_position = msg.position[7]
                
                # 方法2：通过joint_name查找'right_gripper_joint'
                elif 'right_gripper_joint' in msg.name:
                    gripper_idx = msg.name.index('right_gripper_joint')
                    if gripper_idx < len(msg.position):
                        gripper_position = msg.position[gripper_idx]
                
                # 将归一化的夹爪位置(0.0-1.0)转换为OpenArm手指位置(0.0-0.044m)
                # OpenArm的两个手指是镜像控制的，所以使用相同的位置
                finger_position = gripper_position * 0.67
                self.right_arm_positions[7] = finger_position  # finger1
                self.right_arm_positions[8] = finger_position  # finger2
                
                # 每1000次输出一次夹爪调试信息
                if self.gripper_debug_counter % 100 == 1:
                    self.get_logger().info(
                        f'右臂夹爪控制: 归一化位置={gripper_position:.3f} -> 手指位置={finger_position:.5f}m'
                    )
                
                self.right_arm_received = True
                self.last_right_arm_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f'处理右臂关节命令时出错: {e}')
    
    def initialize_controller(self):
        """初始化OpenArm控制器"""
        if self.controller is not None:
            # 已初始化，停止定时器
            if self.init_timer is not None:
                self.init_timer.cancel()
                self.init_timer = None
            return
            
        if not (self.left_arm_received and self.right_arm_received):
            # 必须等待左右臂关节命令数据后才能初始化（安全要求）
            return
        
        try:
            self.get_logger().info('开始初始化OpenArm控制器...')
            
            # 使用openarm_single_control.py的OpenArmBimanualController类
            self.controller = OpenArmBimanualController(
                usb_serial_number=self.init_params['usb_serial_number'],
                urdf_path=self.init_params['urdf_path'],
                enable_gravity_compensation=self.init_params['enable_gravity_compensation'],
                gravity_compensation_factor=self.init_params['gravity_compensation_factor']
            )
            
            self.get_logger().info('✅ OpenArm控制器初始化成功！')
            
            # 启动控制定时器
            control_period = 1.0 / self.control_freq
            self.control_timer = self.create_timer(control_period, self.control_loop)
            
            self.get_logger().info(f'控制定时器已启动，周期: {control_period*1000:.2f}ms')
            
            # 停止初始化定时器
            if self.init_timer is not None:
                self.init_timer.cancel()
                self.init_timer = None
            
        except Exception as e:
            self.get_logger().error(f'初始化OpenArm控制器失败: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def control_loop(self):
        """控制循环 - 调用openarm_single_control.py的位置控制接口"""
        try:
            # 检查左右臂命令超时
            time_since_left = (self.get_clock().now() - self.last_left_arm_time).nanoseconds / 1e9
            time_since_right = (self.get_clock().now() - self.last_right_arm_time).nanoseconds / 1e9
            
            if time_since_left > 1.0 or time_since_right > 1.0:
                self.get_logger().warn('关节命令数据超时', throttle_duration_sec=2.0)
                return
            
            # 调用openarm_single_control.py的位置控制接口
            # 所有底层逻辑（重力补偿计算、电机控制）都在那里完成
            self.controller.control_position(
                left_arm_positions=self.left_arm_positions,
                right_arm_positions=self.right_arm_positions
            )
            
            # 控制成功，重置错误计数
            self.consecutive_errors = 0
            
        except Exception as e:
            self.consecutive_errors += 1
            self.get_logger().error(f'❌ 控制循环出错 ({self.consecutive_errors}/{self.max_consecutive_errors}): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            
            # 连续错误过多，停止控制循环并关闭节点
            if self.consecutive_errors >= self.max_consecutive_errors:
                self.get_logger().fatal(
                    f'🚨 连续{self.consecutive_errors}次控制失败！可能是USB通信故障或电机失能！'
                    '\n正在紧急停止控制循环...'
                )
                # 停止控制定时器
                if self.control_timer:
                    self.control_timer.cancel()
                    self.control_timer = None
                
                # 尝试安全关闭控制器
                try:
                    if self.controller:
                        self.controller.close()
                except:
                    pass
                
                self.get_logger().fatal('控制器已紧急停止，请检查硬件连接和电机状态！')
    
    def shutdown(self):
        """关闭节点"""
        self.get_logger().info('正在关闭OpenArm硬件控制节点...')
        
        # 停止控制定时器
        if self.control_timer:
            self.control_timer.cancel()
            self.get_logger().info('控制定时器已停止')
        
        # 停止初始化定时器
        if self.init_timer:
            self.init_timer.cancel()
        
        # 关闭控制器
        if self.controller:
            try:
                self.controller.close()
                self.get_logger().info('OpenArm控制器已关闭')
            except Exception as e:
                self.get_logger().error(f'关闭控制器失败: {e}')
        
        self.get_logger().info('OpenArm硬件控制节点已安全关闭')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = OpenArmHardwareControlNode()
    
    def signal_handler(sig, frame):
        """信号处理器"""
        node.get_logger().info('接收到中断信号')
        node.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
