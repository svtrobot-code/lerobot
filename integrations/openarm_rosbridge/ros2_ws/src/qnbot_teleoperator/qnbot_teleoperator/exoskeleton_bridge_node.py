#!/usr/bin/env python3
"""
外骨骼-仿真桥接节点
接收 retargeting 后的关节数据 (/left_arm/joint_command, /right_arm/joint_command)
转换为控制器话题 (Float64MultiArray) 和 夹爪动作 (GripperCommand)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from control_msgs.action import GripperCommand
import numpy as np

class ExoskeletonBridgeNode(Node):
    def __init__(self):
        super().__init__('exoskeleton_bridge_node')
        
        # 参数
        self.declare_parameter('gripper_threshold', 0.0001) # 夹爪变化的最小阈值 (米)
        self.gripper_threshold = self.get_parameter('gripper_threshold').value
        
        # 夹爪缩放因子：外骨骼归一化值(0-1) -> 机械臂物理值(米)
        # 默认0.044表示：外骨骼1.0对应机械臂4.4cm，外骨骼0.1对应机械臂4.4mm
        self.declare_parameter('gripper_scaling_factor', 0.044)
        self.gripper_scale = self.get_parameter('gripper_scaling_factor').value

        # 夹爪动作节流与滤波参数
        self.declare_parameter('gripper_min_interval', 0.0)  # 秒
        self.declare_parameter('gripper_filter_alpha', 1.0)  # 0-1
        self.declare_parameter('gripper_close_snap', 0.2)    # 归一化0-1
        self.declare_parameter('gripper_open_snap', 0.3)     # 归一化0-1
        self.gripper_min_interval = float(self.get_parameter('gripper_min_interval').value)
        self.gripper_filter_alpha = float(self.get_parameter('gripper_filter_alpha').value)
        self.gripper_close_snap = float(self.get_parameter('gripper_close_snap').value)
        self.gripper_open_snap = float(self.get_parameter('gripper_open_snap').value)

        # Episode gating (optional): when inactive, skip publishing arm/gripper commands.
        self.declare_parameter('episode_gate_topic', '/leopen/episode_active')
        self.episode_gate_topic = self.get_parameter('episode_gate_topic').value
        self.episode_active = True
        self._episode_gate_seen = False

        def on_episode_gate(msg: Bool) -> None:
            self.episode_active = bool(msg.data)
            self._episode_gate_seen = True

        self.create_subscription(Bool, self.episode_gate_topic, on_episode_gate, 10)

        # ---------------- 左臂 ----------------
        # 订阅
        self.left_arm_sub = self.create_subscription(
            JointState,
            '/left_arm/joint_command',
            self.left_arm_callback,
            10
        )
        
        # 发布: 手臂位置控制
        self.left_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/left_forward_position_controller/commands',
            10
        )
        
        # Action Client: 夹爪控制
        self.left_gripper_client = ActionClient(self, GripperCommand, '/left_gripper_controller/gripper_cmd')
        
        # ---------------- 右臂 ----------------
        # 订阅
        self.right_arm_sub = self.create_subscription(
            JointState,
            '/right_arm/joint_command',
            self.right_arm_callback,
            10
        )
        
        # 发布: 手臂位置控制
        self.right_arm_pub = self.create_publisher(
            Float64MultiArray,
            '/right_forward_position_controller/commands',
            10
        )
        
        # Action Client: 夹爪控制
        self.right_gripper_client = ActionClient(self, GripperCommand, '/right_gripper_controller/gripper_cmd')

        # ---------------- 状态记录 ----------------
        self.last_left_gripper_pos = 0.0  # 初始化为0，避免启动时突然跳变
        self.last_right_gripper_pos = 0.0 # 初始化为0
        self.left_gripper_goal_handle = None
        self.right_gripper_goal_handle = None
        self.filtered_left_gripper_pos = 0.0
        self.filtered_right_gripper_pos = 0.0
        self.last_left_gripper_send_time = 0.0
        self.last_right_gripper_send_time = 0.0
        self.left_gripper_closed_latch = False
        self.right_gripper_closed_latch = False
        
        # 日志计数器
        self.log_counter_left = 0
        self.log_counter_right = 0
        self.log_interval = 50  # 每50次打印一次日志
        self.gripper_log_counter_left = 0
        self.gripper_log_counter_right = 0
        self.gripper_log_interval = 100  # 夹爪日志每100次打印一次
        
        self.get_logger().info(
            f'Exoskeleton Bridge Node 已启动\n'
            f'  夹爪阈值: {self.gripper_threshold}m\n'
            f'  夹爪缩放因子: {self.gripper_scale} (外骨骼1.0 -> 机械臂{self.gripper_scale}m)\n'
            f'  夹爪最小间隔: {self.gripper_min_interval}s\n'
            f'  夹爪滤波系数: {self.gripper_filter_alpha}\n'
            f'  夹爪闭合吸附: {self.gripper_close_snap} -> 解锁: {self.gripper_open_snap}'
        )

    def left_arm_callback(self, msg: JointState):
        self._process_arm_command(
            msg, 
            self.left_arm_pub, 
            self.left_gripper_client, 
            'left',
            '/left_gripper_controller/gripper_cmd'
        )

    def right_arm_callback(self, msg: JointState):
        self._process_arm_command(
            msg, 
            self.right_arm_pub, 
            self.right_gripper_client, 
            'right',
            '/right_gripper_controller/gripper_cmd'
        )

    def _process_arm_command(self, msg: JointState, arm_pub, gripper_client, side, action_name):
        try:
            if not self.episode_active:
                return

            # 1. 处理手臂关节 (前7个)
            if len(msg.position) >= 7:
                arm_positions = list(msg.position[:7])
                
                # 发布 Float64MultiArray
                cmds = Float64MultiArray()
                cmds.data = arm_positions
                arm_pub.publish(cmds)
                
                # ---------------- 日志打印 ----------------
                if side == 'left':
                    self.log_counter_left += 1
                    current_counter = self.log_counter_left
                else:
                    self.log_counter_right += 1
                    current_counter = self.log_counter_right
                
                if current_counter % self.log_interval == 0:
                    # 简略打印关节数据 (保留2位小数)
                    short_pos = [f"{x:.2f}" for x in arm_positions[:4]] 
                    self.get_logger().info(
                        f'[{side.upper()}] 收到: {short_pos}... -> 发布: {short_pos}... (Float64MultiArray)'
                    )

            # 2. 处理夹爪
            # 逻辑参考 openarm_hardware_control_node.py
            gripper_position = 0.0
            has_gripper_data = False

            # 方法1：检查是否有第8个位置（夹爪归一化位置0.0-1.0）
            if len(msg.position) >= 8:
                gripper_position = msg.position[7]
                has_gripper_data = True
            
            # 方法2：通过joint_name查找 gripper_joint
            elif f'{side}_gripper_joint' in msg.name:
                try:
                    gripper_idx = msg.name.index(f'{side}_gripper_joint')
                    if gripper_idx < len(msg.position):
                        gripper_position = msg.position[gripper_idx]
                        has_gripper_data = True
                except ValueError:
                    pass

            if has_gripper_data:
                # 夹爪吸附/滞回，减少接近闭合时的抖动
                if side == 'left':
                    if gripper_position <= self.gripper_close_snap:
                        self.left_gripper_closed_latch = True
                    elif self.left_gripper_closed_latch and gripper_position >= self.gripper_open_snap:
                        self.left_gripper_closed_latch = False
                    if self.left_gripper_closed_latch:
                        gripper_position = 0.0
                else:
                    if gripper_position <= self.gripper_close_snap:
                        self.right_gripper_closed_latch = True
                    elif self.right_gripper_closed_latch and gripper_position >= self.gripper_open_snap:
                        self.right_gripper_closed_latch = False
                    if self.right_gripper_closed_latch:
                        gripper_position = 0.0

                # 应用缩放因子：将外骨骼的归一化值(0-1)转换为机械臂的物理值(米)
                # 例如：gripper_scale=0.02 时，外骨骼0.1 -> 机械臂0.002m(2mm)
                raw_gripper_position = float(gripper_position)
                open_pos = self.gripper_scale
                close_pos = -0.040 if side == 'left' else -0.030
                final_gripper_pos = close_pos if raw_gripper_position < 0.5 else open_pos

                # 夹爪滤波，减少抖动
                if side == 'left':
                    filtered = (self.gripper_filter_alpha * final_gripper_pos +
                                (1.0 - self.gripper_filter_alpha) * self.filtered_left_gripper_pos)
                    self.filtered_left_gripper_pos = filtered
                    final_gripper_pos = filtered
                else:
                    filtered = (self.gripper_filter_alpha * final_gripper_pos +
                                (1.0 - self.gripper_filter_alpha) * self.filtered_right_gripper_pos)
                    self.filtered_right_gripper_pos = filtered
                    final_gripper_pos = filtered
                
                # 调试日志：显示原始值和缩放后的值（降低频率）
                if side == 'left':
                    self.gripper_log_counter_left += 1
                    if self.gripper_log_counter_left % self.gripper_log_interval == 0:
                        self.get_logger().info(
                            f'[{side.upper()}] 夹爪原始值: {gripper_position:.4f} -> 缩放后: {final_gripper_pos:.4f}m (缩放因子: {self.gripper_scale})'
                        )
                else:
                    self.gripper_log_counter_right += 1
                    if self.gripper_log_counter_right % self.gripper_log_interval == 0:
                        self.get_logger().info(
                            f'[{side.upper()}] 夹爪原始值: {gripper_position:.4f} -> 缩放后: {final_gripper_pos:.4f}m (缩放因子: {self.gripper_scale})'
                        )
                
                self._send_gripper_action(gripper_client, final_gripper_pos, side)

        except Exception as e:
            self.get_logger().error(f'Error processing {side} arm command: {e}')

    def _send_gripper_action(self, client, position, side):
        # 检查 Action Server 是否连接
        if not client.server_is_ready():
            # 避免刷屏，可以用 throttle
            # self.get_logger().warn(f'{side} gripper action server not ready', throttle_duration_sec=5)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        last_send_time = self.last_left_gripper_send_time if side == 'left' else self.last_right_gripper_send_time
        if (now - last_send_time) < self.gripper_min_interval:
            return

        # 检查变化阈值
        last_pos = self.last_left_gripper_pos if side == 'left' else self.last_right_gripper_pos
        
        # 如果变化量超过阈值，才发送新命令
        if abs(position - last_pos) > self.gripper_threshold:
            # 发送 Action
            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = float(position)
            goal_msg.command.max_effort = 30.0 # 固定力矩
            
            # 异步发送，不等待结果，以免阻塞
            future = client.send_goal_async(goal_msg)
            # future.add_done_callback(self.goal_response_callback)
            
            # 更新最后位置
            if side == 'left':
                self.last_left_gripper_pos = position
                self.last_left_gripper_send_time = now
            else:
                self.last_right_gripper_pos = position
                self.last_right_gripper_send_time = now
                
            self.get_logger().info(
                f'[{side.upper()}] 夹爪动作请求: {position:.4f} (Action: {client._action_name})'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ExoskeletonBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

