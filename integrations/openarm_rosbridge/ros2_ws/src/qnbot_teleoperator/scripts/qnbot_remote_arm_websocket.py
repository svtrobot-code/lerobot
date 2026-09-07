#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QnBot外骨骼数据WebSocket发送器
通过WebSocket将外骨骼数据实时发送给机器人控制系统

功能特点：
1. 100Hz高频率实时数据发送
2. 速度限制滤波（最大1rad/s）
3. 直接位置控制
4. 安全保护机制
5. 摇杆控制小车移动和滑台升降

使用方法：
python3 qnbot_remote_arm_websocket.py --arm_mode left               # 仅左臂
python3 qnbot_remote_arm_websocket.py --arm_mode right              # 仅右臂
python3 qnbot_remote_arm_websocket.py --arm_mode both               # 双臂同步
python3 qnbot_remote_arm_websocket.py --arm_mode both_arms_only     # 仅双臂（无滑台、小车）
python3 qnbot_remote_arm_websocket.py --arm_mode both --no_filter   # 双臂同步（轻量级滤波）
python3 qnbot_remote_arm_websocket.py --arm_mode both --no_filter --smooth_alpha 0.3  # 自定义平滑度

作者：QnBot团队
版本：2.0 (WebSocket版本)
"""

import asyncio
import websockets
import json
import numpy as np
import time
import math
import argparse
import threading
import signal
import sys
import os
from remote_manipulator_data_reader import RemoteManipulatorReader, RemoteManipulatorData
import struct

class QnbotWebSocketSender:
    def __init__(self, arm_mode='right', enable_filter=True, smooth_alpha=0.2, websocket_url='ws://0.0.0.0:19091'):
        """
        初始化WebSocket发送器
        
        Args:
            arm_mode: 控制模式 ('left', 'right', 'both', 'both_arms_only')
            enable_filter: 是否启用完整速度滤波
            smooth_alpha: 平滑滤波系数
            websocket_url: WebSocket服务器地址
        """
        # 机械臂控制模式配置
        self.arm_mode = arm_mode
        self.control_left = arm_mode in ['left', 'both', 'both_arms_only']
        self.control_right = arm_mode in ['right', 'both', 'both_arms_only']
        
        # 滑台和小车控制配置（仅双臂模式禁用）
        self.enable_vehicle_control = arm_mode != 'both_arms_only'
        
        # 滤波控制配置
        self.enable_filter = enable_filter
        
        # 平滑滤波参数（用于"禁用滤波"模式的基础抖动抑制）
        self.smooth_alpha = smooth_alpha
        
        # WebSocket配置
        self.websocket_url = websocket_url
        self.websocket = None
        self.websocket_connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 2.0
        
        # 控制参数
        self.control_rate = 100.0  # 100Hz控制频率
        self.time_step = 1.0 / self.control_rate
        self.max_joint_velocity = 24  # 最大关节速度 2rad/s
        
        # ================= 摇杆和小车滑台控制参数 =================
        # 摇杆数据范围和死区设置
        self.joystick_min = 200
        self.joystick_max = 3896
        self.joystick_center = 2048
        self.joystick_deadzone = 300
        
        # 小车速度限制
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 1.0
        
        # 滑台速度控制参数
        self.max_elevator_velocity = 0.15
        self.max_elevator_acceleration = 0.15
        self.current_elevator_velocity = 0.0
        self.target_elevator_velocity = 0.0
        
        # 滤波参数
        self.elevator_filter_alpha = 0.1
        # ================================================================
        
        # ================= 按钮控制状态管理 =================
        # 按钮状态（1=未按下，0=按下）
        self.left_button_last_state = 1
        self.right_button_last_state = 1
        
        # 控制启停状态
        self.left_arm_control_enabled = False
        self.right_arm_control_enabled = False
        self.left_joystick_control_enabled = False
        self.right_joystick_control_enabled = False
        
        # 按钮点击计数器
        self.left_button_click_count = 0
        self.right_button_click_count = 0
        
        # ================= 数据发送状态管理 =================
        # 数据发送状态：'not_started', 'transmitting', 'paused'
        self.transmission_state = 'not_started'
        
        # 1秒长按检测相关
        self.transmission_long_press_duration = 1.0
        self.transmission_long_press_start_time = None
        self.transmission_long_press_triggered = False
        
        # 暂停时保存的数据
        self.paused_positions_left = None
        self.paused_positions_right = None
        self.paused_gripper_left = None
        self.paused_gripper_right = None
        # ========================================================
        # ============================================================
        
        # ================= 长按和回零控制状态 =================
        # 长按检测相关
        self.left_button_press_start_time = None
        self.right_button_press_start_time = None
        self.long_press_duration = 2.0
        self.long_press_triggered = False
        
        # 防抖检测相关
        self.debounce_duration = 0.05
        
        # 点击检测相关
        self.left_button_pressed = False
        self.right_button_pressed = False
        
        # 回零流程控制
        self.homing_active = False
        self.homing_start_time = None
        self.homing_current_joint_left = 0
        self.homing_current_joint_right = 0
        self.homing_joint_interval = 1.0
        self.homing_last_joint_time_left = None
        self.homing_last_joint_time_right = None
        self.total_joints_to_home = 8
        
        # 插值回零参数
        self.interpolation_steps = 3
        self.step_duration = self.homing_joint_interval / self.interpolation_steps
        
        # 回零顺序映射
        self.homing_joint_order = [1, 0, 2, 3, 4, 5, 6, 7]
        
        # 记录关节开始位置
        self.homing_start_positions_left = None
        self.homing_start_positions_right = None
        self.homing_start_gripper_left = None
        self.homing_start_gripper_right = None
        # ============================================================
        
        # 机械臂关节配置
        self.num_joints = 7
        
        # 回零目标位置
        self.homing_positions = np.zeros(self.num_joints)
        self.homing_gripper_position = -0.0120
        
        # 关节限制 (弧度)
        self.joint_limits = {
            # 左臂关节限制
            'left_arm_joint_1': (-2.356194, 2.356194),
            'left_arm_joint_2': (-1.832592, 1.832599),
            'left_arm_joint_3': (-2.094395, 2.094395),
            'left_arm_joint_4': (-1.614426, 1.614433),
            'left_arm_joint_5': (-2.094395, 2.094395),
            'left_arm_joint_6': (-1.570796, 1.570796),
            'left_arm_joint_7': (-1.745329, 1.221730),
            # 右臂关节限制
            'right_arm_joint_1': (-2.356194, 2.356194),
            'right_arm_joint_2': (-1.832592, 1.832599),
            'right_arm_joint_3': (-2.094395, 2.094395),
            'right_arm_joint_4': (-1.614426, 1.614433),
            'right_arm_joint_5': (-2.094395, 2.094395),
            'right_arm_joint_6': (-1.570796, 1.570796),
            'right_arm_joint_7': (-1.745329, 1.221730)
        }
        
        # 夹爪限制（米）
        self.gripper_limits = {
            'left_gripper_joint': (-0.0120, 0.0),
            'right_gripper_joint': (-0.0120, 0.0)
        }
        
        # 扳机数据映射参数
        self.trigger_min = 700
        self.trigger_max = 2700
        
        # 当前关节状态
        self.current_positions_left = np.zeros(self.num_joints)
        self.current_positions_right = np.zeros(self.num_joints)
        self.target_positions_left = np.zeros(self.num_joints)
        self.target_positions_right = np.zeros(self.num_joints)
        self.filtered_positions_left = np.zeros(self.num_joints)
        self.filtered_positions_right = np.zeros(self.num_joints)
        self.last_filtered_positions_left = np.zeros(self.num_joints)
        self.last_filtered_positions_right = np.zeros(self.num_joints)
        
        # 夹爪状态
        self.current_gripper_position_left = 0.0
        self.current_gripper_position_right = 0.0
        self.target_gripper_position_left = 0.0
        self.target_gripper_position_right = 0.0
        self.filtered_gripper_position_left = 0.0
        self.filtered_gripper_position_right = 0.0
        self.last_filtered_gripper_position_left = 0.0
        self.last_filtered_gripper_position_right = 0.0
        
        # 初始化标志
        self.exoskeleton_initialized = False
        self.control_active = False
        
        # 统计信息
        self.stats = {
            'total_sent': 0,
            'send_errors': 0,
            'connection_errors': 0,
            'last_send_time': 0
        }
        
        # 创建外骨骼数据读取器
        self.exoskeleton_reader = RemoteManipulatorReader(
            port=os.environ.get("EXO_PORT", "/dev/ttyACM0"),
            baudrate=2000000
        )
        
        # 事件循环和任务
        self.loop = None
        self.control_task = None
        self.websocket_task = None
        self.running = True
        
        print(f'{arm_mode}外骨骼WebSocket发送器初始化完成')
    
    def log_info(self, message):
        """日志输出"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        print(f"[{timestamp}] INFO: {message}")
    
    def log_error(self, message):
        """错误日志输出"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        print(f"[{timestamp}] ERROR: {message}")
    
    def log_debug(self, message):
        """调试日志输出"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        print(f"[{timestamp}] DEBUG: {message}")
    
    async def connect_websocket(self):
        """连接WebSocket服务器"""
        while self.running and self.reconnect_attempts < self.max_reconnect_attempts:
            try:
                self.log_info(f"尝试连接WebSocket服务器: {self.websocket_url}")
                self.websocket = await websockets.connect(self.websocket_url)
                self.websocket_connected = True
                self.reconnect_attempts = 0
                self.log_info("WebSocket连接成功")
                return True
                
            except Exception as e:
                self.reconnect_attempts += 1
                self.stats['connection_errors'] += 1
                self.log_error(f"WebSocket连接失败 (尝试 {self.reconnect_attempts}/{self.max_reconnect_attempts}): {e}")
                
                if self.reconnect_attempts < self.max_reconnect_attempts:
                    self.log_info(f"{self.reconnect_delay}秒后重试...")
                    await asyncio.sleep(self.reconnect_delay)
                
        return False
    
    async def send_websocket_message(self, data):
        """发送WebSocket消息"""
        if not self.websocket_connected or not self.websocket:
            return False
        
        try:
            # 转换为JSON格式
            message = json.dumps(data)
            await self.websocket.send(message)
            self.stats['total_sent'] += 1
            self.stats['last_send_time'] = time.time()
            return True
            
        except websockets.exceptions.ConnectionClosed:
            self.log_error("WebSocket连接已关闭")
            self.websocket_connected = False
            return False
        except Exception as e:
            self.stats['send_errors'] += 1
            self.log_error(f"发送WebSocket消息失败: {e}")
            return False
    
    def _wait_for_exoskeleton_data(self):
        """等待外骨骼数据可用"""
        timeout = 10.0
        start_time = time.time()
        
        while not self.exoskeleton_initialized:
            data = self.exoskeleton_reader.get_latest_data()
            
            if data.timestamp > 0:
                if self.control_left:
                    raw_data_left = np.array(data.arm_joint_left_rad[:self.num_joints])
                    # 应用关节符号反转（左臂J2, J3添加负号）
                    inverted_data_left = self._apply_joint_sign_inversion(raw_data_left, 'left')
                    self.target_positions_left = self._apply_joint_limits(inverted_data_left, 'left')
                
                if self.control_right:
                    raw_data_right = np.array(data.arm_joint_right_rad[:self.num_joints])
                    # 应用关节符号反转（右臂J1-J5添加负号）
                    inverted_data_right = self._apply_joint_sign_inversion(raw_data_right, 'right')
                    self.target_positions_right = self._apply_joint_limits(inverted_data_right, 'right')
                
                self.exoskeleton_initialized = True
                self.log_info(f'{self.arm_mode}侧外骨骼数据已初始化')
                break
            
            if time.time() - start_time > timeout:
                self.log_error('等待外骨骼数据超时')
                break
            
            time.sleep(0.1)
    
    def _apply_joint_limits(self, positions, arm_side):
        """应用关节限制"""
        limited_positions = np.zeros(self.num_joints)
        
        joint_names = [f'{arm_side}_arm_joint_{i+1}' for i in range(self.num_joints)]
        
        for i, joint_name in enumerate(joint_names):
            if i < len(positions):
                min_val, max_val = self.joint_limits[joint_name]
                limited_positions[i] = np.clip(positions[i], min_val, max_val)
        
        return limited_positions
    
    def _apply_joint_sign_inversion(self, positions, arm_side):
        """应用特定关节的符号反转
        
        Args:
            positions: 关节位置数组（弧度）
            arm_side: 'left' 或 'right'
            
        Returns:
            应用符号反转后的关节位置数组
        """
        inverted_positions = positions.copy()
        
        if arm_side == 'right':
            # 右臂：J1, J2, J3, J4, J5 (索引 0-4) 添加负号
            for i in range(5):  # J1-J5 对应索引 0-4
                if i < len(inverted_positions):
                    inverted_positions[i] = -inverted_positions[i]
        elif arm_side == 'left':
            # 左臂：J2, J3 (索引 1-2) 添加负号
            for i in [1, 2]:  # J2, J3 对应索引 1, 2
                if i < len(inverted_positions):
                    inverted_positions[i] = -inverted_positions[i]
        
        return inverted_positions
    
    def _apply_joint_sign_inversion_raw(self, raw_values, arm_side):
        """应用特定关节的符号反转（原始编码器数据）
        
        Args:
            raw_values: 原始编码器数据数组
            arm_side: 'left' 或 'right'
            
        Returns:
            应用符号反转后的原始编码器数据数组
        """
        inverted_values = raw_values.copy() if hasattr(raw_values, 'copy') else list(raw_values)
        
        if arm_side == 'right':
            # 右臂：J1, J2, J3, J4, J5 (索引 0-4) 添加负号
            for i in range(5):  # J1-J5 对应索引 0-4
                if i < len(inverted_values):
                    inverted_values[i] = -inverted_values[i]
        elif arm_side == 'left':
            # 左臂：J2, J3 (索引 1-2) 添加负号
            for i in [1, 2]:  # J2, J3 对应索引 1, 2
                if i < len(inverted_values):
                    inverted_values[i] = -inverted_values[i]
        
        return inverted_values
    
    def map_trigger_to_gripper(self, trigger_value, arm_side):
        """映射扳机数据到夹爪位置"""
        trigger_value = np.clip(trigger_value, self.trigger_min, self.trigger_max)
        trigger_normalized = (trigger_value - self.trigger_min) / (self.trigger_max - self.trigger_min)
        
        gripper_joint_name = f'{arm_side}_gripper_joint'
        gripper_min, gripper_max = self.gripper_limits[gripper_joint_name]
        gripper_position = gripper_min + trigger_normalized * (gripper_max - gripper_min)
        
        return gripper_position
    
    # ================= 摇杆数据处理方法 =================
    def apply_joystick_deadzone(self, raw_value):
        """应用摇杆死区处理"""
        if abs(raw_value - self.joystick_center) <= self.joystick_deadzone:
            return self.joystick_center
        return raw_value
    
    def map_joystick_to_velocity(self, joystick_value, max_velocity):
        """将摇杆值映射到速度"""
        filtered_value = self.apply_joystick_deadzone(joystick_value)
        offset = filtered_value - self.joystick_center
        
        if abs(offset) <= self.joystick_deadzone:
            return 0.0
        
        if offset > 0:
            max_offset = self.joystick_max - self.joystick_center - self.joystick_deadzone
            normalized_offset = (offset - self.joystick_deadzone) / max_offset
        else:
            max_offset = self.joystick_center - self.joystick_min - self.joystick_deadzone
            normalized_offset = (offset + self.joystick_deadzone) / max_offset
        
        normalized_offset = np.clip(normalized_offset, -1.0, 1.0)
        velocity = normalized_offset * max_velocity
        
        return velocity
    
    def apply_elevator_velocity_filter(self, target_velocity):
        """应用滑台速度滤波和加速度限制"""
        velocity_change = target_velocity - self.current_elevator_velocity
        max_velocity_change = self.max_elevator_acceleration * self.time_step
        
        if abs(velocity_change) > max_velocity_change:
            if velocity_change > 0:
                velocity_change = max_velocity_change
            else:
                velocity_change = -max_velocity_change
        
        self.current_elevator_velocity += velocity_change
        
        filtered_velocity = (self.elevator_filter_alpha * target_velocity + 
                           (1 - self.elevator_filter_alpha) * self.current_elevator_velocity)
        
        filtered_velocity = np.clip(filtered_velocity, -self.max_elevator_velocity, self.max_elevator_velocity)
        
        return filtered_velocity
    # =============================================================
    
    # ================= 按钮控制处理方法 =================
    def detect_button_clicks(self, exo_data):
        """检测按钮点击事件"""
        current_time = time.time()
        
        # 当前外骨骼按钮是 active-low bitmask:
        # 空闲通常 raw=31；按下某个按钮时某一 bit 从 1 变 0，
        # 例如 31->29 / 27 / 23。原代码按 0/1 判断，无法识别。
        left_button_raw = int(exo_data.joystick_left[2]) & 0xFF
        right_button_raw = int(exo_data.joystick_right[2]) & 0xFF

        # 低 5 位任意一位变 0，认为该侧按钮被按下。
        # 输出仍保持旧逻辑：1=未按下，0=按下。
        left_button_current = 0 if (left_button_raw & 0x1F) != 0x1F else 1
        right_button_current = 0 if (right_button_raw & 0x1F) != 0x1F else 1
        
        left_button_clicked = False
        right_button_clicked = False
        
        # 处理左按钮状态变化
        if self.left_button_last_state == 1 and left_button_current == 0:
            self.left_button_pressed = True
            self.left_button_press_start_time = current_time
            
        elif self.left_button_last_state == 0 and left_button_current == 1:
            if (self.left_button_pressed and not self.long_press_triggered and 
                self.left_button_press_start_time is not None):
                press_duration = current_time - self.left_button_press_start_time
                if press_duration >= self.debounce_duration:
                    left_button_clicked = True
                    self.log_debug(f"左按钮完成有效点击 (持续{press_duration*1000:.0f}ms)")
                else:
                    self.log_debug(f"左按钮点击过短，忽略 (持续{press_duration*1000:.0f}ms < {self.debounce_duration*1000:.0f}ms)")
            
            self.left_button_pressed = False
            self.left_button_press_start_time = None
            self.long_press_triggered = False
        
        # 处理右按钮状态变化
        if self.right_button_last_state == 1 and right_button_current == 0:
            self.right_button_pressed = True
            self.right_button_press_start_time = current_time
            
        elif self.right_button_last_state == 0 and right_button_current == 1:
            if (self.right_button_pressed and not self.long_press_triggered and 
                self.right_button_press_start_time is not None):
                press_duration = current_time - self.right_button_press_start_time
                if press_duration >= self.debounce_duration:
                    right_button_clicked = True
                    self.log_debug(f"右按钮完成有效点击 (持续{press_duration*1000:.0f}ms)")
                else:
                    self.log_debug(f"右按钮点击过短，忽略 (持续{press_duration*1000:.0f}ms < {self.debounce_duration*1000:.0f}ms)")
            
            self.right_button_pressed = False
            self.right_button_press_start_time = None
            self.long_press_triggered = False
        
        # 检测双边长按
        if left_button_current == 0 and right_button_current == 0:
            # 先检测1秒长按用于数据发送控制
            self.check_dual_long_press_for_transmission(current_time)
            # 再检测2秒长按用于回零 - 已注释掉
            # self.check_dual_long_press_for_homing(current_time)
        else:
            # 按钮释放时重置1秒长按状态
            self.transmission_long_press_start_time = None
            self.transmission_long_press_triggered = False
        
        # 处理左按钮点击
        if left_button_clicked:  # 移除了homing_active检查
            self.left_button_click_count += 1
            self.left_arm_control_enabled = not self.left_arm_control_enabled
            
            if self.enable_vehicle_control:
                self.left_joystick_control_enabled = not self.left_joystick_control_enabled
                status = "启用" if self.left_arm_control_enabled else "禁用"
                self.log_info(f"左按钮点击 #{self.left_button_click_count}: 左臂和左摇杆控制已{status}")
            else:
                status = "启用" if self.left_arm_control_enabled else "禁用"
                self.log_info(f"左按钮点击 #{self.left_button_click_count}: 左臂控制已{status}")
        
        # 处理右按钮点击
        if right_button_clicked:  # 移除了homing_active检查
            self.right_button_click_count += 1
            self.right_arm_control_enabled = not self.right_arm_control_enabled
            
            if self.enable_vehicle_control:
                self.right_joystick_control_enabled = not self.right_joystick_control_enabled
                status = "启用" if self.right_arm_control_enabled else "禁用"
                self.log_info(f"右按钮点击 #{self.right_button_click_count}: 右臂和右摇杆控制已{status}")
            else:
                status = "启用" if self.right_arm_control_enabled else "禁用"
                self.log_info(f"右按钮点击 #{self.right_button_click_count}: 右臂控制已{status}")
        
        self.left_button_last_state = left_button_current
        self.right_button_last_state = right_button_current
        
        return left_button_clicked, right_button_clicked
    
    def check_dual_long_press_for_transmission(self, current_time):
        """检测双边1秒长按触发数据发送状态切换"""
        both_buttons_pressed = (self.left_button_pressed and self.right_button_pressed and
                               self.left_button_press_start_time is not None and 
                               self.right_button_press_start_time is not None)
        
        if both_buttons_pressed and not self.transmission_long_press_triggered:
            if self.transmission_long_press_start_time is None:
                self.transmission_long_press_start_time = current_time
            
            press_duration = current_time - self.transmission_long_press_start_time
            
            # 每0.2秒显示一次提示信息
            if int(press_duration * 5) != int((press_duration - 0.01) * 5):
                remaining = self.transmission_long_press_duration - press_duration
                if remaining > 0:
                    if self.transmission_state == 'not_started':
                        self.log_info(f"🚀 双边长按检测中... 剩余{remaining:.1f}秒开始数据发送")
                    elif self.transmission_state == 'transmitting':
                        self.log_info(f"⏸️ 双边长按检测中... 剩余{remaining:.1f}秒暂停发送")
                    elif self.transmission_state == 'paused':
                        self.log_info(f"▶️ 双边长按检测中... 剩余{remaining:.1f}秒恢复发送")
            
            if press_duration >= self.transmission_long_press_duration:
                self.transmission_long_press_triggered = True
                self.toggle_transmission_state()
    
    def toggle_transmission_state(self):
        """切换数据发送状态"""
        if self.transmission_state == 'not_started':
            self.transmission_state = 'transmitting'
            self.log_info("🚀 数据发送已开始！")
            
        elif self.transmission_state == 'transmitting':
            self.transmission_state = 'paused'
            # 保存当前位置数据用于暂停期间发送
            self.save_current_positions_for_pause()
            self.log_info("⏸️ 数据发送已暂停，机械臂将保持当前位置")
            
        elif self.transmission_state == 'paused':
            self.transmission_state = 'transmitting'
            self.log_info("▶️ 数据发送已恢复，开始使用新的位置控制")
    
    def save_current_positions_for_pause(self):
        """保存当前位置用于暂停期间发送"""
        if self.control_left:
            self.paused_positions_left = self.filtered_positions_left.copy()
            self.paused_gripper_left = self.filtered_gripper_position_left
        
        if self.control_right:
            self.paused_positions_right = self.filtered_positions_right.copy()
            self.paused_gripper_right = self.filtered_gripper_position_right
        
        self.log_debug("已保存当前位置用于暂停期间发送")
    
    def process_new_exoskeleton_data(self, exo_data):
        """处理新的外骨骼数据"""
        # 处理左臂控制
        if self.control_left and self.left_arm_control_enabled:
            raw_targets_left = np.array(exo_data.arm_joint_left_rad[:self.num_joints])
            # 应用关节符号反转（左臂J2, J3添加负号）
            inverted_targets_left = self._apply_joint_sign_inversion(raw_targets_left, 'left')
            limited_targets_left = self._apply_joint_limits(inverted_targets_left, 'left')
            
            if self.enable_filter:
                filtered_targets_left = self.apply_velocity_filter(limited_targets_left, self.last_filtered_positions_left)
            else:
                filtered_targets_left = self.apply_smooth_filter(limited_targets_left, self.filtered_positions_left)
            
            # 处理左侧扳机数据控制夹爪
            trigger_value_left = exo_data.joystick_left[3]
            gripper_target_left = self.map_trigger_to_gripper(trigger_value_left, 'left')
            
            # 检查位置变化
            position_change_left = np.linalg.norm(filtered_targets_left - self.filtered_positions_left)
            gripper_change_left = abs(gripper_target_left - self.filtered_gripper_position_left)
            
            if position_change_left > 0.001:
                self.last_filtered_positions_left = self.filtered_positions_left.copy()
                self.filtered_positions_left = filtered_targets_left.copy()
            
            if gripper_change_left > 0.0001:
                self.filtered_gripper_position_left = gripper_target_left
        
        # 处理右臂控制
        if self.control_right and self.right_arm_control_enabled:
            raw_targets_right = np.array(exo_data.arm_joint_right_rad[:self.num_joints])
            # 应用关节符号反转（右臂J1-J5添加负号）
            inverted_targets_right = self._apply_joint_sign_inversion(raw_targets_right, 'right')
            limited_targets_right = self._apply_joint_limits(inverted_targets_right, 'right')
            
            if self.enable_filter:
                filtered_targets_right = self.apply_velocity_filter(limited_targets_right, self.last_filtered_positions_right)
            else:
                filtered_targets_right = self.apply_smooth_filter(limited_targets_right, self.filtered_positions_right)
            
            # 处理右侧扳机数据控制夹爪
            trigger_value_right = exo_data.joystick_right[3]
            gripper_target_right = self.map_trigger_to_gripper(trigger_value_right, 'right')
            
            # 检查位置变化
            position_change_right = np.linalg.norm(filtered_targets_right - self.filtered_positions_right)
            gripper_change_right = abs(gripper_target_right - self.filtered_gripper_position_right)
            
            if position_change_right > 0.001:
                self.last_filtered_positions_right = self.filtered_positions_right.copy()
                self.filtered_positions_right = filtered_targets_right.copy()
            
            if gripper_change_right > 0.0001:
                self.filtered_gripper_position_right = gripper_target_right
    
    def use_paused_position_data(self):
        """使用暂停时保存的位置数据"""
        # 使用保存的位置数据，不更新filtered_positions
        if self.control_left and self.paused_positions_left is not None:
            # 在暂停模式下保持使用保存的位置
            pass  # filtered_positions_left 保持不变
        
        if self.control_right and self.paused_positions_right is not None:
            # 在暂停模式下保持使用保存的位置
            pass  # filtered_positions_right 保持不变
    
    def check_dual_long_press_for_homing(self, current_time):
        """检测双边长按触发回零功能"""
        # 检查触发条件
        arms_enabled = []
        if self.control_left:
            arms_enabled.append(self.left_arm_control_enabled)
        if self.control_right:
            arms_enabled.append(self.right_arm_control_enabled)
        
        if any(arms_enabled) or self.homing_active or self.long_press_triggered:
            return
        
        both_buttons_pressed = (self.left_button_pressed and self.right_button_pressed and
                               self.left_button_press_start_time is not None and 
                               self.right_button_press_start_time is not None)
        
        if both_buttons_pressed:
            min_press_start_time = max(self.left_button_press_start_time, self.right_button_press_start_time)
            press_duration = current_time - min_press_start_time
            
            if int(press_duration * 2) != int((press_duration - 0.01) * 2):
                remaining = self.long_press_duration - press_duration
                if remaining > 0:
                    self.log_info(f"🔄 双边长按检测中... 剩余{remaining:.1f}秒触发回零")
            
            if press_duration >= self.long_press_duration:
                self.long_press_triggered = True
                self.start_homing_sequence()
    
    def start_homing_sequence(self):
        """开始机械臂回零流程"""
        self.log_info("🔄 双边长按2秒检测到！开始机械臂回零流程...")
        
        self.homing_active = True
        self.homing_start_time = time.time()
        self.homing_current_joint_left = 0
        self.homing_current_joint_right = 0
        self.homing_last_joint_time_left = None
        self.homing_last_joint_time_right = None
        
        # 禁用所有控制
        self.left_arm_control_enabled = False
        self.right_arm_control_enabled = False
        self.left_joystick_control_enabled = False
        self.right_joystick_control_enabled = False
        
        self.log_info("回零参数: 关节2→关节1→关节3-7→夹爪，每个关节间隔3秒，插值平滑运动")
    
    def process_homing_sequence(self):
        """处理回零流程的时序控制"""
        if not self.homing_active:
            return
        
        current_time = time.time()
        
        # 处理左臂回零
        if (self.control_left and 
            self.homing_current_joint_left < self.total_joints_to_home):
            
            if self.homing_last_joint_time_left is None:
                self.homing_start_positions_left = self.current_positions_left.copy()
                self.homing_start_gripper_left = self.current_gripper_position_left
                self.homing_last_joint_time_left = current_time
                elapsed_time = 0.0
                self.home_single_joint('left', self.homing_current_joint_left, elapsed_time, log_message=True)
            elif current_time - self.homing_last_joint_time_left >= self.homing_joint_interval:
                self.homing_current_joint_left += 1
                if self.homing_current_joint_left < self.total_joints_to_home:
                    actual_joint_index = self.homing_joint_order[self.homing_current_joint_left]
                    if actual_joint_index >= self.num_joints:
                        self.homing_start_gripper_left = self.current_gripper_position_left
                    
                    elapsed_time = 0.0
                    self.home_single_joint('left', self.homing_current_joint_left, elapsed_time, log_message=True)
                self.homing_last_joint_time_left = current_time
            else:
                elapsed_time = current_time - self.homing_last_joint_time_left
                self.home_single_joint('left', self.homing_current_joint_left, elapsed_time, log_message=False)
        
        # 处理右臂回零
        if (self.control_right and 
            self.homing_current_joint_right < self.total_joints_to_home):
            
            if self.homing_last_joint_time_right is None:
                self.homing_start_positions_right = self.current_positions_right.copy()
                self.homing_start_gripper_right = self.current_gripper_position_right
                self.homing_last_joint_time_right = current_time
                elapsed_time = 0.0
                self.home_single_joint('right', self.homing_current_joint_right, elapsed_time, log_message=True)
            elif current_time - self.homing_last_joint_time_right >= self.homing_joint_interval:
                self.homing_current_joint_right += 1
                if self.homing_current_joint_right < self.total_joints_to_home:
                    actual_joint_index = self.homing_joint_order[self.homing_current_joint_right]
                    if actual_joint_index >= self.num_joints:
                        self.homing_start_gripper_right = self.current_gripper_position_right
                    
                    elapsed_time = 0.0
                    self.home_single_joint('right', self.homing_current_joint_right, elapsed_time, log_message=True)
                self.homing_last_joint_time_right = current_time
            else:
                elapsed_time = current_time - self.homing_last_joint_time_right
                self.home_single_joint('right', self.homing_current_joint_right, elapsed_time, log_message=False)
        
        # 检查回零流程是否完成
        left_done = not self.control_left or self.homing_current_joint_left >= self.total_joints_to_home
        right_done = not self.control_right or self.homing_current_joint_right >= self.total_joints_to_home
        
        if left_done and right_done:
            self.finish_homing_sequence()
    
    def home_single_joint(self, arm_side, step_index, elapsed_time, log_message=False):
        """回零单个关节（带插值）"""
        actual_joint_index = self.homing_joint_order[step_index]
        
        if actual_joint_index < self.num_joints:
            if log_message:
                self.log_info(f"🔄 {arm_side}臂关节{actual_joint_index + 1}回零中... (插值平滑运动)")
            
            if arm_side == 'left':
                current_positions = self.current_positions_left.copy()
                start_positions = self.homing_start_positions_left
            else:
                current_positions = self.current_positions_right.copy()
                start_positions = self.homing_start_positions_right
            
            target_positions = current_positions.copy()
            
            # 处理已经回零的关节
            for i in range(step_index):
                completed_joint_index = self.homing_joint_order[i]
                if completed_joint_index < self.num_joints:
                    target_positions[completed_joint_index] = 0.0
            
            # 处理当前正在回零的关节
            if start_positions is not None:
                start_value = start_positions[actual_joint_index]
                target_value = 0.0
                interpolated_value = self.calculate_interpolated_position(
                    start_value, target_value, elapsed_time, self.homing_joint_interval
                )
                target_positions[actual_joint_index] = interpolated_value
            else:
                target_positions[actual_joint_index] = 0.0
            
            # 更新对应臂的位置
            if arm_side == 'left':
                self.filtered_positions_left = target_positions
            else:
                self.filtered_positions_right = target_positions
            
        elif actual_joint_index == self.num_joints:
            # 夹爪回零
            if log_message:
                self.log_info(f"🔄 {arm_side}夹爪回零中... (插值平滑运动)")
            
            if arm_side == 'left':
                start_gripper = self.homing_start_gripper_left
            else:
                start_gripper = self.homing_start_gripper_right
            
            if start_gripper is not None:
                interpolated_gripper = self.calculate_interpolated_position(
                    start_gripper, self.homing_gripper_position, elapsed_time, self.homing_joint_interval
                )
            else:
                interpolated_gripper = self.homing_gripper_position
            
            # 更新对应夹爪位置
            if arm_side == 'left':
                self.filtered_gripper_position_left = interpolated_gripper
            else:
                self.filtered_gripper_position_right = interpolated_gripper
    
    def calculate_interpolated_position(self, start_value, target_value, elapsed_time, total_time):
        """计算插值位置"""
        if elapsed_time >= total_time:
            return target_value
        
        current_step = min(int(elapsed_time / self.step_duration), self.interpolation_steps - 1)
        
        step_start_time = current_step * self.step_duration
        step_elapsed = elapsed_time - step_start_time
        step_progress = min(step_elapsed / self.step_duration, 1.0)
        
        if current_step == 0:
            point1 = start_value + (target_value - start_value) * (1.0 / 3.0)
            return start_value + (point1 - start_value) * step_progress
        elif current_step == 1:
            point1 = start_value + (target_value - start_value) * (1.0 / 3.0)
            point2 = start_value + (target_value - start_value) * (2.0 / 3.0)
            return point1 + (point2 - point1) * step_progress
        else:
            point2 = start_value + (target_value - start_value) * (2.0 / 3.0)
            return point2 + (target_value - point2) * step_progress
    
    def finish_homing_sequence(self):
        """完成回零流程"""
        self.homing_active = False
        self.long_press_triggered = False
        total_time = time.time() - self.homing_start_time
        
        self.homing_start_positions_left = None
        self.homing_start_positions_right = None
        self.homing_start_gripper_left = None
        self.homing_start_gripper_right = None
        
        self.log_info(f"✅ 机械臂回零流程完成！总耗时: {total_time:.1f}秒")
    # =========================================================
    
    def apply_velocity_filter(self, new_targets, last_filtered_positions):
        """应用速度限制滤波器"""
        filtered_targets = np.zeros(self.num_joints)
        
        for i in range(self.num_joints):
            desired_change = new_targets[i] - last_filtered_positions[i]
            max_change = self.max_joint_velocity * self.time_step
            
            if abs(desired_change) > max_change:
                if desired_change > 0:
                    limited_change = max_change
                else:
                    limited_change = -max_change
                
                filtered_targets[i] = last_filtered_positions[i] + limited_change
            else:
                filtered_targets[i] = new_targets[i]
        
        return filtered_targets
    
    def apply_smooth_filter(self, new_targets, last_positions):
        """应用轻量级平滑滤波器"""
        smooth_targets = self.smooth_alpha * new_targets + (1 - self.smooth_alpha) * last_positions
        return smooth_targets
    
    def prepare_websocket_message(self, exo_data):
        """准备WebSocket消息 - 发送原始外骨骼协议数据"""
        # 构建51字节的二进制帧数据：帧头(1) + 有效载荷(48) + 校验和(1) + 帧尾(1)
        
        # 帧头
        frame_data = bytearray([0xAA])  # FRAME_HEADER = 0xAA
        
        # 有效载荷 (48字节)
        payload = bytearray()
        
        # 左摇杆数据 (4个int16 = 8字节)
        # 如果左摇杆控制未启用，发送中心值（静止状态）
        if self.left_joystick_control_enabled:
            left_joystick = exo_data.joystick_left[:4]
        else:
            left_joystick = [self.joystick_center, self.joystick_center, 
                           exo_data.joystick_left[2], exo_data.joystick_left[3]]  # 保留按键和扳机
        
        for value in left_joystick:
            payload.extend(struct.pack('<h', int(value)))
        
        # 右摇杆数据 (4个int16 = 8字节)
        # 如果右摇杆控制未启用，发送中心值（静止状态）
        if self.right_joystick_control_enabled:
            right_joystick = exo_data.joystick_right[:4]
        else:
            right_joystick = [self.joystick_center, self.joystick_center, 
                            exo_data.joystick_right[2], exo_data.joystick_right[3]]  # 保留按键和扳机
        
        for value in right_joystick:
            payload.extend(struct.pack('<h', int(value)))
        
        # 左臂关节数据 (8个int16 = 16字节)
        # 如果左臂控制启用且我们有处理后的数据，使用滤波后的数据
        # 否则使用原始数据（需要应用符号反转）
        for i in range(8):
            if (self.control_left and self.left_arm_control_enabled and 
                i < len(self.filtered_positions_left)):
                # 临时代码：屏蔽左手第5个关节（索引4），强制发送0
                if i == 4:  # 第5个关节（索引从0开始）
                    encoder_value = 0
                else:
                    # 将滤波后的弧度值转换回编码器值
                    encoder_value = int(self.filtered_positions_left[i] / (2 * math.pi / 16384))
            elif i < len(exo_data.arm_joint_left):
                # 临时代码：屏蔽左手第5个关节（索引4），强制发送0
                if i == 4:  # 第5个关节（索引从0开始）
                    encoder_value = 0
                else:
                    # 应用符号反转后使用原始数据
                    inverted_left_data = self._apply_joint_sign_inversion_raw(exo_data.arm_joint_left, 'left')
                    encoder_value = int(inverted_left_data[i])
            else:
                encoder_value = 0
            payload.extend(struct.pack('<h', encoder_value))
        
        # 右臂关节数据 (8个int16 = 16字节)
        # 如果右臂控制启用且我们有处理后的数据，使用滤波后的数据
        # 否则使用原始数据（需要应用符号反转）
        for i in range(8):
            if (self.control_right and self.right_arm_control_enabled and 
                i < len(self.filtered_positions_right)):
                # 将滤波后的弧度值转换回编码器值
                encoder_value = int(self.filtered_positions_right[i] / (2 * math.pi / 16384))
            elif i < len(exo_data.arm_joint_right):
                # 应用符号反转后使用原始数据
                inverted_right_data = self._apply_joint_sign_inversion_raw(exo_data.arm_joint_right, 'right')
                encoder_value = int(inverted_right_data[i])
            else:
                encoder_value = 0
            payload.extend(struct.pack('<h', encoder_value))
        
        # 确保payload正好是48字节
        while len(payload) < 48:
            payload.append(0)
        payload = payload[:48]
        
        # 添加payload到帧数据
        frame_data.extend(payload)
        
        # 计算校验和 (对payload进行XOR)
        checksum = 0
        for byte in payload:
            checksum ^= byte
        frame_data.append(checksum)
        
        # 帧尾
        frame_data.append(0x55)  # FRAME_TAIL = 0x55
        
        # 构建符合机器人端期望格式的消息
        message_data = {
            'deviceId': 'qnbot-exoskeleton',
            'timestamp': int(time.time() * 1000),  # 毫秒时间戳
            'type': 'exoskeleton-realtime',
            'data': {
                'type': 'binary',
                'length': 51,
                'data': list(frame_data),  # 51字节的数组
                'parsed': {
                    'header': frame_data[0],
                    'payload': list(frame_data[1:49]),  # 48字节payload
                    'checksum': frame_data[49],
                    'footer': frame_data[50],
                    'checksumValid': True
                }
            }
        }
        
        return message_data
    
    async def control_loop(self):
        """主控制循环"""
        self.log_info("控制循环已启动 - 100Hz")
        debug_counter = 0
        
        while self.running:
            try:
                # 获取最新外骨骼数据
                exo_data = self.exoskeleton_reader.get_latest_data()
                
                if exo_data.timestamp == 0:
                    await asyncio.sleep(self.time_step)
                    continue
                
                # 检测按钮点击事件
                self.detect_button_clicks(exo_data)
                
                # 处理回零流程 - 已注释掉
                # self.process_homing_sequence()
                
                # 根据数据发送状态决定是否处理新数据
                if self.transmission_state == 'transmitting':
                    # 正常模式：处理新的外骨骼数据
                    self.process_new_exoskeleton_data(exo_data)
                elif self.transmission_state == 'paused':
                    # 暂停模式：使用保存的位置数据
                    self.use_paused_position_data()
                # 'not_started' 状态：不处理数据，等待开始指令
                

                
                # 准备并发送WebSocket消息（仅在非not_started状态）
                if self.transmission_state != 'not_started':
                    message_data = self.prepare_websocket_message(exo_data)
                    
                    # 尝试发送数据
                    if self.websocket_connected:
                        success = await self.send_websocket_message(message_data)
                        if not success:
                            # 连接断开，尝试重连
                            self.websocket_connected = False
                            self.log_error("WebSocket连接断开，尝试重连...")
                            if await self.connect_websocket():
                                await self.send_websocket_message(message_data)
                
                # 调试信息输出
                debug_counter += 1
                if debug_counter % 50 == 0:  # 每0.5秒记录一次
                    control_status = []
                    if self.control_left:
                        left_status = "启用" if self.left_arm_control_enabled else "禁用"
                        control_status.append(f"左臂:{left_status}")
                    
                    if self.control_right:
                        right_status = "启用" if self.right_arm_control_enabled else "禁用"
                        control_status.append(f"右臂:{right_status}")
                    
                    left_joy_status = "启用" if self.left_joystick_control_enabled else "禁用"
                    right_joy_status = "启用" if self.right_joystick_control_enabled else "禁用"
                    control_status.append(f"左摇杆:{left_joy_status}")
                    control_status.append(f"右摇杆:{right_joy_status}")
                    
                    # 添加数据发送状态
                    transmission_status_map = {
                        'not_started': '⏹️未开始',
                        'transmitting': '▶️发送中',
                        'paused': '⏸️已暂停'
                    }
                    transmission_status = transmission_status_map.get(self.transmission_state, self.transmission_state)
                    
                    debug_info = f'WebSocket状态: {"已连接" if self.websocket_connected else "断开"}, 数据发送: {transmission_status}, 控制状态: {", ".join(control_status)}'
                    
                    # 回零进度显示 - 已注释掉
                    # if self.homing_active:
                    #     homing_progress_left = f"{self.homing_current_joint_left}/{self.total_joints_to_home}" if self.control_left else "N/A"
                    #     homing_progress_right = f"{self.homing_current_joint_right}/{self.total_joints_to_home}" if self.control_right else "N/A"
                    #     debug_info += f', 🔄回零中: 左臂{homing_progress_left}, 右臂{homing_progress_right}'
                    
                    debug_info += f', 已发送: {self.stats["total_sent"]}, 错误: {self.stats["send_errors"]}'
                    
                    # 添加数据格式调试信息
                    if 'message_data' in locals():
                        frame_data = message_data['data']['data']
                        debug_info += f', 帧长度: {len(frame_data)}, 帧头: 0x{frame_data[0]:02X}, 帧尾: 0x{frame_data[-1]:02X}'
                        if message_data['data']['parsed']['checksumValid']:
                            debug_info += ', 校验✓'
                        else:
                            debug_info += ', 校验✗'
                    
                    self.log_debug(debug_info)
                
            except Exception as e:
                self.log_error(f'控制循环异常: {str(e)}')
            
            # 控制循环频率
            await asyncio.sleep(self.time_step)
    
    async def websocket_monitor(self):
        """WebSocket连接监控"""
        while self.running:
            if not self.websocket_connected:
                self.log_info("尝试连接WebSocket...")
                await self.connect_websocket()
            
            await asyncio.sleep(5.0)  # 每5秒检查一次连接状态
    
    async def run(self):
        """运行WebSocket发送器"""
        try:
            # 启动外骨骼数据读取
            if not self.exoskeleton_reader.start():
                self.log_error('无法启动外骨骼数据读取器')
                return
            
            self.log_info('外骨骼数据读取器已启动')
            
            # 等待获取外骨骼数据
            self.log_info('等待外骨骼数据...')
            self._wait_for_exoskeleton_data()
            
            # 启动WebSocket连接
            if not await self.connect_websocket():
                self.log_error('无法连接到WebSocket服务器')
                return
            
            self.control_active = True
            self.log_info(f'{self.arm_mode}外骨骼WebSocket发送器已启动')
            
            if self.enable_filter:
                self.log_info(f'完整速度滤波已启用 - 最大关节速度限制: {self.max_joint_velocity} rad/s')
            else:
                self.log_info(f'轻量级平滑滤波模式 - 平滑系数: {self.smooth_alpha} (快速响应+抖动抑制)')
            
            # 启动控制循环和连接监控
            self.control_task = asyncio.create_task(self.control_loop())
            self.websocket_task = asyncio.create_task(self.websocket_monitor())
            
            # 等待任务完成
            await asyncio.gather(self.control_task, self.websocket_task)
            
        except KeyboardInterrupt:
            self.log_info("收到中断信号，正在关闭...")
        except Exception as e:
            self.log_error(f"运行异常: {e}")
        finally:
            await self.shutdown()
    
    async def shutdown(self):
        """安全关闭"""
        self.log_info('正在安全关闭WebSocket发送器...')
        
        self.running = False
        self.control_active = False
        
        # 停止回零流程 - 已注释掉
        # if self.homing_active:
        #     self.homing_active = False
        #     self.log_info('已停止回零流程')
        
        # 取消任务
        if self.control_task:
            self.control_task.cancel()
        if self.websocket_task:
            self.websocket_task.cancel()
        
        # 关闭WebSocket连接
        if self.websocket and not self.websocket.closed:
            await self.websocket.close()
            self.log_info('WebSocket连接已关闭')
        
        # 停止外骨骼数据读取
        if hasattr(self, 'exoskeleton_reader'):
            self.exoskeleton_reader.stop()
            self.log_info('外骨骼数据读取器已停止')
        
        self.log_info('WebSocket发送器已安全关闭')


def get_arm_mode_interactive():
    """交互式获取控制模式"""
    print("\n" + "="*60)
    print("QnBot外骨骼WebSocket发送器")
    print("="*60)
    print("请选择控制模式:")
    print("1. 左臂控制 (left)")
    print("2. 右臂控制 (right)")
    print("3. 双臂同步控制 (both)")
    print("4. 仅双臂控制（无滑台、小车）(both_arms_only)")
    print("="*60)
    
    while True:
        try:
            choice = input("请输入选择 (1/2/3/4): ").strip()
            if choice == '1':
                return 'left'
            elif choice == '2':
                return 'right'
            elif choice == '3':
                return 'both'
            elif choice == '4':
                return 'both_arms_only'
            else:
                print("无效输入，请输入 1、2、3 或 4")
        except KeyboardInterrupt:
            print("\n程序已取消")
            sys.exit(0)


async def main():
    """主函数"""
    try:
        # 解析命令行参数
        parser = argparse.ArgumentParser(description="QnBot外骨骼WebSocket发送器")
        parser.add_argument("--arm_mode", type=str, choices=['left', 'right', 'both', 'both_arms_only'], 
                          help="机械臂控制模式 (不指定则交互式选择)")
        parser.add_argument("--no_filter", action='store_true', 
                          help="禁用完整速度滤波，使用轻量级平滑滤波（快速响应+抖动抑制）")
        parser.add_argument("--smooth_alpha", type=float, default=0.2,
                          help="平滑滤波系数 (0.1-0.5)，越小越平滑但响应越慢，默认0.2")
        parser.add_argument("--websocket_url", type=str, default="ws://0.0.0.0:19091",
                          help="WebSocket服务器地址，默认ws://0.0.0.0:19091")
        parsed_args = parser.parse_args()
        
        # 获取控制模式
        if parsed_args.arm_mode:
            arm_mode = parsed_args.arm_mode
        else:
            arm_mode = get_arm_mode_interactive()
        
        # 创建WebSocket发送器
        enable_filter = not parsed_args.no_filter
        smooth_alpha = max(0.05, min(0.8, parsed_args.smooth_alpha))
        
        sender = QnbotWebSocketSender(
            arm_mode=arm_mode, 
            enable_filter=enable_filter, 
            smooth_alpha=smooth_alpha,
            websocket_url=parsed_args.websocket_url
        )
        
        print("\n" + "="*60)
        mode_desc = {
            'left': '左臂',
            'right': '右臂', 
            'both': '双臂同步',
            'both_arms_only': '仅双臂（无滑台、小车）'
        }
        print(f"QnBot {mode_desc[arm_mode]}外骨骼WebSocket发送器")
        print(f"WebSocket服务器: {parsed_args.websocket_url}")
        print("控制频率: 100Hz")
        
        if enable_filter:
            print(f"最大关节速度: {sender.max_joint_velocity} rad/s")
            print("控制方式: 完整速度滤波位置控制")
        else:
            print(f"控制方式: 轻量级平滑滤波（快速响应+抖动抑制），平滑系数: {smooth_alpha}")
        
        if arm_mode == 'left':
            print("扳机控制: 左摇杆扳机控制左夹爪")
        elif arm_mode == 'right':
            print("扳机控制: 右摇杆扳机控制右夹爪")
        else:
            print("扳机控制: 左摇杆扳机控制左夹爪，右摇杆扳机控制右夹爪")
        
        print("扳机范围: 700(松开) ~ 2700(夹紧)")
        print()
        
        print("数据发送控制:")
        print("  初始状态: 默认不发送数据，等待开始指令")
        print("  开始发送: 双边同时长按1秒开始数据发送")
        print("  暂停发送: 运行中双边同时长按1秒暂停发送（保持当前位置）")
        print("  恢复发送: 暂停中双边同时长按1秒恢复发送（使用新位置）")
        print()
        
        # 根据模式显示不同的控制信息
        if arm_mode == 'both_arms_only':
            print("模式说明: 仅控制双臂，小车和滑台控制已禁用")
            print()
            print("按钮控制:")
            print("  左摇杆按钮: 启用/禁用 左臂控制")
            print("  右摇杆按钮: 启用/禁用 右臂控制")
        else:
            print("摇杆控制:")
            print("  左摇杆Y轴: 小车前进/后退 (最大1.0m/s)")
            print("  右摇杆X轴: 小车左转/右转 (最大1.0rad/s)")
            print("  右摇杆Y轴: 滑台上升/下降 (最大0.15m/s，带加速度限制)")
            print("  死区范围: 中心值±300 (自动校准为静止状态)")
            print()
            print("按钮控制:")
            print("  左摇杆按钮: 启用/禁用 左臂控制 + 左摇杆控制")
            print("  右摇杆按钮: 启用/禁用 右臂控制 + 右摇杆控制")
        
        print("  操作方式: 短按(按下-松开)切换控制状态")
        print("  防抖保护: 按钮需按下至少50ms才算有效点击")
        print("  初始状态: 所有控制均为禁用，需要按钮激活")
        print()
        print("高级功能:")
        print("  数据发送控制: 双边同时长按1秒控制数据发送开始/暂停/恢复")
        # print("  机械臂回零: 所有控制禁用时，双边同时长按2秒触发回零")
        # print("  回零顺序: 关节2→关节1→关节3-7→夹爪，每个关节间隔3秒")
        # print("  运动方式: 3阶段插值平滑运动 (起始→1/3→2/3→目标)")
        # print("  并行执行: 左右臂同时进行回零，总耗时约24秒")
        print("按 Ctrl+C 退出")
        print("="*60 + "\n")
        
        # 设置信号处理
        def signal_handler(signum, frame):
            print("\n收到中断信号，正在关闭...")
            sender.running = False
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # 运行发送器
        await sender.run()
        
    except KeyboardInterrupt:
        print("\n用户中断，正在安全关闭...")
    except Exception as e:
        print(f"\n程序异常: {str(e)}")


if __name__ == '__main__':
    # 运行异步主函数
    asyncio.run(main()) 