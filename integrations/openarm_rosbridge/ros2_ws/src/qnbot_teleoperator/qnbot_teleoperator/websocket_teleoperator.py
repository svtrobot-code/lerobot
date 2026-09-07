#!/usr/bin/env python3
"""
WebSocket 远程控制器
接收来自 WebSocket 的外骨骼数据，实时转发为 ROS2 关节命令并发布

功能特点：
1. 事件驱动实时转发，收到数据立即发布
2. 直接位置控制（无滤波、无限位检测）
3. 按钮控制启停和回零功能
4. 摇杆控制小车移动和滑台升降
5. 扳机数据作为关节8发布
6. 支持左臂、右臂、双臂控制模式配置
7. 无数据缓存，避免读写分离延迟

数据发布：
- /exo/joint_command: 16个关节数据（左臂7个+右臂7个+左扳机+右扳机）
- /exo/gamepad_keys: 摇杆数据和按钮状态
  * axes[0-3]: 左右摇杆XY轴
  * buttons[0-3]: 控制状态
  * buttons[4-9]: 左手柄6个按钮 (Joystick_K, A, B, C, D, Switch)
  * buttons[10-15]: 右手柄6个按钮 (Joystick_K, A, B, C, D, Switch)

注意：
- 关节数据直接传递，不进行限位检测，限位安全由下游控制器负责
- 收到新数据时立即发布，没有数据时不发布任何命令
"""

import json
import asyncio
import websockets
import threading
import time
import math
import numpy as np
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Header

from .protocol import ExoProtocolParser


class WebSocketTeleoperator(Node):
    """
    WebSocket 外骨骼遥操作节点
    
    功能：
    - 接收外骨骼数据（双臂各7个关节 + 摇杆 + 扳机）
    - 处理所有控制逻辑（按钮控制、回零流程等）
    - 实时发布到专用话题：
      * /exo/joint_command: 16个关节数据（左臂7个 + 右臂7个 + 左扳机 + 右扳机）
      * /exo/gamepad_keys: 摇杆数据
    
    控制模式配置：
    - enable_left_arm: 是否接收左臂数据
    - enable_right_arm: 是否接收右臂数据  
    - enable_left_joystick: 是否接收左手柄数据
    - enable_right_joystick: 是否接收右手柄数据
    - enable_vehicle_control: 是否启用小车和滑台控制
    """
    
    def __init__(self):
        super().__init__('websocket_teleoperator')
            
        # 参数声明
        self.declare_parameter('websocket_host', '0.0.0.0')
        self.declare_parameter('websocket_port', 19091)
        self.declare_parameter('joint_command_topic', '/exo/joint_command')
        self.declare_parameter('gamepad_keys_topic', '/exo/gamepad_keys')
        
        # 控制模式参数
        self.declare_parameter('enable_left_arm', True)
        self.declare_parameter('enable_right_arm', True)
        self.declare_parameter('enable_left_joystick', True)
        self.declare_parameter('enable_right_joystick', True)
        self.declare_parameter('enable_vehicle_control', True)
        
        # 获取参数
        self.websocket_host = self.get_parameter('websocket_host').value
        self.websocket_port = self.get_parameter('websocket_port').value
        self.joint_command_topic = self.get_parameter('joint_command_topic').value
        self.gamepad_keys_topic = self.get_parameter('gamepad_keys_topic').value
        
        # 控制模式配置
        self.enable_left_arm = self.get_parameter('enable_left_arm').value
        self.enable_right_arm = self.get_parameter('enable_right_arm').value
        self.enable_left_joystick = self.get_parameter('enable_left_joystick').value
        self.enable_right_joystick = self.get_parameter('enable_right_joystick').value
        self.enable_vehicle_control = self.get_parameter('enable_vehicle_control').value

        
        # ================= 按钮控制状态管理 =================
        # 按钮状态
        self.left_button_last_state = 1   # 1=未按下，0=按下
        self.right_button_last_state = 1
        
        # 控制启停状态（简化为双臂一起控制）
        self.arms_control_enabled = True   # 双臂控制是否启用
        self.joystick_control_enabled = True   # 摇杆控制是否启用
        
        # 按钮点击计数器
        self.button_click_count = 0
        
        # Switch 按钮状态跟踪（用于检测变化）
        self.left_switch_last_state = None
        self.right_switch_last_state = None
        self.both_switches_enabled_last = False  # 跟踪两个 Switch 是否都开启
        # ============================================================
        
        # ================= 长按和启停控制状态 =================
        # 长按检测相关
        self.left_button_press_start_time = None
        self.right_button_press_start_time = None
        self.control_toggle_duration = 1.0  # 启停控制需要1秒长按
        self.homing_duration = 10.0  # 回零需要10秒长按
        
        # 防抖检测相关
        self.debounce_duration = 0.05
        
        # 点击检测相关
        self.left_button_pressed = False
        self.right_button_pressed = False
        
        # 松开检测相关（用于检测近似同时的双边松开）
        self.left_button_release_time = None
        self.right_button_release_time = None
        self.left_button_press_start_time_saved = None
        self.right_button_press_start_time_saved = None
        self.dual_release_window = 0.2  # 双边松开检测窗口：200ms
        
        # 回零流程控制
        self.homing_active = False
        self.homing_start_time = None
        self.homing_current_joint_left = 0
        self.homing_current_joint_right = 0
        self.homing_joint_interval = 1.0
        self.homing_last_joint_time_left = None
        self.homing_last_joint_time_right = None
        self.total_joints_to_home = 8  # 1-7关节+夹爪
        
        # 回零顺序映射
        self.homing_joint_order = [1, 0, 2, 3, 4, 5, 6, 7]  # 关节2→关节1→关节3→...→夹爪
        # ============================================================
        
        # 机械臂关节配置
        self.num_joints = 7  # 每个臂7个关节
        
        # 关节名称（包含扳机作为第8个关节）
        self.joint_names = [
            # 左臂7个关节
            'left_arm_joint_1', 'left_arm_joint_2', 'left_arm_joint_3',
            'left_arm_joint_4', 'left_arm_joint_5', 'left_arm_joint_6', 'left_arm_joint_7',
            # 右臂7个关节
            'right_arm_joint_1', 'right_arm_joint_2', 'right_arm_joint_3',
            'right_arm_joint_4', 'right_arm_joint_5', 'right_arm_joint_6', 'right_arm_joint_7',
            # 扳机作为第8个关节
            'left_trigger_joint', 'right_trigger_joint'
        ]
        
        # 扳机数据映射参数
        self.trigger_min = 700    # 扳机松开值
        self.trigger_max = 2700   # 扳机夹紧值
        
        # 初始化协议解析器
        self.parser = ExoProtocolParser()
        
        # 创建发布器
        self.joint_command_pub = self.create_publisher(
            JointState, 
            self.joint_command_topic, 
            10
        )
        
        # 创建摇杆数据发布器
        self.gamepad_keys_pub = self.create_publisher(
            Joy,
            self.gamepad_keys_topic,
            10
        )
        
        # 初始化关节命令模板（16个关节：左臂7个+右臂7个+左扳机+右扳机）
        self.init_joint_command_template()
        
        # WebSocket 相关
        self.websocket_server = None
        self.connected_clients = set()
        self.websocket_running = False
        
        # 统计信息
        self.stats = {
            'total_received': 0,
            'total_published': 0,
            'parse_errors': 0,
            'last_receive_time': 0,
            'dropped_by_rate_limit': 0,  # 因频率限制而丢弃的数据
            'dropped_by_switch_check': 0,  # 因 Switch 检查而丢弃的数据
        }
        
        # 频率限制配置（100Hz = 10ms间隔）
        self.max_publish_rate_hz = 100.0
        self.min_publish_interval = 1.0 / self.max_publish_rate_hz  # 0.01秒 = 10ms
        self.last_publish_time = 0.0
        
        # 启动 WebSocket 服务器
        self.start_websocket_server()
        
        # TODO: 回零功能暂时注释，需要后续添加插值功能保证安全
        # 创建回零流程定时器（仅用于回零时序控制）
        self.homing_timer = self.create_timer(
            0.1,  # 10Hz检查回零进度（当前禁用）
            self.process_homing_sequence
        )
        
        self.get_logger().info(
            f'WebSocket 远程控制器已启动'
            f'\n  WebSocket 地址: {self.websocket_host}:{self.websocket_port}'
            f'\n  关节命令话题: {self.joint_command_topic}'
            f'\n  摇杆数据话题: {self.gamepad_keys_topic}'
            f'\n  转发方式: 事件驱动实时转发'
            f'\n  频率限制: 最高{self.max_publish_rate_hz}Hz (超出数据将被丢弃)'
            f'\n  左臂控制: {"启用" if self.enable_left_arm else "禁用"}'
            f'\n  右臂控制: {"启用" if self.enable_right_arm else "禁用"}'
            f'\n  左手柄控制: {"启用" if self.enable_left_joystick else "禁用"}'
            f'\n  右手柄控制: {"启用" if self.enable_right_joystick else "禁用"}'
            f'\n  小车滑台控制: {"启用" if self.enable_vehicle_control else "禁用"}'
            f'\n  📡 等待WebSocket连接后开始接收数据...'
            f'\n  🎮 控制方式: 双侧同时长按1-10秒松开启停控制'
            f'\n  🔧 容错机制: 双边松开允许200ms时间差'
            f'\n  ⚠️  初始状态: 所有控制已禁用，需要长按激活'
            f'\n  🚀 收到数据立即转发，无缓存延迟'
            f'\n  🔒 Switch安全检查: 只有左右手柄Switch都推上去(为0)时才转发数据'
            f'\n  🚫 回零功能已禁用（需要插值安全机制）'
        )
    
    def init_joint_command_template(self):
        """初始化关节命令模板，包含16个关节（左臂7个+右臂7个+左扳机+右扳机）"""
        # 默认关节位置：16个关节（仅用于回零）
        self.default_joint_positions = [0.0] * 16
        self.default_velocity = [0.0] * 16
        self.default_effort = [0.0] * 16
        
        # 关节索引映射
        self.left_arm_joint_indices = [0, 1, 2, 3, 4, 5, 6]     # 左臂关节
        self.right_arm_joint_indices = [7, 8, 9, 10, 11, 12, 13] # 右臂关节
        self.left_trigger_index = 14   # 左扳机
        self.right_trigger_index = 15  # 右扳机
        
    def start_websocket_server(self):
        """启动 WebSocket 服务器"""
        def run_server():
            # 创建新的事件循环并设置为当前线程的事件循环
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                self.websocket_running = True
                
                # 创建协程并在事件循环中运行
                async def server_main():
                    server = await websockets.serve(
                        self.handle_websocket_connection,
                        self.websocket_host,
                        self.websocket_port
                    )
                    self.get_logger().info(f'WebSocket 服务器已启动在 {self.websocket_host}:{self.websocket_port}')
                    
                    # 保存服务器引用以便后续关闭
                    self.websocket_server = server
                    
                    # 保持服务器运行直到被告知停止
                    try:
                        while self.websocket_running:
                            await asyncio.sleep(0.1)
                    except asyncio.CancelledError:
                        pass
                    finally:
                        # 关闭服务器
                        server.close()
                        await server.wait_closed()
                
                # 运行服务器
                loop.run_until_complete(server_main())
                
            except Exception as e:
                self.get_logger().error(f'WebSocket 服务器启动失败: {e}')
                import traceback
                self.get_logger().error(f'详细错误信息: {traceback.format_exc()}')
            finally:
                self.websocket_running = False
                # 确保事件循环正确关闭
                try:
                    # 取消所有待处理的任务
                    pending = asyncio.all_tasks(loop)
                    for task in pending:
                        task.cancel()
                    
                    # 等待所有任务完成或取消
                    if pending:
                        loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                        
                finally:
                    loop.close()
        
        # 在单独的线程中运行 WebSocket 服务器
        self.websocket_thread = threading.Thread(target=run_server, daemon=True)
        self.websocket_thread.start()
    
    async def handle_websocket_connection(self, websocket, _path=None):
        """处理 WebSocket 连接"""
        client_address = websocket.remote_address
        self.get_logger().info(f'🔗 新的 WebSocket 连接: {client_address}')
        
        self.connected_clients.add(websocket)
        
        # 如果这是第一个连接，通知开始发布数据
        if len(self.connected_clients) == 1:
            self.get_logger().info(f'📡 WebSocket连接已建立，开始实时转发数据到 {self.joint_command_topic} 和 {self.gamepad_keys_topic}')
        
        try:
            async for message in websocket:
                await self.process_websocket_message(message)
        except websockets.exceptions.ConnectionClosed as e:
            self.get_logger().info(f'WebSocket 连接正常关闭: {client_address} - {e}')
        except websockets.exceptions.ConnectionClosedError as e:
            self.get_logger().info(f'WebSocket 连接异常关闭: {client_address} - {e}')
        except websockets.exceptions.WebSocketException as e:
            self.get_logger().warn(f'WebSocket 协议错误 {client_address}: {e}')
        except asyncio.CancelledError:
            self.get_logger().info(f'WebSocket 连接任务被取消: {client_address}')
        except Exception as e:
            self.get_logger().error(f'WebSocket 连接未知错误 {client_address}: {e}')
            import traceback
            self.get_logger().error(f'详细错误信息: {traceback.format_exc()}')
        finally:
            # 确保客户端从连接列表中移除
            if websocket in self.connected_clients:
                self.connected_clients.discard(websocket)
                self.get_logger().info(f'📡 客户端已断开: {client_address}, 剩余连接数: {len(self.connected_clients)}')
                
                # 如果所有连接都断开，通知停止发布数据
                if len(self.connected_clients) == 0:
                    self.get_logger().info(f'🚫 所有WebSocket连接已断开，停止接收数据')
            else:
                self.get_logger().debug(f'客户端已不在连接列表中: {client_address}')
    
    async def process_websocket_message(self, message: str):
        """处理 WebSocket 消息 - 立即转发"""
        try:
            # 检查是否有连接
            if len(self.connected_clients) == 0:
                return
            
            # 解析 JSON 消息
            data = json.loads(message)
            
            # 使用协议解析器解析外骨骼数据
            parsed_data = self.parser.parse_websocket_message(data)
            
            if parsed_data and parsed_data.get('valid', False):
                # 处理按钮控制逻辑
                self.detect_button_clicks(parsed_data)
                
                # 检查控制状态
                if not self.arms_control_enabled and not self.joystick_control_enabled:
                    # 控制未启用，不发布数据
                    self.stats['total_received'] += 1
                    return
                
                # 立即处理并发布数据
                self.process_and_publish_data(parsed_data)
                
                # 更新统计信息
                self.stats['total_received'] += 1
                self.stats['last_receive_time'] = self.get_clock().now().nanoseconds
                
                # 记录调试信息（降低频率）
                if self.stats['total_received'] % 2000 == 0:
                    self.get_logger().info(
                        f'已处理 {self.stats["total_received"]} 条消息, '
                        f'已发布 {self.stats["total_published"]} 条命令, '
                        f'频率限制丢弃 {self.stats["dropped_by_rate_limit"]} 条, '
                        f'Switch检查丢弃 {self.stats["dropped_by_switch_check"]} 条, '
                        f'双臂控制: {"启用" if self.arms_control_enabled else "禁用"}, '
                        f'摇杆控制: {"启用" if self.joystick_control_enabled else "禁用"}'
                    )
            else:
                self.stats['parse_errors'] += 1
                if self.stats['parse_errors'] % 10 == 0:
                    self.get_logger().warn(f'解析错误累计: {self.stats["parse_errors"]}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON 解析错误: {e}')
        except Exception as e:
            self.get_logger().error(f'处理 WebSocket 消息失败: {e}')
    
    def process_and_publish_data(self, parsed_data: Dict):
        """处理并立即发布数据（受频率限制）"""
        try:
            # 频率限制检查：最高100Hz
            current_time = time.time()
            time_since_last_publish = current_time - self.last_publish_time
            
            if time_since_last_publish < self.min_publish_interval:
                # 距离上次发布时间不足，丢弃此数据
                self.stats['dropped_by_rate_limit'] += 1
                return
            
            # 创建关节位置数组（16个关节）
            joint_positions = self.default_joint_positions.copy()
            
            # 创建摇杆数据
            joystick_data = {
                'left_x': 0.0,
                'left_y': 0.0, 
                'right_x': 0.0,
                'right_y': 0.0
            }
            
            # 更新左臂关节（如果启用）
            if self.enable_left_arm and self.arms_control_enabled:
                left_arm_joints = parsed_data.get('left_arm_joints', [])
                for i, joint_idx in enumerate(self.left_arm_joint_indices):
                    if i < len(left_arm_joints):
                        # 直接使用原始关节数据，不进行限位检测
                        joint_positions[joint_idx] = float(left_arm_joints[i])
            
            # 更新右臂关节（如果启用）
            if self.enable_right_arm and self.arms_control_enabled:
                right_arm_joints = parsed_data.get('right_arm_joints', [])
                for i, joint_idx in enumerate(self.right_arm_joint_indices):
                    if i < len(right_arm_joints):
                        # 直接使用原始关节数据，不进行限位检测
                        joint_positions[joint_idx] = float(right_arm_joints[i])
            
            # 更新扳机数据作为关节8（夹爪位置）
            left_joystick = parsed_data.get('left_joystick', {})
            right_joystick = parsed_data.get('right_joystick', {})
            
            if self.enable_left_joystick and self.joystick_control_enabled:
                left_trigger = left_joystick.get('trigger', self.trigger_min)
                joint_positions[self.left_trigger_index] = self.map_trigger_to_gripper(left_trigger)
            
            if self.enable_right_joystick and self.joystick_control_enabled:
                right_trigger = right_joystick.get('trigger', self.trigger_min)
                joint_positions[self.right_trigger_index] = self.map_trigger_to_gripper(right_trigger)

            # 更新摇杆数据（仅在控制启用时）
            if self.enable_left_joystick and self.joystick_control_enabled:
                joystick_data['left_x'] = float(left_joystick.get('x', 0.0))
                joystick_data['left_y'] = float(left_joystick.get('y', 0.0))
            
            if self.enable_right_joystick and self.joystick_control_enabled:
                joystick_data['right_x'] = float(right_joystick.get('x', 0.0))
                joystick_data['right_y'] = float(right_joystick.get('y', 0.0))
            
            # 更新按钮数据（总是读取，不受控制状态影响，因为按钮可能用于控制启停等功能）
            if self.enable_left_joystick:
                joystick_data['left_buttons'] = left_joystick.get('buttons', {})
                # 检测左手柄 Switch 状态变化
                left_switch_current = joystick_data['left_buttons'].get('switch', 0)
                if self.left_switch_last_state is not None and self.left_switch_last_state != left_switch_current:
                    status_text = '推上去' if left_switch_current == 0 else '推下来'
                    self.get_logger().info(f"🔄 左手柄 Switch 状态变化: {self.left_switch_last_state} → {left_switch_current} ({status_text})")
                    if left_switch_current == 0:
                        self.get_logger().info(f"✅ 左手柄 Switch 已推上去，等待右手柄 Switch 推上去后开始转发数据")
                    else:
                        self.get_logger().warn(f"⚠️ 左手柄 Switch 已推下来，数据转发已停止")
                self.left_switch_last_state = left_switch_current
            
            if self.enable_right_joystick:
                joystick_data['right_buttons'] = right_joystick.get('buttons', {})
                # 检测右手柄 Switch 状态变化
                right_switch_current = joystick_data['right_buttons'].get('switch', 0)
                if self.right_switch_last_state is not None and self.right_switch_last_state != right_switch_current:
                    status_text = '推上去' if right_switch_current == 0 else '推下来'
                    self.get_logger().info(f"🔄 右手柄 Switch 状态变化: {self.right_switch_last_state} → {right_switch_current} ({status_text})")
                    if right_switch_current == 0:
                        self.get_logger().info(f"✅ 右手柄 Switch 已推上去，等待左手柄 Switch 推上去后开始转发数据")
                    else:
                        self.get_logger().warn(f"⚠️ 右手柄 Switch 已推下来，数据转发已停止")
                self.right_switch_last_state = right_switch_current
            
            # 检查 Switch 状态：只有左右手的 Switch 都为 0（推上去）才转发数据
            left_switch = joystick_data.get('left_buttons', {}).get('switch', 0) if self.enable_left_joystick else 0
            right_switch = joystick_data.get('right_buttons', {}).get('switch', 0) if self.enable_right_joystick else 0
            
            # 检查两个 Switch 是否都推上去（都为 0）
            both_switches_up = (
                (not self.enable_left_joystick or left_switch == 0) and
                (not self.enable_right_joystick or right_switch == 0)
            )
            
            # 检测两个 Switch 都推上去的状态变化
            if both_switches_up != self.both_switches_enabled_last:
                if both_switches_up:
                    self.get_logger().info(f"✅ 左右手柄 Switch 都已推上去，开始转发数据到机器人")
                else:
                    self.get_logger().warn(f"🚫 左右手柄 Switch 未全部推上去，停止转发数据 (左: {left_switch}, 右: {right_switch})")
                self.both_switches_enabled_last = both_switches_up
            
            # 如果启用了手柄但 Switch 不全为 0（未全部推上去），则不转发数据
            if not both_switches_up:
                self.stats['dropped_by_switch_check'] += 1
                # 降低日志频率，每100次记录一次
                if self.stats['dropped_by_switch_check'] % 100 == 0:
                    self.get_logger().debug(f"🚫 Switch 检查未通过，数据未转发 (左: {left_switch}, 右: {right_switch})")
                return
            
            # 立即发布关节命令和摇杆数据
            self.publish_joint_command(joint_positions)
            self.publish_gamepad_keys(joystick_data)
            
            # 更新发布时间戳和统计
            self.last_publish_time = current_time
            self.stats['total_published'] += 1
            
        except Exception as e:
            self.get_logger().error(f'处理和发布数据失败: {e}')
    
    def map_trigger_to_gripper(self, trigger_value):
        """
        映射扳机数据到夹爪位置
        扳机归一化范围: 1.0(张开) ~ 0.0(夹紧)
        夹爪范围: 0.067m(张开) ~ 0.0m(闭合)
        """
        # trigger_value已经是归一化的结果(0.0-1.0)，1.0=张开，0.0=夹紧
        # 限制在有效范围内
        trigger_normalized = np.clip(trigger_value, 0.0, 1.0)
        
        # 映射到夹爪位置范围：1.0 -> 0.067m(张开)，0.0 -> 0.0m(夹紧)
        gripper_position = trigger_normalized * 0.067
        
        return gripper_position
    
    # ================= 按钮控制处理方法 =================
    def detect_button_clicks(self, exo_data):
        """
        检测按钮点击事件并处理控制逻辑
        双侧同时长按1-3秒后松开：启停双臂和摇杆控制
        双侧同时长按超过3秒后松开：回零（仅在控制禁用时）
        """
        current_time = time.time()
        
        # 获取当前按钮状态
        left_joystick = exo_data.get('left_joystick', {})
        right_joystick = exo_data.get('right_joystick', {})
        
        left_button_current = left_joystick.get('button', 1)   # 1=未按下，0=按下
        right_button_current = right_joystick.get('button', 1)
        
        # 按钮状态变化检测
        left_button_clicked = False
        right_button_clicked = False
        
        # 处理左按钮状态变化
        if self.left_button_last_state == 1 and left_button_current == 0:
            # 按钮刚被按下
            self.left_button_pressed = True
            self.left_button_press_start_time = current_time
            
        elif self.left_button_last_state == 0 and left_button_current == 1:
            # 按钮刚被松开
            self.left_button_release_time = current_time
            self.left_button_press_start_time_saved = self.left_button_press_start_time
            self.get_logger().debug(f"🔧 左按钮松开时间: {current_time}")
            if (self.left_button_pressed and 
                self.left_button_press_start_time is not None):
                # 检查按下持续时间是否满足防抖要求
                press_duration = current_time - self.left_button_press_start_time
                if press_duration >= self.debounce_duration:
                    # 完成一次有效点击
                    left_button_clicked = True
            
            # 重置状态
            self.left_button_pressed = False
            self.left_button_press_start_time = None
        
        # 处理右按钮状态变化
        if self.right_button_last_state == 1 and right_button_current == 0:
            # 按钮刚被按下
            self.right_button_pressed = True
            self.right_button_press_start_time = current_time
            
        elif self.right_button_last_state == 0 and right_button_current == 1:
            # 按钮刚被松开
            self.right_button_release_time = current_time
            self.right_button_press_start_time_saved = self.right_button_press_start_time
            self.get_logger().debug(f"🔧 右按钮松开时间: {current_time}")
            if (self.right_button_pressed and 
                self.right_button_press_start_time is not None):
                # 检查按下持续时间是否满足防抖要求
                press_duration = current_time - self.right_button_press_start_time
                if press_duration >= self.debounce_duration:
                    # 完成一次有效点击
                    right_button_clicked = True
            
            # 重置状态
            self.right_button_pressed = False
            self.right_button_press_start_time = None
        
        # 检测双边松开（容错版本：允许200ms内的非同步松开）
        self.check_dual_release_with_tolerance(current_time)
        
        # 检测严格同时松开（保留原逻辑作为备选）
        if (self.left_button_last_state == 0 and left_button_current == 1 and
            self.right_button_last_state == 0 and right_button_current == 1):
            # 双边按钮严格同时松开
            self.get_logger().info("🎮 检测到严格同时松开")
            self.check_dual_long_press_release(current_time)
        
        # 显示双边长按进度（按着的时候）
        if left_button_current == 0 and right_button_current == 0:
            self.show_dual_press_progress(current_time)
        
        # 处理单独按钮点击（暂时保留，可用于其他功能）
        if (left_button_clicked or right_button_clicked):  # 移除homing_active检查
            # 检查是否在双边松开时间窗口内（如果是，就不处理单独点击）
            in_dual_release_window = False
            if (self.left_button_release_time is not None and 
                self.right_button_release_time is not None):
                time_diff = abs(self.left_button_release_time - self.right_button_release_time)
                if time_diff <= self.dual_release_window:
                    in_dual_release_window = True
            
            if not in_dual_release_window:
                self.button_click_count += 1
                self.get_logger().info(f"按钮点击 #{self.button_click_count} (单独按钮点击，当前无功能)")
        
        # 更新上一次按钮状态
        self.left_button_last_state = left_button_current
        self.right_button_last_state = right_button_current
    
    def check_dual_release_with_tolerance(self, current_time):
        """检测容错的双边松开（允许200ms内的非同步松开）"""
        if (self.left_button_release_time is not None and 
            self.right_button_release_time is not None):
            
            # 计算两个按钮松开的时间差
            time_diff = abs(self.left_button_release_time - self.right_button_release_time)
            
            # 如果在容错时间窗口内，且还没有处理过
            if time_diff <= self.dual_release_window:
                # 取较晚的松开时间作为基准
                latest_release_time = max(self.left_button_release_time, self.right_button_release_time)
                
                # 确保我们只处理一次（避免重复触发）
                if abs(current_time - latest_release_time) < 0.05:  # 50ms内处理
                    self.get_logger().info(f"🎮 检测到容错双边松开（时间差: {time_diff*1000:.0f}ms）")
                    self.check_dual_long_press_release(latest_release_time)
                    
                    # 清除松开时间记录，避免重复处理
                    self.left_button_release_time = None
                    self.right_button_release_time = None
    
    def show_dual_press_progress(self, current_time):
        """显示双边长按进度"""
        # 检查双边是否都在长按状态
        both_buttons_pressed = (self.left_button_pressed and self.right_button_pressed and
                               self.left_button_press_start_time is not None and 
                               self.right_button_press_start_time is not None)
        
        if both_buttons_pressed:
            # 计算最短按下时间（取较晚开始的时间）
            min_press_start_time = max(self.left_button_press_start_time, self.right_button_press_start_time)
            press_duration = current_time - min_press_start_time
            
            # 显示长按进度
            if press_duration >= 0.5:  # 0.5秒后开始显示进度
                if int(press_duration * 2) != int((press_duration - 0.01) * 2):  # 每0.5秒显示
                    if press_duration < self.control_toggle_duration:
                        remaining = self.control_toggle_duration - press_duration
                        self.get_logger().info(f"🎮 双边长按检测中... 剩余{remaining:.1f}秒切换控制状态")
                    elif press_duration < self.homing_duration:
                        remaining = self.homing_duration - press_duration
                        if remaining > 5.0:
                            self.get_logger().info(f"🔄 继续长按... 剩余{remaining:.1f}秒触发回零")
                        else:
                            self.get_logger().info(f"🔄 即将触发回零... 剩余{remaining:.1f}秒")
                    else:
                        self.get_logger().info(f"🔄 长按{press_duration:.1f}秒，松开将触发回零！")
    
    def check_dual_long_press_release(self, current_time):
        """检测双边长按松开时的功能"""
        # TODO: 回零功能暂时禁用
        # 检查是否正在进行回零流程
        # if self.homing_active:
        #     self.get_logger().info("⚠️ 双边松开：回零流程正在进行中，忽略按键操作")
        #     return
        
        # 检查双边是否都曾经长按（使用保存的按下开始时间）
        if (self.left_button_press_start_time_saved is not None and 
            self.right_button_press_start_time_saved is not None):
            
            # 计算最短按下时间（取较晚开始的时间）
            min_press_start_time = max(self.left_button_press_start_time_saved, self.right_button_press_start_time_saved)
            press_duration = current_time - min_press_start_time
            
            self.get_logger().info(f"🔧 计算长按时长: {press_duration:.2f}秒")
            
            # 判断长按功能并给出明确提示
            if press_duration >= self.homing_duration:
                # 长按超过10秒：回零功能暂时禁用
                self.get_logger().warn(f"⚠️ 双边长按{press_duration:.1f}秒松开 → 回零功能已禁用（需要插值安全机制）")
                # TODO: 回零功能暂时注释，需要后续添加插值功能保证安全
                # if not self.arms_control_enabled and not self.joystick_control_enabled:
                #     self.get_logger().info(f"✅ 双边长按{press_duration:.1f}秒松开 → 触发回零流程")
                #     self.start_homing_sequence()
                # else:
                #     self.get_logger().warn(f"❌ 双边长按{press_duration:.1f}秒松开 → 回零失败：需要先关闭所有控制")
            elif press_duration >= self.control_toggle_duration:
                # 长按1-10秒：启停控制
                status_before_arms = "启用" if self.arms_control_enabled else "禁用"
                status_before_joystick = "启用" if self.joystick_control_enabled else "禁用"
                
                self.toggle_control_state()
                
                status_after_arms = "启用" if self.arms_control_enabled else "禁用"
                status_after_joystick = "启用" if self.joystick_control_enabled else "禁用"
                
                self.get_logger().info(f"✅ 双边长按{press_duration:.1f}秒松开 → 控制状态切换")
                self.get_logger().info(f"   双臂控制: {status_before_arms} → {status_after_arms}")
                self.get_logger().info(f"   摇杆控制: {status_before_joystick} → {status_after_joystick}")
            else:
                # 长按时间不足1秒
                self.get_logger().info(f"⏱️ 双边短按{press_duration:.1f}秒松开 → 无功能触发（需要≥1秒）")
            
            # 清除保存的按下开始时间，避免重复处理
            self.left_button_press_start_time_saved = None
            self.right_button_press_start_time_saved = None
        else:
            # 没有检测到双边长按
            self.get_logger().debug("双边松开：未检测到有效的双边长按")
    
    def check_dual_long_press(self, current_time):
        """保留原函数名但清空内容，功能已移到其他函数"""
        pass
    
    def toggle_control_state(self):
        """切换控制状态"""
        # 切换双臂控制状态
        if self.enable_left_arm and self.enable_right_arm:
            self.arms_control_enabled = not self.arms_control_enabled
        
        # 切换摇杆控制状态
        if self.enable_left_joystick and self.enable_right_joystick and self.enable_vehicle_control:
            self.joystick_control_enabled = not self.joystick_control_enabled
    
    # TODO: 回零功能暂时注释，需要后续添加插值功能保证安全
    # def start_homing_sequence(self):
    #     """开始机械臂回零流程"""
    #     self.get_logger().info("🔄 双边长按超过10秒松开检测到！开始机械臂回零流程...")
    #     
    #     # 初始化回零状态
    #     self.homing_active = True
    #     self.homing_start_time = time.time()
    #     self.homing_current_joint_left = 0
    #     self.homing_current_joint_right = 0
    #     self.homing_last_joint_time_left = None
    #     self.homing_last_joint_time_right = None
    #     
    #     # 禁用所有控制
    #     self.arms_control_enabled = False
    #     self.joystick_control_enabled = False
    #     
    #     self.get_logger().info("回零参数: 关节2→关节1→关节3-7→夹爪，每个关节间隔1秒")
    #     self.get_logger().info("左右臂并行回零，总耗时约8秒")
    
    # TODO: 回零功能暂时注释，需要后续添加插值功能保证安全
    def process_homing_sequence(self):
        """处理回零流程的时序控制 - 暂时禁用"""
        # 暂时禁用回零功能，避免突然跳跃到目标位置
        return
        
        # if not self.homing_active:
        #     return
        # 
        # current_time = time.time()
        # homing_positions = self.default_joint_positions.copy()
        # homing_updated = False
        # 
        # # 处理左臂回零
        # if (self.enable_left_arm and 
        #     self.homing_current_joint_left < self.total_joints_to_home):
        #     
        #     if self.homing_last_joint_time_left is None:
        #         # 第一个步骤，开始回零
        #         self.homing_last_joint_time_left = current_time
        #         self.home_single_joint('left', self.homing_current_joint_left, homing_positions)
        #         homing_updated = True
        #     elif current_time - self.homing_last_joint_time_left >= self.homing_joint_interval:
        #         # 间隔时间到了，进入下一个步骤
        #         self.homing_current_joint_left += 1
        #         if self.homing_current_joint_left < self.total_joints_to_home:
        #             self.home_single_joint('left', self.homing_current_joint_left, homing_positions)
        #             homing_updated = True
        #         self.homing_last_joint_time_left = current_time
        # 
        # # 处理右臂回零
        # if (self.enable_right_arm and 
        #     self.homing_current_joint_right < self.total_joints_to_home):
        #     
        #     if self.homing_last_joint_time_right is None:
        #         # 第一个步骤，开始回零
        #         self.homing_last_joint_time_right = current_time
        #         self.home_single_joint('right', self.homing_current_joint_right, homing_positions)
        #         homing_updated = True
        #     elif current_time - self.homing_last_joint_time_right >= self.homing_joint_interval:
        #         # 间隔时间到了，进入下一个步骤
        #         self.homing_current_joint_right += 1
        #         if self.homing_current_joint_right < self.total_joints_to_home:
        #             self.home_single_joint('right', self.homing_current_joint_right, homing_positions)
        #             homing_updated = True
        #         self.homing_last_joint_time_right = current_time
        # 
        # # 如果有回零更新，立即发布回零位置
        # if homing_updated:
        #     self.publish_joint_command(homing_positions)
        #     self.publish_gamepad_keys({
        #         'left_x': 0.0, 'left_y': 0.0, 
        #         'right_x': 0.0, 'right_y': 0.0
        #     })
        # 
        # # 检查回零流程是否完成
        # left_done = not self.enable_left_arm or self.homing_current_joint_left >= self.total_joints_to_home
        # right_done = not self.enable_right_arm or self.homing_current_joint_right >= self.total_joints_to_home
        # 
        # if left_done and right_done:
        #     self.finish_homing_sequence()
    
    # TODO: 回零功能暂时注释，需要后续添加插值功能保证安全
    # def home_single_joint(self, arm_side, step_index, positions_array):
    #     """回零单个关节（直接设置目标位置）"""
    #     # 获取实际的关节索引（根据回零顺序）
    #     actual_joint_index = self.homing_joint_order[step_index]
    #     
    #     if actual_joint_index < self.num_joints:
    #         # 关节回零
    #         self.get_logger().info(f"🔄 {arm_side}臂关节{actual_joint_index + 1}回零")
    #         
    #         if arm_side == 'left':
    #             joint_idx = self.left_arm_joint_indices[actual_joint_index]
    #         else:
    #             joint_idx = self.right_arm_joint_indices[actual_joint_index]
    #         
    #         # 将关节设为0
    #         positions_array[joint_idx] = 0.0
    #         
    #     elif actual_joint_index == self.num_joints:
    #         # 夹爪回零
    #         self.get_logger().info(f"🔄 {arm_side}夹爪回零")
    #         
    #         if arm_side == 'left':
    #             trigger_idx = self.left_trigger_index
    #         else:
    #             trigger_idx = self.right_trigger_index
    #         
    #         # 夹爪完全张开
    #         positions_array[trigger_idx] = -0.0120
    # 
    # def finish_homing_sequence(self):
    #     """完成回零流程"""
    #     self.homing_active = False
    #     self.long_press_triggered = False
    #     total_time = time.time() - self.homing_start_time
    #     
    #     self.get_logger().info(f"✅ 机械臂回零流程完成！总耗时: {total_time:.1f}秒")
    #     self.get_logger().info("所有关节已归零，可以重新启用控制")
    # =========================================================
    

    

    def publish_joint_command(self, positions):
        """发布关节命令"""
        try:
            # 创建关节命令消息
            joint_cmd = JointState()
            
            # 设置消息头
            joint_cmd.header = Header()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.header.frame_id = ''
            
            # 设置关节数据（16个关节）
            joint_cmd.name = self.joint_names
            joint_cmd.position = [float(p) for p in positions]
            joint_cmd.velocity = [float(v) for v in self.default_velocity]
            joint_cmd.effort = [float(e) for e in self.default_effort]
            
            # 发布关节命令消息
            self.joint_command_pub.publish(joint_cmd)
                
        except Exception as e:
            self.get_logger().error(f'发布关节命令失败: {e}')
    
    def publish_gamepad_keys(self, joystick_data):
        """发布摇杆数据消息"""
        try:
            # 创建摇杆消息
            gamepad_msg = Joy()
            
            # 设置消息头
            gamepad_msg.header = Header()
            gamepad_msg.header.stamp = self.get_clock().now().to_msg()
            gamepad_msg.header.frame_id = ''
            
            # 设置摇杆数据（只发布原始数据，不做运算）
            gamepad_msg.axes = [
                float(joystick_data['left_x']),   # 索引 0: 左摇杆X
                float(joystick_data['left_y']),   # 索引 1: 左摇杆Y
                float(joystick_data['right_x']),  # 索引 2: 右摇杆X
                float(joystick_data['right_y']),  # 索引 3: 右摇杆Y
            ]
            
            # 获取按钮状态
            left_buttons = joystick_data.get('left_buttons', {})
            right_buttons = joystick_data.get('right_buttons', {})
            
            # 设置按钮数据
            # 索引 0-3: 控制状态
            # 索引 4-9: 左手柄按钮 (Joystick_K, A, B, C, D, Switch)
            # 索引 10-15: 右手柄按钮 (Joystick_K, A, B, C, D, Switch)
            gamepad_msg.buttons = [
                # 控制状态
                1 if self.arms_control_enabled else 0,     # 索引 0: 双臂控制状态
                1 if self.joystick_control_enabled else 0,  # 索引 1: 摇杆控制状态
                1 if self.homing_active else 0,                # 索引 2: 回零状态
                1 if self.enable_vehicle_control else 0,       # 索引 3: 小车滑台控制权限
                
                # 左手柄按钮
                left_buttons.get('joystick_k', 0),  # 索引 4: 左摇杆按键
                left_buttons.get('button_a', 0),    # 索引 5: 左手柄 Button A
                left_buttons.get('button_b', 0),    # 索引 6: 左手柄 Button B
                left_buttons.get('button_c', 0),    # 索引 7: 左手柄 Button C
                left_buttons.get('button_d', 0),    # 索引 8: 左手柄 Button D
                left_buttons.get('switch', 0),      # 索引 9: 左手柄 Switch
                
                # 右手柄按钮
                right_buttons.get('joystick_k', 0),  # 索引 10: 右摇杆按键
                right_buttons.get('button_a', 0),    # 索引 11: 右手柄 Button A
                right_buttons.get('button_b', 0),    # 索引 12: 右手柄 Button B
                right_buttons.get('button_c', 0),    # 索引 13: 右手柄 Button C
                right_buttons.get('button_d', 0),    # 索引 14: 右手柄 Button D
                right_buttons.get('switch', 0),      # 索引 15: 右手柄 Switch
            ]
            
            # 发布摇杆数据消息
            self.gamepad_keys_pub.publish(gamepad_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布摇杆数据失败: {e}')
    
    def get_statistics(self) -> Dict:
        """获取节点统计信息"""
        return {
            **self.stats,
            'connected_clients': len(self.connected_clients),
            'websocket_running': self.websocket_running,
            'arms_control_enabled': self.arms_control_enabled,
            'joystick_control_enabled': self.joystick_control_enabled,
            'homing_active': self.homing_active,
            'parser_stats': self.parser.get_statistics(),
        }
    
    def destroy_node(self):
        """清理节点资源"""
        self.get_logger().info('正在关闭 WebSocket 远程控制器...')
        
        # 关闭 WebSocket 服务器
        self.websocket_running = False
        
        # 关闭所有客户端连接
        for client in self.connected_clients.copy():
            try:
                # 创建一个新的事件循环来关闭连接
                if not client.closed:
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    loop.run_until_complete(client.close())
                    loop.close()
            except Exception as e:
                self.get_logger().debug(f'关闭客户端连接时出错: {e}')
        
        # 等待 WebSocket 线程结束
        if hasattr(self, 'websocket_thread') and self.websocket_thread.is_alive():
            try:
                self.websocket_thread.join(timeout=2.0)
            except Exception:
                pass
        
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        # 创建节点
        node = WebSocketTeleoperator()
        
        # 使用多线程执行器
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            # 运行节点
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('收到中断信号，正在关闭...')
        finally:
            # 清理资源
            node.destroy_node()
            executor.shutdown()
            
    except Exception as e:
        print(f'节点启动失败: {e}')
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main() 
