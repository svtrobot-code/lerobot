#!/usr/bin/env python3
"""
WebSocket 远程控制器启动文件
启动 WebSocket 远程控制器，处理所有控制逻辑并发布到 /exo/* 话题

使用方法：
ros2 launch qnbot_teleoperator websocket_teleoperator.launch.py
ros2 launch qnbot_teleoperator websocket_teleoperator.launch.py enable_left_arm:=true enable_right_arm:=false
ros2 launch qnbot_teleoperator websocket_teleoperator.launch.py enable_vehicle_control:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'enable_left_arm',
            default_value='true',
            description='是否接收左臂数据'
        ),
        
        DeclareLaunchArgument(
            'enable_right_arm',
            default_value='true',
            description='是否接收右臂数据'
        ),
        
        DeclareLaunchArgument(
            'enable_left_joystick',
            default_value='true',
            description='是否接收左手柄数据'
        ),
        
        DeclareLaunchArgument(
            'enable_right_joystick',
            default_value='true',
            description='是否接收右手柄数据'
        ),
        
        DeclareLaunchArgument(
            'enable_vehicle_control',
            default_value='true',
            description='是否启用小车和滑台控制'
        ),
        
        DeclareLaunchArgument(
            'websocket_host',
            default_value='0.0.0.0',
            description='WebSocket服务器主机地址'
        ),
        
        DeclareLaunchArgument(
            'websocket_port',
            default_value='19091',
            description='WebSocket服务器端口'
        ),
        
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='100.0',
            description='外骨骼数据发布频率 (Hz)'
        ),
        
        DeclareLaunchArgument(
            'timeout_ms',
            default_value='100',
            description='数据超时时间 (ms)'
        ),
        
        # 启动信息
        LogInfo(
            msg=['启动QnBot WebSocket远程控制系统']
        ),
        
        LogInfo(
            msg=['WebSocket服务器: ', LaunchConfiguration('websocket_host'), ':', LaunchConfiguration('websocket_port')]
        ),
        
        # 启动WebSocket远程控制器（接收外骨骼数据，处理控制逻辑，发布到/exo/*话题）
        Node(
            package='qnbot_teleoperator',
            executable='websocket_teleoperator',
            name='websocket_teleoperator',
            output='screen',
            parameters=[{
                'websocket_host': LaunchConfiguration('websocket_host'),
                'websocket_port': LaunchConfiguration('websocket_port'),
                'joint_command_topic': '/exo/joint_command',
                'gamepad_keys_topic': '/exo/gamepad_keys',
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'timeout_ms': LaunchConfiguration('timeout_ms'),
                'enable_left_arm': LaunchConfiguration('enable_left_arm'),
                'enable_right_arm': LaunchConfiguration('enable_right_arm'),
                'enable_left_joystick': LaunchConfiguration('enable_left_joystick'),
                'enable_right_joystick': LaunchConfiguration('enable_right_joystick'),
                'enable_vehicle_control': LaunchConfiguration('enable_vehicle_control'),
            }],
            emulate_tty=True,
        ),
        
        # 启动信息
        LogInfo(
            msg=['WebSocket远程控制系统启动完成']
        ),
        
        LogInfo(
            msg=['发布话题: /exo/joint_command (16个关节数据), /exo/gamepad_keys (摇杆和控制状态)']
        ),
        
        LogInfo(
            msg=['控制功能: 按钮启停、回零流程、摇杆控制、扳机夹爪控制']
        ),
    ]) 