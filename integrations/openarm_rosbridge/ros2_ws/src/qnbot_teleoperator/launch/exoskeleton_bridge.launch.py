import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    gripper_threshold_arg = DeclareLaunchArgument(
        'gripper_threshold',
        default_value='0.0001',
        description='夹爪位置变化的最小阈值 (m)，用于减少Action请求频率'
    )
    
    gripper_scaling_factor_arg = DeclareLaunchArgument(
        'gripper_scaling_factor',
        default_value='0.044',
        description='夹爪数值缩放因子：外骨骼归一化值(0-1) -> 机械臂物理值(米)。默认0.044表示外骨骼1.0对应机械臂4.4cm'
    )

    gripper_min_interval_arg = DeclareLaunchArgument(
        'gripper_min_interval',
        default_value='0.0',
        description='夹爪动作最小间隔(秒)，避免高频抖动导致反复开合'
    )

    gripper_filter_alpha_arg = DeclareLaunchArgument(
        'gripper_filter_alpha',
        default_value='1.0',
        description='夹爪低通滤波系数(0-1)，值越小越平滑'
    )

    gripper_close_snap_arg = DeclareLaunchArgument(
        'gripper_close_snap',
        default_value='0.2',
        description='夹爪闭合吸附阈值(归一化0-1)，低于该值直接视为闭合'
    )

    gripper_open_snap_arg = DeclareLaunchArgument(
        'gripper_open_snap',
        default_value='0.3',
        description='夹爪解锁阈值(归一化0-1)，高于该值才解除闭合吸附'
    )

    # 桥接节点
    bridge_node = Node(
        package='qnbot_teleoperator',
        executable='exoskeleton_bridge_node',
        name='exoskeleton_bridge_node',
        output='screen',
        parameters=[{
            'gripper_threshold': LaunchConfiguration('gripper_threshold'),
            'gripper_scaling_factor': LaunchConfiguration('gripper_scaling_factor'),
            'gripper_min_interval': LaunchConfiguration('gripper_min_interval'),
            'gripper_filter_alpha': LaunchConfiguration('gripper_filter_alpha'),
            'gripper_close_snap': LaunchConfiguration('gripper_close_snap'),
            'gripper_open_snap': LaunchConfiguration('gripper_open_snap')
        }]
    )

    return LaunchDescription([
        gripper_threshold_arg,
        gripper_scaling_factor_arg,
        gripper_min_interval_arg,
        gripper_filter_alpha_arg,
        gripper_close_snap_arg,
        gripper_open_snap_arg,
        bridge_node
    ])

