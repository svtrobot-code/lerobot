#!/usr/bin/env python3
"""
OpenArm硬件控制Launch文件
启动实际机械臂电机控制节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成Launch描述"""
    
    # 获取URDF文件的默认路径
    # 使用与openarm_single_control.py相同的URDF文件
    pkg_share = FindPackageShare('qnbot_teleoperator').find('qnbot_teleoperator')
    # pkg_share返回的是 .../install/qnbot_teleoperator/share/qnbot_teleoperator
    # 需要回到install目录再找到lib路径
    install_dir = os.path.dirname(os.path.dirname(pkg_share))  # 回到install/qnbot_teleoperator
    default_urdf_path = os.path.join(
        install_dir,
        'lib/python3.10/site-packages/qnbot_teleoperator/openarm/mode/openarm_bimanual.urdf'
    )
    
    # 声明Launch参数
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf_path,
        description='URDF文件路径（用于Pinocchio重力补偿计算）'
    )
    
    usb_serial_number_arg = DeclareLaunchArgument(
        'usb_serial_number',
        default_value='1F1AD573CBBDF4AC08D448446A81E925',
        description='USB CAN适配器序列号'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='100.0',
        description='控制频率（Hz）- 默认100Hz与原始代码一致'
    )
    
    gravity_compensation_factor_arg = DeclareLaunchArgument(
        'gravity_compensation_factor',
        default_value='0.8',
        description='重力补偿系数（0.0-1.0）'
    )
    
    enable_gravity_compensation_arg = DeclareLaunchArgument(
        'enable_gravity_compensation',
        default_value='true',
        description='是否启用重力补偿'
    )
    
    # OpenArm硬件控制节点
    openarm_hardware_control_node = Node(
        package='qnbot_teleoperator',
        executable='openarm_hardware_control_node',
        name='openarm_hardware_control_node',
        output='screen',
        parameters=[{
            'urdf_path': LaunchConfiguration('urdf_path'),
            'usb_serial_number': LaunchConfiguration('usb_serial_number'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'gravity_compensation_factor': LaunchConfiguration('gravity_compensation_factor'),
            'enable_gravity_compensation': LaunchConfiguration('enable_gravity_compensation'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        urdf_path_arg,
        usb_serial_number_arg,
        control_frequency_arg,
        gravity_compensation_factor_arg,
        enable_gravity_compensation_arg,
        openarm_hardware_control_node,
    ])

