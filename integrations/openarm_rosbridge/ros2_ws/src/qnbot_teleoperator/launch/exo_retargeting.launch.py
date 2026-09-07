#!/usr/bin/env python3
"""
外骨骼重定向节点启动文件
启动外骨骼数据重定向到目标机器人的节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    declare_robot_type = DeclareLaunchArgument(
        'robot_type',
        default_value='OpenArm',
        description='机器人类型 (OpenArm等) - 自动加载对应配置文件'
    )
    
    declare_enable_left_arm = DeclareLaunchArgument(
        'enable_left_arm_retargeting',
        default_value='true',
        description='是否启用左臂重定向'
    )
    
    declare_enable_right_arm = DeclareLaunchArgument(
        'enable_right_arm_retargeting', 
        default_value='true',
        description='是否启用右臂重定向'
    )

    declare_enable_pose_alignment = DeclareLaunchArgument(
        'enable_pose_alignment',
        default_value='true',
        description='是否启用姿态对齐以避免启用瞬间跳变'
    )
    
    # 外骨骼重定向节点
    exo_retargeting_node = Node(
        package='qnbot_teleoperator',
        executable='exo_retargeting_node',
        name='exo_retargeting_node',
        output='screen',
        parameters=[{
            'robot_type': LaunchConfiguration('robot_type'),
            'enable_left_arm_retargeting': LaunchConfiguration('enable_left_arm_retargeting'),
            'enable_right_arm_retargeting': LaunchConfiguration('enable_right_arm_retargeting'),
            'enable_pose_alignment': LaunchConfiguration('enable_pose_alignment'),
        }],
        remappings=[
            # 可以在这里添加话题重映射
        ]
    )
    
    return LaunchDescription([
        declare_robot_type,
        declare_enable_left_arm,
        declare_enable_right_arm,
        declare_enable_pose_alignment,
        exo_retargeting_node,
    ])
