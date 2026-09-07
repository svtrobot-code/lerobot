#!/usr/bin/env python3
"""
外骨骼显示launch文件 (ROS2版本)
用于在rviz中显示和测试外骨骼模型
使用独立名空间避免与机器人joint_states话题冲突
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    """根据参数配置节点"""
    
    # 获取参数值
    source_value = LaunchConfiguration('source').perform(context)
    namespace_value = LaunchConfiguration('namespace').perform(context)
    
    # 获取xacro文件路径并生成robot_description
    xacro_file = PathJoinSubstitution([
        FindPackageShare('qnbot_teleoperator'),
        'resource', 'urdf', 'qnbot_exoskeleton.xacro'
    ])

    # 使用ParameterValue确保传参为字符串
    robot_description_content = ParameterValue(
        Command(['xacro', ' ', xacro_file]),
        value_type=str
    )
    
    # 根据source参数设置remappings
    remappings = []
    if source_value == 'exo_command':
        # 使用外骨骼专用的joint_command话题
        remappings = [(f'/{namespace_value}/joint_states', '/exo/joint_command')]
    elif source_value == 'gui':
        # GUI模式下也使用外骨骼专用的joint_command话题
        remappings = [(f'/{namespace_value}/joint_states', '/exo/joint_command')]
    
    # robot_state_publisher节点 - 添加frame_prefix参数
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace_value,
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 50.0,
            'frame_prefix': f'{namespace_value}/'
        }],
        remappings=remappings
    )
    
    # 创建节点列表
    nodes = [robot_state_publisher_node]
    
    # 根据source参数添加相应的joint_state_publisher
    if source_value == 'gui':
        # joint_state_publisher_gui（GUI版本）- 重映射到外骨骼话题
        joint_state_publisher_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=namespace_value,
            output='screen',
            parameters=[{
                'robot_description': robot_description_content
            }],
            remappings=[
                # 将GUI发布的joint_states重映射到外骨骼专用话题
                (f'/{namespace_value}/joint_states', '/exo/joint_command')
            ]
        )
        nodes.append(joint_state_publisher_gui_node)
    
    # rviz配置文件路径
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('qnbot_teleoperator'),
        'config', 'exoskeleton_display.rviz'
    ])
    
    # rviz节点 - 保持在全局名空间，但配置TF前缀
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name=f'rviz2_{namespace_value}',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{
            'robot_description': robot_description_content
        }],
        remappings=[
            ('/robot_description', f'/{namespace_value}/robot_description'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    nodes.append(rviz_node)
    
    return nodes


def generate_launch_description():
    """生成launch描述"""
    
    # 声明launch参数
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否启动rviz可视化'
    )
    
    # 添加数据源选择参数
    source_arg = DeclareLaunchArgument(
        'source',
        default_value='exo_command',
        description='joint_states数据源：gui(GUI位置发布器，发布到/exo/joint_command) 或 exo_command(外骨骼命令话题)',
        choices=['gui', 'exo_command']
    )
    
    # 添加名空间参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='exoskeleton',
        description='外骨骼系统的ROS名空间，避免与机器人话题冲突'
    )
    
    # 创建launch描述
    return LaunchDescription([
        rviz_arg,
        source_arg,
        namespace_arg,
        OpaqueFunction(function=launch_setup)
    ]) 