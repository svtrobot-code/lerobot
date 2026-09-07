#!/usr/bin/env python3
"""
OpenArm机器人外骨骼遥操作显示Launch文件
将外骨骼TF树合并到OpenArm双臂机器人肩部

使用方法:
ros2 launch qnbot_teleoperator openarm_teleoperator.launch.py

功能:
1. 启动OpenArm双臂机器人模型（robot_state_publisher）
2. 启动外骨骼显示（包含TF发布）
3. 启动OpenArm外骨骼TF桥接节点（将外骨骼TF映射到OpenArm肩部）
4. 启动OpenArm手臂关节合并节点（合并外骨骼命令到joint_states）
5. 启动RViz可视化

参数:
- arm_type: OpenArm手臂类型 (默认: 'v10')
- ee_type: 末端执行器类型 (默认: 'openarm_hand')
- use_gui: 是否启动joint_state_publisher_gui (默认: true)
- use_exoskeleton: 是否启用外骨骼集成 (默认: true)
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def robot_state_publisher_spawner(context: LaunchContext, arm_type, ee_type):
    """生成OpenArm robot_state_publisher节点"""
    arm_type_str = context.perform_substitution(arm_type)
    ee_type_str = context.perform_substitution(ee_type)

    xacro_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "urdf", "robot", f"{arm_type_str}.urdf.xacro"
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "ee_type": ee_type_str,
            "bimanual": "true",  # 强制使用双臂配置
        }
    ).toprettyxml(indent="  ")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def generate_launch_description():
    """生成launch描述"""
    
    # 获取包目录
    openarm_pkg_dir = get_package_share_directory('openarm_description')
    qnbot_pkg_dir = get_package_share_directory('qnbot_teleoperator')
    
    # 声明launch参数
    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='v10',
        description='OpenArm手臂类型 (e.g., v10)'
    )
    
    ee_type_arg = DeclareLaunchArgument(
        'ee_type',
        default_value='openarm_hand',
        description='末端执行器类型 (e.g., openarm_hand or none)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时钟'
    )
    
    use_exoskeleton_arg = DeclareLaunchArgument(
        'use_exoskeleton',
        default_value='true',
        description='启用外骨骼TF桥接集成'
    )
    
    # 获取LaunchConfiguration
    arm_type = LaunchConfiguration('arm_type')
    ee_type = LaunchConfiguration('ee_type')
    
    # OpenArm Robot State Publisher
    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner,
        args=[arm_type, ee_type]
    )
    
    # 包含外骨骼显示launch文件
    exoskeleton_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qnbot_teleoperator'),
                'launch',
                'exoskeleton_display.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': 'false',  # 不启动额外的RViz
            'source': 'exo_command',  # 使用外骨骼命令话题作为数据源
            'namespace': 'exoskeleton'  # 使用外骨骼名空间
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_exoskeleton'))
    )
    
    # OpenArm TF桥接节点 - 将外骨骼手臂TF映射到OpenArm肩膀
    openarm_tf_bridge_node = Node(
        package='qnbot_teleoperator',
        executable='openarm_exo_tf_bridge_node',
        name='openarm_exo_tf_bridge_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'exo_namespace': 'exoskeleton',
            'robot_base_frame': 'world',
            'publish_rate': 50.0,
            'enable_left_arm': True,
            'enable_right_arm': True
        }],
        condition=IfCondition(LaunchConfiguration('use_exoskeleton')),
        output='screen'
    )
    
    # OpenArm手臂关节合并节点 - 将手臂命令合并到joint_states
    openarm_arm_joint_merger_node = Node(
        package='qnbot_teleoperator',
        executable='openarm_arm_joint_merger',
        name='openarm_arm_joint_merger',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(LaunchConfiguration('use_exoskeleton')),
        output='screen'
    )
    
    # RViz节点 - 使用OpenArm的双臂配置
    rviz_config_path = os.path.join(openarm_pkg_dir, 'rviz', 'bimanual.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # 参数声明
        arm_type_arg,
        ee_type_arg,
        use_sim_time_arg,
        use_exoskeleton_arg,
        
        # 节点启动
        robot_state_publisher_loader,
        exoskeleton_display_launch,      # 添加外骨骼显示
        openarm_tf_bridge_node,          # 添加OpenArm TF桥接节点
        openarm_arm_joint_merger_node,   # 添加OpenArm手臂关节合并节点
        rviz_node
    ])

