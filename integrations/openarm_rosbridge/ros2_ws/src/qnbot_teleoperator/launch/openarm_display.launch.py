#!/usr/bin/env python3
"""
OpenArm机器人显示Launch文件（带外骨骼集成）
用于启动OpenArm双臂机器人显示和外骨骼集成

使用方法:
ros2 launch qnbot_teleoperator openarm_display.launch.py use_exoskeleton:=true
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
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration('use_sim_time')
            }],
        )
    ]


def generate_launch_description():
    """生成launch描述"""
    
    # 获取包目录
    openarm_pkg_dir = get_package_share_directory('openarm_description')
    
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
        description='使用仿真（Gazebo）时钟'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='启动joint_state_publisher_gui'
    )
    
    use_exoskeleton_arg = DeclareLaunchArgument(
        'use_exoskeleton',
        default_value='false',
        description='启用外骨骼TF桥接集成'
    )
    
    launch_exo_model_arg = DeclareLaunchArgument(
        'launch_exo_model',
        default_value='true',
        description='是否启动外骨骼模型显示（如果已在外部启动可设为false）'
    )
    
    # 获取LaunchConfiguration
    arm_type = LaunchConfiguration('arm_type')
    ee_type = LaunchConfiguration('ee_type')
    
    # OpenArm Robot State Publisher
    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner,
        args=[arm_type, ee_type]
    )
    
    # Joint State Publisher GUI节点
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('/joint_states', '/joint_states_gui')  # 重映射避免冲突
        ],
        condition=IfCondition(LaunchConfiguration('use_gui')),
        output='screen'
    )
    
    # Joint State Publisher节点（无GUI）
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=UnlessCondition(LaunchConfiguration('use_gui')),
        output='screen'
    )
    
    # 包含外骨骼显示launch文件
    # 仅当 use_exoskeleton=true 且 launch_exo_model=true 时启动
    from launch.substitutions import PythonExpression
    exoskeleton_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qnbot_teleoperator'),
                'launch',
                'exoskeleton_display.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': 'false',  # 不启动额外的RViz，使用主RViz
            'source': 'exo_command',  # 使用外骨骼命令话题作为数据源
            'namespace': 'exoskeleton'  # 使用外骨骼名空间
        }.items(),
        condition=IfCondition(PythonExpression([
            LaunchConfiguration('use_exoskeleton'), " and ", LaunchConfiguration('launch_exo_model')
        ]))
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
        use_gui_arg,
        use_exoskeleton_arg,
        launch_exo_model_arg,
        
        # 节点启动
        robot_state_publisher_loader,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        exoskeleton_display_launch,      # 添加外骨骼显示
        openarm_tf_bridge_node,          # 添加OpenArm TF桥接节点
        openarm_arm_joint_merger_node,   # 添加OpenArm手臂关节合并节点
        rviz_node
    ])

