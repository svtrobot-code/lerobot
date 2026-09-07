import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'qnbot_teleoperator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'qnbot_teleoperator.openarm.src': ['*.so'],
        'qnbot_teleoperator.openarm.mode': ['*.urdf', '*.xml', '*.stl', '*.dae', '*.STL', '*.TXT'],
        'qnbot_teleoperator.openarm.mode.meshes': ['**/*.stl', '**/*.dae'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/websocket_teleoperator.launch.py',
            'launch/exoskeleton_display.launch.py',
            'launch/exo_retargeting.launch.py',
            'launch/openarm_teleoperator.launch.py',
            'launch/openarm_display.launch.py',
            'launch/openarm_hardware_control.launch.py',
            'launch/exoskeleton_bridge.launch.py'
        ]),
        # Configuration files
        ('share/' + package_name + '/config', [
            'config/qnbot_teleoperator_config.yaml',
            'config/exoskeleton_display.rviz',
            'config/retargeting_OpenArm.yaml'
        ]),
        # Kinematics chain files
        ('share/' + package_name + '/config/target', glob('config/target/*.yaml')),
        # URDF/xacro files
        ('share/' + package_name + '/resource/urdf', [
            'resource/urdf/qnbot_exoskeleton.xacro',
            'resource/urdf/qnbot_exoskeleton_right.xacro',
            'resource/urdf/qnbot_exoskeleton_left.xacro'
        ]),
        # Mesh files
        ('share/' + package_name + '/resource/meshs', glob('resource/meshs/*.STL')),
    ],
    install_requires=[
        'setuptools', 
        'websockets', 
        'asyncio',
        'numpy',
        'pin'  # Pinocchio for gravity compensation
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='WebSocket远程控制器，接收外骨骼数据并转换为机器人关节命令',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'websocket_teleoperator = qnbot_teleoperator.websocket_teleoperator:main',
            'exo_protocol_parser = qnbot_teleoperator.exo_protocol_parser:main',
            'exo_retargeting_node = qnbot_teleoperator.exo_retargeting_node:main',
            'openarm_exo_tf_bridge_node = qnbot_teleoperator.openarm_exo_tf_bridge_node:main',
            'openarm_arm_joint_merger = qnbot_teleoperator.openarm_arm_joint_merger:main',
            'openarm_hardware_control_node = qnbot_teleoperator.openarm_hardware_control_node:main',
            'exoskeleton_bridge_node = qnbot_teleoperator.exoskeleton_bridge_node:main',
        ],
    },
)
