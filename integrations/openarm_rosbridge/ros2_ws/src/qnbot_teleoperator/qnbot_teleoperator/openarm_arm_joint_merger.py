#!/usr/bin/env python3
"""
OpenArm手臂关节合并节点
订阅/left_arm/joint_command和/right_arm/joint_command话题
将手臂关节状态合并到完整的joint_state消息中发布

该节点会：
1. 订阅左右手臂的关节命令话题
2. 从joint_state_publisher_gui订阅其他关节的状态（如果有）
3. 合并所有关节状态并发布到/joint_states话题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy


class OpenArmArmJointMerger(Node):
    def __init__(self):
        super().__init__('openarm_arm_joint_merger')
        
        # OpenArm手臂关节名称（7自由度）
        # 基于openarm_arm.xacro的关节定义
        self.left_arm_joints = [
            'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3', 
            'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6', 
            'openarm_left_joint7'
        ]
        
        self.right_arm_joints = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3', 
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6', 
            'openarm_right_joint7'
        ]
        
        # 如果有末端执行器（手），可能还有额外的关节
        # 这里先只处理7自由度手臂
        
        # 所有OpenArm机器人关节名称
        self.all_joint_names = self.left_arm_joints + self.right_arm_joints
        
        # 关节状态缓存
        self.left_arm_positions = [0.0] * len(self.left_arm_joints)
        self.right_arm_positions = [0.0] * len(self.right_arm_joints)
        self.other_joint_positions = {}
        
        # 锁定机制
        self.lock = threading.Lock()
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # 创建订阅器
        self.left_arm_sub = self.create_subscription(
            JointState,
            '/left_arm/joint_command',
            self.left_arm_callback,
            qos_profile
        )
        
        self.right_arm_sub = self.create_subscription(
            JointState,
            '/right_arm/joint_command',
            self.right_arm_callback,
            qos_profile
        )
        
        # 订阅joint_state_publisher_gui发布的其他关节状态（如果有）
        self.gui_joint_sub = self.create_subscription(
            JointState,
            '/joint_states_gui',
            self.gui_joint_callback,
            qos_profile
        )
        
        # 创建发布器 - 发布到标准的/joint_states话题
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            qos_profile
        )
        
        # 创建定时器定期发布合并后的关节状态
        self.timer = self.create_timer(0.02, self.publish_merged_joint_state)  # 50Hz
        
        self.get_logger().info('✅ OpenArm手臂关节合并节点已启动')
        self.get_logger().info(f'   - 订阅左臂话题: /left_arm/joint_command')
        self.get_logger().info(f'   - 订阅右臂话题: /right_arm/joint_command')
        self.get_logger().info(f'   - 订阅GUI话题: /joint_states_gui')
        self.get_logger().info(f'   - 发布合并话题: /joint_states')
        self.get_logger().info(f'   - 支持的关节总数: {len(self.all_joint_names)}')
    
    def left_arm_callback(self, msg: JointState):
        """处理左臂关节命令"""
        with self.lock:
            if len(msg.position) >= len(self.left_arm_joints):
                self.left_arm_positions = list(msg.position[:len(self.left_arm_joints)])
                self.get_logger().debug(f'更新左臂关节: {self.left_arm_positions}')
    
    def right_arm_callback(self, msg: JointState):
        """处理右臂关节命令"""
        with self.lock:
            if len(msg.position) >= len(self.right_arm_joints):
                self.right_arm_positions = list(msg.position[:len(self.right_arm_joints)])
                self.get_logger().debug(f'更新右臂关节: {self.right_arm_positions}')
    
    def gui_joint_callback(self, msg: JointState):
        """处理joint_state_publisher_gui发布的其他关节状态"""
        with self.lock:
            # 只保存非手臂关节的状态
            for i, joint_name in enumerate(msg.name):
                if (joint_name not in self.left_arm_joints and 
                    joint_name not in self.right_arm_joints and
                    i < len(msg.position)):
                    self.other_joint_positions[joint_name] = msg.position[i]
    
    def publish_merged_joint_state(self):
        """发布合并后的关节状态"""
        with self.lock:
            # 创建合并后的关节状态消息
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.header.frame_id = ""
            
            joint_state_msg.name = []
            joint_state_msg.position = []
            
            # 添加左臂关节
            for i, joint_name in enumerate(self.left_arm_joints):
                joint_state_msg.name.append(joint_name)
                joint_state_msg.position.append(self.left_arm_positions[i])
            
            # 添加右臂关节
            for i, joint_name in enumerate(self.right_arm_joints):
                joint_state_msg.name.append(joint_name)
                joint_state_msg.position.append(self.right_arm_positions[i])
            
            # 添加其他关节（如果有）
            for joint_name, position in self.other_joint_positions.items():
                joint_state_msg.name.append(joint_name)
                joint_state_msg.position.append(position)
            
            # 发布合并后的关节状态
            self.joint_state_pub.publish(joint_state_msg)
            
            # 定期输出统计信息
            if hasattr(self, '_publish_count'):
                self._publish_count += 1
            else:
                self._publish_count = 1
                
            if self._publish_count % 500 == 0:  # 每10秒输出一次（50Hz * 500 = 10s）
                self.get_logger().info(
                    f'📊 关节状态发布统计: 总计{self._publish_count}次, '
                    f'左臂{len([p for p in self.left_arm_positions if abs(p) > 0.001])}个关节有运动, '
                    f'右臂{len([p for p in self.right_arm_positions if abs(p) > 0.001])}个关节有运动'
                )


def main(args=None):
    rclpy.init(args=args)
    
    openarm_arm_joint_merger = OpenArmArmJointMerger()
    
    try:
        rclpy.spin(openarm_arm_joint_merger)
    except KeyboardInterrupt:
        pass
    finally:
        openarm_arm_joint_merger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

