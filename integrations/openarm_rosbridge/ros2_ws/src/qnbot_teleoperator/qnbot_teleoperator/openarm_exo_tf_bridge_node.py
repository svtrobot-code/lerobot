#!/usr/bin/env python3
"""
OpenArm外骨骼TF桥接节点 (OpenArm Exoskeleton TF Bridge Node)
将外骨骼的手臂TF重新映射到OpenArm机器人的肩膀位置

功能：
1. 监听外骨骼的TF数据（exoskeleton/left_base_link, exoskeleton/right_base_link）
2. 将外骨骼手臂的TF重新发布到OpenArm机器人的肩膀位置
3. 创建完整的TF树，将外骨骼手臂与OpenArm身体结合

TF映射关系：
- 外骨骼 exoskeleton/left_base_link  -> OpenArm openarm_body_link0 + 左肩偏移
- 外骨骼 exoskeleton/right_base_link -> OpenArm openarm_body_link0 + 右肩偏移

重要设计说明：
- 外骨骼绑定到固定的躯干框架(openarm_body_link0)
- 通过偏移量将外骨骼放置在肩膀位置，但保持运动独立性
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import tf2_ros
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class OpenArmExoTfBridgeNode(Node):
    """
    OpenArm外骨骼TF桥接节点
    
    功能：
    - 监听外骨骼的TF（在exoskeleton名空间中）
    - 将外骨骼手臂TF重新映射到OpenArm机器人的肩膀位置
    - 发布完整的TF树
    """
    
    def __init__(self):
        super().__init__('openarm_exo_tf_bridge_node')
        
        # 参数声明
        self.declare_parameter('exo_namespace', 'exoskeleton')
        self.declare_parameter('robot_base_frame', 'world')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('enable_left_arm', True)
        self.declare_parameter('enable_right_arm', True)
        
        # 获取参数
        self.exo_namespace = self.get_parameter('exo_namespace').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_left_arm = self.get_parameter('enable_left_arm').value
        self.enable_right_arm = self.get_parameter('enable_right_arm').value
        self.arm_scale_factor = 0.7
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 初始化TF相关组件
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 创建定时器，定期发布动态TF
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_dynamic_transforms,
            callback_group=self.callback_group
        )
        
        # 状态变量
        self.transform_stats = {
            'left_arm_published': 0,
            'right_arm_published': 0,
            'transform_errors': 0,
        }
        
        # 位姿输出计数器
        self.pose_output_counter = 0
        
        self.get_logger().info(
            f'OpenArm外骨骼TF桥接节点已启动'
            f'\n  外骨骼名空间: {self.exo_namespace}'
            f'\n  机器人基座框架: {self.robot_base_frame}'
            f'\n  发布频率: {self.publish_rate} Hz'
            f'\n  左臂启用: {"是" if self.enable_left_arm else "否"}'
            f'\n  右臂启用: {"是" if self.enable_right_arm else "否"}'
            f'\n  手臂缩放因子: {self.arm_scale_factor:.3f}'
            f'\n  🎯 位姿输出: 每100次输出Link7位姿信息 (xyz + RPY)'
            f'\n  🔗 开始TF桥接...'
        )
    
    def publish_dynamic_transforms(self):
        """发布动态TF变换，将外骨骼手臂映射到OpenArm肩膀"""
        try:
            current_time = self.get_clock().now()
            
            # 发布左臂变换
            if self.enable_left_arm:
                self.publish_arm_transform('left', current_time)
            
            # 发布右臂变换
            if self.enable_right_arm:
                self.publish_arm_transform('right', current_time)
            
            # 每100次输出外骨骼Link7位姿信息
            self.pose_output_counter += 1
            if self.pose_output_counter % 100 == 0:
                self.output_link7_poses()
            
            # 定期输出统计信息
            total_published = self.transform_stats['left_arm_published'] + self.transform_stats['right_arm_published']
            if total_published > 0 and total_published % 1000 == 0:
                self.get_logger().info(
                    f'TF桥接统计 - 左臂: {self.transform_stats["left_arm_published"]}, '
                    f'右臂: {self.transform_stats["right_arm_published"]}, '
                    f'错误: {self.transform_stats["transform_errors"]}'
                )
                
        except Exception as e:
            self.transform_stats['transform_errors'] += 1
            self.get_logger().error(f'发布动态变换失败: {e}')
    
    def publish_arm_transform(self, arm_side: str, current_time):
        """
        发布单个手臂的TF变换
        
        Args:
            arm_side: 'left' 或 'right'
            current_time: 当前时间戳
        """
        try:
            # 外骨骼手臂框架名称（在命名空间中）
            exo_arm_frame = f'{self.exo_namespace}/{arm_side}_base_link'
            
            # OpenArm肩膀固定安装点
            # 基于v10.urdf.xacro中的配置：
            # left_arm_base_xyz="0.0 0.031 0.698"
            # left_arm_base_rpy="-1.5708 0 0"
            # right_arm_base_xyz="0.0 -0.031 0.698"
            # right_arm_base_rpy="1.5708 0 0"
            shoulder_frame = 'openarm_body_link0'
            
            if arm_side == 'left':
                # 左肩膀相对于躯干的偏移
                shoulder_offset = (0.0, 0.031, 0.698)
                # 左臂RPY: -90度绕X轴
                base_rpy = (0.0, 0.0, 0.0)
            else:
                # 右肩膀相对于躯干的偏移
                shoulder_offset = (0.0, -0.031, 0.698)
                # 右臂RPY: 90度绕X轴
                base_rpy = (0.0, 0.0, 0.0)
            
            # 创建新的变换：将外骨骼手臂base_link映射到OpenArm肩膀
            try:
                arm_transform = TransformStamped()
                arm_transform.header.stamp = current_time.to_msg()
                arm_transform.header.frame_id = shoulder_frame  # OpenArm躯干
                arm_transform.child_frame_id = f'exo_{arm_side}_base_link'  # 外骨骼手臂base_link（不带命名空间前缀）
                
                # 设置偏移变换
                arm_transform.transform.translation.x = shoulder_offset[0]
                arm_transform.transform.translation.y = shoulder_offset[1] 
                arm_transform.transform.translation.z = shoulder_offset[2]
                
                # 设置旋转 - 对齐外骨骼Z轴与OpenArm手臂坐标系
                # 外骨骼Z轴向上，需要旋转使其对齐OpenArm的关节轴
                if arm_side == 'left':
                    # 左臂：组合基础旋转和外骨骼对齐
                    # 先应用base_rpy，然后调整外骨骼方向
                    quat_base = tf_transformations.quaternion_from_euler(base_rpy[0], base_rpy[1], base_rpy[2])
                    quat_align = tf_transformations.quaternion_from_euler(-math.pi/2, 0, 0)
                    quat = tf_transformations.quaternion_multiply(quat_base, quat_align)
                else:
                    # 右臂：组合基础旋转和外骨骼对齐
                    quat_base = tf_transformations.quaternion_from_euler(base_rpy[0], base_rpy[1], base_rpy[2])
                    quat_align = tf_transformations.quaternion_from_euler(math.pi/2, 0, 0)
                    quat = tf_transformations.quaternion_multiply(quat_base, quat_align)
                
                arm_transform.transform.rotation.x = quat[0]
                arm_transform.transform.rotation.y = quat[1]
                arm_transform.transform.rotation.z = quat[2]
                arm_transform.transform.rotation.w = quat[3]
                
                # 发布变换
                self.tf_broadcaster.sendTransform(arm_transform)
                
                # 递归发布外骨骼手臂的所有子链接
                self.republish_arm_tree(arm_side, f'exo_{arm_side}_base_link', current_time)
                
                # 更新统计
                if arm_side == 'left':
                    self.transform_stats['left_arm_published'] += 1
                else:
                    self.transform_stats['right_arm_published'] += 1
                
                # 定期输出绑定状态信息
                if self.transform_stats[f'{arm_side}_arm_published'] % 500 == 1:
                    self.get_logger().info(
                        f'✅ {arm_side}臂绑定状态: '
                        f'固定参考框架={shoulder_frame}, '
                        f'偏移=({shoulder_offset[0]:.3f}, {shoulder_offset[1]:.3f}, {shoulder_offset[2]:.3f})'
                    )
                    
            except Exception as inner_e:
                self.get_logger().debug(f'创建{arm_side}臂变换失败: {inner_e}')
            
        except Exception as e:
            self.transform_stats['transform_errors'] += 1
            self.get_logger().debug(f'发布{arm_side}臂变换失败: {e}')
    
    def republish_arm_tree(self, arm_side: str, new_parent_frame: str, current_time):
        """
        递归重新发布外骨骼手臂的所有子链接
        
        Args:
            arm_side: 'left' 或 'right'
            new_parent_frame: 新的父框架名称 (应该是 exo_{arm_side}_base_link)
            current_time: 当前时间戳
        """
        try:
            # 外骨骼手臂的关节名称列表
            arm_links = [
                f'{arm_side}_link1',
                f'{arm_side}_link2', 
                f'{arm_side}_link3',
                f'{arm_side}_link4',
                f'{arm_side}_link5',
                f'{arm_side}_link6',
                f'{arm_side}_link7',
                f'{arm_side}_link8'
            ]
            
            # 重新发布每个链接的TF
            current_parent = new_parent_frame  # 从exo_{arm_side}_base_link开始
            
            for i, link_name in enumerate(arm_links):
                try:
                    # 原始外骨骼框架名称（带命名空间）
                    exo_link_frame = f'{self.exo_namespace}/{link_name}'
                    
                    # 确定当前链接的原始父框架
                    if i == 0:
                        original_parent = f'{self.exo_namespace}/{arm_side}_base_link'
                    else:
                        original_parent = f'{self.exo_namespace}/{arm_links[i-1]}'
                    
                    # 获取原始变换
                    # 使用非阻塞方式检查TF是否存在
                    # 注意：timeout=0表示立即返回，不等待
                    try:
                        if self.tf_buffer.can_transform(original_parent, exo_link_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0)):
                            transform = self.tf_buffer.lookup_transform(
                                original_parent,
                                exo_link_frame,
                                rclpy.time.Time()
                            )
                        else:
                            # 如果TF暂时不可用，直接跳出循环，不阻塞
                            break
                    except Exception:
                        break
                    
                    # 创建新的变换
                    new_transform = TransformStamped()
                    new_transform.header.stamp = current_time.to_msg()
                    new_transform.header.frame_id = current_parent
                    new_transform.child_frame_id = f'exo_{link_name}'
                    
                    # 复制变换数据并应用缩放因子
                    new_transform.transform.translation.x = transform.transform.translation.x * self.arm_scale_factor
                    new_transform.transform.translation.y = transform.transform.translation.y * self.arm_scale_factor
                    new_transform.transform.translation.z = transform.transform.translation.z * self.arm_scale_factor
                    
                    # 旋转保持不变
                    new_transform.transform.rotation = transform.transform.rotation
                    
                    # 发布变换
                    self.tf_broadcaster.sendTransform(new_transform)
                    
                    # 更新父框架为当前链接
                    current_parent = f'exo_{link_name}'
                    
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    # 某些链接可能还未可用，跳过
                    break
                    
        except Exception as e:
            self.get_logger().debug(f'重新发布{arm_side}臂TF树失败: {e}')
    
    def output_link7_poses(self):
        """输出外骨骼左右臂Link7相对于openarm_body_link0的位姿信息"""
        try:
            base_frame = 'openarm_body_link0'
            pose_info = []
            
            # 获取左臂Link7位姿
            if self.enable_left_arm:
                left_pose = self.get_link7_pose('left', base_frame)
                if left_pose:
                    x, y, z, roll, pitch, yaw = left_pose
                    pose_info.append(f'左臂Link7: xyz=({x:.3f}, {y:.3f}, {z:.3f}), rpy=({math.degrees(roll):.1f}°, {math.degrees(pitch):.1f}°, {math.degrees(yaw):.1f}°)')
                else:
                    pose_info.append('左臂Link7: 数据不可用')
            
            # 获取右臂Link7位姿
            if self.enable_right_arm:
                right_pose = self.get_link7_pose('right', base_frame)
                if right_pose:
                    x, y, z, roll, pitch, yaw = right_pose
                    pose_info.append(f'右臂Link7: xyz=({x:.3f}, {y:.3f}, {z:.3f}), rpy=({math.degrees(roll):.1f}°, {math.degrees(pitch):.1f}°, {math.degrees(yaw):.1f}°)')
                else:
                    pose_info.append('右臂Link7: 数据不可用')
            
            # 输出位姿信息
            if pose_info:
                self.get_logger().info(
                    f'🎯 外骨骼Link7位姿 (相对于{base_frame}):\n  ' + '\n  '.join(pose_info)
                )
                
        except Exception as e:
            self.get_logger().debug(f'输出Link7位姿失败: {e}')
    
    def get_link7_pose(self, arm_side: str, base_frame: str):
        """
        获取指定臂的Link7相对于base_frame的位姿
        
        Args:
            arm_side: 'left' 或 'right'
            base_frame: 基准框架名称
            
        Returns:
            (x, y, z, roll, pitch, yaw) 或 None
        """
        try:
            # 外骨骼Link7框架名称
            link7_frame = f'exo_{arm_side}_link7'
            
            # 获取变换
            # 使用非阻塞方式，避免卡顿
            if self.tf_buffer.can_transform(base_frame, link7_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0)):
                transform = self.tf_buffer.lookup_transform(
                    base_frame,
                    link7_frame,
                    rclpy.time.Time()
                )
            else:
                return None
            
            # 提取位置
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # 提取RPY角度
            rot = transform.transform.rotation
            roll, pitch, yaw = tf_transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            
            return (x, y, z, roll, pitch, yaw)
            
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            # TF数据不可用
            return None
        except Exception as e:
            self.get_logger().debug(f'获取{arm_side}臂Link7位姿失败: {e}')
            return None
    
    def get_bridge_statistics(self):
        """获取桥接统计信息"""
        return self.transform_stats
    
    def destroy_node(self):
        """清理节点资源"""
        self.get_logger().info('正在关闭OpenArm外骨骼TF桥接节点...')
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        # 创建节点
        node = OpenArmExoTfBridgeNode()
        
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
        print(f'OpenArm TF桥接节点启动失败: {e}')
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

