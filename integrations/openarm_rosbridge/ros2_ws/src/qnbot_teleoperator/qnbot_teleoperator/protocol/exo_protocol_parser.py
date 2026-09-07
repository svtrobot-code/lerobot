#!/usr/bin/env python3
"""
外骨骼协议解析器
解析来自 WebSocket 的外骨骼数据，提取关节角度信息、摇杆数据和IMU数据
"""

import struct
import numpy as np
from typing import Dict, List, Optional, Tuple, Any


class ExoProtocolParser:
    """外骨骼数据协议解析器"""
    
    # 协议常量
    FRAME_HEADER = 0xAA
    FRAME_TAIL = 0x55
    
    # 支持的载荷长度（不含帧头/校验/帧尾）
    # 基础版: 摇杆(8*int16) + 关节(16*int16) = 48B
    # 躯干IMU版(四元数): 基础48B + (fAcc[3]+fGyro[3]+fQuat[4]) = 48 + (10*4) = 88B
    # 躯干+附加IMU版(四元数): 48 + 40(躯干) + 40(附加) = 128B
    DATA_LEN_BASE = 48
    DATA_LEN_TORSO_IMU_QUAT = 88
    DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT = 128
    SUPPORTED_DATA_LENGTHS = [DATA_LEN_BASE, DATA_LEN_TORSO_IMU_QUAT, DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT]
    
    # 编码器转换常量：16384对应2π弧度
    ENCODER_TO_RADIAN = (2 * np.pi) / 16384
    
    # 数据字段偏移量 (字节)
    JOYSTICK_LEFT_OFFSET = 0      # 左摇杆数据，8字节 (4个int16)
    JOYSTICK_RIGHT_OFFSET = 8     # 右摇杆数据，8字节 (4个int16)
    ARM_JOINT_LEFT_OFFSET = 16    # 左臂关节数据，16字节 (8个int16)
    ARM_JOINT_RIGHT_OFFSET = 32   # 右臂关节数据，16字节 (8个int16)
    
    # 摇杆数据范围
    JOYSTICK_MIN = 200
    JOYSTICK_MAX = 3896
    JOYSTICK_CENTER = 2048
    JOYSTICK_DEADZONE = 200
    
    # 扳机数据范围
    TRIGGER_MIN = 700   # 松开
    TRIGGER_MAX = 3700  # 夹紧
    
    def __init__(self):
        """初始化解析器"""
        self.last_parsed_data = None
        self.parse_count = 0
        
    def calculate_checksum(self, data: bytes) -> int:
        """计算校验和"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    def validate_frame(self, data: bytes, data_len: int) -> bool:
        """
        验证数据帧格式
        
        Args:
            data: 数据帧字节
            data_len: 载荷长度（不含帧头/校验/帧尾）
            
        Returns:
            bool: 是否通过验证
        """
        frame_size = 1 + data_len + 2  # header + payload + checksum + tail
        
        if len(data) < frame_size:
            return False
            
        # 检查帧头
        if data[0] != self.FRAME_HEADER:
            return False
            
        # 检查帧尾
        if data[frame_size - 1] != self.FRAME_TAIL:
            return False
            
        # 检查校验和（对载荷数据进行异或）
        payload = data[1:frame_size - 2]
        expected_checksum = self.calculate_checksum(payload)
        actual_checksum = data[frame_size - 2]
        
        return expected_checksum == actual_checksum
    
    def parse_websocket_message(self, message: Dict) -> Optional[Dict]:
        """
        解析来自 WebSocket 的消息
        
        Args:
            message: WebSocket 消息，格式如下：
            {
                "deviceId": "qnbot-exoskeleton",
                "timestamp": 1755862784204,
                "type": "exoskeleton-realtime",
                "data": {
                    "type": "binary",
                    "length": 51/91/131,  # 支持不同长度
                    "data": [170, 0, 0, ...],  # 字节数组
                    "parsed": {
                        "header": 170,
                        "payload": [...],
                        "checksum": 0,
                        "footer": 85,
                        "checksumValid": true
                    }
                }
            }
            
            Returns:
            解析后的数据字典，格式：
            {
                "timestamp": int,
                "device_id": str,
                "left_arm_joints": List[float],      # 8个关节的弧度值
                "right_arm_joints": List[float],     # 8个关节的弧度值
                "left_arm_joints_raw": List[int],    # 8个关节的原始值
                "right_arm_joints_raw": List[int],   # 8个关节的原始值
                "left_joystick": Dict,               # x, y, button, trigger, buttons
                "right_joystick": Dict,              # x, y, button, trigger, buttons
                "torso_imu": Dict,                   # acc, gyro, quat (如果有)
                "extra_imu": Dict,                   # acc, gyro, quat (如果有)
                "format_version": int,               # 1/2/3
                "valid": bool
            }
            
            其中 joystick 的 buttons 字段格式：
            {
                "joystick_k": int,  # 摇杆按键，1=按下，0=未按下
                "button_a": int,    # A按钮
                "button_b": int,    # B按钮
                "button_c": int,    # C按钮
                "button_d": int,    # D按钮
                "switch": int,      # 开关
            }
        """
        try:
            # 验证消息格式
            if not isinstance(message, dict):
                return None
                
            if message.get("type") != "exoskeleton-realtime":
                return None
                
            data_section = message.get("data", {})
            if not data_section or data_section.get("type") != "binary":
                return None
                
            # 获取原始数据
            raw_data = data_section.get("data", [])
            if not raw_data:
                return None
                
            # 转换为字节数组
            data_bytes = bytes(raw_data)
            
            # 尝试解析不同长度的数据格式
            parsed_data = None
            for data_len in self.SUPPORTED_DATA_LENGTHS:
                frame_size = 1 + data_len + 2  # header + payload + checksum + tail
                
                # 检查长度是否匹配
                if len(data_bytes) != frame_size:
                    continue
                    
                # 验证帧格式
                if not self.validate_frame(data_bytes, data_len):
                    continue
                    
                # 解析数据
                parsed_data = self._parse_frame_data(data_bytes, data_len)
                if parsed_data:
                    break
            
            if not parsed_data:
                return None
                
            # 添加元数据
            parsed_data["timestamp"] = message.get("timestamp", 0)
            parsed_data["device_id"] = message.get("deviceId", "unknown")
            parsed_data["valid"] = True
            
            self.last_parsed_data = parsed_data
            self.parse_count += 1
            
            return parsed_data
            
        except Exception as e:
            print(f"解析 WebSocket 消息失败: {e}")
            return None
    
    def _parse_frame_data(self, data_bytes: bytes, data_len: int) -> Optional[Dict]:
        """
        解析数据帧
        
        Args:
            data_bytes: 完整的数据帧
            data_len: 载荷长度（不含帧头/校验/帧尾）
            
        Returns:
            解析后的数据字典
        """
        try:
            # 提取有效载荷 (跳过帧头，去掉校验和和帧尾)
            frame_size = 1 + data_len + 2
            payload = data_bytes[1:frame_size - 2]
            
            # 解析左摇杆数据 (4个int16)
            left_joystick_raw = struct.unpack('<4h', payload[self.JOYSTICK_LEFT_OFFSET:self.JOYSTICK_LEFT_OFFSET + 8])
            left_joystick = self._parse_joystick_data(left_joystick_raw)
            
            # 解析右摇杆数据 (4个int16)
            right_joystick_raw = struct.unpack('<4h', payload[self.JOYSTICK_RIGHT_OFFSET:self.JOYSTICK_RIGHT_OFFSET + 8])
            right_joystick = self._parse_joystick_data(right_joystick_raw)
            
            # 解析左臂关节数据 (8个int16)
            left_arm_raw = struct.unpack('<8h', payload[self.ARM_JOINT_LEFT_OFFSET:self.ARM_JOINT_LEFT_OFFSET + 16])
            left_arm_joints = [self._encoder_to_radian(joint) for joint in left_arm_raw]
            left_arm_joints_raw = list(left_arm_raw)
            
            # 解析右臂关节数据 (8个int16)
            right_arm_raw = struct.unpack('<8h', payload[self.ARM_JOINT_RIGHT_OFFSET:self.ARM_JOINT_RIGHT_OFFSET + 16])
            right_arm_joints = [self._encoder_to_radian(joint) for joint in right_arm_raw]
            right_arm_joints_raw = list(right_arm_raw)
            
            # 基础数据
            result = {
                "left_arm_joints": left_arm_joints,
                "right_arm_joints": right_arm_joints,
                "left_arm_joints_raw": left_arm_joints_raw,
                "right_arm_joints_raw": right_arm_joints_raw,
                "left_joystick": left_joystick,
                "right_joystick": right_joystick,
                "torso_imu": {},
                "extra_imu": {},
                "format_version": 1
            }
            
            offset = 48  # 基础数据偏移
            
            # 解析IMU数据（如果有）
            if data_len == self.DATA_LEN_BASE:
                result["format_version"] = 1
            elif data_len == self.DATA_LEN_TORSO_IMU_QUAT:
                # 躯干IMU(四元数): fAcc[3], fGyro[3], fQuat[4]
                result["format_version"] = 2
                torso_acc = struct.unpack('<3f', payload[offset:offset + 12])
                offset += 12
                torso_gyro = struct.unpack('<3f', payload[offset:offset + 12])
                offset += 12
                torso_quat = struct.unpack('<4f', payload[offset:offset + 16])
                offset += 16
                
                result["torso_imu"] = {
                    "acc": list(torso_acc),
                    "gyro": list(torso_gyro),
                    "quat": list(torso_quat)
                }
            elif data_len == self.DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT:
                # 躯干IMU + 附加IMU (四元数)
                result["format_version"] = 3
                torso_acc = struct.unpack('<3f', payload[offset:offset + 12])
                offset += 12
                torso_gyro = struct.unpack('<3f', payload[offset:offset + 12])
                offset += 12
                torso_quat = struct.unpack('<4f', payload[offset:offset + 16])
                offset += 16
                
                extra_acc = struct.unpack('<3f', payload[offset:offset + 12])
                offset += 12
                extra_gyro = struct.unpack('<3f', payload[offset:offset + 12])
                offset += 12
                extra_quat = struct.unpack('<4f', payload[offset:offset + 16])
                offset += 16
                
                result["torso_imu"] = {
                    "acc": list(torso_acc),
                    "gyro": list(torso_gyro),
                    "quat": list(torso_quat)
                }
                result["extra_imu"] = {
                    "acc": list(extra_acc),
                    "gyro": list(extra_gyro),
                    "quat": list(extra_quat)
                }
            
            return result
            
        except Exception as e:
            print(f"解析数据帧失败: {e}")
            return None
    
    def _parse_joystick_data(self, raw_data: Tuple[int, int, int, int]) -> Dict:
        """解析摇杆数据"""
        x, y, button_info, trigger = raw_data
        
        # 解析按钮信息（button_info的低字节包含6个按钮状态）
        button_byte = button_info & 0xFF
        buttons = self._parse_button_info(button_byte)
        
        return {
            "x": self._normalize_joystick_axis(x),
            "y": self._normalize_joystick_axis(y),
            "button": button_info,  # 保留原始值以兼容旧代码
            "buttons": buttons,  # 新增：独立的按钮状态
            "trigger": self._normalize_trigger(trigger)
        }
    
    def _parse_button_info(self, button_byte: int) -> Dict:
        """
        解析按钮信息字节
        
        按钮编码格式：
        Bit 0: Joystick_K (摇杆按键)
        Bit 1: Button_A
        Bit 2: Button_B
        Bit 3: Button_C
        Bit 4: Button_D
        Bit 5: Switch (开关)
        
        Args:
            button_byte: 按钮信息字节
            
        Returns:
            Dict: 包含各个按钮状态的字典，1表示按下，0表示未按下
        """
        return {
            "joystick_k": (button_byte >> 0) & 0x01,  # Bit 0
            "button_a": (button_byte >> 1) & 0x01,    # Bit 1
            "button_b": (button_byte >> 2) & 0x01,    # Bit 2
            "button_c": (button_byte >> 3) & 0x01,    # Bit 3
            "button_d": (button_byte >> 4) & 0x01,    # Bit 4
            "switch": (button_byte >> 5) & 0x01,      # Bit 5
        }
    
    def _normalize_joystick_axis(self, value: int) -> float:
        """归一化摇杆轴值到 [-1.0, 1.0]"""
        # 应用死区
        if abs(value - self.JOYSTICK_CENTER) < self.JOYSTICK_DEADZONE:
            return 0.0
            
        # 归一化
        if value > self.JOYSTICK_CENTER:
            return (value - self.JOYSTICK_CENTER) / (self.JOYSTICK_MAX - self.JOYSTICK_CENTER)
        else:
            return (value - self.JOYSTICK_CENTER) / (self.JOYSTICK_CENTER - self.JOYSTICK_MIN)
    
    def _normalize_trigger(self, value: int) -> float:
        """归一化扳机值到 [0.0, 1.0]，张开=1.0，夹紧=0.0"""
        if value <= self.TRIGGER_MIN:
            return 1.0  # 松开扳机 -> 夹爪张开
        elif value >= self.TRIGGER_MAX:
            return 0.0  # 按紧扳机 -> 夹爪夹紧
        else:
            # 反向映射：扳机值越大，归一化结果越小
            normalized = (value - self.TRIGGER_MIN) / (self.TRIGGER_MAX - self.TRIGGER_MIN)
            return 1.0 - normalized
    
    def _encoder_to_radian(self, encoder_value: int) -> float:
        """将编码器值转换为弧度"""
        return encoder_value * self.ENCODER_TO_RADIAN
    
    def get_statistics(self) -> Dict:
        """获取解析统计信息"""
        return {
            "total_parsed": self.parse_count,
            "last_parse_time": self.last_parsed_data.get("timestamp", 0) if self.last_parsed_data else 0,
        }


def main():
    """测试函数"""
    parser = ExoProtocolParser()
    
    print("测试外骨骼协议解析器")
    print("=" * 60)
    
    # 测试基础格式 (48字节载荷)
    print("\n1. 测试基础格式 (48字节载荷):")
    test_message_base = {
        "deviceId": "qnbot-exoskeleton",
        "timestamp": 1755862784204,
        "type": "exoskeleton-realtime",
        "data": {
            "type": "binary",
            "length": 51,
            "data": [170] + [0] * 49 + [85],  # 简单的测试数据
        }
    }
    
    result = parser.parse_websocket_message(test_message_base)
    if result:
        print("  ✓ 解析成功")
        print(f"  - 格式版本: {result['format_version']}")
        print(f"  - 左臂关节数(弧度): {len(result['left_arm_joints'])} 个")
        print(f"  - 右臂关节数(弧度): {len(result['right_arm_joints'])} 个")
        print(f"  - 左臂关节数(原始): {len(result['left_arm_joints_raw'])} 个")
        print(f"  - 右臂关节数(原始): {len(result['right_arm_joints_raw'])} 个")
        print(f"  - 左摇杆: x={result['left_joystick']['x']:.2f}, y={result['left_joystick']['y']:.2f}, "
              f"button={result['left_joystick']['button']}, trigger={result['left_joystick']['trigger']:.2f}")
        print(f"  - 左手柄按钮: {result['left_joystick']['buttons']}")
        print(f"  - 右摇杆: x={result['right_joystick']['x']:.2f}, y={result['right_joystick']['y']:.2f}, "
              f"button={result['right_joystick']['button']}, trigger={result['right_joystick']['trigger']:.2f}")
        print(f"  - 右手柄按钮: {result['right_joystick']['buttons']}")
        print(f"  - 时间戳: {result['timestamp']}")
    else:
        print("  ✗ 解析失败")
    
    # 测试带躯干IMU格式 (88字节载荷)
    print("\n2. 测试带躯干IMU格式 (88字节载荷):")
    test_message_imu = {
        "deviceId": "qnbot-exoskeleton",
        "timestamp": 1755862784205,
        "type": "exoskeleton-realtime",
        "data": {
            "type": "binary",
            "length": 91,
            "data": [170] + [0] * 89 + [85],
        }
    }
    
    result = parser.parse_websocket_message(test_message_imu)
    if result:
        print("  ✓ 解析成功")
        print(f"  - 格式版本: {result['format_version']}")
        print(f"  - 躯干IMU数据: {result.get('torso_imu', {})}")
    else:
        print("  ✗ 解析失败")
    
    # 测试带躯干+附加IMU格式 (128字节载荷)
    print("\n3. 测试带躯干+附加IMU格式 (128字节载荷):")
    test_message_full = {
        "deviceId": "qnbot-exoskeleton",
        "timestamp": 1755862784206,
        "type": "exoskeleton-realtime",
        "data": {
            "type": "binary",
            "length": 131,
            "data": [170] + [0] * 129 + [85],
        }
    }
    
    result = parser.parse_websocket_message(test_message_full)
    if result:
        print("  ✓ 解析成功")
        print(f"  - 格式版本: {result['format_version']}")
        print(f"  - 躯干IMU数据: {result.get('torso_imu', {})}")
        print(f"  - 附加IMU数据: {result.get('extra_imu', {})}")
    else:
        print("  ✗ 解析失败")
    
    # 统计信息
    print("\n" + "=" * 60)
    stats = parser.get_statistics()
    print(f"总解析次数: {stats['total_parsed']}")
    print(f"最后解析时间: {stats['last_parse_time']}")
    print("=" * 60)


if __name__ == "__main__":
    main() 