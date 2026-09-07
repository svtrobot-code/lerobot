from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("openarm_rosbridge_bimanual_teleop")
@dataclass(kw_only=True)
class OpenArmRosbridgeBimanualTeleopConfig(TeleoperatorConfig):
    """External exoskeleton targets received from ROS2 through rosbridge."""

    id: str | None = "openarm_exoskeleton"
    rosbridge_host: str = "127.0.0.1"
    rosbridge_port: int = 9090
    rosbridge_transport: str = "asyncio"
    connection_timeout_s: float = 10.0
    left_arm_command_topic: str = "/left_forward_position_controller/commands"
    right_arm_command_topic: str = "/right_forward_position_controller/commands"
    left_joint_command_topic: str = "/left_arm/joint_command"
    right_joint_command_topic: str = "/right_arm/joint_command"
    command_timeout_s: float = 2.0
    wait_for_commands_s: float = 10.0
    gripper_output_mode: str = "bridge_binary_meters"
    gripper_open_m: float = 0.044
    left_gripper_closed_m: float = -0.040
    right_gripper_closed_m: float = -0.030

    def __post_init__(self) -> None:
        if self.gripper_output_mode not in {"raw", "bridge_binary_meters"}:
            raise ValueError("gripper_output_mode must be 'raw' or 'bridge_binary_meters'.")
        if self.rosbridge_transport not in {"asyncio", "twisted"}:
            raise ValueError("rosbridge_transport must be 'asyncio' or 'twisted'.")
