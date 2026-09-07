from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("openarm_rosbridge_bimanual")
@dataclass(kw_only=True)
class OpenArmRosbridgeBimanualConfig(RobotConfig):
    """OpenArm dual-arm state and command transport over rosbridge WebSocket."""

    id: str | None = "openarm_rosbridge_bimanual"
    rosbridge_host: str = "127.0.0.1"
    rosbridge_port: int = 9090
    rosbridge_transport: str = "asyncio"
    connection_timeout_s: float = 10.0

    joint_states_topic: str = "/joint_states"
    joint_state_timeout_s: float = 2.0
    wait_for_joint_states_s: float = 10.0
    left_arm_joint_names: list[str] = field(
        default_factory=lambda: [f"openarm_left_joint{i}" for i in range(1, 8)]
    )
    right_arm_joint_names: list[str] = field(
        default_factory=lambda: [f"openarm_right_joint{i}" for i in range(1, 8)]
    )
    left_gripper_joint_name: str = "openarm_left_finger_joint1"
    right_gripper_joint_name: str = "openarm_right_finger_joint1"

    # Keep false while the exoskeleton ROS2 stack controls the robot during recording.
    active_control: bool = False
    # ``forward_position`` publishes seven arm joints directly to the ROS2 controllers.
    # ``exoskeleton_bridge`` publishes a JointState containing seven arm joints plus a
    # normalized gripper command, reusing the project's ROS2 bridge for arm and gripper control.
    command_transport: str = "forward_position"
    left_arm_command_topic: str = "/left_forward_position_controller/commands"
    right_arm_command_topic: str = "/right_forward_position_controller/commands"
    left_bridge_command_topic: str = "/left_arm/joint_command"
    right_bridge_command_topic: str = "/right_arm/joint_command"
    gripper_open_m: float = 0.044
    left_gripper_closed_m: float = -0.040
    right_gripper_closed_m: float = -0.030
    max_delta_rad: float = 0.15

    left_joint_signs: list[float] = field(default_factory=list)
    right_joint_signs: list[float] = field(default_factory=list)
    left_joint_offsets: list[float] = field(default_factory=list)
    right_joint_offsets: list[float] = field(default_factory=list)
    apply_transforms_to_observation: bool = True
    apply_transforms_to_action: bool = True

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.rosbridge_transport not in {"asyncio", "twisted"}:
            raise ValueError("rosbridge_transport must be 'asyncio' or 'twisted'.")
        if self.command_transport not in {"forward_position", "exoskeleton_bridge"}:
            raise ValueError("command_transport must be 'forward_position' or 'exoskeleton_bridge'.")
        if len(self.left_arm_joint_names) != 7 or len(self.right_arm_joint_names) != 7:
            raise ValueError("OpenArm bimanual configuration requires exactly 7 joints per arm.")
        if self.gripper_open_m <= self.left_gripper_closed_m:
            raise ValueError("gripper_open_m must be greater than left_gripper_closed_m.")
        if self.gripper_open_m <= self.right_gripper_closed_m:
            raise ValueError("gripper_open_m must be greater than right_gripper_closed_m.")
