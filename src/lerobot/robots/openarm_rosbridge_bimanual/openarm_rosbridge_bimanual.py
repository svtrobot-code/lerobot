import logging
import math
import threading
import time
from functools import cached_property
from typing import Any

import numpy as np

from lerobot.cameras import make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.import_utils import require_package

from ..robot import Robot
from .config_openarm_rosbridge_bimanual import OpenArmRosbridgeBimanualConfig

logger = logging.getLogger(__name__)


class OpenArmRosbridgeBimanual(Robot):
    """Read OpenArm state and RGB cameras without importing ROS2 Python bindings.

    In passive mode, ROS2/exoskeleton remains responsible for executing commands. This
    object only mirrors the teleoperator action into the LeRobot dataset.
    """

    config_class = OpenArmRosbridgeBimanualConfig
    name = "openarm_rosbridge_bimanual"

    def __init__(self, config: OpenArmRosbridgeBimanualConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self._connected = False
        self._ros: Any = None
        self._joint_topic: Any = None
        self._left_command_topic: Any = None
        self._right_command_topic: Any = None
        self._lock = threading.Lock()
        self._latest_positions: dict[str, float] = {}
        self._last_joint_state_time: float | None = None
        self._last_camera_frames: dict[str, np.ndarray] = {}
        self._left_signs = self._vector(config.left_joint_signs, 1.0, "left_joint_signs")
        self._right_signs = self._vector(config.right_joint_signs, 1.0, "right_joint_signs")
        self._left_offsets = self._vector(config.left_joint_offsets, 0.0, "left_joint_offsets")
        self._right_offsets = self._vector(config.right_joint_offsets, 0.0, "right_joint_offsets")
        self._last_left_command: list[float] | None = None
        self._last_right_command: list[float] | None = None

    @staticmethod
    def _vector(values: list[float], default: float, label: str) -> list[float]:
        if not values:
            return [default] * 7
        if len(values) != 7:
            raise ValueError(f"{label} must contain 7 values, got {len(values)}.")
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"{label} contains a non-finite value.")
        return [float(value) for value in values]

    @staticmethod
    def _transform(value: float, sign: float, offset: float, inverse: bool = False) -> float:
        if sign == 0:
            raise ValueError("Joint sign cannot be zero.")
        return (value - offset) / sign if inverse else value * sign + offset

    @cached_property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        features: dict[str, type | tuple[int, int, int]] = dict(self.action_features)
        for name, camera in self.config.cameras.items():
            features[name] = (camera.height, camera.width, 3)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        features = {f"left_joint{i}.pos": float for i in range(1, 8)}
        features.update({f"right_joint{i}.pos": float for i in range(1, 8)})
        features["left_gripper.pos"] = float
        features["right_gripper.pos"] = float
        return features

    @property
    def is_connected(self) -> bool:
        return (
            self._connected
            and bool(self._ros and self._ros.is_connected)
            and all(camera.is_connected for camera in self.cameras.values())
        )

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        require_package("roslibpy", "openarm-rosbridge")
        import roslibpy

        self._ros = roslibpy.Ros(
            host=self.config.rosbridge_host,
            port=self.config.rosbridge_port,
            transport=self.config.rosbridge_transport,
        )
        self._ros.run(timeout=self.config.connection_timeout_s)
        if not self._ros.is_connected:
            raise ConnectionError(
                f"Could not connect to rosbridge at ws://{self.config.rosbridge_host}:"
                f"{self.config.rosbridge_port}."
            )

        def receive_joint_state(message: dict[str, Any]) -> None:
            names = message.get("name", [])
            positions = message.get("position", [])
            if not isinstance(names, list) or not isinstance(positions, list):
                return
            valid = {
                str(name): float(position)
                for name, position in zip(names, positions, strict=False)
                if isinstance(position, int | float) and math.isfinite(float(position))
            }
            with self._lock:
                self._latest_positions.update(valid)
                self._last_joint_state_time = time.monotonic()

        self._joint_topic = roslibpy.Topic(
            self._ros, self.config.joint_states_topic, "sensor_msgs/msg/JointState"
        )
        self._joint_topic.subscribe(receive_joint_state)

        if self.config.active_control:
            use_bridge = self.config.command_transport == "exoskeleton_bridge"
            self._left_command_topic = roslibpy.Topic(
                self._ros,
                self.config.left_bridge_command_topic if use_bridge else self.config.left_arm_command_topic,
                "sensor_msgs/msg/JointState" if use_bridge else "std_msgs/msg/Float64MultiArray",
            )
            self._right_command_topic = roslibpy.Topic(
                self._ros,
                self.config.right_bridge_command_topic if use_bridge else self.config.right_arm_command_topic,
                "sensor_msgs/msg/JointState" if use_bridge else "std_msgs/msg/Float64MultiArray",
            )
            self._left_command_topic.advertise()
            self._right_command_topic.advertise()

        self._connected = True
        try:
            self._wait_for_joint_state()
            if self.config.active_control:
                with self._lock:
                    positions = dict(self._latest_positions)
                self._last_left_command = [
                    float(positions[name]) for name in self.config.left_arm_joint_names
                ]
                self._last_right_command = [
                    float(positions[name]) for name in self.config.right_arm_joint_names
                ]
            for camera in self.cameras.values():
                camera.connect()
        except Exception:
            self._disconnect_resources()
            raise
        logger.info(
            "%s connected to ws://%s:%d and %d camera(s).",
            self,
            self.config.rosbridge_host,
            self.config.rosbridge_port,
            len(self.cameras),
        )

    def _wait_for_joint_state(self) -> None:
        deadline = time.monotonic() + self.config.wait_for_joint_states_s
        required = set(
            self.config.left_arm_joint_names
            + self.config.right_arm_joint_names
            + [self.config.left_gripper_joint_name, self.config.right_gripper_joint_name]
        )
        while time.monotonic() < deadline:
            with self._lock:
                missing = required - self._latest_positions.keys()
            if not missing:
                return
            time.sleep(0.05)
        raise TimeoutError(
            f"JointState {self.config.joint_states_topic} did not provide required joints: {sorted(missing)}"
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def _joint_values(
        self, positions: dict[str, float], names: list[str], signs: list[float], offsets: list[float]
    ) -> list[float]:
        values = [float(positions[name]) for name in names]
        if self.config.apply_transforms_to_observation:
            return [self._transform(value, signs[i], offsets[i]) for i, value in enumerate(values)]
        return values

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        with self._lock:
            positions = dict(self._latest_positions)
            state_time = self._last_joint_state_time
        if state_time is None or time.monotonic() - state_time > self.config.joint_state_timeout_s:
            raise TimeoutError(f"JointState stream {self.config.joint_states_topic} is stale.")

        left = self._joint_values(
            positions, self.config.left_arm_joint_names, self._left_signs, self._left_offsets
        )
        right = self._joint_values(
            positions, self.config.right_arm_joint_names, self._right_signs, self._right_offsets
        )
        observation: RobotObservation = {
            **{f"left_joint{i + 1}.pos": value for i, value in enumerate(left)},
            **{f"right_joint{i + 1}.pos": value for i, value in enumerate(right)},
            "left_gripper.pos": float(positions[self.config.left_gripper_joint_name]),
            "right_gripper.pos": float(positions[self.config.right_gripper_joint_name]),
        }
        for name, camera in self.cameras.items():
            try:
                frame = camera.read_latest()
                self._last_camera_frames[name] = frame
            except (RuntimeError, TimeoutError):
                frame = self._last_camera_frames.get(name)
                if frame is None:
                    cfg = self.config.cameras[name]
                    frame = np.zeros((cfg.height, cfg.width, 3), dtype=np.uint8)
            observation[name] = frame
        return observation

    def _safe_arm_command(
        self,
        action: RobotAction,
        side: str,
        signs: list[float],
        offsets: list[float],
        previous: list[float] | None,
    ) -> list[float]:
        targets = [float(action[f"{side}_joint{i}.pos"]) for i in range(1, 8)]
        if self.config.apply_transforms_to_action:
            targets = [
                self._transform(value, signs[i], offsets[i], inverse=True) for i, value in enumerate(targets)
            ]
        if previous is not None and self.config.max_delta_rad > 0:
            delta = self.config.max_delta_rad
            targets = [
                max(old - delta, min(old + delta, new)) for old, new in zip(previous, targets, strict=True)
            ]
        return targets

    @staticmethod
    def _normalized_gripper_command(value: float, closed_m: float, open_m: float) -> float:
        if not math.isfinite(value):
            raise ValueError("Gripper action contains a non-finite value.")
        return max(0.0, min(1.0, (value - closed_m) / (open_m - closed_m)))

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        if not self.config.active_control:
            return dict(action)
        if self._left_command_topic is None or self._right_command_topic is None:
            raise DeviceNotConnectedError(f"{self} command publishers are not connected.")
        import roslibpy

        left = self._safe_arm_command(
            action, "left", self._left_signs, self._left_offsets, self._last_left_command
        )
        right = self._safe_arm_command(
            action, "right", self._right_signs, self._right_offsets, self._last_right_command
        )
        if self.config.command_transport == "exoskeleton_bridge":
            left_gripper = float(action["left_gripper.pos"])
            right_gripper = float(action["right_gripper.pos"])
            left_normalized = self._normalized_gripper_command(
                left_gripper, self.config.left_gripper_closed_m, self.config.gripper_open_m
            )
            right_normalized = self._normalized_gripper_command(
                right_gripper, self.config.right_gripper_closed_m, self.config.gripper_open_m
            )
            self._left_command_topic.publish(
                roslibpy.Message(
                    {
                        "name": [*self.config.left_arm_joint_names, "left_gripper_joint"],
                        "position": [*left, left_normalized],
                        "velocity": [],
                        "effort": [],
                    }
                )
            )
            self._right_command_topic.publish(
                roslibpy.Message(
                    {
                        "name": [*self.config.right_arm_joint_names, "right_gripper_joint"],
                        "position": [*right, right_normalized],
                        "velocity": [],
                        "effort": [],
                    }
                )
            )
        else:
            self._left_command_topic.publish(roslibpy.Message({"data": left}))
            self._right_command_topic.publish(roslibpy.Message({"data": right}))
        self._last_left_command = left
        self._last_right_command = right
        sent = dict(action)
        sent.update({f"left_joint{i + 1}.pos": value for i, value in enumerate(left)})
        sent.update({f"right_joint{i + 1}.pos": value for i, value in enumerate(right)})
        return sent

    def _disconnect_resources(self) -> None:
        for camera in self.cameras.values():
            if camera.is_connected:
                camera.disconnect()
        if self._joint_topic is not None:
            self._joint_topic.unsubscribe()
        for topic in (self._left_command_topic, self._right_command_topic):
            if topic is not None:
                topic.unadvertise()
        if self._ros is not None:
            self._ros.terminate()
        self._joint_topic = None
        self._left_command_topic = None
        self._right_command_topic = None
        self._ros = None
        self._connected = False

    @check_if_not_connected
    def disconnect(self) -> None:
        self._disconnect_resources()
