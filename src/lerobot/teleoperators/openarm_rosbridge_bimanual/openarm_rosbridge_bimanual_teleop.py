import math
import threading
import time
from functools import cached_property
from typing import Any

from lerobot.lerobot_types import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.import_utils import require_package

from ..teleoperator import Teleoperator
from .config_openarm_rosbridge_bimanual_teleop import OpenArmRosbridgeBimanualTeleopConfig


class OpenArmRosbridgeBimanualTeleop(Teleoperator):
    """Expose OpenArm exoskeleton ROS2 target topics as LeRobot actions."""

    config_class = OpenArmRosbridgeBimanualTeleopConfig
    name = "openarm_rosbridge_bimanual_teleop"

    def __init__(self, config: OpenArmRosbridgeBimanualTeleopConfig):
        super().__init__(config)
        self.config = config
        self._connected = False
        self._ros: Any = None
        self._topics: list[Any] = []
        self._lock = threading.Lock()
        self._values: dict[str, list[float] | float] = {}
        self._times: dict[str, float] = {}

    @cached_property
    def action_features(self) -> dict[str, type]:
        features = {f"left_joint{i}.pos": float for i in range(1, 8)}
        features.update({f"right_joint{i}.pos": float for i in range(1, 8)})
        features["left_gripper.pos"] = float
        features["right_gripper.pos"] = float
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected and bool(self._ros and self._ros.is_connected)

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

        def arm_callback(side: str):
            def callback(message: dict[str, Any]) -> None:
                data = message.get("data", [])
                if not isinstance(data, list) or len(data) < 7:
                    return
                values = [float(value) for value in data[:7]]
                if not all(math.isfinite(value) for value in values):
                    return
                with self._lock:
                    self._values[side] = values
                    self._times[side] = time.monotonic()

            return callback

        def gripper_callback(side: str):
            def callback(message: dict[str, Any]) -> None:
                positions = message.get("position", [])
                if not isinstance(positions, list) or len(positions) < 8:
                    return
                value = float(positions[7])
                if not math.isfinite(value):
                    return
                key = f"{side}_gripper"
                with self._lock:
                    self._values[key] = value
                    self._times[key] = time.monotonic()

            return callback

        specs = (
            (self.config.left_arm_command_topic, "std_msgs/msg/Float64MultiArray", arm_callback("left")),
            (self.config.right_arm_command_topic, "std_msgs/msg/Float64MultiArray", arm_callback("right")),
            (self.config.left_joint_command_topic, "sensor_msgs/msg/JointState", gripper_callback("left")),
            (self.config.right_joint_command_topic, "sensor_msgs/msg/JointState", gripper_callback("right")),
        )
        try:
            for name, message_type, callback in specs:
                topic = roslibpy.Topic(self._ros, name, message_type)
                topic.subscribe(callback)
                self._topics.append(topic)
            self._connected = True
            self._wait_for_commands()
        except Exception:
            self._disconnect_resources()
            raise

    def _wait_for_commands(self) -> None:
        required = {"left", "right", "left_gripper", "right_gripper"}
        deadline = time.monotonic() + self.config.wait_for_commands_s
        while time.monotonic() < deadline:
            with self._lock:
                if required <= self._times.keys():
                    return
            time.sleep(0.05)
        with self._lock:
            missing = required - self._times.keys()
        raise TimeoutError(f"Exoskeleton command topics are missing: {sorted(missing)}")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        required = {"left", "right", "left_gripper", "right_gripper"}
        now = time.monotonic()
        with self._lock:
            stale = [
                key
                for key in required
                if key not in self._times or now - self._times[key] > self.config.command_timeout_s
            ]
            if stale:
                raise TimeoutError(f"Exoskeleton command streams are stale: {sorted(stale)}")
            left = list(self._values["left"])
            right = list(self._values["right"])
            left_gripper = float(self._values["left_gripper"])
            right_gripper = float(self._values["right_gripper"])

        if self.config.gripper_output_mode == "bridge_binary_meters":
            left_gripper = (
                self.config.left_gripper_closed_m if left_gripper < 0.5 else self.config.gripper_open_m
            )
            right_gripper = (
                self.config.right_gripper_closed_m if right_gripper < 0.5 else self.config.gripper_open_m
            )
        return {
            **{f"left_joint{i + 1}.pos": value for i, value in enumerate(left)},
            **{f"right_joint{i + 1}.pos": value for i, value in enumerate(right)},
            "left_gripper.pos": left_gripper,
            "right_gripper.pos": right_gripper,
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback

    def _disconnect_resources(self) -> None:
        for topic in self._topics:
            topic.unsubscribe()
        self._topics.clear()
        if self._ros is not None:
            self._ros.terminate()
        self._ros = None
        self._connected = False

    @check_if_not_connected
    def disconnect(self) -> None:
        self._disconnect_resources()
