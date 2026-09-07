# OpenArm ROSBridge integration

This directory contains the ROS2 source packages migrated from the existing
OpenArm/exoskeleton workspace. Build `ros2_ws` with ROS2 Humble's system Python.
Run LeRobot from its separate Python 3.12 environment and connect through the
`rosbridge_server` WebSocket on port 9090.

See `examples/openarm_rosbridge/README.zh-CN.md` for the installation, topic
checks, three-camera configuration, and recording command.
