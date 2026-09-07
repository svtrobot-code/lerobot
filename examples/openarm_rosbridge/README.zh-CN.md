# OpenArm 双臂、外骨骼与三路 RGB 数据采集

此适配针对以下架构：OpenArm/CAN 和外骨骼节点运行在 Ubuntu 22.04 的 ROS2 Humble
环境；LeRobot 0.6.2 运行在独立的 Python 3.12 Conda 环境，通过 ROSBridge 9090
端口读取 ROS2 话题。LeRobot 进程不导入 Humble 的 `rclpy`，因此不会混用 Python
3.10 与 Python 3.12 的二进制扩展。

## 1. 创建 Conda 环境

不要继续使用之前的 Python 3.10 环境。当前源码的 `pyproject.toml` 明确要求
Python `>=3.12`，建议固定为 3.12：

```bash
conda deactivate
conda create -n lerobot-openarm python=3.12 -y
conda activate lerobot-openarm
python --version
python -m pip install --upgrade pip setuptools wheel
cd /你的路径/lerobot
python -m pip install --upgrade torch torchvision \
  --index-url https://download.pytorch.org/whl/cu128
python -m pip install -e ".[core_scripts,training,openarm-rosbridge,pi,peft]"
```

`pip install -e .` 本身可以执行，但只安装基础依赖，缺少录制、视频编码、训练和
ROSBridge 所需的可选依赖，所以这里安装带 extras 的 editable 包。

RTX 5060 机器先用 `nvidia-smi` 确认驱动；当前仓库锁定的 Linux PyTorch 源是 CUDA
12.8，源码注释给出的最低驱动版本是 570.86。安装后用下面命令确认 PyTorch 能识别
显卡。不要在 Conda 环境中安装 ROS2 Humble 的 `rclpy`。

```bash
python -c "import torch; print(torch.__version__, torch.version.cuda); print(torch.cuda.get_device_name(0)); print(torch.cuda.get_device_capability(0))"
```

## 2. 构建并启动 ROS2 硬件栈

仓库内 `integrations/openarm_rosbridge/ros2_ws/src` 保存了迁移过来的 OpenArm ROS2、
CAN、描述文件和外骨骼遥操作源码。ROS2 构建必须在系统 Python/ROS2 终端中进行：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y ros-humble-rosbridge-server python3-colcon-common-extensions
cd /你的路径/lerobot/integrations/openarm_rosbridge/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

构建成功后开 4 个 ROS2 终端；每个终端都执行前两条 `source`。先启动双臂硬件和
forward position controller：

```bash
source /opt/ros/humble/setup.bash
source /你的路径/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  robot_controller:=forward_position_controller \
  right_can_interface:=can0 left_can_interface:=can1 launch_rviz:=false
```

第二个终端启动外骨骼 WebSocket 19091 接收器，第三个终端做重定向，第四个终端把
重定向目标送给 OpenArm 控制器和夹爪：

```bash
ros2 launch qnbot_teleoperator websocket_teleoperator.launch.py \
  websocket_host:=0.0.0.0 websocket_port:=19091 \
  enable_left_arm:=true enable_right_arm:=true enable_vehicle_control:=false

ros2 launch qnbot_teleoperator exo_retargeting.launch.py \
  robot_type:=OpenArm enable_left_arm_retargeting:=true enable_right_arm_retargeting:=true

ros2 launch qnbot_teleoperator exoskeleton_bridge.launch.py
```

再开一个已经 source ROS2 工作区的终端启动 LeRobot 所需的 ROSBridge：

```bash
source /opt/ros/humble/setup.bash
source /你的路径/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

## 3. 采集前检查

```bash
ls -l /dev/video0 /dev/video2 /dev/video4
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video2 --list-formats-ext
v4l2-ctl -d /dev/video4 --list-formats-ext

ros2 topic hz /joint_states
ros2 topic hz /left_forward_position_controller/commands
ros2 topic hz /right_forward_position_controller/commands
ros2 topic echo /left_arm/joint_command --once
ros2 topic echo /right_arm/joint_command --once
```

`/joint_states` 必须包含 14 个手臂关节和两个夹爪关节，默认名称写在
`OpenArmRosbridgeBimanualConfig` 中。实际名称不一致时，应修改 YAML 中的
`left_arm_joint_names`、`right_arm_joint_names` 和两个 gripper joint name。

## 4. 开始录制

OpenArm 示例配置默认启动 Rerun 实时界面，并在每条 episode 开始前等待回车。
等待期间，相机画面、双臂关节状态和遥操作动作曲线会持续刷新，但不会写入数据集。
在录制命令所在的终端按 `Enter` 后才开始当前 episode；录制结束并完成环境复位后，
下一条 episode 会再次等待 `Enter`。单条录制时长由 `dataset.episode_time_s` 控制，
当前示例设置为每条 120 秒、共采集 100 条。

录制终端只激活 Conda 环境，不 source ROS2：

```bash
conda activate lerobot-openarm
cd /你的路径/lerobot
lerobot-record --config_path=examples/openarm_rosbridge/record_openarm_dual_rgb.yaml
```

排查 USB 带宽或相机故障时，可先只启用 `/dev/video0`：

```bash
lerobot-record --config_path=examples/openarm_rosbridge/record_openarm_single_rgb.yaml
```

第一次先保留 `num_episodes: 3`、15 FPS、30 秒，检查输出目录
`data/openarm_rgb_test`。确认三路视频、观测关节和 action 目标都正确后，再修改任务、
集数、时长和数据集目录。若需要预览，可把 YAML 的 `display_data` 改为 `true`。

录制模式下 `robot.active_control` 必须保持 `false`：外骨骼 ROS2 节点已经向控制器发
命令，LeRobot 只记录同一目标，避免重复发布。策略推理时才将它设为 `true`，并把
`command_transport` 设为 `exoskeleton_bridge`。此时 LeRobot 向左右
`/left_arm/joint_command`、`/right_arm/joint_command` 发布 7 轴目标和归一化夹爪目标，
现有 `exoskeleton_bridge_node` 再负责手臂控制器与夹爪 Action。连接成功时会用当前实际
姿态初始化动作限幅，避免第一条模型动作绕过 `max_delta_rad`。

## 5. 训练入口

完成有效数据采集后，可先用 ACT 做基线：

```bash
conda activate lerobot-openarm
cd /你的路径/lerobot
lerobot-train \
  --dataset.repo_id=local/openarm_rgb_test \
  --dataset.root=data/openarm_rgb_test \
  --policy.type=act \
  --output_dir=outputs/train/openarm_act \
  --job_name=openarm_act \
  --policy.device=cuda \
  --wandb.enable=false
```

本地数据集读取参数可能随所选策略变化；执行前先用 `lerobot-train --help` 检查当前
0.6.2 CLI。训练前应人工抽查 episode，错误动作、断流和相机错位的数据不要混入。

## 6. PI0.5 LoRA 训练与真机推理

单张 RTX 5060 只有 8 GB 显存，使用
`examples/openarm_rosbridge/train_pi05_openarm.yaml` 中的 BF16、LoRA rank 4、batch 1、
4 步梯度累积和梯度检查点配置。先把数据集的 `single_task` 改成每条演示真实执行的任务描述；
VLA 会把这段文字作为条件，`Test ... data collection` 不适合作为正式训练指令。

先做 10 步冒烟测试（使用一个新的输出目录）：

```bash
lerobot-train --config_path=examples/openarm_rosbridge/train_pi05_openarm.yaml \
  --steps=10 --eval_steps=0 --save_freq=0 \
  --output_dir=outputs/train/openarm_pi05_smoke
```

确认没有显存溢出、数据字段错误或视频解码错误后启动正式训练：

```bash
lerobot-train --config_path=examples/openarm_rosbridge/train_pi05_openarm.yaml
```

如果之前已用测试文字录完 100 条，并且 100 条确实都是同一个实际任务，可先备份数据，
再一次性改正任务标签。例如：

```bash
cp -a data/openarm_single_rgb_100ep data/openarm_single_rgb_100ep.backup
lerobot-edit-dataset \
  --repo_id local/openarm_single_rgb_100ep \
  --root data/openarm_single_rgb_100ep \
  --operation.type modify_tasks \
  --operation.new_task "Pick up the object with both arms and place it in the target area"
```

推理时保留 OpenArm hardware、`exoskeleton_bridge_node` 和 rosbridge 9090，停止外骨骼
WebSocket 接收器与 `exo_retargeting_node`，避免它们和策略同时发布控制目标。检查控制器
和夹爪 Action Server 后运行：

```bash
ros2 control list_controllers
ros2 action list | grep gripper_cmd
ros2 topic info /left_arm/joint_command -v
ros2 topic info /right_arm/joint_command -v

conda activate lerobot-openarm
cd /你的路径/lerobot
lerobot-rollout --config_path=examples/openarm_rosbridge/rollout_pi05_openarm.yaml
```

程序加载后机械臂保持不动，在交互终端输入 `/start` 才开始执行，输入 `/stop` 立即停止
当前段。第一次测试先断开负载、降低控制器速度/力矩，并让操作员随时能急停。推理 YAML
中的 `task` 必须与训练数据使用的任务描述一致；若输出路径不同，应把 `policy.path` 改为
实际 checkpoint 的 `pretrained_model` 目录。
