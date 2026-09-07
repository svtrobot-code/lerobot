# Vinci OpenArm 双臂 VLA 完整操作手册

本文档适用于下面这套设备和源码：

- Ubuntu 22.04 x86_64
- ROS2 Humble
- NVIDIA GeForce RTX 5060，8 GB 显存
- 32 GB 系统内存
- OpenArm 双臂
- 一套外骨骼遥操作设备
- 三个普通 RGB USB 相机：`/dev/video0`、`/dev/video2`、`/dev/video4`
- LeRobot 0.6.2 源码目录：`~/Downloads/lerobot`
- Python 3.12 Conda 环境
- π0.5 基础模型：`lerobot/pi05_base`

本文所有命令都直接在终端中逐条执行，不使用自定义 `.sh` 脚本。

## 1. 整体架构

系统分成两个相互隔离的 Python 环境：

1. ROS2 Humble 使用 Ubuntu 系统 Python 3.10，负责 OpenArm 硬件、CAN、外骨骼接收、
   动作重定向、控制器和夹爪。
2. LeRobot 使用 Conda Python 3.12，负责相机、数据集、π0.5 训练和模型推理。
3. 两套环境通过 `rosbridge_server` 的 WebSocket 9090 端口通信。

不要在激活 Conda 环境后执行 ROS2 节点，也不要在 LeRobot Conda 环境里安装
ROS2 Humble 的 `rclpy`。ROS2 和 Python 3.12 的二进制模块版本不兼容。

数据采集时的控制链路：

```text
外骨骼
  -> WebSocket 19091 接收节点
  -> exo_retargeting_node
  -> /left_arm/joint_command 和 /right_arm/joint_command
  -> exoskeleton_bridge_node
  -> OpenArm 手臂控制器和夹爪 Action Server

LeRobot
  <- rosbridge 9090
  <- /joint_states、外骨骼动作话题和三个 RGB 相机
  -> 写入 LeRobotDataset
```

π0.5 自主推理时的控制链路：

```text
RGB 相机 + /joint_states
  -> LeRobot π0.5
  -> /left_arm/joint_command 和 /right_arm/joint_command
  -> exoskeleton_bridge_node
  -> OpenArm 手臂控制器和夹爪 Action Server
```

自主推理时必须停止外骨骼 WebSocket 接收节点和 `exo_retargeting_node`，避免两个程序
同时发布控制命令。

## 2. 把源码复制到 Ubuntu 机器

应将当前修改后的整个 `E:\downloads\lerobot` 目录同步到 Ubuntu：

```text
~/Downloads/lerobot
```

进入源码目录并检查关键文件：

```bash
cd ~/Downloads/lerobot
pwd
ls examples/openarm_rosbridge
ls integrations/openarm_rosbridge/ros2_ws/src
```

应该至少能看到：

```text
examples/openarm_rosbridge/test_fff.yaml
examples/openarm_rosbridge/record_openarm_single_rgb.yaml
examples/openarm_rosbridge/record_openarm_dual_rgb.yaml
examples/openarm_rosbridge/train_pi05_openarm.yaml
examples/openarm_rosbridge/rollout_pi05_openarm.yaml
integrations/openarm_rosbridge/ros2_ws/src/openarm_ros2
integrations/openarm_rosbridge/ros2_ws/src/qnbot_teleoperator
```

## 3. 安装系统软件

更新软件索引：

```bash
sudo apt update
```

安装构建、视频、USB 相机和 CAN 调试工具：

```bash
sudo apt install -y git git-lfs build-essential cmake pkg-config
sudo apt install -y ffmpeg v4l-utils can-utils
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo apt install -y ros-humble-rosbridge-server
```

检查 ROS2 Humble：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

如果系统提示找不到 `/opt/ros/humble/setup.bash`，需要先按照 ROS2 官方方法安装
ROS2 Humble Desktop，然后再继续。

首次使用 `rosdep` 时执行：

```bash
sudo rosdep init
rosdep update
```

如果 `sudo rosdep init` 提示已经初始化，可以忽略该提示，继续执行 `rosdep update`。

## 4. 安装 Conda

如果机器已经可以运行 `conda --version`，跳到下一节。

下载 Miniconda：

```bash
cd ~/Downloads
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
```

安装：

```bash
bash Miniconda3-latest-Linux-x86_64.sh
```

按安装程序提示接受许可证并初始化 Conda。安装结束后关闭并重新打开终端，然后检查：

```bash
conda --version
```

如果新终端仍找不到 Conda，可以执行：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda init bash
```

然后重新打开终端。

## 5. 创建 LeRobot Python 3.12 环境

当前 LeRobot 0.6.2 的 `pyproject.toml` 要求 Python `>=3.12`。不要继续使用之前创建的
Python 3.10 环境 `test_aaa`。

创建新环境：

```bash
conda create -n lerobot-pi05 python=3.12 -y
```

激活环境：

```bash
conda activate lerobot-pi05
```

检查 Python：

```bash
python --version
which python
```

预期 Python 为 3.12，路径位于 `envs/lerobot-pi05`。

升级 Python 安装工具：

```bash
python -m pip install --upgrade pip setuptools wheel
```

## 6. 安装 RTX 5060 对应的 PyTorch 和 LeRobot

先检查显卡和驱动：

```bash
nvidia-smi
```

当前仓库为 Linux 配置了 CUDA 12.8 PyTorch wheel，源码注释注明最低 NVIDIA 驱动版本
为 570.86。安装 PyTorch：

```bash
python -m pip install --upgrade torch torchvision \
  --index-url https://download.pytorch.org/whl/cu128
```

进入源码目录：

```bash
cd ~/Downloads/lerobot
```

使用 editable 模式安装完整依赖：

```bash
python -m pip install -e ".[core_scripts,training,openarm-rosbridge,pi,peft]"
```

这里必须带上可选依赖：

- `core_scripts`：录制、回放、相机、数据集和 Rerun 界面。
- `training`：训练器和训练依赖。
- `openarm-rosbridge`：`roslibpy`，连接 ROSBridge。
- `pi`：π0 和 π0.5 依赖。
- `peft`：LoRA 微调和 LoRA checkpoint 加载。

单独执行 `pip install -e .` 虽然可以安装 LeRobot 主包，但不包含本流程需要的全部可选依赖。

检查安装：

```bash
lerobot-info
```

检查 PyTorch 是否真正识别 RTX 5060：

```bash
python -c "import torch; print('torch=', torch.__version__); print('cuda=', torch.version.cuda); print('available=', torch.cuda.is_available()); print('gpu=', torch.cuda.get_device_name(0)); print('capability=', torch.cuda.get_device_capability(0))"
```

`available` 必须是 `True`，`gpu` 应显示 RTX 5060。

## 7. 构建 OpenArm ROS2 工作区

先退出 Conda，回到系统 Python：

```bash
conda deactivate
```

加载 ROS2 Humble：

```bash
source /opt/ros/humble/setup.bash
```

进入 ROS2 工作区：

```bash
cd ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws
```

安装 ROS2 包依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

构建：

```bash
colcon build --symlink-install
```

加载构建结果：

```bash
source install/setup.bash
```

确认包已经被 ROS2 找到：

```bash
ros2 pkg prefix openarm_bringup
ros2 pkg prefix qnbot_teleoperator
```

以后每次修改 `integrations/openarm_rosbridge/ros2_ws/src` 内的 ROS2 源码，都要重新执行：

```bash
cd ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

LeRobot Python 源码使用了 editable 安装，修改 `src/lerobot` 后通常不需要重新安装；如果
修改了 `pyproject.toml` 或依赖，重新执行 `python -m pip install -e ...`。

## 8. 检查 CAN 和相机设备

检查 CAN 接口：

```bash
ip -details link show can0
ip -details link show can1
```

如果项目原来的 OpenArm 安装流程已经配置好 CAN，接口应处于 `UP`。不要在不确定电机
CAN/CAN-FD 参数时随意改波特率。

检查三个相机设备：

```bash
ls -l /dev/video0 /dev/video2 /dev/video4
v4l2-ctl --list-devices
```

分别检查支持的分辨率、帧率和编码：

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video2 --list-formats-ext
v4l2-ctl -d /dev/video4 --list-formats-ext
```

配置文件使用 640×480、30 FPS、MJPG。如果某个相机不支持这些参数，应把对应 YAML
的 `width`、`height`、`fps` 或 `fourcc` 改成相机实际支持的组合。

USB 相机编号重启后可能变化。正式采集前必须重新运行 `v4l2-ctl --list-devices`，确认
前视、左腕和右腕相机没有互换。条件允许时，后续应改用稳定的 `/dev/v4l/by-id/...` 路径。

## 9. 数据采集需要启动的 ROS2 节点

数据采集需要五个终端。所有 ROS2 终端都不要激活 Conda。

### 9.1 终端一：启动 OpenArm 双臂硬件和控制器

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
```

启动双臂：

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  robot_controller:=forward_position_controller \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  launch_rviz:=false
```

### 9.2 终端二：启动外骨骼 WebSocket 19091 接收器

```bash
source /opt/ros/humble/setup.bash
source ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
```

```bash
ros2 launch qnbot_teleoperator websocket_teleoperator.launch.py \
  websocket_host:=0.0.0.0 \
  websocket_port:=19091 \
  enable_left_arm:=true \
  enable_right_arm:=true \
  enable_vehicle_control:=false
```

### 9.3 终端三：启动外骨骼到 OpenArm 的动作重定向

```bash
source /opt/ros/humble/setup.bash
source ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
```

```bash
ros2 launch qnbot_teleoperator exo_retargeting.launch.py \
  robot_type:=OpenArm \
  enable_left_arm_retargeting:=true \
  enable_right_arm_retargeting:=true
```

### 9.4 终端四：启动手臂和夹爪控制桥

```bash
source /opt/ros/humble/setup.bash
source ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
```

```bash
ros2 launch qnbot_teleoperator exoskeleton_bridge.launch.py
```

### 9.5 终端五：启动 ROSBridge 9090

```bash
source /opt/ros/humble/setup.bash
source ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
```

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

## 10. 数据采集前检查

新开一个 ROS2 检查终端：

```bash
source /opt/ros/humble/setup.bash
source ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
```

列出话题：

```bash
ros2 topic list
```

检查关节状态频率：

```bash
ros2 topic hz /joint_states
```

检查 `/joint_states` 内容：

```bash
ros2 topic echo /joint_states --once
```

必须包含左右各 7 个手臂关节以及两个夹爪关节。默认名称是：

```text
openarm_left_joint1 ... openarm_left_joint7
openarm_right_joint1 ... openarm_right_joint7
openarm_left_finger_joint1
openarm_right_finger_joint1
```

检查外骨骼重定向输出：

```bash
ros2 topic echo /left_arm/joint_command --once
ros2 topic echo /right_arm/joint_command --once
```

检查控制器输出频率：

```bash
ros2 topic hz /left_forward_position_controller/commands
ros2 topic hz /right_forward_position_controller/commands
```

检查夹爪 Action Server：

```bash
ros2 action list | grep gripper_cmd
```

预期包含左右夹爪：

```text
/left_gripper_controller/gripper_cmd
/right_gripper_controller/gripper_cmd
```

检查 ROSBridge 端口：

```bash
ss -lntp | grep 9090
```

在所有检查正常后，先手动遥操双臂做小幅度动作，确认左右臂、关节方向和夹爪开合正确。

## 11. 理解数据采集配置

单相机配置：

```text
examples/openarm_rosbridge/test_fff.yaml
examples/openarm_rosbridge/record_openarm_single_rgb.yaml
```

三相机配置：

```text
examples/openarm_rosbridge/record_openarm_dual_rgb.yaml
```

虽然文件名包含 `dual_rgb`，配置内实际包含三路画面：

```text
front       -> /dev/video0
wrist_left  -> /dev/video2
wrist_right -> /dev/video4
```

关键采集参数：

```yaml
dataset:
  fps: 15
  num_episodes: 100
  episode_time_s: 120
  reset_time_s: 30
  video: true
  push_to_hub: false
  no_stamp: true

display_data: true
display_mode: rerun
wait_for_enter: true
```

含义：

- 总共录制 100 条 episode。
- 每条正式数据 120 秒。
- 两条之间有 30 秒环境复位时间，复位阶段不写入数据。
- 数据集控制频率为 15 FPS。
- Rerun 实时显示相机、机器人状态和动作曲线。
- 每条 episode 都等待终端回车，不会启动后自动录制。
- `robot.active_control` 在采集配置中必须保持 `false`，因为 ROS2 外骨骼链路已经控制机械臂。

每帧主要包含：

- 一路或三路 RGB 图像。
- 左臂 7 个关节位置。
- 右臂 7 个关节位置。
- 左右夹爪位置。
- 外骨骼给出的左右臂目标动作和夹爪目标动作，共 16 维 action。
- 时间戳、frame index、episode index、task index 和任务文字。

## 12. 修改正式任务描述和保存目录

π0.5 是视觉语言动作模型。`single_task` 必须描述演示实际执行的任务，不能继续使用：

```text
Test OpenArm bimanual data collection with one RGB camera
```

例如实际任务是双臂拿起物体并放入托盘，可以设置：

```yaml
single_task: Pick up the object with both arms and place it in the tray
```

建议使用清晰、一致、可从视频中判断完成情况的英文指令。同一个数据集如果包含多个不同
任务，应给不同 episode 设置相应任务标签，不能全部写成同一句话。

正式单相机数据集可使用：

```yaml
repo_id: local/openarm_single_rgb_100ep
root: data/openarm_single_rgb_100ep
```

开始前确认目标目录不存在：

```bash
cd ~/Downloads/lerobot
ls data/openarm_single_rgb_100ep
```

如果提示不存在，可以开始新数据集。如果目录已存在，应选择：

- 使用 `--resume=true` 追加数据；或者
- 修改 YAML 使用新的 `repo_id` 和 `root`。

不要在未备份时覆盖已有正式数据集。

## 13. 先做单相机短测试

编辑 `examples/openarm_rosbridge/test_fff.yaml`，第一次测试建议临时设置：

```yaml
dataset:
  num_episodes: 3
  episode_time_s: 30
  reset_time_s: 15
```

开启 LeRobot 终端。这个终端只激活 Conda，不加载 ROS2：

```bash
conda activate lerobot-pi05
cd ~/Downloads/lerobot
```

启动录制：

```bash
lerobot-record --config_path=examples/openarm_rosbridge/test_fff.yaml
```

程序启动后的行为：

1. 连接 rosbridge 9090。
2. 等待 `/joint_states` 和外骨骼动作话题。
3. 打开 `/dev/video0`。
4. 弹出 Rerun 实时界面。
5. 输出 `Ready for episode 0 ... press ENTER to start`。
6. 此时只预览，不保存帧。
7. 焦点切回运行 `lerobot-record` 的终端并按 `Enter`，才开始正式录制。

录制中的键盘控制：

- `→`：提前结束当前 episode，并保存已经录制的部分。
- `←`：放弃当前 episode，并重新录制这一条。
- `Esc`：停止整个录制任务。

每条录制完成后进入复位阶段。复位阶段用外骨骼把环境和机械臂恢复到下一条演示所需的
起始状态；复位阶段不写入数据。下一条仍然需要按 `Enter`。

## 14. 正式录制 100 条单相机数据

当前 π0.5 低显存配置推荐先使用单相机数据。确认 `test_fff.yaml` 中参数为：

```yaml
dataset:
  repo_id: local/openarm_single_rgb_100ep
  root: data/openarm_single_rgb_100ep
  single_task: Pick up the object with both arms and place it in the tray
  fps: 15
  num_episodes: 100
  episode_time_s: 120
  reset_time_s: 30
```

启动：

```bash
conda activate lerobot-pi05
cd ~/Downloads/lerobot
lerobot-record --config_path=examples/openarm_rosbridge/test_fff.yaml
```

如果中途正常停止，之后只需要追加 20 条，执行：

```bash
lerobot-record \
  --config_path=examples/openarm_rosbridge/test_fff.yaml \
  --resume=true \
  --dataset.num_episodes=20
```

恢复录制时 `--dataset.num_episodes` 表示本次新增条数，不是数据集最终总数。

## 15. 切换到三个相机采集

确认三个相机同时工作后运行：

```bash
conda activate lerobot-pi05
cd ~/Downloads/lerobot
lerobot-record --config_path=examples/openarm_rosbridge/record_openarm_dual_rgb.yaml
```

三个相机如果出现打开失败、画面卡住或超时，按顺序处理：

1. 单独测试每个 `/dev/videoX`。
2. 保持 `fourcc: MJPG`，避免未压缩 YUYV 占用大量 USB 带宽。
3. 把相机接到不同 USB 控制器，而不只是不同插口。
4. 将相机采集 FPS 从 30 降到 15。
5. 必要时把分辨率从 640×480 降低。
6. 先回到单相机完成整条数据链路验证。

训练和推理必须使用一致的相机特征。若使用三相机数据训练，推理 YAML 也必须提供
`front`、`wrist_left`、`wrist_right` 三个同名相机；当前
`rollout_pi05_openarm.yaml` 是单相机版本。

## 16. 检查录制结果

单相机数据目录应类似：

```text
data/openarm_single_rgb_100ep/
├── data/
├── meta/
└── videos/
```

这是正常的 LeRobotDataset 目录结构：

- `data/`：状态、动作、索引和时间戳等表格数据。
- `meta/`：数据特征、episode、任务、统计量和数据集信息。
- `videos/`：每个相机的压缩视频。

检查磁盘占用和文件数量：

```bash
cd ~/Downloads/lerobot
du -sh data/openarm_single_rgb_100ep
find data/openarm_single_rgb_100ep/data -type f | wc -l
find data/openarm_single_rgb_100ep/videos -type f | wc -l
find data/openarm_single_rgb_100ep/meta -maxdepth 2 -type f -print
```

用 Rerun 检查第 0 条：

```bash
lerobot-dataset-viz \
  --repo-id local/openarm_single_rgb_100ep \
  --root data/openarm_single_rgb_100ep \
  --mode local \
  --episode-index 0
```

还应抽查中间和最后几条，例如：

```bash
lerobot-dataset-viz \
  --repo-id local/openarm_single_rgb_100ep \
  --root data/openarm_single_rgb_100ep \
  --mode local \
  --episode-index 50
```

检查重点：

- 图像是否流畅、方向正确、没有黑帧和长时间重复帧。
- 图像动作与机械臂状态时间上是否同步。
- 16 维状态和 16 维动作曲线是否连续。
- 左右臂有没有交换。
- 夹爪开合数据是否正确变化。
- 任务描述是否对应实际演示。
- 每条演示是否完整成功，没有碰撞和错误操作。

错误、失败、断流或明显不同步的 episode 不应进入正式 π0.5 训练。

## 17. 修正已经录制的数据集任务文字

如果所有 episode 都是同一个实际任务，但采集时误用了测试文字，先备份：

```bash
cd ~/Downloads/lerobot
cp -a data/openarm_single_rgb_100ep data/openarm_single_rgb_100ep.backup
```

再原地修改任务：

```bash
lerobot-edit-dataset \
  --repo_id local/openarm_single_rgb_100ep \
  --root data/openarm_single_rgb_100ep \
  --operation.type modify_tasks \
  --operation.new_task "Pick up the object with both arms and place it in the tray"
```

这个操作会修改 `meta/tasks.parquet`、数据文件中的 `task_index` 和 episode 元数据。

## 18. π0.5 训练配置说明

训练配置文件：

```text
examples/openarm_rosbridge/train_pi05_openarm.yaml
```

当前关键参数：

```yaml
dataset:
  repo_id: local/openarm_single_rgb_100ep
  root: data/openarm_single_rgb_100ep
  return_uint8: true
  eval_split: 0.05

policy:
  path: lerobot/pi05_base
  device: cuda
  dtype: bfloat16
  use_amp: true
  gradient_checkpointing: true
  chunk_size: 30
  n_action_steps: 10
  num_inference_steps: 10
  tokenizer_max_length: 64

batch_size: 1
steps: 30000

accelerator:
  mixed_precision: bf16
  gradient_accumulation:
    steps: 4

peft:
  method_type: LORA
  r: 4
  lora_alpha: 8

ema:
  enable: false
```

选择这些参数是因为 RTX 5060 只有 8 GB 显存。完整参数微调通常无法放入 8 GB；LoRA
只训练少量适配参数，同时使用 BF16、batch 1 和梯度检查点降低显存占用。32 GB 系统
内存不能代替 GPU 显存。

π0.5 会自动将当前 16 维状态和 16 维动作填充到模型支持的最大 32 维。单相机 RGB
画面会由策略预处理器转换成模型需要的 224×224 输入，状态和动作使用数据集统计量做
quantile normalization。

## 19. 下载模型前的 Hugging Face 准备

`lerobot/pi05_base` 从 Hugging Face Hub 下载。先确认 Ubuntu 机器能访问 Hugging Face。
如果下载时遇到鉴权提示，执行：

```bash
hf auth login
```

然后粘贴自己的 Hugging Face access token。模型和数据缓存通常位于用户缓存目录，首次
加载需要下载数 GB 文件，应提前检查系统盘空间：

```bash
df -h
du -sh ~/.cache/huggingface 2>/dev/null
```

## 20. 先运行 10 步 π0.5 训练冒烟测试

激活环境：

```bash
conda activate lerobot-pi05
cd ~/Downloads/lerobot
```

降低 CUDA 内存碎片风险：

```bash
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
```

启动 10 步测试，并使用独立输出目录：

```bash
lerobot-train \
  --config_path=examples/openarm_rosbridge/train_pi05_openarm.yaml \
  --steps=10 \
  --eval_steps=0 \
  --save_freq=0 \
  --output_dir=outputs/train/openarm_pi05_smoke
```

另开终端监控显卡：

```bash
watch -n 1 nvidia-smi
```

冒烟测试必须确认：

- 成功下载并加载 `lerobot/pi05_base`。
- 数据集被识别为正确的一路 RGB、16 维 state 和 16 维 action。
- 视频能够正常解码。
- 日志开始输出 loss。
- loss 不是 `nan` 或 `inf`。
- 没有 `CUDA out of memory`。
- 能生成最终测试 checkpoint。

每次重新运行冒烟测试都要换一个不存在的输出目录，例如：

```bash
lerobot-train \
  --config_path=examples/openarm_rosbridge/train_pi05_openarm.yaml \
  --steps=10 \
  --eval_steps=0 \
  --save_freq=0 \
  --output_dir=outputs/train/openarm_pi05_smoke_02
```

## 21. 正式训练 π0.5

确认配置中的数据集路径正确：

```bash
ls data/openarm_single_rgb_100ep/meta
ls data/openarm_single_rgb_100ep/data
ls data/openarm_single_rgb_100ep/videos
```

开始正式训练：

```bash
conda activate lerobot-pi05
cd ~/Downloads/lerobot
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
lerobot-train --config_path=examples/openarm_rosbridge/train_pi05_openarm.yaml
```

输出目录：

```text
outputs/train/openarm_pi05_lora
```

每 5000 步保存一次 checkpoint，最终模型路径通常是：

```text
outputs/train/openarm_pi05_lora/checkpoints/last/pretrained_model
```

检查 checkpoint：

```bash
find outputs/train/openarm_pi05_lora/checkpoints -maxdepth 3 -type f | head -50
ls -la outputs/train/openarm_pi05_lora/checkpoints/last
ls outputs/train/openarm_pi05_lora/checkpoints/last/pretrained_model
```

## 22. 训练中断后继续训练

使用最后一个 checkpoint 的训练配置和状态恢复：

```bash
conda activate lerobot-pi05
cd ~/Downloads/lerobot
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
```

```bash
lerobot-train \
  --config_path=outputs/train/openarm_pi05_lora/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

恢复时会读取模型、LoRA adapter、optimizer、scheduler、训练步数和随机数状态。不要把
普通的新训练 YAML 与 `--resume=true` 混用；恢复应指向 checkpoint 中的
`train_config.json` 或 `pretrained_model` 目录。

## 23. RTX 5060 显存不足时的处理

如果出现 `CUDA out of memory`，先确认没有其他进程占用显存：

```bash
nvidia-smi
```

结束不需要的 GPU 图形、训练或推理进程，然后使用更小配置重新开始一个训练目录：

```bash
conda activate lerobot-pi05
cd ~/Downloads/lerobot
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
```

```bash
lerobot-train \
  --config_path=examples/openarm_rosbridge/train_pi05_openarm.yaml \
  --policy.chunk_size=20 \
  --policy.n_action_steps=5 \
  --policy.tokenizer_max_length=48 \
  --output_dir=outputs/train/openarm_pi05_lora_small
```

不要通过增大系统 swap 来期待解决 GPU OOM。若上述 LoRA 配置仍然无法稳定训练，应在
16 GB、24 GB 或更大显存的 GPU 上训练，再把 checkpoint 复制回 RTX 5060 机器推理。

## 24. π0.5 推理前关闭遥操作节点

自主推理时停止下面两个采集阶段使用的进程：

```text
websocket_teleoperator.launch.py
exo_retargeting.launch.py
```

可以在对应终端按 `Ctrl+C` 停止。

推理时保留并重新确认下面三个部分：

1. `openarm.bimanual.launch.py` 双臂硬件与控制器。
2. `exoskeleton_bridge.launch.py` 手臂和夹爪执行桥。
3. `rosbridge_websocket_launch.xml` 9090 服务。

为避免桥节点保留采集结束时的状态，建议推理前也重新启动
`exoskeleton_bridge.launch.py`。

## 25. π0.5 真机推理所需 ROS2 终端

### 25.1 推理终端一：OpenArm 硬件

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
```

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  robot_controller:=forward_position_controller \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  launch_rviz:=false
```

### 25.2 推理终端二：执行桥

```bash
source /opt/ros/humble/setup.bash
source ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
ros2 launch qnbot_teleoperator exoskeleton_bridge.launch.py
```

### 25.3 推理终端三：ROSBridge

```bash
source /opt/ros/humble/setup.bash
source ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

### 25.4 推理前检查终端

```bash
source /opt/ros/humble/setup.bash
source ~/Downloads/lerobot/integrations/openarm_rosbridge/ros2_ws/install/setup.bash
```

```bash
ros2 control list_controllers
ros2 action list | grep gripper_cmd
ros2 topic hz /joint_states
ros2 topic info /left_arm/joint_command -v
ros2 topic info /right_arm/joint_command -v
```

在启动 LeRobot 之前，`/left_arm/joint_command` 和 `/right_arm/joint_command` 不应还有
`exo_retargeting_node` 发布者。

## 26. 检查并修改 π0.5 推理配置

推理文件：

```text
examples/openarm_rosbridge/rollout_pi05_openarm.yaml
```

检查模型路径：

```yaml
policy:
  path: outputs/train/openarm_pi05_lora/checkpoints/last/pretrained_model
```

检查任务文字。推理文字应与训练数据采用同一种表达：

```yaml
task: Pick up the object with both arms and place it in the tray
```

检查安全相关配置：

```yaml
robot:
  active_control: true
  command_transport: exoskeleton_bridge
  max_delta_rad: 0.05

interactive: true
return_to_initial_position: false
```

含义：

- 模型动作通过现有 ROS2 bridge 同时控制双臂和两个夹爪。
- 每个 15 Hz 控制周期的关节目标变化限制为 0.05 rad。
- 连接时用机械臂当前实际姿态初始化限幅，避免第一条策略动作突然跳变。
- 启动程序后不自动运动。
- 退出时不自动把机械臂拉回启动姿态。

RTC 配置用于异步生成动作块：

```yaml
inference:
  type: rtc
  queue_threshold: 20
  rtc:
    mode: guided
    execution_horizon: 10
    max_guidance_weight: 10.0
```

## 27. 启动 π0.5 真机推理

新开 LeRobot 终端。这个终端只激活 Conda，不加载 ROS2：

```bash
conda activate lerobot-pi05
cd ~/Downloads/lerobot
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
```

启动：

```bash
lerobot-rollout --config_path=examples/openarm_rosbridge/rollout_pi05_openarm.yaml
```

程序会加载模型、相机和 `/joint_states`，并打开 Rerun。因为设置了
`interactive: true`，此时机械臂保持不动。

确认画面、机械臂状态和任务文字正确后，在运行 `lerobot-rollout` 的终端输入：

```text
/start
```

然后按回车，才会开始模型控制。

停止当前执行：

```text
/stop
```

结束程序可在停止后按 `Ctrl+C`。

首次真机推理建议：

- 不安装易碎或尖锐工具。
- 工作空间内不放人手和无关物体。
- 先降低硬件控制器速度、力矩或电流限制。
- 从远离奇异位形和关节极限的姿态开始。
- 安排一名操作员只负责观察和急停。
- 先运行几秒，检查双臂方向和夹爪逻辑，再逐渐延长时间。

## 28. 推理速度不足时

RTX 5060 运行 π0.5 可能不能持续达到 15 Hz 模型推理频率。RTC 会在后台生成动作块，
但不能消除显卡本身的计算限制。

先观察 GPU：

```bash
watch -n 1 nvidia-smi
```

可以先减少流匹配推理步数测试延迟：

```bash
lerobot-rollout \
  --config_path=examples/openarm_rosbridge/rollout_pi05_openarm.yaml \
  --policy.num_inference_steps=5
```

减少推理步数会提高速度，但可能降低动作质量。应先离线或空载测试，再决定是否用于正式
任务。不要直接修改 `fps` 来掩盖训练数据和执行时间尺度不一致的问题；当前数据集和 rollout
都使用 15 FPS。

## 29. 单相机和三相机模型不能直接混用

单相机训练得到的模型期望：

```text
observation.images.front
```

三相机训练得到的模型期望：

```text
observation.images.front
observation.images.wrist_left
observation.images.wrist_right
```

训练、验证和真机推理的相机键名与数量必须一致。若后续决定使用三相机：

1. 使用三相机配置重新采集正式数据。
2. 把 `train_pi05_openarm.yaml` 的 dataset 路径改成三相机数据集。
3. 给 `rollout_pi05_openarm.yaml` 增加同名的两个腕部相机。
4. 重新训练模型。

8 GB 显存下，三路图像会增加 π0.5 激活显存和计算量，可能只能把训练放到更大显存的
机器上完成。

## 30. 常见报错检查顺序

### 30.1 无法连接 `ws://127.0.0.1:9090`

```bash
ss -lntp | grep 9090
ros2 node list | grep rosbridge
```

确认 rosbridge 节点仍在运行，并且 LeRobot 与 ROS2 位于同一台机器。如果不在同一台机器，
YAML 的 `rosbridge_host` 应改成 ROS2 机器 IP，并检查网络和防火墙。

### 30.2 等待 `/joint_states` 超时

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

确认关节名称与 YAML 默认名称完全一致。名称不一致时修改机器人配置中的：

```text
left_arm_joint_names
right_arm_joint_names
left_gripper_joint_name
right_gripper_joint_name
```

### 30.3 等待外骨骼 action 超时

```bash
ros2 topic echo /left_arm/joint_command --once
ros2 topic echo /right_arm/joint_command --once
```

确认外骨骼 WebSocket 19091 客户端已连接，且 `exo_retargeting_node` 正常运行。

### 30.4 相机打开失败或卡顿

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
fuser /dev/video0
```

检查设备编号是否变化，以及是否被其他程序占用。三个相机同时出错时，优先按 USB 带宽
问题排查。

### 30.5 Rerun 界面出现但没有开始录制

这是 `wait_for_enter: true` 的预期行为。把输入焦点切回启动 `lerobot-record` 的终端，
按 `Enter` 后才会开始当前 episode。

### 30.6 输出目录已经存在

新训练默认不覆盖已有目录。为新实验指定新的目录：

```bash
lerobot-train \
  --config_path=examples/openarm_rosbridge/train_pi05_openarm.yaml \
  --output_dir=outputs/train/openarm_pi05_lora_02
```

已有训练需要继续时使用 `--resume=true` 和 checkpoint 的 `train_config.json`。

### 30.7 π0.5 模型下载失败

```bash
hf auth login
```

检查网络、Hugging Face token、磁盘空间和 `~/.cache/huggingface` 权限。

### 30.8 训练出现 CUDA OOM

```bash
nvidia-smi
```

确认使用的是 LoRA 配置、`batch_size: 1`、BF16、梯度检查点，并按第 23 节进一步减少
`chunk_size` 和 `tokenizer_max_length`。若仍 OOM，需要更大显存训练设备。

## 31. 推荐的完整执行顺序

第一次部署按下面顺序执行：

1. 同步最新 LeRobot/OpenArm 适配源码到 Ubuntu。
2. 安装系统软件和 ROS2 依赖。
3. 创建 `lerobot-pi05` Python 3.12 Conda 环境。
4. 安装 CUDA 12.8 PyTorch。
5. 执行带 extras 的 `pip install -e`。
6. 构建 ROS2 工作区。
7. 检查 CAN、三个相机和夹爪 Action Server。
8. 启动五个数据采集 ROS2/LeRobot 终端。
9. 用单相机、3 条、每条 30 秒做测试采集。
10. 用 Rerun 检查图像、状态、动作和时间同步。
11. 设置真实任务文字，录制 100 条正式数据。
12. 抽查数据并移除失败演示。
13. 运行 10 步 π0.5 LoRA 冒烟训练。
14. 运行正式 30000 步训练。
15. 推理前停止外骨骼接收和重定向节点。
16. 保留硬件节点、执行桥和 rosbridge。
17. 检查推理 YAML 的模型路径、相机和任务文字。
18. 启动 `lerobot-rollout`。
19. 在终端输入 `/start` 后进行空载短时间测试。
20. 用 `/stop` 停止，检查动作和日志后再逐步增加测试时间。

