# 程序运行手册

## 环境说明

| 环境名 | 用途 |
|---|---|
| `ros-humble` | ROS2（Robostack），用于 ROS2 节点、rosbridge、相机、手柄、数据采集 |
| `panthera` | Panthera 机械臂控制，用于所有 `code/panthera/` 下的程序及 foot_pedal |
| `xr` | Pico 4 Ultra XR 头显 SDK，用于 `code/pico/main.py` |

ROS2 **没有**系统级安装，全部通过 robostack 在 `ros-humble` conda 环境中运行。

---

## 前置：构建 ROS 包

首次使用或代码有改动时，需要编译 `code/ros_pkgs`：

```bash
conda activate ros-humble
cd /path/to/ros/code/ros_pkgs
colcon build --symlink-install
```

---

## 前置：rosbridge（所有程序都需要先启动）

所有程序均通过 `roslibpy` 连接 rosbridge websocket（默认 `localhost:9090`）。  
**必须最先启动 rosbridge，其他程序才能连上 ROS。**

```bash
conda activate ros-humble
source code/ros_pkgs/install/setup.zsh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## 一键启动脚本：`start.sh`

**作用**：用 tmux 批量启动所有常用服务，适合数据采集场景。  
会依次在各 tmux 窗口中启动：rosbridge、相机 1-3、foot pedal。

```bash
./start.sh
```

| tmux 窗口 | 内容 |
|---|---|
| 0 - rosbridge | ROS websocket 桥 |
| 1 - camera_1 | 相机 1（left） |
| 2 - camera_2 | 相机 2（extern） |
| 3 - camera_3 | 相机 3 |
| 5 - foot_pedal | 脚踏板 |

> 注：camera_4 默认注释掉，根据需要手动开启。  
> 脚踏板以 `sudo` + `panthera` 环境中的 Python 运行（evdev 需要 root）。

---

## 1. 相机发布节点（ROS2 包）

**文件**：[code/ros_pkgs/src/camera/camera/camera_publisher.py](code/ros_pkgs/src/camera/camera/camera_publisher.py)

**作用**：读取 Intel RealSense 相机，将彩色（可选深度）图像发布到 ROS topic。每个相机跑一个独立节点，通过 `serial_number` 区分。

**运行**：
```bash
conda activate ros-humble
source code/ros_pkgs/install/setup.zsh
ros2 run camera camera_publisher --config configs/camera_1.yaml
```

**发布 topic**（以 `camera_name` 为命名空间）：
- `/{camera_name}/color/image_raw` — `sensor_msgs/Image`
- `/{camera_name}/color/camera_info` — `sensor_msgs/CameraInfo`
- `/{camera_name}/depth/image_raw` — `sensor_msgs/Image`（仅 `enable_depth: true`）
- `/{camera_name}/depth/camera_info` — `sensor_msgs/CameraInfo`（仅 `enable_depth: true`）

### Config：`configs/camera_1.yaml`（以 camera_1 为例）

```yaml
serial_number: '260322273623'  # RealSense 序列号；'' 则自动选第一个设备
camera_name: left              # topic 命名空间，例如 /left/color/image_raw
enable_depth: false            # 是否发布深度流
publish_rate: 30.0             # 发布频率（Hz）
image_width: 640               # 图像宽度（像素）
image_height: 480              # 图像高度（像素）
```

| 参数 | 说明 |
|---|---|
| `serial_number` | 相机序列号，用 `rs-enumerate-devices` 查询；空字符串则接第一个 |
| `camera_name` | topic 前缀（如 `left`、`extern`、`right`），同时是 TF frame 前缀 |
| `enable_depth` | 是否同时采集并发布深度图（16UC1，单位 mm） |
| `publish_rate` | 发布帧率，默认 30 Hz |
| `image_width/height` | 采集分辨率，必须与 RealSense 支持的分辨率匹配 |

> 现有配置：`camera_1.yaml` → `left`（序列号 260322273623）；`camera_2.yaml` → `extern`（序列号 260322275556）

---

## 2. 手柄发布节点（ROS2 包）

**文件**：[code/ros_pkgs/src/joystick/joystick/joystick_publisher.py](code/ros_pkgs/src/joystick/joystick/joystick_publisher.py)

**作用**：读取 USB 游戏手柄（pygame），将摇杆/扳机/按键映射为机械臂末端速度指令（Twist）和夹爪/复位信号，发布到 ROS topic。

**运行**：
```bash
conda activate ros-humble
source code/ros_pkgs/install/setup.zsh
ros2 run joystick joystick_publisher --config configs/joystick.yaml
```

**发布 topic**：
- `/joystick/right/cmd` — `geometry_msgs/Twist`，末端线速度 + 角速度增量
- `/joystick/right/gripper` — `std_msgs/Int8`，1=开夹，-1=关夹，0=空闲
- `/joystick/right/reset` — `std_msgs/Bool`，True=复位

**按键映射**：

| 输入 | ROS 输出 |
|---|---|
| 左摇杆 X | `linear.x` |
| 左摇杆 Y | `linear.y` |
| RT - LT | `linear.z`（上抬 - 下压） |
| 右摇杆 X | `angular.x`（roll） |
| 右摇杆 Y | `angular.y`（pitch） |
| B/X 键 | `angular.z`（yaw -/+） |
| RB | 夹爪开（+1） |
| LB | 夹爪关（-1） |
| A 键 | 复位 |

### Config：`configs/joystick.yaml`

```yaml
device_id: 0        # pygame 手柄索引（0 = 第一个连接的手柄）
publish_rate: 100.0 # 发布频率（Hz）
trans_scale: 1      # 位移指令缩放系数
rot_scale: 1        # 旋转指令缩放系数

topics:
  cmd:     /joystick/right/cmd      # Twist 指令 topic
  gripper: /joystick/right/gripper  # 夹爪 topic
  reset:   /joystick/right/reset    # 复位 topic
```

| 参数 | 说明 |
|---|---|
| `device_id` | pygame 手柄编号，多手柄时用 1、2 区分 |
| `publish_rate` | 发布频率，越高延迟越低 |
| `trans_scale` | 摇杆→位移缩放；越大单步幅度越大 |
| `rot_scale` | 摇杆→旋转缩放；越大旋转幅度越大 |
| `topics.*` | 发布 topic 名；需与 joystick_control 的订阅 topic 对应 |

---

## 3. 数据采集节点（ROS2 包）

**文件**：[code/ros_pkgs/src/data_collector/data_collector/data_collector_node.py](code/ros_pkgs/src/data_collector/data_collector/data_collector_node.py)

**作用**：多 topic 同步数据采集节点。订阅相机图像和机械臂关节/末端状态，以固定频率对齐时间戳后保存到本地文件；由脚踏板控制录制的开始/停止；录制结束后可自动播放回放以验证数据质量。

**运行**：
```bash
conda activate ros-humble
source code/ros_pkgs/install/setup.zsh
ros2 run data_collector data_collector_node --config configs/data_collection.yaml
```

**交互方式（通过脚踏板 topic）**：

| 脚踏板 | topic | 操作 |
|---|---|---|
| 中（press） | `/foot_pedal/press` | 第一次按下=开始录制；再按=停止录制 |
| 右（save） | `/foot_pedal/save` | 验证完毕后保存本条数据 |
| 左（delete） | `/foot_pedal/delete` | 验证完毕后删除本条数据 |

**保存结构**：
```
<output_dir>/<YYYYMMDD_HHMMSS>/<000001>/
    <timestamp_ns>_left.npy
    <timestamp_ns>_extern.npy
    <timestamp_ns>_left_joint.json
    <timestamp_ns>_left_ee.json
    ...
```

### Config：`configs/data_collection.yaml`（核心字段）

```yaml
recording:
  frequency: 15.0          # 录制帧率（Hz），即每秒保存多少个快照
  output_dir: ./data/xxx   # 数据保存根目录
  buffer_size: 100         # 每个 topic 的内存环形缓冲区大小（条）
  foot_pedal_topic: /foot_pedal/press   # 开始/停止 topic
  save_topic:   /foot_pedal/save        # 保存确认 topic
  delete_topic: /foot_pedal/delete      # 删除 topic

topics:
  - name: left             # 保存文件名后缀
    topic: /left/color/image_raw
    msg_type: sensor_msgs/Image
    is_primary: true       # 以此 topic 的时间戳为对齐基准（建议选低频 topic）
  - name: left_joint
    topic: /follower_left/joint_states
    msg_type: sensor_msgs/JointState
    is_primary: false
  # ... 更多 topic

verify:
  enabled: true            # 录制停止后是否自动验证
  primary: left            # 对齐基准（与 topics 中 is_primary 一致）
  alignment_warn_ms: 10    # 时间对齐误差超过此值（ms）则警告
  dropout_threshold: 1.5   # 帧间隔 > median_gap × 此值 视为丢帧
  joint_topics: [left_joint, right_joint]  # 用于轨迹平滑度分析
  ee_topics: [left_ee, right_ee]
  plot: false              # 是否显示分析图表
```

| 参数 | 说明 |
|---|---|
| `recording.frequency` | 保存帧率；通常设为相机帧率的一半以下（相机 30Hz→录 15Hz） |
| `recording.output_dir` | 任务名称目录，每次运行会在其下创建带时间戳的子目录 |
| `recording.buffer_size` | 越大对时间对齐越宽容，但消耗更多内存 |
| `topics[].name` | 文件名中的标识符，也用于 verify 中的引用 |
| `topics[].is_primary` | 全局只能有一个 true；时间戳以此 topic 为基准对齐其他 topic |
| `verify.alignment_warn_ms` | 时间对齐偏差阈值，超出则在控制台打印警告 |
| `verify.dropout_threshold` | 丢帧检测灵敏度，越小越严格 |

---

## 4. 脚踏板发布节点

**文件**：[code/foot_pedal/main.py](code/foot_pedal/main.py)

**作用**：监听 USB 脚踏板的 evdev 事件，将按键释放映射为 `std_msgs/Empty` 发布到对应 ROS topic（经 rosbridge）。需要 `sudo` 权限才能读取 evdev 设备。

**运行**（需在 rosbridge 启动后）：
```bash
sudo /path/to/miniforge3/envs/panthera/bin/python \
    code/foot_pedal/main.py --config configs/foot_pedal.yaml
```

> `start.sh` 会自动用绝对路径以 sudo 启动。

**发布 topic**：由 config 决定，默认：
- `/foot_pedal/press` — `std_msgs/Empty`
- `/foot_pedal/save` — `std_msgs/Empty`
- `/foot_pedal/delete` — `std_msgs/Empty`

### Config：`configs/foot_pedal.yaml`

```yaml
rosbridge:
  host: "localhost"
  port: 9090

pedals:
  - device: "KM-key08"                          # evdev 设备名中包含此字符串
    phys: "usb-0000:09:00.0-4.4.3/input0"       # evdev 物理路径（区分多个同型号设备）
    key: "KEY_SPACE"                            # 对应的按键码（用 sudo evtest 查询）
    topic: "/foot_pedal/press"                  # 按键释放时发布到此 topic
  - device: "KM-key08"
    phys: "usb-0000:09:00.0-4.4.2/input0"
    key: "KEY_SPACE"
    topic: "/foot_pedal/save"
  - device: "KM-key08"
    phys: "usb-0000:09:00.0-4.4.4/input0"
    key: "KEY_SPACE"
    topic: "/foot_pedal/delete"
```

| 参数 | 说明 |
|---|---|
| `rosbridge.host/port` | rosbridge 地址，需与正在运行的 rosbridge 一致 |
| `pedals[].device` | evdev 设备名的子字符串，用于匹配设备 |
| `pedals[].phys` | evdev 物理路径，用于区分同名多设备（可用 `sudo evtest` 查询） |
| `pedals[].key` | 按键名（evdev ecodes），如 `KEY_SPACE`、`KEY_ENTER` |
| `pedals[].topic` | 按键触发时发布的 ROS topic |

> 查询设备信息：`sudo evtest`

---

## 5. Pico 4 Ultra XR 遥操作发布端

**文件**：[code/pico/main.py](code/pico/main.py)

**作用**：从 Pico 4 Ultra 头显读取双手控制器位姿和按键，经坐标变换后将末端速度指令（Twist）和夹爪指令发布到 ROS（经 rosbridge）。左/右 Grip 键分别激活左/右臂控制；双击 Grip 复位；Y 键急停。

**环境**：`xr`（需要 `xrobotoolkit_sdk`）

**运行**：
```bash
conda activate xr
cd code/pico
python main.py --config ../../configs/pico.yaml
```

**操作说明**：

| 操作 | 效果 |
|---|---|
| 右手 Grip 按下（单击） | 激活右臂控制，以当前位姿为基准校准 |
| 右手 Grip 按住 | 控制右臂末端（持续发布位姿偏移） |
| 右手 Grip 双击 | 右臂回零（复位到安全位置），需重新按下校准 |
| 左手同理 | 控制左臂 |
| B 键（右手） | 右夹爪张开 |
| A 键（右手） | 右夹爪闭合 |
| Y 键（左手） | 左夹爪张开 |
| X 键（左手） | 左夹爪闭合 |
| Ctrl+C | 退出，自动发布急停信号 |

**发布 topic**（以右臂为例，左臂同理）：
- `/joystick/right/cmd` — `geometry_msgs/Twist`
- `/joystick/right/gripper` — `std_msgs/Int8`
- `/joystick/right/reset` — `std_msgs/Bool`
- `/joystick/right/init` — `std_msgs/Bool`
- `/joystick/emergency` — `std_msgs/Bool`

### Config：`configs/pico.yaml`

```yaml
rosbridge:
  host: "localhost"
  port: 9090

grip:
  threshold: 0.5          # Grip 键按下阈值（0.0~1.0）
  double_tap_window_s: 0.4 # 双击识别时间窗口（秒）

scale:
  translation_m: 1.0      # XR 位移 → Twist linear 的缩放系数
  rotation_rad: 1.0       # XR 旋转 → Twist angular 的缩放系数
  max_delta_pos_m: 0.3    # 单帧最大位移幅值（m），超出则截断方向不变

deadzone:
  pos_m: 0.008            # 平移死区半径（m），约为 tracker 噪声的 2~3 倍
  rot_rad: 0.03           # 旋转死区（rad ≈ 1.7°）

filter:
  alpha: 0.08             # EMA 低通滤波系数（0~1），越小越平滑但延迟越大

# XR 坐标系 → 机械臂坐标系映射矩阵（行向量 = 机械臂轴在 XR 坐标系中的方向）
coord_mapping:
  pos: [[0, 0, -1],       # 机械臂 X ← -XR Z
        [-1, 0, 0],       # 机械臂 Y ← -XR X
        [0, 1, 0]]        # 机械臂 Z ←  XR Y
  rot: [[0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0]]
  rot_sign: [1, 1, 1]     # 各旋转轴方向符号（1 或 -1）

topics:
  publish:
    right_cmd:     "/joystick/right/cmd"
    left_cmd:      "/joystick/left/cmd"
    right_gripper: "/joystick/right/gripper"
    left_gripper:  "/joystick/left/gripper"
    right_reset:   "/joystick/right/reset"
    left_reset:    "/joystick/left/reset"
    right_init:    "/joystick/right/init"
    left_init:     "/joystick/left/init"
    emergency:     "/joystick/emergency"
```

| 参数 | 说明 |
|---|---|
| `grip.threshold` | Grip 模拟轴超过此值视为"按下" |
| `grip.double_tap_window_s` | 两次 Grip 在此时间窗内视为双击（触发复位） |
| `scale.translation_m` | 手柄移动量 → 机械臂末端移动量的比例；越大越灵敏 |
| `scale.rotation_rad` | 手柄旋转量 → 机械臂末端旋转量的比例 |
| `scale.max_delta_pos_m` | 单帧位移幅值上限，防止手柄大幅晃动时机械臂突变 |
| `deadzone.pos_m` | 平移死区；手的微小抖动在此范围内会被过滤掉 |
| `deadzone.rot_rad` | 旋转死区 |
| `filter.alpha` | EMA 滤波强度；`0.08` 为适度滤波，增大则更平滑但响应变慢 |
| `coord_mapping.pos/rot` | 3×3 正交矩阵，将 XR 世界坐标系映射到机械臂坐标系 |
| `coord_mapping.rot_sign` | 对旋转分量逐轴取反，用于匹配机械臂旋转约定 |

---

## 6. Joystick 控制节点（Panthera 端）

**文件**：[code/panthera/joystick_control/main.py](code/panthera/joystick_control/main.py)

**作用**：接收来自手柄或 Pico 发布的 Twist/夹爪/复位 topic，通过 IK 将末端速度增量转换为关节目标，控制单臂 Panthera 机械臂。启动时先移动到安全位置，退出时也返回安全位置。同时向 ROS 发布关节状态和末端 Pose。

**环境**：`panthera`

**运行**（需先启动 rosbridge）：
```bash
conda activate panthera
cd code/panthera/joystick_control
# 左臂
python main.py --config ../../../configs/pathera_left.yaml
# 右臂
python main.py --config ../../../configs/pathera_right.yaml
```

**订阅 topic**：
- `topics.subscribe.cmd` — `geometry_msgs/Twist`，末端速度增量
- `topics.subscribe.gripper` — `std_msgs/Int8`，夹爪指令
- `topics.subscribe.reset` — `std_msgs/Bool`，复位信号

**发布 topic**：
- `topics.publish.joints` — `sensor_msgs/JointState`
- `topics.publish.ee` — `geometry_msgs/PoseStamped`

### Config：`configs/pathera_left.yaml` / `configs/pathera_right.yaml`

```yaml
motor_config_path: "/path/to/configs/robot_param/Follower_left.yaml"
                        # 机器人参数文件路径（关节限位、力矩限制等）

rosbridge:
  host: "localhost"
  port: 9090

rates:
  control_hz: 50.0       # 控制循环频率（Hz）
  publish_hz: 100.0      # 关节状态发布频率（Hz）
  watchdog_timeout_s: 0.5 # 超过此时间未收到 cmd 则切换到保持模式

ik_control:
  kp: [30.0, 50.0, 60.0, 25.0, 15.0, 10.0]  # 6 关节 PD 位置增益
  kd: [ 3.0,  5.0,  6.0,  2.5,  1.5,  1.0]  # 6 关节 PD 速度增益

safe_position:
  joint_pos: [0.0, 0.5, 0.6, 0.0, 0.0, 0.0]  # 安全位置各关节角（rad）
  joint_vel: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]  # 移动到安全位置的速度（rad/s）

twist_scale:
  translation_m: 0.005   # 每帧收到的 Twist linear 缩放为末端位移（m）
  rotation_rad: 0.003    # 每帧收到的 Twist angular 缩放为末端旋转（rad）

joints:
  names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]

topics:
  subscribe:
    cmd:     "/joystick/left/cmd"
    gripper: "/joystick/left/gripper"
    reset:   "/joystick/left/reset"
  publish:
    joints:  "/follower_left/joint_states"
    ee:      "/follower_left/end_effector"
```

| 参数 | 说明 |
|---|---|
| `motor_config_path` | 机器人 YAML 路径，包含关节限位、URDF、电机参数 |
| `rates.control_hz` | IK 控制循环频率；过高会增加计算负担 |
| `rates.watchdog_timeout_s` | 超时后机械臂切换为重力补偿保持模式（防止指令延迟时漂移） |
| `ik_control.kp/kd` | 各关节 PD 增益；kp 越大跟踪越紧，kd 越大阻尼越强（防振荡） |
| `safe_position.joint_pos` | 启动和退出时的目标关节角（rad），应设为无碰撞的中性位 |
| `twist_scale.translation_m` | 每帧 Twist 中 linear 的缩放；与 Pico/手柄端的 scale 共同决定灵敏度 |
| `topics.subscribe.cmd` | 必须与手柄/Pico 端发布的 topic 名一致 |

---

## 7. 主从遥操作节点（Leader-Follower）

**文件**：[code/panthera/leader_follower/main.py](code/panthera/leader_follower/main.py)

**作用**：将 Leader 臂的关节位置/速度实时映射到 Follower 臂（位置+速度前馈 + 重力补偿 + 摩擦力补偿的 PD 控制）。支持多对主从同时运行。向 ROS 发布 Follower 的关节状态和末端 Pose。

**环境**：`panthera`

**运行**（需先启动 rosbridge）：
```bash
conda activate panthera
cd code/panthera/leader_follower
python main.py --config ../../../configs/follower_leader.yaml
```

**发布 topic**（每对各一组）：
- `/follower_left/joint_states` — `sensor_msgs/JointState`
- `/follower_left/end_effector` — `geometry_msgs/PoseStamped`
- `/follower_right/joint_states`
- `/follower_right/end_effector`

### Config：`configs/follower_leader.yaml`

```yaml
rosbridge:
  host: "localhost"
  port: 9090

publish_hz: 100.0          # Follower 状态发布频率（Hz）

pairs:
  - name: "pair_1"
    leader:   "/path/to/configs/robot_param/Leader_left.yaml"
    follower: "/path/to/configs/robot_param/Follower_left.yaml"
    topics:
      publish:
        joint_states: "/follower_left/joint_states"
        end_effector: "/follower_left/end_effector"
  - name: "pair_2"
    leader:   "/path/to/configs/robot_param/Leader_right.yaml"
    follower: "/path/to/configs/robot_param/Follower_right.yaml"
    topics:
      publish:
        joint_states: "/follower_right/joint_states"
        end_effector: "/follower_right/end_effector"

control:
  kp: [10.0, 21.0, 21.0, 16.0, 13.0, 1.0]   # Follower 各关节位置增益
  kd: [1.0,   2.0,  2.0,  0.9,  0.8,  0.1]  # Follower 各关节速度增益
  gripper_kp: 4.0                             # 夹爪位置增益
  gripper_kd: 0.4                             # 夹爪速度增益
  friction:
    Fc: [0.15, 0.12, 0.12, 0.12, 0.04, 0.04] # 各关节库仑摩擦力系数
    Fv: [0.05, 0.05, 0.05, 0.03, 0.02, 0.02] # 各关节粘性摩擦力系数
    vel_threshold: 0.02                       # 摩擦补偿速度阈值（rad/s）
  tor_threshold: [0.5, 1.0, 1.0, 0.5, 0.3, 0.3] # 力矩死区（小于此值的力矩差不参与反馈）
```

| 参数 | 说明 |
|---|---|
| `pairs[].leader/follower` | 各臂的机器人参数 YAML 路径 |
| `pairs[].topics.publish` | Follower 状态发布 topic，需与数据采集的订阅 topic 一致 |
| `control.kp/kd` | Follower 跟随增益；kp 大跟随快但易振荡，kd 大阻尼强但响应慢 |
| `control.friction.Fc` | 库仑（静）摩擦补偿；补偿电机启动时的摩擦力 |
| `control.friction.Fv` | 粘性（动）摩擦补偿；补偿运动时随速度增大的摩擦力 |
| `control.tor_threshold` | 力矩死区，过滤传感器噪声引起的微小力矩反馈 |

---

## 8. Pico 位置控制节点（pico_control）

**文件**：[code/panthera/pico_control/main.py](code/panthera/pico_control/main.py)

**作用**：专为 Pico 遥操作设计的位置控制节点。与 joystick_control（增量速度控制）不同，pico_control 接收的是**相对于校准基准的绝对末端位姿偏移**。启动后先移动到安全位置，然后等待 Pico 发来的 `init` 信号来校准基准末端位姿；校准完成后，将 Pico 控制器的实时偏移量加到基准位姿上作为 IK 目标，通过带速度限幅的 PD 平滑跟踪执行。

内部结构分三个线程：
- **控制线程**（50 Hz）：消费 Pico 偏移量 → 更新 hard target → smooth target PD 步进 → IK → 电机指令
- **发布线程**（100 Hz）：将关节状态 + 末端 Pose 发布到 ROS topic
- **主线程**：管理 rosbridge 订阅、启动/停止信号

**环境**：`panthera`

**运行**（需先启动 rosbridge 和 Pico 发布端）：
```bash
conda activate panthera
cd code/panthera/pico_control

# 左臂
python main.py --config ../../../configs/pico_control_left.yaml
# 右臂
python main.py --config ../../../configs/pico_control_right.yaml
```

**启动后工作流**：
1. 程序移动机械臂到安全位置
2. 打印 `请按 Pico init 键完成校准`，等待 Grip 单击
3. Grip 单击 → 记录当前末端位姿为基准 → 控制循环激活
4. Grip 按住期间，末端跟踪 Pico 偏移量
5. Grip 双击 → 复位到安全位置，需重新校准

**订阅 topic**：
- `cmd` — `geometry_msgs/Twist`，Pico 发出的绝对位姿偏移（linear=位移偏移 m，angular=旋转偏移 rad）
- `gripper` — `std_msgs/Int8`，1=开夹，-1=关夹
- `reset` — `std_msgs/Bool`，物理复位到安全位置（优先级最高）
- `init` — `std_msgs/Bool`，记录当前末端位姿为位置控制校准基准

**发布 topic**：
- `joints` — `sensor_msgs/JointState`（6 关节 + gripper）
- `ee` — `geometry_msgs/PoseStamped`

### Config：`configs/pico_control_left.yaml` / `configs/pico_control_right.yaml`

```yaml
motor_config_path: "/path/to/configs/robot_param/Follower_right.yaml"
                        # 机器人参数文件路径

rosbridge:
  host: "localhost"
  port: 9090

rates:
  control_hz: 50.0          # 控制循环频率（Hz）
  publish_hz: 100.0         # 关节状态发布频率（Hz）
  watchdog_timeout_s: 0.5   # 超过此时间未收到 cmd 则切换到 hold 模式

ik_control:
  kp: [30.0, 50.0, 60.0, 25.0, 15.0, 10.0]  # 6 关节 PD 位置增益
  kd: [ 3.0,  5.0,  6.0,  2.5,  1.5,  1.0]  # 6 关节 PD 速度增益

ik_params:
  max_joint_step_rad: 0.15  # 每控制周期最大关节位移（rad），防止 IK 奇异点跳变

safe_position:
  joint_pos: [0.0, 0.5, 0.6, 0.0, 0.0, 0.0]  # 安全位置各关节角（rad）
  joint_vel: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]  # 移动到安全位置的速度（rad/s）

twist_scale:
  translation_m: 0.7  # Pico 偏移量（m）→ 机械臂末端位移缩放（越大越灵敏）
  rotation_rad: 1.0   # Pico 旋转偏移（rad）→ 末端旋转缩放

smoothing:
  max_linear_vel_m_s:    0.2  # 末端线速度上限（m/s），防止运动过猛
  max_angular_vel_rad_s: 0.5  # 末端角速度上限（rad/s）
  tracking_gain_hz:      40   # smooth target P 增益（Hz），时间常数 ≈ 1/40 s
  damping_ratio:         0.6  # D/P 比例；1.0=临界阻尼，<1=欠阻尼（有超调）

joints:
  names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]

topics:
  subscribe:
    cmd:     "/joystick/right/cmd"     # 左臂改为 /joystick/left/cmd
    gripper: "/joystick/right/gripper"
    reset:   "/joystick/right/reset"
    init:    "/joystick/right/init"    # joystick_control 无此项，pico_control 特有
  publish:
    joints:  "/follower_right/joint_states"
    ee:      "/follower_right/end_effector"
```

| 参数 | 说明 |
|---|---|
| `ik_params.max_joint_step_rad` | 单步 IK 关节跳变上限（rad），超过则截断并保持方向，防止奇异点附近飞车 |
| `twist_scale.translation_m` | Pico 偏移量→末端位移缩放；Pico 输出的是绝对偏移（m），0.7 表示 Pico 移动 1m 时末端偏移 0.7m |
| `smoothing.tracking_gain_hz` | smooth target 向 hard target 步进的 P 增益；越大响应越快，越小越平滑 |
| `smoothing.damping_ratio` | D/P 比；0.6 为欠阻尼（响应快但可能有轻微超调） |
| `topics.subscribe.init` | Pico Grip 单击时发出，机器人记录当前末端位姿为控制基准；校准前 cmd 不生效 |
| `rates.watchdog_timeout_s` | Grip 松开后超过此时间无 cmd，切换到 hold 模式（重力补偿保持位置） |

---

## 9. VLA 动作执行节点（vla_control）

**文件**：[code/panthera/vla_control/main.py](code/panthera/vla_control/main.py)

**作用**：订阅 VLA 模型推理结果（`/vla/actions`，Float32MultiArray），将动作 chunk 入队并以固定频率逐步执行到机械臂关节。同时向 ROS 发布关节状态，供 VLA bridge 读取作为模型输入。支持单臂（left/right）和双臂（dual）模式。

**环境**：`panthera`

**运行**（需先启动 rosbridge 和 VLA bridge）：
```bash
conda activate panthera
cd code/panthera/vla_control
python main.py --config ../../../configs/vla_control.yaml
```

**订阅 topic**：
- `/vla/actions` — `std_msgs/Float32MultiArray`，VLA 推理结果（14 维双臂或 7 维单臂）

**发布 topic**：
- `/vla_robot/right/joint_states` — `sensor_msgs/JointState`
- `/vla_robot/right/end_effector` — `geometry_msgs/PoseStamped`
- `/vla/trigger` — `std_msgs/String`（向 VLA bridge 请求新的推理）

### Config：`configs/vla_control.yaml`

```yaml
# "dual" 双臂 14 维；"right"/"left" 单臂 7 维
mode: "right"

rosbridge:
  host: "localhost"
  port: 9090

rates:
  control_hz: 50.0   # 动作执行频率（Hz），决定每步 VLA action 的速度
  publish_hz: 50.0   # 关节状态发布频率（Hz）

arms:
  right:
    motor_config_path: "/path/to/Follower_right.yaml"
    joint_names: ["joint_1", ..., "joint_6"]
    topics:
      joint_states: "/vla_robot/right/joint_states"
      end_effector: "/vla_robot/right/end_effector"
  left:
    motor_config_path: "/path/to/Follower_left.yaml"
    joint_names: ["joint_1", ..., "joint_6"]
    topics:
      joint_states: "/vla_robot/left/joint_states"
      end_effector: "/vla_robot/left/end_effector"

safe_position:
  joint_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  joint_vel: [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]

control:
  kp: [10.0, 21.0, 21.0, 16.0, 13.0, 1.0]
  kd: [1.0,   2.0,  2.0,  0.9,  0.8, 0.1]
  gripper_kp: 4.0
  gripper_kd: 0.4
  exec_vel: 1.0      # 预留参数，Joint_Pos_Vel 模式使用

prompt: "Close the top drawer."   # 任务语言指令，发给 VLA bridge

topics:
  subscribe:
    vla_actions: "/vla/actions"   # 接收 VLA 推理动作
  publish:
    trigger: "/vla/trigger"       # 向 VLA bridge 请求推理
```

| 参数 | 说明 |
|---|---|
| `mode` | `dual` 双臂（14 维动作），`right`/`left` 单臂（7 维动作）；需与 VLA bridge 一致 |
| `rates.control_hz` | 动作执行频率；越高每步越小，动作越平滑 |
| `arms.*.motor_config_path` | 各臂机器人参数路径；未填写的臂不会初始化 |
| `safe_position` | 启动和退出时的目标关节角，VLA 模式通常设为 [0,0,0,0,0,0] |
| `prompt` | 任务描述文字，发给 VLA bridge 作为语言条件输入 |

---

## 10. VLA 推理 Bridge

**文件**：[code/VLA/ros_bridge.py](code/VLA/ros_bridge.py)

**作用**：加载 openpi VLA 模型，订阅相机图像和关节状态，收到 trigger 信号后调用 `policy.infer()` 做推理，将动作 chunk 以 `Float32MultiArray` 发布到 `/vla/actions`。支持单臂（left/right）和双臂（dual）模式。

**依赖**：需要安装 `openpi`（单独仓库），及 `cv2`、`roslibpy`。

**运行**：
```bash
# 根据 openpi 的安装方式激活对应环境
cd code/VLA
python ros_bridge.py --config ../../configs/vla_ros_bridge.yaml
```

**订阅 topic**：
- `/vla/trigger` — `std_msgs/String`，触发推理（data 字段作为语言指令）
- `/camera_extern/color/image_raw` — 外部相机图像
- `/camera_left/color/image_raw`、`/camera_right/color/image_raw` — 腕部相机
- `/vla_robot/right/joint_states`、`/vla_robot/left/joint_states` — 关节状态

**发布 topic**：
- `/vla/actions` — `std_msgs/Float32MultiArray`，推理结果（chunk × action_dim）

### Config：`configs/vla_ros_bridge.yaml`

```yaml
mode: "dual"     # "dual"=14 维双臂；"right"/"left"=7 维单臂；需与 vla_control 一致

rosbridge:
  host: "localhost"
  port: 9090

model:
  config_name: "pi05_ht_left_arm_lora_open_top_drawer"   # openpi 模型配置名
  checkpoint_dir: "/path/to/checkpoints/xxx/6000"        # 模型 checkpoint 目录
  fallback_prompt: "Close the top drawer."               # trigger 无语言时的默认 prompt
  image_height: 180   # 模型输入图像高度（像素）
  image_width: 320    # 模型输入图像宽度（像素）

joints:
  right_names: ["joint_1", ..., "joint_6"]   # 必须与 vla_control 发布的 name 一致
  left_names:  ["joint_1", ..., "joint_6"]
  gripper_name: "gripper"

topics:
  cameras:
    extern:      "/camera_extern/color/image_raw"
    left_wrist:  "/camera_left/color/image_raw"
    right_wrist: "/camera_right/color/image_raw"
  subscribe:
    trigger:      "/vla/trigger"
    right_joints: "/vla_robot/right/joint_states"
    left_joints:  "/vla_robot/left/joint_states"
  publish:
    actions: "/vla/actions"
```

| 参数 | 说明 |
|---|---|
| `mode` | 推理模式，必须与 vla_control 的 mode 保持一致 |
| `model.config_name` | openpi 中注册的模型配置名 |
| `model.checkpoint_dir` | 训练好的 checkpoint 目录（包含 params 文件） |
| `model.fallback_prompt` | trigger topic 的 data 为空时使用的默认任务描述 |
| `model.image_height/width` | 模型期望的输入图像分辨率；相机图像会自动 resize 到此大小 |
| `joints.gripper_name` | JointState 消息中夹爪关节的名字，用于提取夹爪位置 |
| `topics.cameras.extern` | 外部全景相机 topic；三种模式都需要 |
| `topics.cameras.left/right_wrist` | 腕部相机 topic；按 mode 自动选取 |

---

## 典型使用场景

### 场景一：Pico 遥操作 + 数据采集（推荐）

```
rosbridge  →  camera (×3)  →  foot_pedal
                ↓
pico/main.py（xr 环境，Pico 控制器读取端）
                ↓
pico_control/main.py × 2（panthera 环境，左/右臂各一个）
                ↓
data_collector_node（ros-humble 环境）
```

1. 启动 `./start.sh`（rosbridge、相机、foot_pedal）
2. 启动 `pico_control`（左臂）+ `pico_control`（右臂）
3. 启动 `data_collector_node`
4. 在 xr 环境启动 `pico/main.py`
5. 在 Pico 头显中 **Grip 单击**完成校准（左右手各一次）
6. Grip 按住遥操机械臂，脚踏板控制录制开始/停止/保存

> 详细步骤见 [docs/pico_data_collection.md](pico_data_collection.md)

### 场景二：主从遥操作 + 数据采集

1. 启动 `./start.sh`
2. 启动 `leader_follower`（自动控制双臂）
3. 启动 `data_collector_node`

> 详细步骤见 [docs/leader_follower_data_collection.md](leader_follower_data_collection.md)


---

## Robot Param 文件（`configs/robot_param/`）

各机械臂实例的低层参数，被各控制节点通过 `motor_config_path` 引用。

```yaml
# configs/robot_param/Follower_left.yaml 示例
robot:
  name: "Panthera-HT"
  param_file: "...motor_param/6dof_Panthera_params_follower_left.yaml"
  joint_limits:
    lower: [-2.4, -0.1, -0.1, -1.6, -1.7, -2.5]   # 各关节下限（rad）
    upper: [ 2.4,  3.2,  4.0,  1.6,  1.7,  2.5]   # 各关节上限（rad）
  gripper_limits:
    lower: 0.0   # 夹爪关闭位置
    upper: 2.0   # 夹爪打开位置
  max_torque: [21, 36, 36, 21, 10, 10]             # 各关节最大力矩（N·m）
  velocity_limits: [1.0, ...]                       # 各关节速度限制（rad/s）

urdf:
  file_path: "...Panthera-HT_description_follower.urdf"
  base_link: "base_link"
  end_effector_link: "tool_link"

kinematics:
  joint_names: ["joint1", ..., "joint6"]   # URDF 中的关节名

control:
  default_velocity: 0.5          # 默认运动速度（rad/s）
  default_max_torque: 10.0       # 默认最大力矩（N·m）
  position_tolerance: 0.01       # 到达目标位置的判定容差（rad）
  timeout: 10.0                  # 运动超时时间（s）
```

| 文件 | 对应机器人 |
|---|---|
| `Leader_left.yaml` / `Leader_right.yaml` | 主臂（左/右） |
| `Follower_left.yaml` / `Follower_right.yaml` | 从臂（左/右） |
| `Follower_vision.yaml` | 视觉臂（第三臂） |
