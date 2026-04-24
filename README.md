## Setup

这个 repo 里的代码基于 ROS2 Humble 的语法编写，不建议系统级安装 ROS2 Humble，会导致环境管理比较复杂，容易和 conda 环境产生冲突。推荐使用开源项目 [Robostack](https://robostack.github.io/index.html)。

**Installing ROS**
```bash
# Create a ros-humble desktop environment
conda create -n ros-humble -c conda-forge -c robostack-humble ros-humble-desktop
# Activate the environment
conda activate ros-humble
# Add the robostack channel to the environment
conda config --env --add channels robostack-humble
```

**Installing the tools**
```bash
conda activate ros-humble
conda install -c conda-forge ros-dev-tools
```

**Installing the ros-python bridge**
```bash
conda install -c robostack-humble ros-humble-rosbridge-suite
pip install roslibpy
```

## 编译 ROS 包

所有 ROS 包都在 `code/ros_pkgs/src/` 目录下，使用 `colcon` 进行编译。

```bash
conda activate ros-humble
cd code/ros_pkgs
colcon build
```

编译完成后，需要 source 环境才能使用这些包：

```bash
source install/setup.bash
```

如果只想编译某个特定的包，可以使用 `--packages-select` 参数，例如：

```bash
colcon build --packages-select joystick
```

## ROS 包用法

所有包的配置文件都放在 `configs/` 目录下。每个节点启动时都需要通过 `--config` 参数指定对应的 yaml 配置文件。

### camera

基于 Intel RealSense 的相机节点，支持 RGB 和深度图像发布。每个相机单独启动一个节点，通过 `serial_number` 区分。

**依赖安装：**
```bash
pip install pyrealsense2
conda install -c robostack-humble ros-humble-cv-bridge ros-humble-image-transport
```

**配置文件示例（`configs/camera_1.yaml`）：**
```yaml
serial_number: '136622073828'  # 留空则自动选择第一个设备
camera_name: camera_1          # 决定 topic 的命名空间
enable_depth: false            # 是否发布深度流
publish_rate: 30.0             # Hz
image_width: 640
image_height: 480
```

**启动：**
```bash
ros2 run camera camera_publisher --config configs/camera_1.yaml
```

**发布的 Topics：**
| Topic | 类型 |
|---|---|
| `/<camera_name>/color/image_raw` | `sensor_msgs/Image` |
| `/<camera_name>/color/camera_info` | `sensor_msgs/CameraInfo` |
| `/<camera_name>/depth/image_raw` | `sensor_msgs/Image`（开启深度时） |
| `/<camera_name>/depth/camera_info` | `sensor_msgs/CameraInfo`（开启深度时） |

---

### joystick

通过 pygame 读取手柄输入并发布控制指令。

**依赖安装：**
```bash
pip install pygame
```

**配置文件示例（`configs/joystick.yaml`）：**
```yaml
device_id: 0        # pygame 手柄索引
publish_rate: 100.0 # Hz
trans_scale: 1      # 平移缩放系数
rot_scale: 1        # 旋转缩放系数
```

**启动：**
```bash
ros2 run joystick joystick_publisher --config configs/joystick.yaml
```

**发布的 Topics：**
| Topic | 类型 | 说明 |
|---|---|---|
| `/joystick/cmd` | `geometry_msgs/Twist` | 平移和旋转指令 |
| `/joystick/gripper` | `std_msgs/Int8` | 1=open, -1=close, 0=idle |
| `/joystick/reset` | `std_msgs/Bool` | True=触发 reset |

**按键映射：**
| 按键 | 功能 |
|---|---|
| 左摇杆 X/Y | 平移 X/Y |
| RT / LT | 平移 Z 上/下 |
| 右摇杆 X/Y | Roll / Pitch |
| B / X | Yaw -/+ |
| RB / LB | 夹爪 open/close |
| A | Reset |

---

### data_collector

订阅多个 ROS topic，按照固定频率对齐时间戳并保存到本地。图像保存为 `.npy`，其他消息保存为 `.json`，每个 snapshot 存放在单独的子文件夹中。

**配置文件示例（`configs/data_collection.yaml`）：**
```yaml
recording:
  frequency: 15.0          # Hz，保存频率
  output_dir: ./data/test  # 输出目录
  buffer_size: 100         # 每个 topic 的环形缓冲区大小

topics:
  - name: camera_1                        # 用于文件命名
    topic: /camera_1/color/image_raw
    msg_type: sensor_msgs/Image
    is_primary: false

  - name: camera_2
    topic: /camera_2/color/image_raw
    msg_type: sensor_msgs/Image
    is_primary: true                      # 以该 topic 的时间戳作为对齐基准
```

> `is_primary: true` 的 topic 时间戳作为其他所有 topic 的对齐基准，通常设置在频率最低的 topic 上。

**启动：**
```bash
ros2 run data_collector data_collector_node --config configs/data_collection.yaml
```

## 非 ROS 包的代码（panthera / pico）

`code/panthera/` 和 `code/pico/` 下的代码是普通 Python 程序，通过 `roslibpy` 与 ROS 通信，**不需要** source ROS 环境，可以在任意 conda 环境中运行。

### 前提：启动 rosbridge_server

这些程序依赖 rosbridge_server 作为通信桥梁，运行前必须先在 ros-humble 环境中启动：

```bash
conda activate ros-humble
conda install -c robostack-humble ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

默认监听 `localhost:9090`，启动成功后保持该终端运行。

---

### panthera — joystick_control

通过手柄（joystick ROS 包）控制 Panthera 机械臂，订阅手柄指令，通过 IK 控制末端位姿。

**配置文件示例（`configs/pathera_right.yaml`）：**
```yaml
motor_config_path: "/path/to/Right.yaml"   # Panthera SDK 的电机配置文件路径
rosbridge:
  host: "localhost"
  port: 9090

rates:
  control_hz: 50.0         # 控制频率
  publish_hz: 100.0        # 状态发布频率
  watchdog_timeout_s: 0.5  # 超时保护

safe_position:
  joint_pos: [0.0, 0.5, 0.6, 0.0, 0.0, 0.0]
  joint_vel: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

topics:
  subscribe:
    cmd:     "/joystick/right/cmd"
    gripper: "/joystick/right/gripper"
    reset:   "/joystick/right/reset"
  publish:
    joints:  "/robot/joint_states"
    ee:      "/robot/end_effector"
```

**发布的 Topics：**
| Topic | 类型 | 说明 |
|---|---|---|
| `topics.publish.joints` | `sensor_msgs/JointState` | 6 个关节 + 夹爪，共 7 个 entry |
| `topics.publish.ee` | `geometry_msgs/PoseStamped` | 末端位姿 |

JointState 的 `name` 字段为 `joints.names`（配置文件中的 6 个关节名）加上 `"gripper"`，`position` 和 `velocity` 均来自实时读取。

**启动：**
```bash
python code/panthera/joystick_control/main.py --config configs/pathera_right.yaml
```

---

### panthera — leader_follower

主从遥操作模式，支持任意数量的主从臂对，follower 跟随 leader 的关节运动。

**配置文件示例（`configs/follower_leader.yaml`）：**
```yaml
rosbridge:
  host: "localhost"
  port: 9090

publish_hz: 100.0

pairs:
  - name: "pair_1"
    leader:   "/path/to/Leader_1.yaml"    # Panthera SDK 电机配置
    follower: "/path/to/Follower_1.yaml"
    topics:
      publish:
        joint_states: "/follower_1/joint_states"
        end_effector: "/follower_1/end_effector"
```

**发布的 Topics：**
| Topic | 类型 | 说明 |
|---|---|---|
| `topics.publish.joint_states` | `sensor_msgs/JointState` | 6 个关节 + 夹爪，共 7 个 entry |
| `topics.publish.end_effector` | `geometry_msgs/PoseStamped` | 末端位姿 |

JointState 的 `name` 字段为 `joint_1` … `joint_6` 加上 `"gripper"`，`position` 和 `velocity` 均来自 follower 实时读取。

**启动：**
```bash
python code/panthera/leader_follower/main.py --config configs/follower_leader.yaml
```

---

### pico

通过 Pico 4 Ultra VR 手柄（XRoboToolkit SDK）控制双臂，将手柄的位姿转换为末端控制指令并发布到 ROS。

> 注意：`xrobotoolkit_sdk` 需要单独安装，且通常需要在独立的 conda 环境中运行。

**配置文件需包含以下字段：**
```yaml
rosbridge:
  host: "localhost"
  port: 9090

grip:
  threshold: 0.5           # Grip 按下判定阈值
  double_tap_window_s: 0.4 # 双击识别时间窗口（触发回零）

scale:
  translation_m: 0.005
  rotation_rad: 0.003
  max_delta_pos_m: 0.05

topics:
  publish:
    right_cmd:     "/pico/right/cmd"
    left_cmd:      "/pico/left/cmd"
    right_gripper: "/pico/right/gripper"
    left_gripper:  "/pico/left/gripper"
    right_reset:   "/pico/right/reset"
    left_reset:    "/pico/left/reset"
    emergency:     "/pico/emergency"
```

**发布的 Topics：**
| Topic | 类型 |
|---|---|
| `topics.publish.right_cmd` / `left_cmd` | `geometry_msgs/Twist` |
| `topics.publish.right_gripper` / `left_gripper` | `std_msgs/Int8` |
| `topics.publish.right_reset` / `left_reset` | `std_msgs/Bool` |
| `topics.publish.emergency` | `std_msgs/Bool` |

**启动：**
```bash
python code/pico/main.py --config /path/to/pico_config.yaml
```

**操作说明：**
| 操作 | 功能 |
|---|---|
| 右手 Grip 按住 | 激活右臂控制 |
| 左手 Grip 按住 | 激活左臂控制 |
| Grip 连按两次 | 对应臂回零 |
| Trigger | 夹爪开合 |
| Y 键 | 紧急停止 |

## 致谢

感谢以下开源项目：

- [HighTorque-Robotics/Panthera-HT_SDK](https://github.com/HighTorque-Robotics/Panthera-HT_SDK) — Panthera 机械臂 SDK
- [zhigenzhao/isaaclab_xr_teleop](https://github.com/zhigenzhao/isaaclab_xr_teleop) — XR 手柄遥操作参考实现
- [JeanElsner/panda_mujoco](https://github.com/JeanElsner/panda_mujoco) — Panda 机械臂 MuJoCo 仿真
