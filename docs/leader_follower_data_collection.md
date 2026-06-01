# 主从遥操采数据操作手册

## 概述

### 系统架构

```
操作员双手握住 Leader 臂（重力补偿模式，可自由移动）
            │
            ▼
  leader_follower 程序（panthera 环境）
  ├── Leader 左臂 → 读关节位置/速度
  ├── Follower 左臂 ← 跟踪 Leader（PD 控制）
  ├── Leader 右臂 → 读关节位置/速度
  └── Follower 右臂 ← 跟踪 Leader（PD 控制）
            │
            ▼ 发布 ROS topic（via rosbridge）
  /follower_left/joint_states    /follower_right/joint_states
  /follower_left/end_effector    /follower_right/end_effector
            │
            ▼
  data_collector 节点（ros-humble 环境）
  ├── 相机 left   → /left/color/image_raw    (左腕部相机)
  ├── 相机 right  → /right/color/image_raw   (右腕部相机)
  ├── 相机 extern → /extern/color/image_raw  (外部全景相机)
  ├── Follower 左臂关节 + 末端位姿
  └── Follower 右臂关节 + 末端位姿
            │
            ▼
  脚踏板控制 开始/停止/保存/删除
```

### 需要的终端窗口

| 窗口 | 环境 | 内容 |
|---|---|---|
| tmux（由 `start.sh` 创建） | `ros-humble` | rosbridge + 3 路相机 + 脚踏板 |
| Terminal A | `panthera` | leader_follower 主从控制 |
| Terminal B | `ros-humble` | data_collector 采集节点 |

---

## 第零步：确认 config 路径

所有 config 中的绝对路径都需要指向本机实际位置。  
主要检查以下文件中的 `motor_config_path` / `leader` / `follower` 字段：

```bash
# 确认路径正确
grep -n "home\|path" configs/follower_leader.yaml
```

如果路径不对，修改 `configs/follower_leader.yaml` 中的 `leader` / `follower` 字段，  
以及 `configs/robot_param/` 目录下各 yaml 文件。

---

## 第一步：识别脚踏板并更新 `foot_pedal.yaml`

> **仅在以下情况需要执行**：首次使用、更换 USB 口、或换了新的脚踏板设备。  
> 如果脚踏板配置已确认正确可跳过此步。

### 背景

每个物理踏板在 Linux evdev 系统中有唯一的 `phys` 地址（格式如 `usb-0000:09:00.0-4.4.3/input0`）。  
同一品牌的多个踏板设备名相同（都叫 `KM-key08`），必须用 `phys` 来区分哪个踏板是哪个。  
**换 USB 口后 `phys` 会改变**，需要重新识别。

### 操作步骤

**第 1 步：运行识别脚本**

```bash
cd /path/to/ros
sudo python3 utils/identify_pedals.py
```

输出示例（找到 3 个设备时）：
```
Found 3 KM-key08 device(s):

  /dev/input/event18  phys=usb-0000:09:00.0-4.4.2/input0
  /dev/input/event19  phys=usb-0000:09:00.0-4.4.3/input0
  /dev/input/event20  phys=usb-0000:09:00.0-4.4.4/input0

Press any pedal to identify it (Ctrl+C to quit)...
```

**第 2 步：逐一踩下每个踏板，记录输出**

依次踩下**左踏板**、**中间踏板**、**右踏板**，每次踩下后终端打印：

```
  Pressed!
  path : /dev/input/event19
  phys : usb-0000:09:00.0-4.4.3/input0
  key  : KEY_SPACE
```

将三个踏板的 `phys` 和 `key` 记录下来：

| 踏板 | 功能 | 记录的 `phys` | 记录的 `key` |
|---|---|---|---|
| 左踏板 | delete（删除） | `usb-0000:09:00.0-4.4.?/input0` | `KEY_SPACE` |
| 中间踏板 | press（开始/停止） | `usb-0000:09:00.0-4.4.?/input0` | `KEY_SPACE` |
| 右踏板 | save（保存） | `usb-0000:09:00.0-4.4.?/input0` | `KEY_SPACE` |

按 `Ctrl+C` 退出脚本。

**第 3 步：更新 `configs/foot_pedal.yaml`**

将上面记录的 `phys` 填入对应条目的 `phys` 字段：

```yaml
rosbridge:
  host: "localhost"
  port: 9090

pedals:
  - device: "KM-key08"
    phys: "usb-0000:09:00.0-4.4.3/input0"   # ← 中间踏板的 phys（开始/停止录制）
    key: "KEY_SPACE"
    topic: "/foot_pedal/press"

  - device: "KM-key08"
    phys: "usb-0000:09:00.0-4.4.2/input0"   # ← 右踏板的 phys（保存）
    key: "KEY_SPACE"
    topic: "/foot_pedal/save"

  - device: "KM-key08"
    phys: "usb-0000:09:00.0-4.4.4/input0"   # ← 左踏板的 phys（删除）
    key: "KEY_SPACE"
    topic: "/foot_pedal/delete"
```

> **注意**：`device` 字段填设备名的子字符串（`KM-key08`），  
> `phys` 字段填完整的物理路径，两者配合使用才能精确定位到单个踏板。

**第 4 步：验证配置**

启动 rosbridge 后，单独运行脚踏板程序测试：

```bash
sudo /path/to/miniforge3/envs/panthera/bin/python \
    code/foot_pedal/main.py --config configs/foot_pedal.yaml
```

输出应为：
```
Connected to rosbridge ws://localhost:9090
Advertising /foot_pedal/press
Advertising /foot_pedal/save
Advertising /foot_pedal/delete
Foot pedals ready.
```

踩每个踏板时，终端应打印对应的 topic 名：
```
[KM-key08] key KEY_SPACE → /foot_pedal/press
```

若某踏板踩下无响应，检查 `phys` 是否填错（重新运行 `identify_pedals.py`）。

---

## 第二步：启动 rosbridge + 相机 + 脚踏板

**一条命令启动所有基础服务**：

```bash
cd /path/to/ros
./start.sh
```

`start.sh` 会创建一个 tmux session，各窗口内容：

| tmux 窗口编号 | 名称 | 启动内容 | 对应 config |
|---|---|---|---|
| 0 | rosbridge | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` | — |
| 1 | camera_1 | `ros2 run camera camera_publisher` | `configs/camera_1.yaml` → topic: `/left/color/image_raw` |
| 2 | camera_2 | `ros2 run camera camera_publisher` | `configs/camera_2.yaml` → topic: `/extern/color/image_raw` |
| 3 | camera_3 | `ros2 run camera camera_publisher` | `configs/camera_3.yaml` → topic: `/right/color/image_raw` |
| 5 | foot_pedal | `sudo python foot_pedal/main.py` | `configs/foot_pedal.yaml` |

**验证 rosbridge 已就绪**：tmux 窗口 0 应出现：
```
[INFO] Rosbridge WebSocket server started on port 9090
```

**验证相机已就绪**：
```bash
# 在另一个终端（conda activate ros-humble + source install/setup.zsh）
ros2 topic hz /left/color/image_raw
# 应输出约 30 Hz
```

> **注意**：脚踏板窗口（5）若提示 `Permission denied`，是 sudo 密码需要输入，切换到 tmux 窗口 5 后手动输入。

---

## 第三步：启动主从控制程序

**在 Terminal A 中**：

```bash
conda activate panthera
cd /path/to/ros/code/panthera/leader_follower
python main.py --config ../../../configs/follower_leader.yaml
```

**启动过程输出**（正常应看到）：
```
============================================================
Leader-Follower ROS Bridge
============================================================
[Init] 初始化 pair_1...
  Leader:   /path/to/Leader_left.yaml
  Follower: /path/to/Follower_left.yaml
  Topics:   {'joint_states': '/follower_left/joint_states', 'end_effector': '/follower_left/end_effector'}
[Init] 初始化 pair_2...
...
[Init] ✓ 共初始化 2 对主从臂
[ROS] ✓ 已连接
[Control] 控制循环启动，共 2 对主从臂
```

**此时**：
- Leader 臂处于**重力补偿 + 摩擦力补偿模式**，可以用手自由推动
- Follower 臂会**实时跟随** Leader 臂的关节位置


---

## 第四步：启动数据采集节点

**在 Terminal B 中**：

```bash
conda activate ros-humble
cd /path/to/ros
source code/ros_pkgs/install/setup.zsh
ros2 run data_collector data_collector_node --config configs/data_collection.yaml
```

**启动输出**（正常）：
```
[INFO] Base output directory: ./data/xxx
[INFO] Subscribed to '/left/color/image_raw' [sensor_msgs/Image] → name='left'
[INFO] Subscribed to '/right/color/image_raw' [sensor_msgs/Image] → name='right'
[INFO] Subscribed to '/extern/color/image_raw' [sensor_msgs/Image] → name='extern'
[INFO] Subscribed to '/follower_left/joint_states' [sensor_msgs/JointState] → name='left_joint'
[INFO] Subscribed to '/follower_left/end_effector' [geometry_msgs/PoseStamped] → name='left_ee'
[INFO] Subscribed to '/follower_right/joint_states' [sensor_msgs/JointState] → name='right_joint'
[INFO] Subscribed to '/follower_right/end_effector' [geometry_msgs/PoseStamped] → name='right_ee'
[INFO] Waiting for foot pedal on /foot_pedal/press. Press to START, press again to STOP.
```

---

## 第五步：录制数据

### 脚踏板操作

| 脚踏板 | ROS topic | 功能 |
|---|---|---|
| **中间踏板** | `/foot_pedal/press` | 第一次踩 = 开始录制；再踩 = 停止录制 |
| **右踏板** | `/foot_pedal/save` | 验证完毕后**保留**本条数据 |
| **左踏板** | `/foot_pedal/delete` | 验证完毕后**丢弃**本条数据 |

### 标准录制流程

```
1. 确保机械臂在起始姿态
2. 踩中间踏板 → 开始录制（Terminal B 打印 "Recording STARTED → ./data/xxx/20240101_120000"）
3. 操作 Leader 臂完成任务动作
4. 踩中间踏板 → 停止录制（打印快照数量 + 开始验证）
5. 等待验证报告打印完毕
6. 踩右踏板保存 / 踩左踏板删除
7. 等待 "Ready for next recording." 提示
8. 回到步骤 1 开始下一条
```

### 录制过程中的终端输出

```
[t=1704067200.123] Saved 7/7 topics.
[t=1704067200.190] Saved 7/7 topics.
...
```

每行代表一个快照（15 Hz 时约每 67ms 一行）。`7/7` 表示所有 topic 都有数据。  
如果出现 `6/7`，说明某个 topic 缓冲区为空（相机掉帧或机械臂未发布）。

---

## 第六步：录制后验证（自动）

每次停止录制后，`data_collector` 会自动运行验证并打印报告：

```
════════════════════════════════════════════════════════════
  Post-recording quality check
  Session: ./data/xxx/20240101_120000
════════════════════════════════════════════════════════════

Loaded 630 entries across 7 topic(s):
  extern                          90 snapshots
  left                            90 snapshots
  left_ee                         90 snapshots
  left_joint                      90 snapshots
  right                           90 snapshots
  right_ee                        90 snapshots
  right_joint                     90 snapshots

── Timestamp Alignment (vs primary: 'left') ──
  Topic                          mean(ms)    p95(ms)    max(ms)
  ──────────────────────────────────────────────────────────────
  extern                             3.21       8.10      12.43
  left_ee                            1.05       2.80       4.12
  left_joint                         1.05       2.80       4.12
  right                              3.98       9.20      15.67
  right_ee                           1.08       2.90       4.20
  right_joint                        1.08       2.90       4.20

── Frame Rate & Jitter ──
  Topic                          fps  median_ms    std_ms    max_ms   dropouts
  ────────────────────────────────────────────────────────────────────────────
  extern                       15.02      66.56      1.23     80.12
  left                         15.02      66.54      1.10     78.90
  left_ee                      15.02      66.54      0.85     71.20
  ...

── Trajectory Smoothness ──
  Joint states ('left_joint')  —  7 joints
  Joint               vel_rms    acc_rms   jerk_rms   jerk_peak
  ────────────────────────────────────────────────────────────────────────
  joint_1              0.1234     0.2345     0.1234      0.5678
  ...
```

### 验证指标说明

| 指标 | 正常值 | 含义 |
|---|---|---|
| `mean(ms)` 对齐误差 | < 10 ms | 各 topic 消息与基准 topic 的平均时间偏差 |
| `max(ms)` 对齐误差 | < 33 ms（1帧） | 最大时间偏差；超过 33ms 说明有一帧级别的时间差 |
| `fps` | 接近配置值 15.0 | 实际录制帧率 |
| `dropouts` 丢帧 | 0 | 相邻帧间隔 > 1.5× 中位间隔时计为丢帧 |
| `jerk_peak` 关节抖动 | < 10× `jerk_rms` | 峰值抖动过大（⚠）说明动作有突变 |

**根据报告决策**：
- 数据良好 → 踩**右踏板**保存
- 数据有问题（大量丢帧、严重抖动） → 踩**左踏板**删除，重录

---

## Config 详解

### `configs/follower_leader.yaml`（主从控制）

```yaml
rosbridge:
  host: "localhost"
  port: 9090

publish_hz: 100.0          # Follower 关节状态发布频率
                           # 建议 ≥ data_collection 录制频率的 5 倍（15Hz录→100Hz发布）

pairs:
  - name: "pair_1"
    leader:   "/path/to/configs/robot_param/Leader_left.yaml"    # 左 Leader 臂参数
    follower: "/path/to/configs/robot_param/Follower_left.yaml"  # 左 Follower 臂参数
    topics:
      publish:
        joint_states: "/follower_left/joint_states"   # ← 必须与 data_collection.yaml 一致
        end_effector: "/follower_left/end_effector"

  - name: "pair_2"
    leader:   "/path/to/configs/robot_param/Leader_right.yaml"
    follower: "/path/to/configs/robot_param/Follower_right.yaml"
    topics:
      publish:
        joint_states: "/follower_right/joint_states"  # ← 必须与 data_collection.yaml 一致
        end_effector: "/follower_right/end_effector"

control:
  # ── Follower 跟随增益（影响跟随刚度和稳定性）──
  kp: [10.0, 21.0, 21.0, 16.0, 13.0, 1.0]  # 关节位置增益（越大跟随越紧）
  kd: [1.0,   2.0,  2.0,  0.9,  0.8,  0.1]  # 关节速度增益（越大阻尼越强）
  gripper_kp: 4.0     # 夹爪位置增益
  gripper_kd: 0.4     # 夹爪速度增益
  
  # ── 摩擦力补偿（让 Leader 手感更轻盈）──
  friction:
    Fc: [0.15, 0.12, 0.12, 0.12, 0.04, 0.04]  # 库仑摩擦（启动摩擦补偿）
    Fv: [0.05, 0.05, 0.05, 0.03, 0.02, 0.02]  # 粘性摩擦（运动中摩擦补偿）
    vel_threshold: 0.02                          # 低于此速度视为静止（rad/s）
  
  # ── 力矩死区（过滤传感器噪声）──
  tor_threshold: [0.5, 1.0, 1.0, 0.5, 0.3, 0.3]  # 小于此力矩差不参与反馈（N·m）
```

**调参建议**：

| 场景 | 调整方向 |
|---|---|
| Follower 跟随滞后 | 适当增大 `kp` |
| Follower 振荡抖动 | 适当增大 `kd`，或减小 `kp` |
| Leader 推起来太沉 | 增大 `friction.Fc` / `Fv` |
| Leader 推起来抖动 | 减小 `friction.Fc` / `Fv` |

---

### `configs/data_collection.yaml`（数据采集）

```yaml
recording:
  frequency: 15.0           # 录制帧率（Hz）
                            # 15 Hz = 每条 6 秒的轨迹 ≈ 90 帧
  output_dir: ./data/任务名  # 修改为当前任务的文件夹名（如 close_top_drawer）
  buffer_size: 100          # 每 topic 内存缓冲帧数；100 帧 ≈ 6.7 秒历史
  foot_pedal_topic: /foot_pedal/press
  save_topic:   /foot_pedal/save
  delete_topic: /foot_pedal/delete

topics:
  # ── 相机（图像保存为 .npy，BGR uint8）──
  - name: left               # 左腕部相机
    topic: /left/color/image_raw
    msg_type: sensor_msgs/Image
    is_primary: true         # 以此 topic 的时间戳为对齐基准
                             # 建议选图像（30Hz）而非关节状态（100Hz），
                             # 高频 topic 在 15Hz 采集窗口内总有近邻帧

  - name: right              # 右腕部相机
    topic: /right/color/image_raw
    msg_type: sensor_msgs/Image

  - name: extern             # 外部全景相机
    topic: /extern/color/image_raw
    msg_type: sensor_msgs/Image

  # ── 机械臂状态（保存为 .json）──
  - name: left_joint
    topic: /follower_left/joint_states    # 必须与 follower_leader.yaml 一致
    msg_type: sensor_msgs/JointState

  - name: left_ee
    topic: /follower_left/end_effector
    msg_type: geometry_msgs/PoseStamped

  - name: right_joint
    topic: /follower_right/joint_states
    msg_type: sensor_msgs/JointState

  - name: right_ee
    topic: /follower_right/end_effector
    msg_type: geometry_msgs/PoseStamped

verify:
  enabled: true
  primary: left              # 对齐基准（与上面 is_primary 的 name 一致）
  alignment_warn_ms: 10      # 超过 10ms 的对齐误差打印警告
  dropout_threshold: 1.5     # 帧间隔 > 1.5× 中位间隔 = 丢帧（越严格越小）
  joint_topics:
    - left_joint
    - right_joint
  ee_topics:
    - left_ee
    - right_ee
  plot: false                # true 时录制完自动弹出图像回放和轨迹图
                             # （建议采集时关闭节省时间，验证时单独打开）
  gripper:
    joint_topics: [left_joint, right_joint]
    joint_name: gripper      # JointState 中夹爪关节的名字
    transition_factor: 5.0
```

**每次换任务时必须修改**：
1. `recording.output_dir` → 改为当前任务名（如 `./data/open_top_drawer`）

---

### `configs/camera_*.yaml`（相机）

| 文件 | `camera_name` | `serial_number` | 对应位置 |
|---|---|---|---|
| `camera_1.yaml` | `left` | `260322273623` | 左腕部 |
| `camera_2.yaml` | `extern` | `260322275556` | 外部全景 |
| `camera_3.yaml` | `right` | `260322272645` | 右腕部 |

> 查询相机序列号：`rs-enumerate-devices | grep Serial`

---

## 数据格式

录制结束后，数据保存在 `<output_dir>/<YYYYMMDD_HHMMSS>/` 下。

```
data/
└── open_top_drawer/
    ├── 20240101_120000/          ← 第 1 条轨迹（session）
    │   ├── 000001/               ← 第 1 帧快照
    │   │   ├── 1704067200123456789_left.npy        ← 左腕部图像
    │   │   ├── 1704067200123456789_right.npy       ← 右腕部图像
    │   │   ├── 1704067200123456789_extern.npy      ← 外部相机图像
    │   │   ├── 1704067200124100000_left_joint.json ← 左臂关节状态
    │   │   ├── 1704067200124100000_left_ee.json    ← 左臂末端位姿
    │   │   ├── 1704067200124100000_right_joint.json
    │   │   └── 1704067200124100000_right_ee.json
    │   ├── 000002/               ← 第 2 帧快照
    │   │   └── ...
    │   └── 000090/               ← 第 90 帧（约 6 秒 @ 15Hz）
    ├── 20240101_120115/          ← 第 2 条轨迹
    │   └── ...
    └── ...
```

**图像文件**（`.npy`）：`numpy.ndarray`，shape `(480, 640, 3)`，dtype `uint8`，BGR 编码。

```python
import numpy as np
img = np.load("1704067200123456789_left.npy")  # shape (480, 640, 3)
```

**关节状态文件**（`.json`）：JointState 消息的字典形式。

```json
{
  "header": {"stamp": {"sec": 1704067200, "nanosec": 124100000}, "frame_id": "base_link"},
  "name": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"],
  "position": [0.123, -0.456, 0.789, 0.012, -0.345, 0.678, 0.950],
  "velocity": [0.001, -0.002, 0.003, 0.000, -0.001, 0.002, 0.000],
  "effort": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}
```

**末端位姿文件**（`.json`）：PoseStamped 消息的字典形式。

```json
{
  "header": {"stamp": {"sec": 1704067200, "nanosec": 124100000}, "frame_id": "base_link"},
  "pose": {
    "position": {"x": 0.312, "y": 0.021, "z": 0.245},
    "orientation": {"x": 0.012, "y": 0.034, "z": 0.056, "w": 0.998}
  }
}
```

---

## 常见问题

### 主从控制问题

**Q：程序启动时报 "移动到安全位置失败"**  
- 检查机械臂是否上电、CAN 通信是否正常
- 确认 `motor_config_path` 路径正确

### 数据采集问题

**Q：采集节点打印 `5/7 topics`（有 topic 缺失）**  
- 大概率是某路相机未发布：检查 tmux 对应窗口是否有报错
- 检查相机序列号是否与实际连接的相机一致（`rs-enumerate-devices`）

**Q：对齐误差 `max(ms)` 很大（>50ms）**  
- 相机帧率是否稳定？检查 `ros2 topic hz /left/color/image_raw`
- 增大 `buffer_size`（让对齐器有更大的搜索窗口）
- 降低 `recording.frequency`（从 15 Hz 降到 10 Hz）

**Q：脚踏板不响应**  
- 检查 tmux 窗口 5（foot_pedal）是否在等待 sudo 密码
- 确认脚踏板 USB 已连接：`ls /dev/input/event*` 后用 `sudo evtest` 测试
- 确认 `foot_pedal.yaml` 中的 `phys` 字段与实际设备一致（换 USB 口后 phys 会变）

**Q：录制完验证时 `alignment_warn_ms` 全部有 ⚠**  
- 如果 `max(ms)` 普遍在 30ms 左右，属于正常（相机 30fps = 帧间隔 33ms）
- 超过 100ms 才需要关注，通常是某 topic 发布不稳定

### 退出与重启

**正常退出**：
```
Terminal A（leader_follower）：Ctrl+C
Terminal B（data_collector）：Ctrl+C
tmux session：Ctrl+b → 输入 kill-session
```

**异常退出后 Follower 臂不受控**：  
重新运行 `leader_follower/main.py`，它启动时会先将机械臂移回安全位置。

---

## 快速参考：完整启动命令

```bash
# === 终端 0（tmux）===
cd /path/to/ros
./start.sh
# 等待所有窗口启动完毕（约 5 秒）

# === Terminal A ===
conda activate panthera
cd /path/to/ros/code/panthera/leader_follower
python main.py --config ../../../configs/follower_leader.yaml

# === Terminal B ===
conda activate ros-humble
cd /path/to/ros
source code/ros_pkgs/install/setup.zsh
ros2 run data_collector data_collector_node --config configs/data_collection.yaml

# 之后用脚踏板控制录制
```
