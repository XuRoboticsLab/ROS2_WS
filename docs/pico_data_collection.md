# Pico 遥操采数据操作手册

## 概述

### 控制方式

Pico 4 Ultra 头显的双手控制器作为遥操作输入设备，通过以下方式控制机械臂：

- **位置控制**：以 Grip 按下时机械臂末端的位置为"校准基准"，之后 Pico 控制器相对于按下位置的偏移量，直接映射为机械臂末端相对于基准的位移目标（绝对偏移，非增量速度）。
- **夹爪**：B/A 键控制右夹爪张开/闭合，Y/X 键控制左夹爪张开/闭合。
- **复位**：Grip 双击 → 机械臂回到安全位置，需重新按下校准。
- **急停**：无实时急停按钮；Ctrl+C 退出程序时自动发布急停信号。

与主从遥操（leader_follower）的区别：主从靠物理拖动，位置跟随精度高但需要专用 Leader 臂；Pico 遥操完全无线，适合快速采集和远程遥操。

---

### 系统架构

```
[Pico 4 Ultra 头显] — XR SDK（xr 环境）
         │
         │  code/pico/main.py （configs/pico.yaml）
         │  读取双手控制器位姿 + 按键
         │  坐标变换 → 发布到 rosbridge
         │
         ▼
[rosbridge WebSocket  localhost:9090]
         │
         ├─── /joystick/right/cmd      (Twist: 右手相对校准基准的偏移)
         ├─── /joystick/right/gripper  (Int8: 夹爪指令)
         ├─── /joystick/right/reset    (Bool: 复位)
         ├─── /joystick/right/init     (Bool: 校准基准)
         ├─── /joystick/left/cmd       (左手同理)
         ├─── /joystick/left/...
         └─── /joystick/emergency      (Bool: 急停)
         │
         ├── code/panthera/pico_control/main.py  ← 左臂（panthera 环境）
         │   configs/pico_control_left.yaml
         │   订阅 /joystick/left/*，发布 /follower_left/*
         │
         └── code/panthera/pico_control/main.py  ← 右臂（panthera 环境）
             configs/pico_control_right.yaml
             订阅 /joystick/right/*，发布 /follower_right/*
                      │
                      ▼
         /follower_left/joint_states    /follower_right/joint_states
         /follower_left/end_effector    /follower_right/end_effector
                      │
                      ▼
         data_collector_node（ros-humble 环境）
         ├── 相机 left   → /left/color/image_raw    (左腕部)
         ├── 相机 right  → /right/color/image_raw   (右腕部)
         ├── 相机 extern → /extern/color/image_raw  (外部全景)
         ├── 左臂关节 + 末端位姿
         └── 右臂关节 + 末端位姿
                      │
                      ▼
         脚踏板  开始 / 停止 / 保存 / 删除
```

### 需要的终端窗口

| 窗口 | 环境 | 内容 |
|---|---|---|
| tmux（由 `start.sh` 创建） | `ros-humble` | rosbridge + 3 路相机 + 脚踏板 |
| Terminal A | `panthera` | pico_control 左臂 |
| Terminal B | `panthera` | pico_control 右臂 |
| Terminal C | `ros-humble` | data_collector 采集节点 |
| Terminal D | `xr` | pico/main.py Pico 发布端 |

> 若只采集单臂，省略对应的 pico_control 实例和 data_collection.yaml 中的对应 topic。

---

## 第零步：确认 config 路径

pico_control 和 data_collection 的 config 都含有绝对路径，需确认指向本机实际位置：

```bash
# 检查 pico_control 的 motor_config_path
grep "motor_config_path" configs/pico_control_left.yaml configs/pico_control_right.yaml

# 检查 leader/follower 路径（如有其他 config 引用）
grep -n "home\|path" configs/pico_control_left.yaml configs/pico_control_right.yaml
```

如果路径不对，修改 `configs/pico_control_left.yaml` 和 `configs/pico_control_right.yaml` 中的 `motor_config_path` 字段。

---

## 第一步：识别脚踏板并更新 `foot_pedal.yaml`

> **仅在首次使用、更换 USB 口、或换了新脚踏板时需要执行。**  
> 脚踏板已配置正确可跳过。

每个物理踏板在 Linux evdev 系统中有唯一的 `phys` 地址，同一品牌多个踏板必须用 `phys` 区分。

**第 1 步：运行识别脚本**

```bash
cd /path/to/ros
sudo python3 utils/identify_pedals.py
```

**第 2 步：逐一踩下每个踏板，记录输出**

```
  Pressed!
  path : /dev/input/event19
  phys : usb-0000:09:00.0-4.4.3/input0
  key  : KEY_SPACE
```

记录左/中/右踏板各自的 `phys` 值：

| 踏板 | 功能 | `phys` |
|---|---|---|
| 中间踏板 | press（开始/停止录制） | `usb-0000:09:00.0-4.4.?/input0` |
| 右踏板 | save（保存） | `usb-0000:09:00.0-4.4.?/input0` |
| 左踏板 | delete（删除） | `usb-0000:09:00.0-4.4.?/input0` |

**第 3 步：更新 `configs/foot_pedal.yaml`**

```yaml
rosbridge:
  host: "localhost"
  port: 9090

pedals:
  - device: "KM-key08"
    phys: "usb-0000:09:00.0-4.4.3/input0"   # 中间踏板
    key: "KEY_SPACE"
    topic: "/foot_pedal/press"

  - device: "KM-key08"
    phys: "usb-0000:09:00.0-4.4.2/input0"   # 右踏板
    key: "KEY_SPACE"
    topic: "/foot_pedal/save"

  - device: "KM-key08"
    phys: "usb-0000:09:00.0-4.4.4/input0"   # 左踏板
    key: "KEY_SPACE"
    topic: "/foot_pedal/delete"
```

---

## 第二步：启动 rosbridge + 相机 + 脚踏板

```bash
cd /path/to/ros
./start.sh
```

`start.sh` 创建 tmux session，各窗口：

| tmux 窗口 | 内容 | 对应 config |
|---|---|---|
| 0 - rosbridge | `ros2 launch rosbridge_server ...` | — |
| 1 - camera_1 | 左腕部相机 | `configs/camera_1.yaml` → `/left/color/image_raw` |
| 2 - camera_2 | 外部全景相机 | `configs/camera_2.yaml` → `/extern/color/image_raw` |
| 3 - camera_3 | 右腕部相机 | `configs/camera_3.yaml` → `/right/color/image_raw` |
| 5 - foot_pedal | 脚踏板监听 | `configs/foot_pedal.yaml` |

**验证 rosbridge 就绪**：tmux 窗口 0 出现：
```
[INFO] Rosbridge WebSocket server started on port 9090
```

**验证相机就绪**（另开终端）：
```bash
conda activate ros-humble
source code/ros_pkgs/install/setup.zsh
ros2 topic hz /left/color/image_raw   # 应约 30 Hz
```

> 脚踏板窗口（5）若提示 `Permission denied`，切换到 tmux 窗口 5 手动输入 sudo 密码。

---

## 第三步：启动 pico_control（Panthera 端）

左/右臂各开一个终端：

**Terminal A（左臂）**：
```bash
conda activate panthera
cd /path/to/ros/code/panthera/pico_control
python main.py --config ../../../configs/pico_control_left.yaml
```

**Terminal B（右臂）**：
```bash
conda activate panthera
cd /path/to/ros/code/panthera/pico_control
python main.py --config ../../../configs/pico_control_right.yaml
```

**启动过程正常输出**：
```
============================================================
Panthera Pico Position Control 节点
============================================================
[Init] 初始化机械臂...
[Init] 移动到安全位置...
[Init] ✓ 已到达安全位置
[Init] 初始末端位置: ['0.312', '-0.021', '0.245']
[Init] 请按 Pico init 键完成校准，之后即可开始位置控制
[ROS] 连接 localhost:9090...
[ROS] ✓ 已连接
[ROS] 已订阅 /joystick/right/cmd, /joystick/right/gripper, /joystick/right/reset, /joystick/right/init
[Control] 控制循环启动，频率 50 Hz
[Control] 等待 Pico init 信号以完成校准...
```

**此时机械臂处于安全位置，以重力补偿保持姿态，等待 Pico 校准信号。**

---

## 第四步：启动数据采集节点

**Terminal C**：
```bash
conda activate ros-humble
cd /path/to/ros
source code/ros_pkgs/install/setup.zsh
ros2 run data_collector data_collector_node --config configs/data_collection.yaml
```

**正常输出**：
```
[INFO] Base output directory: ./data/xxx
[INFO] Subscribed to '/left/color/image_raw'  → name='left'
[INFO] Subscribed to '/right/color/image_raw' → name='right'
[INFO] Subscribed to '/extern/color/image_raw' → name='extern'
[INFO] Subscribed to '/follower_left/joint_states'  → name='left_joint'
[INFO] Subscribed to '/follower_left/end_effector'  → name='left_ee'
[INFO] Subscribed to '/follower_right/joint_states' → name='right_joint'
[INFO] Subscribed to '/follower_right/end_effector' → name='right_ee'
[INFO] Waiting for foot pedal on /foot_pedal/press. Press to START, press again to STOP.
```

---

## 第五步：启动 Pico 发布端

**Terminal D**：
```bash
conda activate xr
cd /path/to/ros/code/pico
python main.py --config ../../configs/pico.yaml
```

戴上 Pico 4 Ultra 头显，确认控制器已连接。

---

## 第六步：校准（每次录制前必须执行）

> **这是与主从遥操最关键的区别**：Pico 位置控制以某一参考姿态为"零点"，校准前手柄发出的 cmd 信号会被丢弃。

### 校准步骤

1. 将机械臂摆放到合适的**起始姿态**（可通过此前的 Grip 控制或手动摆放）
2. **右手 Grip 单击** → 右臂接收到 `init` 信号，记录当前末端位姿为基准
3. **左手 Grip 单击** → 左臂同理
4. Terminal A / B 出现：
   ```
   [State] 校准基准已更新
   ```
5. 校准完成，Grip 按住即可控制机械臂

### 控制手势说明

| Pico 操作 | 效果 |
|---|---|
| Grip **按下**（单击） | 激活控制 + 记录当前末端位姿为校准基准 |
| Grip **按住** | 控制末端跟随手柄偏移（相对于校准基准） |
| Grip **松开** | 停止发布指令，机械臂保持当前位置（hold 模式） |
| Grip **双击** | 机械臂复位到安全位置，需重新按下校准 |
| B 键（右手） | 右夹爪张开 |
| A 键（右手） | 右夹爪闭合 |
| Y 键（左手） | 左夹爪张开 |
| X 键（左手） | 左夹爪闭合 |
| Ctrl+C | 退出发布端，自动发布急停信号 |

> **注意**：Grip 松开后超过 `watchdog_timeout_s`（默认 0.5 s）无 cmd 信号，机械臂自动切换到重力补偿保持模式（不回零，原地保持）。重新 Grip 按住即可继续控制，无需重新校准。

---

## 第七步：录制数据

### 脚踏板操作

| 脚踏板 | ROS topic | 功能 |
|---|---|---|
| **中间踏板** | `/foot_pedal/press` | 第一次踩 = 开始录制；再踩 = 停止录制 |
| **右踏板** | `/foot_pedal/save` | 验证后**保留**本条数据 |
| **左踏板** | `/foot_pedal/delete` | 验证后**丢弃**本条数据 |

### 标准录制流程

```
1. 将机械臂摆放到任务起始姿态
2. Grip 单击校准（确认 [State] 校准基准已更新）
3. 踩中间踏板 → 开始录制（Terminal C 打印 "Recording STARTED"）
4. Grip 按住遥操机械臂完成任务动作
5. 踩中间踏板 → 停止录制
6. 等待验证报告打印完毕
7. 踩右踏板保存 / 踩左踏板删除
8. 等待 "Ready for next recording." 提示
9. 将机械臂摆回起始姿态，重新校准，回到步骤 3
```

### 录制过程中的输出

Terminal C 每帧打印：
```
[t=1704067200.123] Saved 7/7 topics.
```
`7/7` 表示全部 topic 均有数据。出现 `6/7` 说明某相机掉帧或机械臂未发布。

---

## 第八步：录制后验证（自动）

停止录制后 `data_collector` 自动打印质量报告：

```
════════════════════════════════════════════════════════════
  Post-recording quality check
  Session: ./data/xxx/20240101_120000
════════════════════════════════════════════════════════════

Loaded 630 entries across 7 topic(s):
  extern       90 snapshots
  left         90 snapshots
  left_ee      90 snapshots
  left_joint   90 snapshots
  right        90 snapshots
  right_ee     90 snapshots
  right_joint  90 snapshots

── Timestamp Alignment (vs primary: 'left') ──
  Topic       mean(ms)  p95(ms)  max(ms)
  extern          3.21     8.10    12.43
  left_ee         1.05     2.80     4.12
  ...

── Frame Rate & Jitter ──
  Topic       fps   median_ms   std_ms   dropouts
  left       15.02      66.54     1.10          0
  ...

── Trajectory Smoothness ──
  Joint       vel_rms   acc_rms   jerk_rms   jerk_peak
  joint_1      0.1234    0.2345     0.1234      0.5678
  ...
```

### 验证指标

| 指标 | 正常值 | 含义 |
|---|---|---|
| `mean(ms)` 对齐误差 | < 10 ms | 各 topic 与基准 topic 的平均时间偏差 |
| `max(ms)` 对齐误差 | < 33 ms（1 帧） | 最大时间偏差；超过 33ms 有一帧级别的时间差 |
| `fps` | 接近配置值 15.0 | 实际录制帧率 |
| `dropouts` 丢帧 | 0 | 相邻帧间隔 > 1.5× 中位间隔时计为丢帧 |
| `jerk_peak` 关节抖动 | < 10× `jerk_rms` | 峰值过大说明有突变（Pico 信号丢失或奇异点跳变） |

Pico 遥操特有的抖动来源：
- **Grip 松开瞬间**：watchdog 超时切换到 hold 模式有约 0.5 s 的静止段，jitter 正常
- **奇异点附近**：IK 无解时 `joint_pos is None`，control_loop 自动 hold，可能出现短暂位置锁定
- **坐标系映射**：如果 jerk_peak 异常大，检查 `pico.yaml` 中 `coord_mapping` 和 `rot_sign` 设置

**根据报告决策**：
- 数据良好 → 踩**右踏板**保存
- 有大量丢帧或严重抖动 → 踩**左踏板**删除，重录

---

## Config 详解

### `configs/pico.yaml`（Pico 发布端）

```yaml
rosbridge:
  host: "localhost"
  port: 9090

grip:
  threshold: 0.5           # Grip 模拟轴超过此值视为"按下"
  double_tap_window_s: 0.4 # 两次 Grip 在此时间窗内视为双击（触发复位）

scale:
  translation_m: 1.0       # XR 位移 → Twist linear 缩放（与 pico_control.twist_scale 共同决定末端灵敏度）
  rotation_rad:  1.0       # XR 旋转 → Twist angular 缩放
  max_delta_pos_m: 0.3     # 离校准点最大位移幅值（超出则截断，防止大幅晃动）

deadzone:
  pos_m:   0.008           # 平移死区半径（m）；手的微小抖动在此范围内被过滤
  rot_rad: 0.03            # 旋转死区（rad ≈ 1.7°）

filter:
  alpha: 0.08              # EMA 低通滤波系数；越小越平滑但延迟越大

coord_mapping:
  pos: [[0, 0, -1],        # 机械臂 X ← -XR Z（向前）
        [-1, 0, 0],        # 机械臂 Y ← -XR X（向左）
        [0, 1, 0]]         # 机械臂 Z ←  XR Y（向上）
  rot: [[0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0]]
  rot_sign: [1, 1, 1]      # 各旋转轴方向符号；某轴旋转方向反了改为 -1

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
| `scale.translation_m` | Pico 端缩放；实际末端灵敏度 = pico scale × pico_control twist_scale（两端相乘） |
| `scale.max_delta_pos_m` | 离校准点的绝对位移上限（m）；超出则截断幅值但保留方向，防止手柄大幅晃动 |
| `deadzone.pos_m` | 过滤手柄静止时的微小抖动；太大会导致小幅运动无响应 |
| `filter.alpha` | EMA 系数 0.08：约 12 帧平滑（响应约 80ms）；增大可减少延迟但增加抖动 |
| `coord_mapping` | 将 XR 世界坐标系映射到机械臂基坐标系；调试时若某轴运动方向相反，修改 `rot_sign` 对应项为 -1 |

---

### `configs/pico_control_left.yaml` / `pico_control_right.yaml`（Panthera 控制端）

```yaml
motor_config_path: "/path/to/configs/robot_param/Follower_left.yaml"
                        # 左臂用 Follower_left.yaml，右臂用 Follower_right.yaml

rosbridge:
  host: "localhost"
  port: 9090

rates:
  control_hz: 50.0          # 控制循环频率（Hz）；越高末端追踪越精确
  publish_hz: 100.0         # 关节状态发布频率；应 ≥ data_collection 录制频率的 5 倍
  watchdog_timeout_s: 0.5   # Grip 松开超过此时间无 cmd → 切换到 hold 模式

ik_control:
  kp: [30.0, 50.0, 60.0, 25.0, 15.0, 10.0]  # 各关节位置增益
  kd: [ 3.0,  5.0,  6.0,  2.5,  1.5,  1.0]  # 各关节速度增益（阻尼）

ik_params:
  max_joint_step_rad: 0.15  # 单控制周期关节位移上限（rad）
                             # 防止 IK 在奇异点附近产生大幅关节跳变

safe_position:
  joint_pos: [0.0, 0.5, 0.6, 0.0, 0.0, 0.0]
  joint_vel: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

twist_scale:
  translation_m: 0.7  # Pico 偏移量 → 末端位移缩放
                       # 最终末端灵敏度 = pico.scale.translation_m × 此值
  rotation_rad: 1.0   # Pico 旋转偏移 → 末端旋转缩放

smoothing:
  max_linear_vel_m_s:    0.2  # 末端线速度上限（m/s）；超过则按方向截断
  max_angular_vel_rad_s: 0.5  # 末端角速度上限（rad/s）
  tracking_gain_hz:      40   # smooth target P 增益；越大响应越快，越小越平滑
  damping_ratio:         0.6  # D/P 比；<1 欠阻尼（有轻微超调但响应快）

joints:
  names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]

topics:
  subscribe:
    cmd:     "/joystick/left/cmd"     # 右臂改为 /joystick/right/cmd
    gripper: "/joystick/left/gripper"
    reset:   "/joystick/left/reset"
    init:    "/joystick/left/init"
  publish:
    joints:  "/follower_left/joint_states"   # 右臂改为 /follower_right/*
    ee:      "/follower_left/end_effector"
```

**调参建议**：

| 场景 | 调整方向 |
|---|---|
| 末端跟随太慢 | 增大 `smoothing.tracking_gain_hz`（如 40→60）或 `smoothing.max_linear_vel_m_s` |
| 末端抖动/振荡 | 减小 `tracking_gain_hz`，或增大 `damping_ratio`（向 1.0 靠拢） |
| 末端对手柄不够灵敏 | 增大 `twist_scale.translation_m`（如 0.7→1.0）或 pico.yaml 的 `scale.translation_m` |
| 末端太灵敏（难控制） | 减小 `twist_scale.translation_m` |
| IK 奇异点附近飞车 | 减小 `ik_params.max_joint_step_rad`（如 0.15→0.10）|
| 关节振荡 | 减小 `ik_control.kp`，增大 `ik_control.kd` |

---

### `configs/data_collection.yaml`（数据采集）

```yaml
recording:
  frequency: 15.0           # 录制帧率（Hz）；与相机 30Hz 相比降半采样
  output_dir: ./data/任务名  # 修改为当前任务文件夹名（如 ./data/close_top_drawer）
  buffer_size: 100
  foot_pedal_topic: /foot_pedal/press
  save_topic:   /foot_pedal/save
  delete_topic: /foot_pedal/delete

topics:
  - name: left
    topic: /left/color/image_raw
    msg_type: sensor_msgs/Image
    is_primary: true         # 以左腕部相机时间戳为对齐基准

  - name: right
    topic: /right/color/image_raw
    msg_type: sensor_msgs/Image

  - name: extern
    topic: /extern/color/image_raw
    msg_type: sensor_msgs/Image

  - name: left_joint
    topic: /follower_left/joint_states    # 必须与 pico_control_left.yaml publish 一致
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
  primary: left
  alignment_warn_ms: 10
  dropout_threshold: 1.5
  joint_topics: [left_joint, right_joint]
  ee_topics: [left_ee, right_ee]
  plot: false
```

> **每次换任务只需修改 `recording.output_dir`**。

---

## 数据格式

录制数据保存在 `<output_dir>/<YYYYMMDD_HHMMSS>/` 下：

```
data/
└── close_top_drawer/
    ├── 20240101_120000/          ← 第 1 条轨迹
    │   ├── 000001/               ← 第 1 帧快照
    │   │   ├── <timestamp>_left.npy         ← 左腕部图像
    │   │   ├── <timestamp>_right.npy        ← 右腕部图像
    │   │   ├── <timestamp>_extern.npy       ← 外部相机图像
    │   │   ├── <timestamp>_left_joint.json  ← 左臂关节状态
    │   │   ├── <timestamp>_left_ee.json     ← 左臂末端位姿
    │   │   ├── <timestamp>_right_joint.json
    │   │   └── <timestamp>_right_ee.json
    │   ├── 000002/ ...
    │   └── 000090/               ← 第 90 帧（6 s @ 15 Hz）
    └── 20240101_120115/          ← 第 2 条轨迹
```

**关节状态**（`.json`）：
```json
{
  "name": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"],
  "position": [0.123, -0.456, 0.789, 0.012, -0.345, 0.678, 0.950],
  "velocity": [0.001, -0.002, 0.003, 0.000, -0.001, 0.002, 0.000],
  "effort": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}
```

**末端位姿**（`.json`）：
```json
{
  "pose": {
    "position":    {"x": 0.312, "y": 0.021, "z": 0.245},
    "orientation": {"x": 0.012, "y": 0.034, "z": 0.056, "w": 0.998}
  }
}
```

---

## 常见问题

### 校准 / 控制问题

**Q：Grip 单击后 Terminal A/B 没有打印 `[State] 校准基准已更新`**
- 确认 Pico 发布端（Terminal D）正在运行且已连接 rosbridge
- 检查 `pico.yaml` 中 `topics.publish.right_init` / `left_init` 与 `pico_control_right.yaml` 中 `topics.subscribe.init` 是否一致

**Q：Grip 按住但机械臂不动**
- 先确认 `[State] 校准基准已更新` 已打印（未校准则 cmd 被丢弃）
- 检查 Terminal A/B 是否有 `[Control] 等待 Pico init 信号以完成校准...` 仍在等待

**Q：机械臂运动方向和预期相反**
- 修改 `pico.yaml` 中 `coord_mapping.rot_sign` 对应轴改为 -1
- 或调整 `coord_mapping.pos` 矩阵中对应行的符号

**Q：机械臂在奇异点附近突然大幅跳动**
- 减小 `ik_params.max_joint_step_rad`（如 0.15→0.10）
- 避免将机械臂操作到关节极限附近

**Q：Grip 松开后机械臂缓慢漂移**
- watchdog 超时后会切换到 hold 模式（重力补偿保持）；如果漂移说明重力补偿有误差
- 检查 `motor_config_path` 指向的机器人参数文件中重力补偿参数

### 数据采集问题

**Q：采集节点打印 `5/7 topics`（有 topic 缺失）**
- 大概率某路相机未发布：检查 tmux 对应窗口是否报错
- `ros2 topic hz /left/color/image_raw` 确认相机在发布
- 检查相机序列号是否与实际设备一致（`rs-enumerate-devices | grep Serial`）

**Q：关节 topic 缺失（`left_joint` 或 `right_joint` 为 0）**
- 确认 pico_control 已启动且已完成校准
- `ros2 topic hz /follower_left/joint_states` 确认发布频率

**Q：验证报告 `jerk_peak` 异常大**
- Pico 遥操时 Grip 松开瞬间有一段静止期（watchdog 超时）属正常
- 如果 jerk_peak 持续很大，考虑减小 `smoothing.tracking_gain_hz` 或增大 `ik_params.max_joint_step_rad` 反而会增大跳变——应减小

**Q：脚踏板不响应**
- 切换到 tmux 窗口 5 确认 foot_pedal 进程正在运行且无报错
- `sudo evtest` 测试踏板是否有物理信号
- 确认 `foot_pedal.yaml` 的 `phys` 字段与实际一致（换 USB 口后 phys 会变）

### 退出与重启

**正常退出**：
```
Terminal A/B（pico_control）：Ctrl+C → 程序自动返回安全位置
Terminal C（data_collector）：Ctrl+C
Terminal D（pico/main.py）：Ctrl+C
tmux session：Ctrl+b → 输入 kill-session
```

**pico_control 异常退出后机械臂不受控**：
重新运行 `pico_control/main.py`，启动时自动移回安全位置并重新等待校准。

---

## 快速参考：完整启动命令

```bash
# === 终端 0（tmux）===
cd /path/to/ros
./start.sh
# 等待各窗口启动完毕（约 5 秒）

# === Terminal A（左臂）===
conda activate panthera
cd /path/to/ros/code/panthera/pico_control
python main.py --config ../../../configs/pico_control_left.yaml

# === Terminal B（右臂）===
conda activate panthera
cd /path/to/ros/code/panthera/pico_control
python main.py --config ../../../configs/pico_control_right.yaml

# === Terminal C（数据采集）===
conda activate ros-humble
cd /path/to/ros
source code/ros_pkgs/install/setup.zsh
ros2 run data_collector data_collector_node --config configs/data_collection.yaml

# === Terminal D（Pico 发布端）===
conda activate xr
cd /path/to/ros/code/pico
python main.py --config ../../configs/pico.yaml

# === 操作流程 ===
# 1. 戴上 Pico 头显，右手 Grip 单击校准右臂，左手 Grip 单击校准左臂
# 2. Grip 按住遥操机械臂到任务起始姿态
# 3. 踩中间踏板 → 开始录制
# 4. Grip 按住完成任务动作
# 5. 踩中间踏板 → 停止录制
# 6. 等待验证报告 → 踩右踏板保存 / 踩左踏板删除
# 7. 重复步骤 2-6
```
