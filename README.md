# 基于OpenCV与MediaPipe的无人机手势识别控制系统

<p align="center">
  <b>🚁  lightweight UAV Gesture Control System  🖐️</b><br>
  <i>基于视觉感知的低成本无人机非接触式控制方案</i>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.8%2B-blue" alt="Python">
  <img src="https://img.shields.io/badge/OpenCV-4.x-green" alt="OpenCV">
  <img src="https://img.shields.io/badge/MediaPipe-0.10%2B-orange" alt="MediaPipe">
  <img src="https://img.shields.io/badge/ROS-Noetic-purple" alt="ROS">
  <img src="https://img.shields.io/badge/PX4-v1.13.0-red" alt="PX4">
  <img src="https://img.shields.io/badge/Hardware-Orange%20Pi%203B-yellow" alt="Orange Pi">
</p>

---

## 📖 项目简介

本项目设计并实现了一套**轻量化、低功耗的无人机手势识别控制系统**，旨在解决传统无人机控制方式存在的携带不便、操作门槛高、场景适配差等痛点问题。

### 🎯 核心目标
- **零额外设备**：摒弃专用遥控器，仅通过手势即可操控无人机
- **低算力适配**：针对ARM架构嵌入式设备优化，无需高性能GPU
- **高实时响应**：端到端指令延迟 ≤ 250ms，满足实时操控需求
- **低成本部署**：可直接运行于消费级、教育级无人机平台

### ✨ 主要创新点
1. **非模型化手势识别算法**：摒弃传统SVM等机器学习模型，基于MediaPipe手部21个关键点，通过自定义几何算法（关节角度计算、向量方向分析）实现高效手势分类
2. **全链路硬件架构**：以香橙派3B为核心，构建"感知-计算-控制"完整物理链路
3. **ROS分布式通信**：实现边缘端（香橙派）与仿真/实机端（虚拟机/飞控）的跨平台协同
4. **工业级自动化部署**：一键启动脚本实现仿真环境快速初始化与进程生命周期管理

---

## 🏗️ 系统架构

### 硬件架构
```
┌─────────────────────────────────────────────────────────────┐
│                    无人机手势控制系统                          │
│                      Hardware Architecture                    │
└─────────────────────────────────────────────────────────────┘

  ┌──────────────┐         USB 2.0          ┌──────────────┐
  │   USB摄像头   │◄──────────────────────►│  香橙派 3B   │
  │  (640×480)   │    视频采集 /dev/video3  │ ARM Cortex-  │
  └──────────────┘                         │ A53 四核1.6G │
                                           │ 4GB DDR3     │
                                           └──────┬───────┘
                                                  │
                                           USB-TTL│Serial
                                                  │
                                           ┌──────▼───────┐
                                           │  PX4 飞控    │
                                           │ (Pixhawk 4)  │
                                           │ MAVLink协议  │
                                           └──────┬───────┘
                                                  │
                                           ┌──────▼───────┐
                                           │   无人机    │
                                           │  (Iris/实机) │
                                           └──────────────┘
```

### 软件架构
```
┌─────────────────────────────────────────────────────────────┐
│                    系统软件架构                                │
│                   Software Architecture                       │
└─────────────────────────────────────────────────────────────┘

  ┌──────────────────┐    ┌──────────────────┐
  │   虚拟机端 (VM)   │    │  香橙派端 (Edge) │
  │  192.168.1.11    │◄──►│  192.168.1.12   │
  │  ROS Master      │    │  ROS Node       │
  └────────┬─────────┘    └────────┬─────────┘
           │                       │
  ┌────────▼─────────┐    ┌────────▼─────────┐
  │ PX4 SITL + Gazebo│    │ 摄像头图像采集    │
  │ MAVROS 通信节点   │    │ (OpenCV + V4L2) │
  └────────┬─────────┘    └────────┬─────────┘
           │                       │
           │    ┌──────────────────┘
           │    │
  ┌────────▼────▼─────────┐
  │   ROS Topic 通信      │
  │ /mavros/setpoint_raw/ │
  │       /local          │
  └────────────────────────┘

  ┌──────────────────────────────────────────────────────────┐
  │                 手势识别核心流程                          │
  │                                                          │
  │  摄像头采集 ──► 镜像翻转 ──► BGR转RGB ──► MediaPipe     │
  │                                                          │
  │  手部检测 ──► 21关键点提取 ──► 滑动窗口平滑 ──► 手势识别  │
  │                                                          │
  │  右手食指指向: 8方向 ──► 无人机移动(WASD)                │
  │  左手张开/握拳: 状态 ──► 起飞/降落                        │
  └──────────────────────────────────────────────────────────┘
```

---

## 📁 项目结构

```
Research-on-UAV-Gesture-Recognition-and-Control-System-Based-on-OpenCV-and-MediaPipe/
├── orangepi_control/          # 香橙派端手势控制程序
│   ├── main.py               # 主程序入口
│   ├── config.py             # 系统配置文件
│   ├── gesture_recognizer.py # 手势识别核心算法
│   ├── landmark_smoother.py  # 关键点滑动窗口平滑滤波
│   ├── drone_controller.py   # MAVROS无人机控制器
│   └── start_orangepi.sh     # 香橙派一键启动脚本
│
├── vm_control/               # 虚拟机端仿真控制程序
│   ├── start_px4_sitl.sh     # PX4 SITL + Gazebo 一键启动脚本
│   └── drone_control.py      # 虚拟机端无人机控制测试脚本
│
└── README.md                 # 项目说明文档
```

---

## 🤖 手势控制说明

### 支持的手势

| 手势 | 说明 | 无人机动作 | 等效按键 |
|------|------|-----------|---------|
| ☝️ **右手食指向上** | 食指伸直，指向摄像头上方 | 无人机前进 | `W` |
| ☝️ **右手食指向下** | 食指伸直，指向摄像头下方 | 无人机后退 | `S` |
| ☝️ **右手食指向左** | 食指伸直，指向摄像头左侧 | 无人机左移 | `A` |
| ☝️ **右手食指向右** | 食指伸直，指向摄像头右侧 | 无人机右移 | `D` |
| ✋ **左手张开** | 五指全部伸直 | 解锁 + 悬停2米 | `1` |
| ✊ **左手握拳** | 五指全部弯曲 | 降落 | `3` |
| ☝️ **左手食指向上** | 左手食指伸直向上 | 上升高度 | `高度+` |
| ☝️ **左手食指向下** | 左手食指伸直向下 | 下降高度 | `高度-` |

### 操作要点
- **手势保持时间**：所有手势需保持 **≥ 0.4秒** 才会触发指令，防止误触发
- **持续移动**：保持指向手势时，每 **0.3秒** 移动一步（0.5米/步）
- **高度限制**：最低 **0.5米**，最高 **5米**，防止撞地或飞丢

---

## 🛠️ 环境搭建

### 硬件需求

| 组件 | 型号 | 说明 |
|------|------|------|
| 核心板 | 香橙派 3B | ARM Cortex-A53 四核 1.6GHz, 4GB DDR3 |
| 摄像头 | USB摄像头 | 640×480分辨率，固定焦距，设备节点 `/dev/video3` |
| 飞控 | Pixhawk 4 | PX4固件，通过USB-TTL串口连接 |
| 仿真主机 | Ubuntu 20.04 VM | Intel i5-12400/16GB RAM（仿真测试用） |

### 香橙派端环境配置

#### 1. 系统初始化
```bash
# 更新系统包
sudo apt-get update && sudo apt-get upgrade -y

# 安装基础依赖
sudo apt-get install -y python3-pip python3-dev build-essential \
    libopencv-dev v4l-utils
```

#### 2. 安装核心库
```bash
# OpenCV (Python)
pip3 install opencv-python

# MediaPipe (需从源码编译，禁用GPU加速)
# 详见论文第4.2.2节 MediaPipe源码级定制与交叉编译
pip3 install mediapipe

# ROS Noetic (根据官方文档安装)
# http://wiki.ros.org/noetic/Installation/Ubuntu

# MAVROS
sudo apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras
```

#### 3. 网络配置
```bash
# 设置静态IP（香橙派）
# 编辑 /etc/netplan/01-netcfg.yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.12/24]
      gateway4: 192.168.1.1
```

### 虚拟机端环境配置

#### 1. 安装ROS Noetic
按照[官方文档](http://wiki.ros.org/noetic/Installation/Ubuntu)完成安装。

#### 2. 安装PX4-Autopilot
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.13.0
bash ./Tools/setup/ubuntu.sh
make px4_sitl_default gazebo
```

#### 3. 配置Gazebo环境
```bash
# 添加到 ~/.bashrc
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
    ~/PX4-Autopilot \
    ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${HOME}/PX4-Autopilot"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${HOME}/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
```

---

## 🚀 快速开始

### 方式一：仿真环境测试（推荐首次使用）

#### 步骤1：启动虚拟机端仿真环境
```bash
cd vm_control/
./start_px4_sitl.sh

# 或使用无头模式（无Gazebo GUI，节省资源）
./start_px4_sitl.sh headless
```

#### 步骤2：启动香橙派端手势控制
```bash
cd orangepi_control/
./start_orangepi.sh

# 或指定虚拟机IP
./start_orangepi.sh 192.168.1.11
```

#### 步骤3：手势操控无人机
1. 面对摄像头，做出 **左手张开** 手势（五指伸直）
2. 保持0.4秒，无人机解锁并悬停在2米高度
3. 使用 **右手食指指向** 控制前后左右移动
4. 使用 **左手食指上下** 控制高度升降
5. 完成操作后，做出 **左手握拳** 手势触发降落

### 方式二：实机部署

将香橙派通过USB-TTL串口线连接Pixhawk 4飞控，修改 `config.py` 中的串口配置后，按上述步骤启动即可。

---

## 📊 系统性能

### 测试环境
- **硬件**：香橙派 3B (ARM Cortex-A53 四核 1.6GHz, 4GB DDR3)
- **摄像头**：USB摄像头 640×480，设备节点 `/dev/video3`
- **仿真**：Ubuntu 20.04 VM, PX4 SITL v1.13.0, Gazebo 11
- **网络**：千兆以太网桥接，RTT 1.32ms

### 核心指标

| 测试维度 | 指标 | 结果 |
|---------|------|------|
| **功能通过率** | 100次功能触发测试 | **100%** |
| **手势识别准确率** | 正常光照 (400 lux) | **93.0%** |
| | 强光环境 (1200 lux) | **85.3%** |
| | 弱光环境 (30 lux) | **86.7%** |
| **指令响应延迟** | 端到端延迟 | **≤ 250ms** |
| **视觉推理帧率** | FPS | **5~9 FPS** |
| **CPU占用率** | 平均CPU使用率 | **28.6%** |
| **内存占用** | 运行时内存 | **187 MB** |
| **稳定运行时间** | 连续运行 | **≥ 2小时** |

### 远距离识别性能

| 距离 | 检测成功率 | 识别准确率 | 推理FPS |
|------|-----------|-----------|---------|
| 0.5m | 100% | 95.2% | 8.5 |
| 1.0m | 98.5% | 93.8% | 7.8 |
| 1.5m | 95.0% | 90.5% | 6.5 |
| 2.0m | 88.5% | 85.2% | 5.2 |

---

## 🔧 核心算法说明

### 1. MediaPipe手部关键点检测
基于MediaPipe Hands双阶段网络架构：
- **BlazePalm**：手掌检测，快速定位手部区域
- **BlazeHand**：关键点回归，输出21个手部关键点坐标

```
关键点索引 (MediaPipe标准):
  0: 手腕
  1-4: 拇指 (掌指关节→指尖)
  5-8: 食指
  9-12: 中指
  13-16: 无名指
  17-20: 小指
```

### 2. 自定义手势识别算法

#### 右手食指指向识别
```python
# 算法原理：计算食指三个关节向量角度差
vec1 = PIP - MCP    # 掌指关节到近端指间关节
vec2 = DIP - PIP    # 近端到远端指间关节
vec3 = Tip - DIP    # 远端指间关节到指尖

# 若相邻向量角度差均 < 45°，判定为伸直
if abs(angle(vec2) - angle(vec1)) < 45° and \
   abs(angle(vec3) - angle(vec2)) < 45°:
    return "食指伸直"

# 计算指向方向（8方向划分）
angle = atan2(vector_y, vector_x)
# 按角度范围匹配方向：上/下/左/右
```

#### 左手状态识别
```python
# 判断手指伸直：比较指尖与PIP关节的y坐标
for finger in [食指, 中指, 无名指, 小指]:
    if tip.y < pip.y:  # 指尖更靠上 = 伸直
        extended_count += 1

# 根据伸直手指数量判定状态
if extended_count == 0: return "握拳"
if extended_count == 4: return "张开"
```

### 3. 滑动窗口平滑滤波
```python
# 使用最近N帧的均值作为当前坐标估计
# 窗口大小5：约0.33秒延迟，有效消除手部微颤
smoothed_x = mean([frame[N-5].x, ..., frame[N].x])
```

### 4. 跳帧推理优化
```python
# RK3566上MediaPipe单帧推理约80ms
# 跳帧策略：每2帧推理1次，中间帧复用上次结果
# 有效推理帧率从5-7fps提升到10-12fps
inference_skip = 2
```

---

## 📚 关键代码模块

### orangepi_control/main.py
系统主程序，整合摄像头采集、MediaPipe检测、手势识别、无人机控制四大模块。

### orangepi_control/gesture_recognizer.py
手势识别核心，实现右手食指指向（8方向）和左手状态（握拳/张开）识别。

### orangepi_control/drone_controller.py
MAVROS无人机控制器，通过ROS Topic发布PositionTarget消息控制无人机位置。

### orangepi_control/config.py
系统配置文件，包含摄像头参数、MediaPipe配置、手势映射表、ROS网络配置等。

### vm_control/start_px4_sitl.sh
PX4 SITL + Gazebo + MAVROS 一键启动脚本，实现：
- 旧进程清理（防止冲突）
- 环境变量配置
- roscore守护进程启动
- PX4 SITL联合launch
- MAVROS连接状态轮询验证

---

## 🌐 ROS分布式通信架构

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS Distributed Architecture              │
└─────────────────────────────────────────────────────────────┘

  ┌─────────────────┐                      ┌─────────────────┐
  │    虚拟机主机    │                      │   香橙派从机     │
  │  IP: 192.168.1.11│◄────以太网桥接────►│ IP: 192.168.1.12│
  │  ROS Master     │       RTT: 1.32ms    │  ROS Node       │
  └────────┬────────┘                      └────────┬────────┘
           │                                        │
  ┌────────▼────────┐                      ┌────────▼────────┐
  │ /mavros/state   │                      │ 摄像头采集      │
  │ /mavros/local_  │                      │ MediaPipe推理   │
  │   position/pose │                      │ 手势识别        │
  └────────┬────────┘                      └────────┬────────┘
           │                                        │
           │         ┌──────────────────┐          │
           │         │ /mavros/setpoint │          │
           └────────►│ _raw/local       │◄─────────┘
                     │ (PositionTarget) │
                     └────────┬─────────┘
                              │
                     ┌────────▼────────┐
                     │   MAVROS节点     │
                     │  (协议转换)      │
                     └────────┬────────┘
                              │ MAVLink
                     ┌────────▼────────┐
                     │   PX4飞控       │
                     └─────────────────┘
```

### 网络配置
```bash
# 虚拟机端 ~/.bashrc
export ROS_MASTER_URI=http://192.168.1.11:11311
export ROS_IP=192.168.1.11

# 香橙派端 ~/.bashrc
export ROS_MASTER_URI=http://192.168.1.11:11311
export ROS_IP=192.168.1.12
```

**⚠️ 重要**：虚拟机端 `ROS_IP` 必须绑定桥接网卡真实IP（192.168.1.11），若误设为127.0.0.1，ROS Master将拒绝香橙派的跨机连接请求。

---

## 📝 开发日志与调试技巧

### 常见问题

#### 1. 摄像头无法打开
```bash
# 检查可用摄像头设备
ls /dev/video*

# 测试摄像头
python3 -c "import cv2; cap = cv2.VideoCapture('/dev/video3'); print(cap.isOpened())"
```

#### 2. ROS连接失败
```bash
# 检查网络连通性
ping 192.168.1.11

# 检查ROS节点
rostopic list
rostopic echo /mavros/state

# 检查话题发布频率
rostopic hz /mavros/setpoint_raw/local
```

#### 3. MAVROS无法连接
```bash
# 查看MAVROS连接状态
rostopic echo /mavros/state

# 检查PX4 SITL是否正常运行
# Gazebo中应显示Iris无人机模型
```

### 性能调优

#### 提升帧率
- 降低分辨率：修改 `config.py` 中 `FRAME_WIDTH` 和 `FRAME_HEIGHT`
- 减小平滑窗口：降低 `SMOOTHING_WINDOW_SIZE`
- 增大跳帧间隔：增大 `inference_skip`

#### 提高识别准确率
- 调整MediaPipe置信度阈值：`min_detection_confidence`
- 修改角度阈值：`index_angle_thresh`
- 增加手势保持时间：`HOLD_THRESHOLD_SEC`

---

## 🔮 未来优化方向

1. **引入深度估计**：结合双目摄像头或ToF传感器，实现三维空间手势轨迹追踪
2. **动态手势识别**：扩展支持"画圈""滑动"等动态手势，丰富控制维度
3. **多机协同控制**：通过手势同时操控多架无人机编队飞行
4. **增强抗干扰能力**：在硬件层面增加串口滤波电路，降低电磁干扰影响
5. **模型轻量化**：探索TensorFlow Lite或ONNX Runtime进一步降低推理延迟

---

## 📜 开源协议

本项目基于MIT协议开源，欢迎Star和Fork。

如有问题，请提交Issue或联系项目维护者。

---

本文参考了https://github.com/lenanh0803/mediapipe-bin 感谢作者#lenanh0803

---

<p align="center">
  <i>让无人机控制像挥手一样简单</i>
</p>
