#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
香橙派手势控制系统 - 主程序
================================================================================

功能说明:
    本程序是香橙派端的手势控制系统主入口，整合以下功能:
    1. 摄像头图像采集（/dev/video3）
    2. MediaPipe手部关键点检测
    3. 手势识别（8方向指向 + 左手张开/握拳）
    4. 无人机控制（通过ROS网络发送给虚拟机的MAVROS）

系统流程图:
    ┌─────────────┐
    │  摄像头采集  │ ← USB摄像头 15fps 640x480
    └──────┬──────┘
           ↓
    ┌─────────────┐
    │  镜像翻转   │ ← 解决摄像头镜像问题
    │  格式转换   │ ← BGR → RGB (MediaPipe要求)
    └──────┬──────┘
           ↓
    ┌─────────────┐
    │ MediaPipe   │ ← 检测手部，输出21个关键点
    │ Hands       │
    └──────┬──────┘
           ↓
    ┌─────────────┐
    │ 关键点平滑   │ ← 滑动窗口滤波，消除抖动
    │ Landmark     │
    │ Smoother    │
    └──────┬──────┘
           ↓
    ┌─────────────┐
    │ 手势识别     │ ← 计算向量角度，判断手势类型
    │ Gesture      │
    │ Recognizer   │
    └──────┬──────┘
           ↓
    ┌─────────────┐
    │ 无人机控制   │ ← 发送ROS指令到MAVROS
    │ Drone        │
    │ Controller   │
    └──────┬──────┘
           ↓
    ┌─────────────┐
    │  UI显示     │ ← OpenCV窗口显示检测结果
    └─────────────┘

手势控制逻辑（手势 = 虚拟键盘）:
    - 食指向上   → 相当于按W键 → 无人机向前
    - 食指向下   → 相当于按S键 → 无人机向后
    - 食指向左   → 相当于按A键 → 无人机向左
    - 食指向右   → 相当于按D键 → 无人机向右
    - 左手张开   → 相当于按1键 → 解锁+悬停2m
    - 左手握拳   → 相当于按3键 → 降落

网络配置:
    香橙派 ROS_MASTER_URI = http://192.168.1.11:11311
    虚拟机 ROS Master 在 192.168.1.11:11311

使用方法:
    1. 先在虚拟机上运行: ./start_px4_sitl.sh
    2. 在香橙派上运行: python3 main.py --vm-ip 192.168.1.11
    3. 或运行启动脚本: ./start_orangepi.sh
================================================================================
"""

import sys
import time
import signal
import argparse
import cv2
import mediapipe as mp
import math

from config import (
    VIDEO_DEVICE,           # 摄像头设备
    TARGET_FPS,            # 目标帧率
    FRAME_WIDTH,           # 帧宽度
    FRAME_HEIGHT,          # 帧高度
    MEDIAPIPE_CONFIG,      # MediaPipe配置
    SMOOTHING_WINDOW_SIZE, # 平滑窗口大小
    HOLD_THRESHOLD_SEC,    # 手势保持阈值
    ROS_MASTER_URI,        # ROS Master地址
)

from gesture_recognizer import GestureRecognizer
from landmark_smoother import LandmarkSmoother
from drone_controller import DroneController


class GestureControlSystem:
    """
    手势控制系统主类
    
    整合摄像头采集、手势识别、无人机控制三大模块
    提供完整的UI可视化界面
    
    使用流程:
        system = GestureControlSystem(video_device="/dev/video3", 
                                      master_uri="http://192.168.1.11:11311")
        system.run()  # 进入主循环
    """

    def __init__(self, video_device=None, master_uri=None, use_ros=True, show_window=True):
        """
        初始化手势控制系统
        
        参数:
            video_device: 摄像头设备路径，默认从config.py读取
            master_uri: ROS Master URI，默认从config.py读取
            use_ros: 是否启用ROS连接
            show_window: 是否显示OpenCV窗口
        """
        # 保存参数
        self.video_device = video_device or VIDEO_DEVICE
        self.master_uri = master_uri or ROS_MASTER_URI
        self.show_window = show_window
        self.running = True
        
        # 初始化MediaPipe手部检测模块
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(**MEDIAPIPE_CONFIG)
        self.mp_drawing = mp.solutions.drawing_utils
        
        # 初始化手势识别器
        self.recognizer = GestureRecognizer(
            {"smoothing_window_size": SMOOTHING_WINDOW_SIZE}
        )
        
        # 初始化无人机控制器
        self.drone = DroneController(master_uri=self.master_uri, use_ros=use_ros)
        
        # 运行时状态变量
        self.cap = None           # 摄像头捕获对象
        self.prev_time = 0        # 上一帧时间戳
        
        # 跳帧推理：每N帧做一次MediaPipe推理，中间帧复用上次结果
        # RK3566上推理约80ms，跳帧可将有效推理帧率从5-7fps提升到10-12fps
        self.inference_skip = 2   # 每2帧推理1次（可调：1=每帧, 2=每2帧, 3=每3帧）
        self.inference_counter = 0
        self.last_gesture_info = {"fps": 0, "gesture": "No Hand", "hand_label": "-", "color": (255, 255, 255), "angle": 0}
        self.last_gesture = "No Hand"
        self.last_raw_landmarks = None
        self.last_hand_landmarks = None
        self.last_color = (255, 255, 255)

    def init_camera(self):
        """
        初始化摄像头
        
        步骤:
            1. 打开摄像头设备
            2. 设置分辨率、帧率、自动对焦
            3. 验证摄像头是否正常工作
        
        返回:
            bool: True=成功, False=失败
        """
        # 打开摄像头
        print(f"[INFO] 正在打开摄像头: {self.video_device}")
        self.cap = cv2.VideoCapture(self.video_device)
        
        # 检查是否成功打开
        if not self.cap.isOpened():
            print(f"[ERROR] 无法打开摄像头 {self.video_device}")
            print("[INFO] 请检查:")
            print("  1. 摄像头是否已连接")
            print("  2. 设备路径是否正确（ls /dev/video*）")
            return False
        
        # 设置摄像头参数
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # 关闭自动对焦
        
        print(f"[OK] 摄像头已初始化: {FRAME_WIDTH}x{FRAME_HEIGHT} @ {TARGET_FPS}fps")
        return True

    def draw_ui(self, frame, gesture_info):
        """
        在图像上绘制UI信息
        
        绘制内容:
            - 左上角: FPS、连接状态、无人机状态、手势信息
            - 右上角: ROS连接信息（虚拟机IP）
            - 右下角: 控制说明
            - 底部: 快捷键提示
        
        参数:
            frame: 原始图像帧
            gesture_info: 手势信息字典
        
        返回:
            frame: 绘制了UI后的图像
        """
        h, w = frame.shape[:2]
        
        # --------------------------------------------------
        # 左上角: 系统状态面板
        # --------------------------------------------------
        cv2.rectangle(frame, (5, 5), (420, 165), (0, 0, 0), -1)
        cv2.rectangle(frame, (5, 5), (420, 165), (0, 255, 0), 2)
        
        # 提取信息
        fps = gesture_info.get("fps", 0)
        gesture = gesture_info.get("gesture", "Unknown")
        hand_label = gesture_info.get("hand_label", "-")
        angle = gesture_info.get("angle", 0)
        
        # 从无人机控制器获取状态
        status = self.drone.get_status()
        
        # 逐行绘制文字
        cv2.putText(frame, f"FPS: {int(fps)}", (15, 33), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"ROS: {'OK' if status['connected'] else 'NO'} | Armed: {status['armed']} | {status['mode']}", 
                   (15, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(frame, f"Pos: ({status['current_pos'][0]:.1f}, {status['current_pos'][1]:.1f}, {status['current_pos'][2]:.1f})", 
                   (15, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 255), 1)
        cv2.putText(frame, f"Target: ({status['target_pos'][0]:.1f}, {status['target_pos'][1]:.1f}, {status['target_pos'][2]:.1f})", 
                   (15, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 255, 200), 1)
        cv2.putText(frame, f"Hand: {hand_label}", (15, 125), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        
        key_equiv = ""
        if gesture == "Left Open":
            key_equiv = " = 按键1(ARM)"
        elif gesture == "Left Fist":
            key_equiv = " = 按键3(LAND)"
        elif "Pointing" in gesture:
            key_map = {
                "Pointing Up": "W", "Pointing Down": "S",
                "Pointing Left": "A", "Pointing Right": "D",
                "Left Pointing Up": "高度+", "Left Pointing Down": "高度-",
            }
            key_equiv = f" = 按键{key_map.get(gesture, '?')}"
        
        cv2.putText(frame, f"Gesture: {gesture}{key_equiv}", (15, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.55, gesture_info.get("color", (255, 255, 255)), 2)
        
        if "Pointing" in gesture:
            cv2.putText(frame, f"Angle: {angle:.1f}deg", (15, 175), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        
        return frame

    def run(self):
        """
        主循环: 运行手势控制系统
        
        工作流程:
            1. 初始化摄像头
            2. 进入while主循环
            3. 每帧: 采集图像 → MediaPipe检测 → 手势识别 → 无人机控制 → 绘制UI
            4. 处理按键输入
            5. 退出时清理资源
        """
        # 初始化摄像头
        if not self.init_camera():
            return
        
        # 等待MAVROS连接
        if not self.drone.wait_connection():
            print("[ERROR] 无法连接到MAVROS，退出")
            return
        
        # 打印启动信息
        print("="*50)
        print("  UAV Gesture Control System - Orange Pi 3B")
        print("="*50)
        print(f"[INFO] ROS Master: {self.master_uri}")
        print("[INFO] 控制: [SPACE]=解锁 [T]=起飞 [L]=降落 [ESC]=退出")
        print("[INFO] 手势需保持 0.4 秒生效")
        print("="*50)
        
        # 主循环
        while self.running and self.cap.isOpened():
            # ---------- a. 读取摄像头图像 ----------
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # ---------- b. 预处理 ----------
            frame = cv2.flip(frame, 1)
            
            self.inference_counter += 1
            do_inference = (self.inference_counter >= self.inference_skip)
            
            if do_inference:
                self.inference_counter = 0
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img_rgb.flags.writeable = False
                results = self.hands.process(img_rgb)
                img_rgb.flags.writeable = True
                
                gesture_info = {
                    "fps": 0,
                    "gesture": "No Hand",
                    "hand_label": "-",
                    "color": (255, 255, 255),
                    "angle": 0,
                }
                gesture = "No Hand"
                color = (255, 255, 255)
                raw_landmarks = None
                hand_landmarks = None
                
                if results.multi_hand_landmarks and results.multi_handedness:
                    hand_landmarks = results.multi_hand_landmarks[0]
                    handedness = results.multi_handedness[0]
                    hand_label = handedness.classification[0].label
                    raw_landmarks = [(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark]
                    gesture, color, _ = self.recognizer.recognize(
                        raw_landmarks, hand_label, frame.shape
                    )
                    angle = 0
                    if "Pointing" in gesture and raw_landmarks:
                        index_mcp = raw_landmarks[5]
                        index_tip = raw_landmarks[8]
                        angle = math.degrees(
                            math.atan2(index_tip[1] - index_mcp[1], index_tip[0] - index_mcp[0])
                        )
                    gesture_info.update({
                        "gesture": gesture,
                        "hand_label": hand_label,
                        "color": color,
                        "angle": angle,
                    })
                
                self.last_gesture_info = gesture_info
                self.last_gesture = gesture
                self.last_raw_landmarks = raw_landmarks
                self.last_hand_landmarks = hand_landmarks
                self.last_color = color
            
            current_time = time.time()
            if (current_time - self.prev_time) > 0:
                fps = 1 / (current_time - self.prev_time)
            else:
                fps = 0
            self.prev_time = current_time
            self.last_gesture_info["fps"] = fps
            
            if self.last_hand_landmarks is not None:
                self.mp_drawing.draw_landmarks(
                    frame, self.last_hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                    mp.solutions.drawing_utils.DrawingSpec(color=self.last_color, thickness=2, circle_radius=4),
                    mp.solutions.drawing_utils.DrawingSpec(color=self.last_color, thickness=2),
                )
            
            frame = self.draw_ui(frame, self.last_gesture_info)
            
            if self.show_window:
                cv2.imshow("UAV Gesture Control - Orange Pi 3B", frame)
            
            if self.last_gesture not in ("No Hand", "Unknown"):
                self.drone.update(self.last_gesture)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  # ESC键
                break
            elif key == ord(" "):  # 空格键解锁
                print("[KEY] 空格键: 解锁")
                self.drone.arm()
            elif key in (ord("t"), ord("T")):  # T键起飞
                print("[KEY] T键: 起飞到2米")
                self.drone.takeoff(2.0)
            elif key in (ord("l"), ord("L")):  # L键降落
                print("[KEY] L键: 降落")
                self.drone.land()
        
        # 退出主循环
        self.cleanup()

    def cleanup(self):
        """
        系统清理，释放资源
        
        在程序退出时调用:
            1. 停止主循环
            2. 停止无人机控制器
            3. 释放摄像头资源
            4. 关闭OpenCV窗口
        """
        self.running = False
        self.drone.stop()
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        print("[INFO] 系统已关闭")


def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    print("\n[INFO] 接收到中断信号，正在退出...")
    sys.exit(0)


def main():
    """
    命令行入口函数
    
    解析命令行参数，创建GestureControlSystem实例并运行
    
    参数:
        -d, --device:   摄像头设备号，默认从config.py
        --vm-ip:        虚拟机IP地址，默认192.168.1.11
        --vm-port:      ROS master端口，默认11311
        --no-window:    不显示窗口(无屏幕模式)
    
    示例:
        python3 main.py --vm-ip 192.168.1.11 --device /dev/video3
    """
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser(description="无人机手势控制系统 - 香橙派3B端")
    parser.add_argument("-d", "--device", type=str, default=None,
                       help="摄像头设备路径 (默认 /dev/video3)")
    parser.add_argument("--vm-ip", type=str, default="192.168.1.11",
                       help="虚拟机IP地址 (默认 192.168.1.11)")
    parser.add_argument("--vm-port", type=int, default=11311,
                       help="ROS master端口 (默认 11311)")
    parser.add_argument("--no-window", action="store_true",
                       help="不显示窗口 (无屏幕模式)")
    
    # 解析参数
    args = parser.parse_args()
    
    # 构建ROS Master URI
    master_uri = f"http://{args.vm_ip}:{args.vm_port}"
    print(f"[INFO] 将连接到 ROS Master: {master_uri}")
    
    # 是否显示窗口
    show_window = not args.no_window
    
    # 创建设备路径
    video_device = args.device
    
    # 创建并运行系统
    system = GestureControlSystem(
        video_device=video_device,
        master_uri=master_uri,
        use_ros=True,
        show_window=show_window,
    )
    system.run()


if __name__ == "__main__":
    main()