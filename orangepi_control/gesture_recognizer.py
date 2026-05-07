#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
手势识别核心模块 (Gesture Recognizer)
============================================================================
功能说明:
    基于 MediaPipe 手部 21 个关键点，实现了两种手势的识别:
    1. 右手: 食指伸直状态 + 食指向量角度 -> 8个方向指向
    2. 左手: 五指伸直状态 -> 握拳 / 张开

核心算法:
    - 右手食指判断: 计算 MCP->PIP, PIP->DIP, DIP->指尖 三个向量的角度差
                    若相邻角度差均 < 45度，则认为手指伸直
    - 右手方向判断: atan2(向量y, 向量x) 计算角度，按阈值划分8个方向
    - 左手状态判断: 比较指尖y坐标与PIP关节y坐标，指尖更靠上=伸直

关键点索引对应关系 (MediaPipe 标准):
    0: 手腕(基准点)
    1-4: 拇指(掌指关节->指尖)
    5-8: 食指(掌指关节->指尖)
    9-12: 中指
    13-16: 无名指
    17-20: 小指

坐标系说明:
    - MediaPipe 返回的坐标是归一化的(0~1)
    - x: 水平方向, 0=左侧, 1=右侧
    - y: 垂直方向, 0=顶部, 1=底部
    - z: 深度, 0=靠近摄像头, 1=远离摄像头

依赖模块:
    - landmark_smoother: 滑动窗口平滑滤波器，消除手部微颤
============================================================================
"""

import math
from landmark_smoother import LandmarkSmoother


class GestureRecognizer:
    """
    手势识别器类
    
    功能:
        - 初始化平滑滤波器和角度阈值
        - 判断右手食指是否伸直
        - 判断手指是否伸直(用于左手)
        - 根据手部关键点识别手势类型
    
    使用方式:
        recognizer = GestureRecognizer({"smoothing_window_size": 5})
        gesture, color, landmarks = recognizer.recognize(raw_landmarks, "Right", frame.shape)
    """

    def __init__(self, config=None):
        """
        初始化手势识别器
        
        参数:
            config: 配置字典，支持以下键:
                - smoothing_window_size: 平滑窗口大小，默认5
                - index_angle_thresh: 食指伸直角度阈值，默认45度
        
        初始化步骤:
            1. 保存配置参数
            2. 创建滑动窗口平滑滤波器(消除手部微颤)
            3. 设置角度判断阈值
        """
        self.config = config or {}

        # 创建平滑滤波器，使用滑动窗口法平滑关键点坐标
        # 窗口大小5表示用最近5帧的均值作为当前坐标
        self.smoother = LandmarkSmoother(
            window_size=self.config.get("smoothing_window_size", 5)
        )

        # 食指伸直判断的阈值: 相邻关节向量角度差必须同时小于此值
        # 值越大越容易判定为伸直，越小越严格
        # 45度是一个经验值: 弯曲时角度差通常>60度，伸直时<30度
        self.angle_thresh = self.config.get("index_angle_thresh", 45)

    def is_index_extended(self, landmarks):
        """
        判断右手食指是否伸直
        
        算法原理:
            手指伸直时，各关节近似在一条直线上
            弯曲时，关节之间有明显的折角
            
            计算三个相邻关节向量的角度:
                vec1: MCP -> PIP  (掌指关节到近端指间关节)
                vec2: PIP -> DIP  (近端到远端指间关节)
                vec3: DIP -> Tip  (远端指间关节到指尖)
            
            如果三个向量方向接近(角度差小)，说明手指伸直
        
        参数:
            landmarks: 平滑后的21个关键点坐标列表
                      每个元素是 (x, y, z) 三元组，值为0~1的归一化坐标
        
        返回:
            bool: True=食指伸直, False=食指弯曲
        """
        # 获取食指的4个关键点(索引5~8)
        mcp = landmarks[5]   # 掌指关节(Metacarpophalangeal Joint)
        pip = landmarks[6]  # 近端指间关节(Proximal Interphalangeal Joint)
        dip = landmarks[7]  # 远端指间关节(Distal Interphalangeal Joint)
        tip = landmarks[8]  # 指尖

        # 计算三个关节向量
        # vec1: 从MCP指向PIP的向量
        vec1 = (pip[0] - mcp[0], pip[1] - mcp[1])
        # vec2: 从PIP指向DIP的向量
        vec2 = (dip[0] - pip[0], dip[1] - pip[1])
        # vec3: 从DIP指向指尖的向量
        vec3 = (tip[0] - dip[0], tip[1] - dip[1])

        # 使用 atan2 计算每个向量的角度(相对于水平轴)
        # atan2(y, x) 返回 -π 到 π 的弧度值
        # 转换为度数方便理解
        angle1 = math.degrees(math.atan2(vec1[1], vec1[0]))
        angle2 = math.degrees(math.atan2(vec2[1], vec2[0]))
        angle3 = math.degrees(math.atan2(vec3[1], vec3[0]))

        # 计算相邻向量之间的角度差
        # 伸直时这三个角度应该接近(手指是直线)
        # 弯曲时角度差会很大
        diff1 = abs(angle2 - angle1)  # vec1与vec2的角度差
        diff2 = abs(angle3 - angle2)  # vec2与vec3的角度差

        # 两个角度差都小于阈值，才判定为伸直
        return diff1 < self.angle_thresh and diff2 < self.angle_thresh

    def is_finger_extended(self, landmarks, tip_id, pip_id):
        """
        判断单根手指是否伸直
        
        算法原理(简化版):
            比较指尖和对应PIP关节的y坐标
            
            由于归一化坐标中 y=0 是图像顶部，y=1 是底部
            当手指伸直向上时，指尖的y值 < PIP关节的y值
            当手指弯曲时，指尖的y值 > PIP关节的y值
        
        参数:
            landmarks: 21个关键点坐标列表
            tip_id: 指尖关键点索引 (拇指=4, 食指=8, 中指=12, ...)
            pip_id: PIP关节关键点索引 (拇指=3, 食指=6, 中指=10, ...)
        
        返回:
            bool: True=手指伸直, False=手指弯曲
        """
        return landmarks[tip_id][1] < landmarks[pip_id][1]

    def recognize(self, raw_landmarks, hand_label, frame_shape):
        """
        ★ 核心方法: 根据手部关键点识别手势类型
        
        工作流程:
            1. 对原始关键点进行平滑滤波
            2. 根据 hand_label 分发到右手/左手识别逻辑
            3. 右手: 食指伸直 -> 计算向量角度 -> 匹配8个方向
            4. 左手: 判断5根手指的伸直状态 -> 握拳/张开/部分伸直
        
        参数:
            raw_landmarks: MediaPipe 返回的21个原始关键点
                          每个元素是 Landmark 对象，有 x, y, z 属性
                          注意: 需要先转换为 (x, y, z) 三元组
            hand_label: 手部标签字符串，"Left" 或 "Right"
            frame_shape: 图像尺寸 (height, width, channels)
        
        返回:
            tuple: (gesture_name, color, smoothed_landmarks)
                - gesture_name: 手势名称字符串
                  * "No Hand": 未检测到手
                  * "Unknown": 手势无法识别
                  * "Pointing Right/Left/Up/Down": 右手食指指向
                  * "Pointing Up-Right/Up-Left/...": 右手斜向指向
                  * "Left Fist": 左手握拳
                  * "Left Open": 左手张开
                  * "Left Partial (N)": 左手部分伸直(N根手指)
                - color: OpenCV 颜色元组 (B, G, R)，用于可视化
                - smoothed_landmarks: 平滑后的关键点坐标(未使用，保留兼容性)
        """
        # 参数检查: 如果没有检测到手，返回默认结果
        if raw_landmarks is None:
            return "No Hand", (255, 255, 255), None

        # 对关键点进行滑动窗口平滑
        # 作用: 消除手部轻微抖动导致的识别抖动
        landmarks = self.smoother.update(raw_landmarks)
        if landmarks is None:
            return "No Hand", (255, 255, 255), None

        # 初始化默认返回值
        gesture = "Unknown"
        color = (255, 255, 255)
        h, w = frame_shape[:2]  # 图像高度和宽度

        # ========================== 右手识别逻辑 ==========================
        if hand_label == "Right":
            # 右手只识别食指指向

            # 先判断食指是否伸直(如果弯曲则无法指向)
            if self.is_index_extended(landmarks):
                # 伸直了，开始计算指向方向

                # 将归一化坐标转换为像素坐标
                # 关键点索引5=MCP(掌指关节), 8=指尖
                index_mcp = (landmarks[5][0] * w, landmarks[5][1] * h)
                index_tip = (landmarks[8][0] * w, landmarks[8][1] * h)

                # 计算食指向量(从MCP指向指尖)
                vector_x = index_tip[0] - index_mcp[0]  # 水平分量
                vector_y = index_tip[1] - index_mcp[1]  # 垂直分量

                # 计算向量角度
                # atan2(y, x) 的结果:
                #   0度: 向右
                #   90度: 向下
                #   180/-180度: 向左
                #   -90度: 向上
                angle = math.degrees(math.atan2(vector_y, vector_x))

                # 定义4个方向的角度范围（去掉斜方向，提升识别速度和可靠性）
                # 每个方向90度范围，覆盖360度无死角
                angle_ranges = [
                    (-45, 45, "Pointing Right", (255, 0, 255)),      # -45~45度: 向右
                    (45, 135, "Pointing Down", (0, 255, 255)),       # 45~135度: 向下
                    (135, 181, "Pointing Left", (255, 128, 0)),      # 135~180度: 向左
                    (-180, -135, "Pointing Left", (255, 128, 0)),    # -180~-135度: 向左
                    (-135, -45, "Pointing Up", (255, 255, 0)),       # -135~-45度: 向上
                ]

                # 遍历角度范围，找到匹配的方向
                for lo, hi, name, c in angle_ranges:
                    if lo <= angle <= hi:
                        gesture = name
                        color = c
                        break

        # ========================== 左手识别逻辑 ==========================
        elif hand_label == "Left":
            # 左手识别优先级（从高到低）：
            # 1. 食指伸直 → 判断上下方向（升降控制）或张开
            # 2. 握拳 / 张开（食指没伸直时）
            # 
            # 注意：必须用is_index_extended()判断食指是否真正伸直
            # 不是简单的y坐标对比，而是检查三个关节的角度连续性
            
            if self.is_index_extended(landmarks):
                # 食指伸直了，检查是否4根都伸直（张开状态）
                index = self.is_finger_extended(landmarks, 8, 6)
                middle = self.is_finger_extended(landmarks, 12, 10)
                ring = self.is_finger_extended(landmarks, 16, 14)
                pinky = self.is_finger_extended(landmarks, 20, 18)
                fingers_extended = sum([index, middle, ring, pinky])
                
                if fingers_extended == 4:
                    # 4根都伸直 = 张开（不是方向！）
                    gesture, color = "Left Open", (0, 255, 0)
                else:
                    # 只有食指伸直（其他弯曲），判断上下方向
                    # 只关心y坐标差，不关心左右
                    # 关键点索引5=MCP(掌指关节), 8=指尖
                    index_mcp_y = landmarks[5][1]
                    index_tip_y = landmarks[8][1]
                    vector_y = index_tip_y - index_mcp_y
                    
                    if vector_y < 0:
                        # 指尖在关节上方（y坐标更小）= 向上
                        gesture, color = "Left Pointing Up", (0, 255, 128)
                    elif vector_y > 0:
                        # 指尖在关节下方（y坐标更大）= 向下
                        gesture, color = "Left Pointing Down", (0, 128, 255)
                    else:
                        # 水平状态，识别为部分伸直
                        gesture, color = "Left Partial", (128, 128, 255)
            else:
                # 食指没伸直，判断握拳/张开（原来的逻辑）
                index = self.is_finger_extended(landmarks, 8, 6)
                middle = self.is_finger_extended(landmarks, 12, 10)
                ring = self.is_finger_extended(landmarks, 16, 14)
                pinky = self.is_finger_extended(landmarks, 20, 18)
                fingers_extended = sum([index, middle, ring, pinky])
                
                if fingers_extended == 0:
                    # 0根手指伸直 = 握拳
                    gesture, color = "Left Fist", (0, 0, 255)
                elif fingers_extended == 4:
                    # 4根手指都伸直 = 张开
                    gesture, color = "Left Open", (0, 255, 0)
                else:
                    # 部分伸直，显示伸直数量（不含拇指）
                    gesture = f"Left Partial ({fingers_extended})"
                    color = (128, 128, 255)

        return gesture, color, landmarks
