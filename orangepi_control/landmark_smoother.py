#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
关键点平滑滤波器 (Landmark Smoother)
============================================================================
功能说明:
    使用滑动窗口法对 MediaPipe 手部21个关键点进行时间域平滑，
    消除手部轻微抖动导致的控制指令抖动。

问题背景:
    摄像头采集频率约15Hz，每帧图像都可能检测到略有不同的关键点坐标。
    即使手完全静止，连续帧之间的关键点坐标也会有小幅波动(噪声)。
    如果直接用原始坐标计算手势，会导致识别结果不稳定，指令频繁跳变。

解决方案:
    使用滑动窗口滤波器，保存最近N帧的关键点坐标，求均值作为当前估计值。
    这样可以有效抑制高频噪声，让关键点轨迹更加平滑。

算法原理:
    滑动窗口均值滤波:
        smoothed[i] = (frame[i] + frame[i-1] + ... + frame[i-window_size+1]) / window_size
    
    示意图(窗口大小=5):
        帧:  [1] [2] [3] [4] [5] [6] [7] [8] [9] ...
        输出:          [3] [4] [5] [6] [7] [8] [9] ...
        (第3帧才有足够的5帧历史数据开始输出)
    
    优点:
        - 实现简单，计算效率高
        - 对脉冲噪声有良好的抑制作用
        - 不引入额外延迟(只用到历史数据)
    
    缺点:
        - 对突变响应较慢(需要填满窗口)
        - 会稍微拖慢响应速度(约 window_size/2 帧)

参数说明:
    - window_size: 滑动窗口大小
      推荐值: 3~7
      太小: 平滑效果不明显
      太大: 响应延迟明显，手势变化后需要更多帧才能反映
============================================================================
"""

import numpy as np


class LandmarkSmoother:
    """
    滑动窗口关键点平滑器
    
    使用方式:
        smoother = LandmarkSmoother(window_size=5)
        
        # 每帧处理时调用
        smoothed = smoother.update(raw_landmarks)
    
    属性:
        window_size: 滑动窗口大小(帧数)
        history: 保存历史帧的列表
    
    注意:
        - 前 window_size-1 帧由于数据不足，update() 会返回 None
        - 每次识别新手势时，建议调用 reset() 清空历史，避免旧数据干扰
    """

    def __init__(self, window_size=5):
        """
        初始化滑动窗口平滑器
        
        参数:
            window_size: 滑动窗口大小，表示用最近多少帧求均值
                       推荐值: 3~7
                       默认值: 5
        
        初始化步骤:
            1. 保存窗口大小
            2. 创建空的历史数据列表
        """
        # 窗口大小: 用最近多少帧的均值作为输出
        self.window_size = window_size

        # history 是一个列表，保存最近 window_size 帧的关键点
        # 每帧是一个 21x3 的列表 [(x,y,z), ...]
        self.history = []

    def update(self, landmarks):
        """
        ★ 核心方法: 更新平滑器并返回平滑后的关键点
        
        工作流程:
            1. 参数检查: 如果 landmarks 为 None，直接返回 None
            2. 将新帧追加到历史数据末尾
            3. 如果历史数据超过窗口大小，移除最老的帧
            4. 如果历史数据为空(初次调用)，返回 None
            5. 对每个关键点，计算窗口内所有帧的均值
        
        参数:
            landmarks: 21个关键点的原始坐标
                     格式: [(x, y, z), (x, y, z), ...] 共21个元素
                     每个元素是包含3个浮点数的元组
        
        返回:
            smoothed_landmarks: 平滑后的关键点坐标
                              格式与输入相同 [(x, y, z), ...]
            None: 如果输入为 None 或历史数据不足
        
        示例:
            smoother = LandmarkSmoother(window_size=5)
            
            # 第1帧: history只有1帧，不足5帧，返回None
            result = smoother.update(frame1_landmarks)  # -> None
            
            # 第5帧: history已有5帧，开始输出
            result = smoother.update(frame5_landmarks)  # -> smoothed
            
            # 第6帧: 移除最老的frame1，加入frame6
            result = smoother.update(frame6_landmarks)  # -> smoothed
        """
        # --------------------------------------------------
        # 步骤1: 参数检查
        # --------------------------------------------------
        if landmarks is None:
            return None

        # --------------------------------------------------
        # 步骤2: 添加新帧到历史数据
        # --------------------------------------------------
        self.history.append(landmarks)

        # --------------------------------------------------
        # 步骤3: 保持历史数据不超过窗口大小
        # --------------------------------------------------
        if len(self.history) > self.window_size:
            # 移除最老的帧(FIFO: First In First Out)
            self.history.pop(0)

        # --------------------------------------------------
        # 步骤4: 检查数据是否足够
        # --------------------------------------------------
        if len(self.history) == 0:
            return None

        # --------------------------------------------------
        # 步骤5: 计算滑动窗口均值
        # --------------------------------------------------
        # 关键点数量: MediaPipe 手部检测返回21个关键点
        num_points = len(landmarks)

        # 初始化输出列表
        avg_landmarks = []

        # 对每个关键点分别计算均值
        for i in range(num_points):
            # 提取历史数据中第 i 个关键点的 x 坐标
            # history 是一个列表，每个元素是一帧的21个关键点
            # history[frame_idx][point_idx] = (x, y, z)
            x_values = [frame[i][0] for frame in self.history]
            y_values = [frame[i][1] for frame in self.history]
            z_values = [frame[i][2] for frame in self.history]

            # 使用 numpy 计算均值(也可以用 sum()/len() 代替)
            avg_x = np.mean(x_values)
            avg_y = np.mean(y_values)
            avg_z = np.mean(z_values)

            avg_landmarks.append((avg_x, avg_y, avg_z))

        return avg_landmarks

    def reset(self):
        """
        重置平滑器，清空所有历史数据
        
        使用场景:
            - 每次识别到新手势类型时调用
            - 避免上一个手势的历史数据影响新手势的识别
            - 相当于"重新开始"平滑过程
        
        示例:
            # 检测到手势从 "Pointing Right" 变成 "Left Fist"
            smoother.reset()  # 清空历史，重新开始平滑
        """
        self.history.clear()
