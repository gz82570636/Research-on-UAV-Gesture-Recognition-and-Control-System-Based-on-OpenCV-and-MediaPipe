#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
香橙派无人机控制器 - DroneController
================================================================================

功能说明:
    本模块运行在香橙派3B上，负责:
    1. 连接虚拟机上的ROS Master (192.168.1.11:11311)
    2. 接收手势识别结果，转换为位置控制指令
    3. 通过MAVROS控制Gazebo中的无人机飞行

核心设计：手势 = 虚拟键盘
    - 食指向上   → 相当于按W键 → 无人机向前飞
    - 食指向下   → 相当于按S键 → 无人机向后飞
    - 食指向左   → 相当于按A键 → 无人机向左飞
    - 食指向右   → 相当于按D键 → 无人机向右飞
    - 左手张开   → 相当于按1键 → 解锁+悬停2m
    - 左手握拳   → 相当于按3键 → 降落

技术实现：
    - 使用位置控制(PositionTarget)，不是速度控制
    - 发布话题：/mavros/setpoint_raw/local
    - 解锁序列：setpoints流 → OFFBOARD → ARM（与drone_control.py一致）
    - 持续手势每0.3秒移动一步0.5米

网络配置：
    - ROS_MASTER_URI = http://192.168.1.11:11311
    - ROS_IP = 192.168.1.12
================================================================================
"""

import os
import time
import threading
import signal

from config import (
    ROS_MASTER_URI,           # ROS Master地址
    ORANGEPI_IP,              # 本机IP
    GESTURE_STEP,             # 手势到位置增量的映射表（XY平面移动）
    ALTITUDE_STEP,            # 高度控制手势映射（Z轴升降）
    HOLD_THRESHOLD_SEC,       # 手势保持阈值
    CONTINUOUS_MOVE_INTERVAL, # 持续移动间隔
    POSITION_STEP,            # 位置步长
    MIN_ALTITUDE,             # 最小高度
    MAX_ALTITUDE,             # 最大高度
)


class DroneController:
    """
    无人机控制器类
    
    核心功能：
        1. ROS网络连接 - 连接到虚拟机的ROS Master
        2. 位置控制 - 通过PositionTarget控制无人机位置
        3. 手势映射 - 将手势转换为位置增量
        4. 状态管理 - 跟踪连接状态、解锁状态、飞行模式
    
    使用流程：
        controller = DroneController()
        controller.arm()                    # 解锁
        controller.takeoff(2.0)            # 起飞到2米
        controller.update("Pointing Up")    # 前进
        controller.land()                   # 降落
    """

    def __init__(self, master_uri=None, use_ros=True):
        """
        初始化无人机控制器
        
        参数:
            master_uri: ROS Master的URI，默认从config.py读取
            use_ros: 是否启用ROS连接，False时为仿真模式
        """
        # 保存参数
        self.use_ros = use_ros
        self.master_uri = master_uri or ROS_MASTER_URI
        
        # 状态变量
        self.connected = False      # 是否连接到MAVROS
        self.is_armed = False       # 电机是否解锁
        self.mode = "UNKNOWN"       # 当前飞行模式
        self.running = True         # 运行标志
        
        # 位置状态
        self.target_x = 0.0         # 目标位置X（前后）
        self.target_y = 0.0         # 目标位置Y（左右）
        self.target_z = 2.0         # 目标高度（上下）
        self.current_x = 0.0        # 当前位置X
        self.current_y = 0.0        # 当前位置Y
        self.current_z = 0.0        # 当前高度
        
        # 线程同步
        self.pos_lock = threading.Lock()  # 位置变量的锁
        self.setpoint_ready = threading.Event()  # setpoint流就绪标志
        self.arm_event = threading.Event()       # arm状态变化事件
        self.mode_event = threading.Event()      # 模式变化事件
        
        # 手势状态
        self.current_gesture = "Unknown"      # 当前手势
        self.last_gesture = "Unknown"         # 上一次手势
        self.gesture_start_time = None        # 手势开始时间
        self.last_step_time = 0               # 上一次移动时间
        self.step_count = 0                   # 移动步数计数
        
        # 如果启用ROS，进行ROS初始化
        if use_ros:
            self._setup_ros()

    def _setup_ros(self):
        """
        ROS网络初始化和节点创建
        
        步骤：
            1. 设置ROS环境变量（ROS_MASTER_URI和ROS_IP）
            2. 导入ROS Python库
            3. 创建ROS节点
            4. 创建发布者和订阅者
            5. 创建服务代理
            6. 启动setpoint发送线程
        """
        try:
            # =============================================================
            # 步骤1: 设置ROS网络环境变量
            # =============================================================
            # ROS_MASTER_URI告诉rospy "ROS Master在哪里"
            # ROS_IP告诉其他机器 "本机的IP地址是什么"
            os.environ["ROS_MASTER_URI"] = self.master_uri
            os.environ["ROS_IP"] = ORANGEPI_IP
            
            print(f"[ROS-NET] 设置 ROS_MASTER_URI = {self.master_uri}")
            print(f"[ROS-NET] 设置 ROS_IP = {ORANGEPI_IP}")
            
            # =============================================================
            # 步骤2: 导入ROS Python库
            # =============================================================
            import rospy
            from mavros_msgs.msg import State, PositionTarget
            from mavros_msgs.srv import CommandBool, SetMode
            from geometry_msgs.msg import PoseStamped
            
            # 保存引用供其他方法使用
            self.ros = rospy
            self.State = State
            self.PositionTarget = PositionTarget
            self.CommandBool = CommandBool
            self.SetMode = SetMode
            self.PoseStamped = PoseStamped
            
            # =============================================================
            # 步骤3: 创建ROS节点
            # =============================================================
            # anonymous=True表示如果同名节点已存在，自动加随机后缀
            # 这样可以多次运行而不冲突
            rospy.init_node("gesture_drone_controller", anonymous=True)
            print("[ROS-NET] ROS节点已初始化: gesture_drone_controller")
            
            # =============================================================
            # 步骤4: 创建发布者（Publisher）
            # =============================================================
            # 发布位置设定点到 /mavros/setpoint_raw/local
            # MAVROS接收后会转换为MAVLink指令发给PX4
            self.raw_pub = rospy.Publisher(
                "/mavros/setpoint_raw/local",  # 话题名
                PositionTarget,                 # 消息类型
                queue_size=10                   # 队列大小
            )
            print("[ROS-NET] 发布者已创建: /mavros/setpoint_raw/local")
            
            # =============================================================
            # 步骤5: 创建订阅者（Subscriber）
            # =============================================================
            # 订阅MAVROS状态，监听连接、解锁、模式变化
            self.state_sub = rospy.Subscriber(
                "/mavros/state",
                State,
                self._state_callback,
                queue_size=5
            )
            
            # 订阅当前位置反馈（可选，用于显示）
            self.pose_sub = rospy.Subscriber(
                "/mavros/local_position/pose",
                PoseStamped,
                self._pose_callback,
                queue_size=5
            )
            
            print("[ROS-NET] 订阅者已创建: /mavros/state, /mavros/local_position/pose")
            
            # =============================================================
            # 步骤6: 等待MAVROS服务上线
            # =============================================================
            # MAVROS提供的服务：set_mode（切换模式）、cmd/arming（解锁）
            print("[ROS-NET] 等待MAVROS服务上线...")
            try:
                rospy.wait_for_service("/mavros/set_mode", timeout=15.0)
                rospy.wait_for_service("/mavros/cmd/arming", timeout=15.0)
                print("[ROS-NET] MAVROS服务已就绪")
            except rospy.ROSException:
                print("[WARN] MAVROS服务等待超时，将以有限功能运行")
            
            # 创建服务代理
            self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            
            # =============================================================
            # 步骤7: 启动setpoint发送线程
            # =============================================================
            # 这个线程以20Hz频率持续发送位置设定点
            # 保持OFFBOARD模式需要持续接收setpoint（≥2Hz）
            self.stream_thread = threading.Thread(target=self._stream_setpoints)
            self.stream_thread.daemon = True  # 设置为守护线程
            self.stream_thread.start()
            print("[ROS-NET] Setpoint流线程已启动（20Hz）")
            
            # 设置Ctrl+C信号处理
            def _sig_handler(signum, frame):
                print("\n[SIGNAL] 接收到中断信号，正在退出...")
                self.emergency_stop()
                exit(0)
            signal.signal(signal.SIGINT, _sig_handler)
            
            # 不在这里设置 connected=True，由 _state_callback 回调决定
            vm_ip = self.master_uri.split("//")[1].split(":")[0]
            print(f"[ROS-NET] ✓ ROS节点已连接到 {vm_ip}")
            
        except ImportError as e:
            print(f"[WARN] ROS库未安装: {e}")
            print("[WARN] 将以仿真模式运行")
            self.use_ros = False
        except Exception as e:
            print(f"[WARN] ROS连接失败: {e}")
            print("[WARN] 将以仿真模式运行")
            self.use_ros = False

    def _make_position_target(self):
        """
        构建PositionTarget消息
        
        PositionTarget用于位置控制，指定无人机应该飞到的目标位置
        使用NED坐标系：
            x: 前进(+)/后退(-)
            y: 左移(+)/右移(-)
            z: 上升(+)/下降(-)
        
        返回:
            PositionTarget消息对象
        """
        msg = self.PositionTarget()
        msg.header.stamp = self.ros.Time.now()  # 时间戳（必须）
        msg.header.frame_id = "map"              # 坐标系
        
        # 使用本地NED坐标系
        msg.coordinate_frame = self.PositionTarget.FRAME_LOCAL_NED
        
        # 类型掩码：只使用位置，忽略速度、加速度、偏航角速度
        # 这样飞控只关心位置，不关心如何到达（飞控自己规划路径）
        msg.type_mask = (
            self.PositionTarget.IGNORE_VX |
            self.PositionTarget.IGNORE_VY |
            self.PositionTarget.IGNORE_VZ |
            self.PositionTarget.IGNORE_AFX |
            self.PositionTarget.IGNORE_AFY |
            self.PositionTarget.IGNORE_AFZ |
            self.PositionTarget.IGNORE_YAW_RATE
        )
        
        # 设置目标位置
        with self.pos_lock:
            msg.position.x = self.target_x
            msg.position.y = self.target_y
            msg.position.z = self.target_z
            msg.yaw = 0.0  # 偏航角（0表示朝北）
        
        return msg

    def _stream_setpoints(self):
        """
        Setpoint流发送线程
        
        以20Hz频率持续发送位置设定点
        这是保持OFFBOARD模式的关键（PX4要求≥2Hz）
        
        重要：必须无条件发送setpoint，不能等待连接确认
        PX4要求setpoint流在切换OFFBOARD模式之前就已经在发送
        
        运行流程：
            1. 构建PositionTarget消息
            2. 发布到/mavros/setpoint_raw/local
            3. 计数，发送5次后标记setpoint_ready
            4. 睡眠50ms（20Hz）
        """
        rate = self.ros.Rate(20)  # 20Hz
        sent_count = 0
        
        while self.running and not self.ros.is_shutdown():
            try:
                msg = self._make_position_target()
                self.raw_pub.publish(msg)
                
                sent_count += 1
                # 发送5次（约0.25秒）后标记就绪
                if sent_count >= 5:
                    self.setpoint_ready.set()
            except Exception:
                pass
            
            rate.sleep()

    def _state_callback(self, msg):
        """
        MAVROS状态回调函数
        
        每当/mavros/state更新时自动调用（通常10Hz）
        更新内部状态变量，触发事件通知
        
        参数:
            msg: State消息，包含connected、armed、mode字段
        """
        # 保存之前的状态，用于检测变化
        prev_armed = self.is_armed
        prev_mode = self.mode
        
        # 更新当前状态
        self.connected = msg.connected
        self.is_armed = msg.armed
        self.mode = msg.mode
        
        # 如果arm状态变化，触发事件
        if prev_armed != msg.armed:
            self.arm_event.set()
        
        # 如果模式变化，触发事件
        if prev_mode != msg.mode:
            self.mode_event.set()

    def _pose_callback(self, msg):
        """
        位置反馈回调函数
        
        订阅/mavros/local_position/pose获取当前位置
        用于显示当前位置，但不用于控制（控制用目标位置）
        
        参数:
            msg: PoseStamped消息，包含当前位置
        """
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

    def wait_connection(self, timeout=30):
        """
        等待连接到MAVROS
        
        参数:
            timeout: 超时时间（秒）
        
        返回:
            True: 连接成功
            False: 连接超时
        """
        print("[INFO] 等待MAVROS连接...")
        start = time.time()
        
        while not self.connected and (time.time() - start) < timeout:
            time.sleep(0.5)
        
        if self.connected:
            print("[OK] 已连接到飞控")
            return True
        
        print("[ERROR] 连接超时！请检查:")
        print("  1. 虚拟机上的start_px4_sitl.sh是否已运行")
        print("  2. Gazebo中是否能看到无人机模型")
        print("  3. 网络连接是否正常（ping 192.168.1.11）")
        return False

    def arm(self, max_retries=5):
        """
        解锁电机并进入OFFBOARD模式
        
        正确的PX4 OFFBOARD解锁序列：
            1. 等待setpoint流就绪（已发送至少5个setpoint）
            2. 等待1秒让setpoint稳定
            3. 切换到OFFBOARD模式（此时飞控开始接收外部指令）
            4. 解锁电机（ARM）
        
        注意：必须先切OFFBOARD再ARM，顺序不能反！
        
        参数:
            max_retries: 最大重试次数
        
        返回:
            True: 解锁成功
            False: 解锁失败
        """
        if not self.use_ros or not self.connected:
            print("[SIM] 仿真模式：无人机已解锁")
            self.is_armed = True
            return True
        
        print("\n" + "="*50)
        print("  解锁 + 进入OFFBOARD模式")
        print("="*50)
        
        # 步骤1: 等待setpoint流就绪
        print("[1/4] 等待setpoint流就绪...")
        if not self.setpoint_ready.wait(timeout=5):
            print("[ERROR] Setpoint流未就绪！")
            return False
        
        # 步骤2: 等待setpoint稳定
        print("[2/4] 等待setpoint稳定...")
        self.ros.sleep(1)
        
        # 步骤3: 切换到OFFBOARD模式
        print("[3/4] 切换到OFFBOARD模式...")
        
        for attempt in range(max_retries):
            # 先检查是否已经在OFFBOARD模式（降落后可能保持）
            if self.mode == "OFFBOARD":
                print("       → 已在OFFBOARD模式")
                break
            
            # 调用MAVROS服务切换模式，检查结果
            result = self.set_mode_srv(0, "OFFBOARD")
            if not result.mode_sent:
                print(f"       → 第{attempt+1}次尝试... 服务调用失败")
                self.ros.sleep(1)
                continue
            
            # 等待模式切换完成（PX4需要时间处理）
            self.ros.sleep(1)
            
            if self.mode == "OFFBOARD":
                print("       → OFFBOARD模式切换成功")
                break
            
            print(f"       → 第{attempt+1}次尝试... 当前模式: {self.mode}")
            self.ros.sleep(1)
        else:
            # 循环正常结束（没有break），检查最终状态
            if self.mode == "OFFBOARD":
                print("       → 已在OFFBOARD模式")
            else:
                print(f"[ERROR] OFFBOARD模式切换失败! 当前模式: {self.mode}")
                return False
        
        # 步骤4: 解锁电机
        print("[4/4] 解锁电机...")
        
        for attempt in range(max_retries):
            # 先检查是否已解锁（可能已自动解锁）
            if self.is_armed:
                print("       → 已解锁")
                break
            
            # 调用MAVROS服务解锁，检查结果
            result = self.arm_srv(True)
            if not result.success:
                print(f"       → 第{attempt+1}次尝试... 服务调用失败: {result}")
                self.ros.sleep(1)
                continue
            
            # 等待解锁完成（PX4需要时间处理）
            self.ros.sleep(1)
            
            if self.is_armed:
                print("       → 解锁成功！")
                print(f"\n{'='*50}")
                print("  ✓ 无人机已就绪")
                print(f"{'='*50}")
                return True
            
            print(f"       → 第{attempt+1}次解锁尝试... 等待状态更新")
            self.ros.sleep(1)
        
        # 循环结束后最终检查
        if self.is_armed:
            print("       → 解锁成功！")
            return True
        
        print("[ERROR] 解锁失败！")
        return False

    def takeoff(self, altitude=2.0):
        """
        起飞到指定高度
        
        实现方式：设置target_z为目标高度
        由于_stream_setpoints线程持续发送setpoint，
        飞控会自动飞到目标高度并保持
        
        参数:
            altitude: 目标高度（米），默认2.0米
        
        返回:
            True: 起飞指令已发送
        """
        if not self.use_ros or not self.connected:
            print(f"[SIM] 仿真模式：起飞至{altitude}米")
            self.target_z = altitude
            return True
        
        # 如果未解锁，先解锁
        if not self.is_armed:
            with self.pos_lock:
                self.target_x = self.current_x
                self.target_y = self.current_y
                self.target_z = altitude
            print("[INFO] 未解锁，先执行解锁...")
            if not self.arm():
                return False
        
        print(f"\n{'='*50}")
        print(f"  起飞至 {altitude}米")
        print("=" * 50)
        
        with self.pos_lock:
            self.target_z = altitude
        
        # 等待飞到目标高度（简单实现）
        print("[INFO] 等待无人机到达目标高度...")
        self.ros.sleep(3)
        
        print(f"[OK] 已设置目标高度：{altitude}米")
        return True

    def land(self):
        """
        降落
        
        实现方式：切换到AUTO.LAND模式
        AUTO.LAND是PX4的自动降落模式，飞控会自动下降并降落
        触地后自动锁定电机
        
        返回:
            True: 降落指令已发送
        """
        if not self.use_ros or not self.connected:
            print("[SIM] 仿真模式：降落")
            self.is_armed = False
            return True
        
        print("\n" + "="*50)
        print("  降落")
        print("="*50)
        
        # 切换到AUTO.LAND模式
        self.mode_event.clear()
        result = self.set_mode_srv(0, "AUTO.LAND")
        
        if result.mode_sent:
            print("[INFO] 已切换到AUTO.LAND模式")
        else:
            print("[WARN] 降落模式切换失败")
        
        # 等待降落完成（最多30秒）
        print("[INFO] 等待降落完成...")
        start = time.time()
        while self.is_armed and (time.time() - start) < 30:
            self.ros.sleep(0.5)
        
        if not self.is_armed:
            print("[OK] 已安全降落并锁定")
        else:
            print("[WARN] 降落超时，强制锁定...")
            self.arm_srv(False)
        
        # 降落后等待PX4状态重置（关键：不等待会导致下次解锁失败）
        print("[INFO] 等待PX4状态重置...")
        self.ros.sleep(2)
        
        return True

    def update(self, gesture):
        """
        核心方法：根据手势更新无人机控制
        
        这是手势识别程序每帧都会调用的方法
        将手势转换为位置增量（相当于按WASD键）
        
        参数:
            gesture: 识别到的手势字符串，例如"Pointing Up"
        
        手势映射（手势 = 虚拟键盘）：
            Pointing Up    → W键 → target_x += 0.5（前进）
            Pointing Down  → S键 → target_x -= 0.5（后退）
            Pointing Left  → A键 → target_y += 0.5（左移）
            Pointing Right → D键 → target_y -= 0.5（右移）
            Left Open      → 按键1 → 解锁+起飞（如果未解锁）
            Left Fist      → 按键3 → 降落（如果已解锁）
        """
        current_time = time.time()
        
        # 保存当前手势
        self.current_gesture = gesture
        
        # =============================================================
        # 步骤1: 检测手势变化，重置计时器
        # =============================================================
        if gesture != self.last_gesture:
            # 手势变了，重新计时
            self.gesture_start_time = current_time
            self.last_gesture = gesture
            self.step_count = 0
            return  # 等待新手势稳定
        
        if self.gesture_start_time is None:
            self.gesture_start_time = current_time
        
        # =============================================================
        # 步骤2: 计算保持时长
        # =============================================================
        hold_duration = current_time - self.gesture_start_time
        
        # 如果手势保持时间还不够，忽略（防止误触发）
        if hold_duration < HOLD_THRESHOLD_SEC:
            return
        
        # =============================================================
        # 步骤3: 根据手势类型执行对应动作
        # =============================================================
        
        # ---- 左手张开 = 解锁+起飞（相当于按1键）----
        if gesture == "Left Open":
            if not self.is_armed:
                self.takeoff(2.0)
                # 重置手势计时，防止连续触发
                self.gesture_start_time = current_time
                self.last_gesture = gesture
            return
        
        # ---- 左手握拳 = 降落（相当于按3键）----
        if gesture == "Left Fist":
            if self.is_armed:
                self.land()
                # 重置手势计时
                self.gesture_start_time = current_time
                self.last_gesture = gesture
            return
        
        # ---- 左手食指上下 = 高度控制（上升/下降）----
        # 只有在解锁且OFFBOARD模式下才响应高度控制
        if gesture in ALTITUDE_STEP:
            if not (self.is_armed and self.mode == "OFFBOARD"):
                return
            
            # 检查是否到了高度调整时间间隔
            if self.step_count == 0 or \
               (current_time - self.last_step_time) >= CONTINUOUS_MOVE_INTERVAL:
                
                # 获取高度增量
                dz = ALTITUDE_STEP[gesture]
                
                # 应用高度增量（加锁保护）
                with self.pos_lock:
                    self.target_z += dz
                    # 限制高度范围（防止飞得太高或太低）
                    self.target_z = max(MIN_ALTITUDE, 
                                       min(MAX_ALTITUDE, self.target_z))
                
                # 更新移动记录
                self.step_count += 1
                self.last_step_time = current_time
                
                # 打印高度调整信息
                action_str = "上升" if dz > 0 else "下降"
                print(f"[GESTURE] {gesture} = {action_str} "
                      f"→ 目标高度: {self.target_z:.1f}m")
            return
        
        # ---- 右手指向 = 方向移动（相当于WASD）----
        # 只有在解锁且OFFBOARD模式下才响应方向控制
        if not (self.is_armed and self.mode == "OFFBOARD"):
            return
        
        if gesture in GESTURE_STEP:
            # 检查是否到了移动时间间隔
            # 首次移动或间隔达到CONTINUOUS_MOVE_INTERVAL
            if self.step_count == 0 or \
               (current_time - self.last_step_time) >= CONTINUOUS_MOVE_INTERVAL:
                
                # 获取位置增量
                step = GESTURE_STEP[gesture]
                dx = step["dx"]
                dy = step["dy"]
                
                # 应用位置增量（加锁保护）
                with self.pos_lock:
                    self.target_x += dx
                    self.target_y += dy
                    # 限制高度范围（防止飞得太高或太低）
                    self.target_z = max(MIN_ALTITUDE, 
                                       min(MAX_ALTITUDE, self.target_z))
                
                # 更新移动记录
                self.step_count += 1
                self.last_step_time = current_time
                
                # 打印移动信息
                key_map = {
                    "Pointing Up": "W", "Pointing Down": "S",
                    "Pointing Left": "A", "Pointing Right": "D",
                    "Pointing Up-Left": "W+A", "Pointing Up-Right": "W+D",
                    "Pointing Down-Left": "S+A", "Pointing Down-Right": "S+D"
                }
                key_equiv = key_map.get(gesture, "?")
                print(f"[GESTURE] {gesture} = 按键{key_equiv} "
                      f"→ 目标位置: ({self.target_x:.1f}, {self.target_y:.1f})")

    def stop(self):
        """
        停止控制器
        
        安全退出，停止所有线程
        """
        self.running = False
        print("[INFO] 控制器已停止")

    def emergency_stop(self):
        """
        紧急停止
        
        立即停止所有运动，切换到AUTO.LOITER，锁定电机
        用于处理紧急情况（如手势失控、通信中断等）
        """
        print("\n[EMERGENCY] 紧急停止！")
        
        # 停止setpoint流
        self.running = False
        
        if self.use_ros and self.connected:
            try:
                # 切换到AUTO.LOITER模式（悬停）
                print("[EMERGENCY] 切换到AUTO.LOITER模式...")
                self.set_mode_srv(0, "AUTO.LOITER")
                self.ros.sleep(1)
                
                # 锁定电机
                print("[EMERGENCY] 锁定电机...")
                self.arm_srv(False)
                self.ros.sleep(1)
                
                print("[EMERGENCY] 紧急停止完成")
            except Exception as e:
                print(f"[EMERGENCY] 错误: {e}")
        
        self.is_armed = False

    def get_status(self):
        """
        获取当前状态
        
        返回:
            字典，包含连接状态、解锁状态、模式、位置等信息
        """
        with self.pos_lock:
            return {
                "connected": self.connected,
                "armed": self.is_armed,
                "mode": self.mode,
                "current_pos": (self.current_x, self.current_y, self.current_z),
                "target_pos": (self.target_x, self.target_y, self.target_z),
                "gesture": self.current_gesture,
                "step_count": self.step_count
            }