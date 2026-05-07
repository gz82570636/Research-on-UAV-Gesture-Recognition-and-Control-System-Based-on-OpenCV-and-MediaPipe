#!/usr/bin/env python3
"""
虚拟机端无人机控制脚本 - PX4 SITL 测试用

操作说明:
  1 - 解锁 + 起飞悬停2m
  2 - WASD前后左右控制
  3 - 降落
  s - 显示状态
  q - 退出

WASD控制:
  W/S - 前进/后退
  A/D - 左移/右移
  Q   - 退出WASD模式
"""

import rospy
import sys
import select
import termios
import tty
import threading
import time
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


local_ned = PositionTarget.FRAME_LOCAL_NED
ignore_vx = PositionTarget.IGNORE_VX
ignore_vy = PositionTarget.IGNORE_VY
ignore_vz = PositionTarget.IGNORE_VZ
ignore_afx = PositionTarget.IGNORE_AFX
ignore_afy = PositionTarget.IGNORE_AFY
ignore_afz = PositionTarget.IGNORE_AFZ
ignore_yaw_rate = PositionTarget.IGNORE_YAW_RATE
POS_ONLY_MASK = ignore_vx | ignore_vy | ignore_vz | ignore_afx | ignore_afy | ignore_afz | ignore_yaw_rate


class DroneControl:

    def __init__(self):
        rospy.init_node('drone_control', anonymous=True)

        self.state = State()
        self.connected = False
        self.armed = False
        self.mode = "UNKNOWN"
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.lock = threading.Lock()

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 2.0

        self.setpoint_ready = threading.Event()
        self.arm_event = threading.Event()
        self.mode_event = threading.Event()
        self.running = True

        self.raw_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)

        rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
        rospy.wait_for_service('/mavros/set_mode', timeout=30)
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        print("[OK] MAVROS 服务已连接")

        stream = threading.Thread(target=self._stream_setpoints)
        stream.daemon = True
        stream.start()

        spinner = threading.Thread(target=self._spin)
        spinner.daemon = True
        spinner.start()

        print("[OK] 无人机控制节点已就绪")

    def _make_target(self):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = local_ned
        msg.type_mask = POS_ONLY_MASK
        msg.position.x = self.target_x
        msg.position.y = self.target_y
        msg.position.z = self.target_z
        msg.yaw = 0.0
        return msg

    def _stream_setpoints(self):
        rate = rospy.Rate(20)
        sent = 0
        while self.running and not rospy.is_shutdown():
            self.raw_pub.publish(self._make_target())
            sent += 1
            if sent >= 5:
                self.setpoint_ready.set()
            rate.sleep()

    def _spin(self):
        while self.running and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def _state_cb(self, msg):
        with self.lock:
            prev_armed = self.armed
            prev_mode = self.mode
            self.state = msg
            self.connected = msg.connected
            self.armed = msg.armed
            self.mode = msg.mode
        if prev_armed != msg.armed:
            self.arm_event.set()
        if prev_mode != msg.mode:
            self.mode_event.set()

    def _pose_cb(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

    def wait_connection(self, timeout=30):
        print("[INFO] 等待 MAVROS 连接...")
        start = time.time()
        while not self.connected and (time.time() - start) < timeout:
            rospy.sleep(0.5)
        if self.connected:
            print("[OK] 已连接到飞控")
            return True
        print("[ERROR] 连接超时! 请检查:")
        print("  1. vm_control/start_px4_sitl.sh 是否已运行")
        print("  2. Gazebo 中是否能看到无人机模型")
        print("  3. rostopic echo /mavros/state 是否有输出")
        return False

    def arm_and_hover(self, altitude=2.0):
        print("\n" + "=" * 50)
        print(f"  解锁 + 起飞到 {altitude}m")
        print("=" * 50)

        if self.armed and self.mode == "OFFBOARD":
            print("[INFO] 无人机已解锁且处于OFFBOARD模式")
            print(f"       直接调整目标高度到 {altitude}m")
            self.target_z = altitude
            return True

        if not self.setpoint_ready.wait(timeout=5):
            print("[ERROR] Setpoint 流未就绪")
            return False

        print("[1/4] 等待 setpoint 流稳定...")
        rospy.sleep(1)

        print("[2/4] 切换到 OFFBOARD 模式...")
        self.mode_event.clear()
        for attempt in range(3):
            result = self.mode_srv(0, "OFFBOARD")
            if result.mode_sent:
                if self.mode_event.wait(timeout=3):
                    if self.mode == "OFFBOARD":
                        print(f"       → OFFBOARD 模式切换成功")
                        break
                print(f"       → 第{attempt + 1}次尝试... 模式: {self.mode}")
            rospy.sleep(1)
        else:
            print(f"[ERROR] OFFBOARD 模式切换失败! 当前模式: {self.mode}")
            print("        可能原因: setpoint 流未稳定或 GPS 未锁定")
            return False

        print("[3/4] 解锁电机 (ARM)...")
        self.arm_event.clear()
        for attempt in range(3):
            result = self.arm_srv(True)
            if result.success:
                if self.arm_event.wait(timeout=3):
                    if self.armed:
                        print("       → 解锁成功!")
                        break
            print(f"       → 第{attempt + 1}次解锁尝试...")
            rospy.sleep(1)
        else:
            print("[ERROR] 解锁失败!")
            return False

        print(f"[4/4] 飞到 {altitude}m...")
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = altitude
        rospy.sleep(3)

        print(f"\n{'=' * 50}")
        print(f"  ✅ 起飞完成!")
        print(f"  高度: {self.current_z:.1f}m  位置: ({self.current_x:.1f}, {self.current_y:.1f})")
        print(f"  模式: {self.mode}  解锁: {self.armed}")
        print(f"{'=' * 50}")
        return True

    def land(self):
        print("\n" + "=" * 50)
        print("  降落")
        print("=" * 50)

        self.mode_event.clear()
        result = self.mode_srv(0, "AUTO.LAND")
        if result.mode_sent:
            print("[INFO] 已切换到 AUTO.LAND 模式")
        else:
            print(f"[WARN] 降落模式切换失败: {result}")

        print("[INFO] 等待降落...")
        start = time.time()
        while self.armed and (time.time() - start) < 30:
            rospy.sleep(0.5)

        if not self.armed:
            print("[OK] 已安全降落并锁定")
        else:
            print("[WARN] 降落超时，尝试强制锁定...")
            self.arm_srv(False)

        with self.lock:
            print(f"  最终状态: armed={self.armed}, mode={self.mode}")
        return True

    def wasd_control(self):
        if not self.armed:
            print("[ERROR] 请先解锁! (按1)")
            return
        if self.mode != "OFFBOARD":
            print(f"[ERROR] 当前模式: {self.mode}, 需要 OFFBOARD 模式!")
            return

        step = 0.5
        self.target_x = round(self.current_x, 1)
        self.target_y = round(self.current_y, 1)
        self.target_z = self.target_z

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        print(f"\n{'=' * 50}")
        print("  WASD 位置控制模式")
        print(f"  W/S: 前进/后退 ({step}m)")
        print(f"  A/D: 左移/右移 ({step}m)")
        print("  Q:   退出控制模式")
        print(f"{'=' * 50}")

        try:
            tty.setraw(fd)
            while self.running and not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == 'q':
                        print("\r[Q] 退出WASD模式")
                        break
                    elif key == 'w':
                        self.target_x += step
                    elif key == 's':
                        self.target_x -= step
                    elif key == 'a':
                        self.target_y += step
                    elif key == 'd':
                        self.target_y -= step

                sys.stdout.write(
                    f"\r  目标: ({self.target_x:.1f}, {self.target_y:.1f}, {self.target_z:.1f})"
                    f"  实际: ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_z:.1f})"
                    f"  [{self.mode}]"
                )
                sys.stdout.flush()

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print()

    def print_status(self):
        with self.lock:
            print(f"\n--- 无人机状态 ---")
            print(f"  连接:   {self.connected}")
            print(f"  解锁:   {self.armed}")
            print(f"  模式:   {self.mode}")
            print(f"  位置:   ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f})")
            print(f"  目标:   ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f})")
            print(f"------------------")

    def shutdown(self):
        self.running = False


def main():
    print("=" * 50)
    print("  无人机控制 - PX4 SITL 测试")
    print("=" * 50)
    print("  1 - 解锁 + 悬停2m")
    print("  2 - WASD 控制模式")
    print("  3 - 降落")
    print("  s - 显示状态")
    print("  q - 退出")
    print("=" * 50)

    drone = DroneControl()

    if not drone.wait_connection():
        return

    while True:
        try:
            cmd = input("\n输入命令: ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == "1":
            drone.arm_and_hover(2.0)
        elif cmd == "2":
            drone.wasd_control()
        elif cmd == "3":
            drone.land()
        elif cmd == "s":
            drone.print_status()
        elif cmd == "q":
            print("[INFO] 退出...")
            drone.shutdown()
            break
        else:
            print(f"[WARN] 未知命令: {cmd}")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n[INFO] 已中断")