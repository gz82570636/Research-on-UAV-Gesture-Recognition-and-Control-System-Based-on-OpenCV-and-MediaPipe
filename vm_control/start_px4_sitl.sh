#!/bin/bash
# ============================================================================
# PX4 SITL + Gazebo + MAVROS 一键启动脚本
#
# 使用方法:
#   ./start_px4_sitl.sh          # 带Gazebo GUI
#   ./start_px4_sitl.sh headless  # 无Gazebo GUI (仅终端)
#
# 启动后在新终端运行:
#   python3 drone_control.py
# ============================================================================

set -e

HEADLESS=false
if [ "$1" = "headless" ]; then
    HEADLESS=true
    echo "[INFO] 无头模式 (无Gazebo GUI)"
fi

echo "=========================================="
echo "  PX4 SITL + Gazebo + MAVROS 启动脚本"
echo "=========================================="

# ---- 1. 清理旧进程 ----
echo ""
echo "[1/5] 清理旧进程..."
pkill -9 -f "roscore" 2>/dev/null || true
pkill -9 -f "rosmaster" 2>/dev/null || true
pkill -9 -f "mavros" 2>/dev/null || true
pkill -9 -f "gzserver" 2>/dev/null || true
pkill -9 -f "gzclient" 2>/dev/null || true
# 注意: 不能用 pkill -f "px4"，因为本脚本名含px4会自杀
for pid in $(pgrep -f "px4" 2>/dev/null); do
    [ "$pid" != "$$" ] && kill -9 "$pid" 2>/dev/null || true
done
sleep 2

# ---- 2. 环境变量 ----
echo "[2/5] 设置环境变量..."

# ROS 环境
source /opt/ros/noetic/setup.bash

# 工作空间
if [ -f "$HOME/gesture_drone_ws/devel/setup.bash" ]; then
    source "$HOME/gesture_drone_ws/devel/setup.bash"
    echo "       gesture_drone_ws: OK"
else
    echo "       gesture_drone_ws: 未找到 (非必须)"
fi

# PX4 / Gazebo 模型路径
if [ -f "$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash" ]; then
    source "$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
        "$HOME/PX4-Autopilot" \
        "$HOME/PX4-Autopilot/build/px4_sitl_default"
    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${HOME}/PX4-Autopilot"
    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${HOME}/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
    echo "       PX4-Autopilot: OK"
else
    echo "[ERROR] PX4-Autopilot 未找到!"
    echo "        请确认 ~/PX4-Autopilot 存在且已编译"
    exit 1
fi

# ---- 3. 启动 roscore ----
echo ""
echo "[3/5] 启动 roscore..."
roscore &
ROSCORE_PID=$!
sleep 2

# 检查 roscore 是否启动
if ! kill -0 $ROSCORE_PID 2>/dev/null; then
    echo "[ERROR] roscore 启动失败!"
    exit 1
fi
echo "       roscore PID: $ROSCORE_PID"

# ---- 4. 启动 PX4 SITL + Gazebo ----
echo ""
echo "[4/5] 启动 PX4 SITL + Gazebo..."

export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.1.11

if [ "$HEADLESS" = true ]; then
    GUI_ARG="gui:=false"
else
    GUI_ARG="gui:=true"
fi

# 使用 PX4 官方 launch 文件启动 SITL + Gazebo + MAVROS
roslaunch px4 mavros_posix_sitl.launch \
    fcu_url:="udp://:14540@127.0.0.1:14557" \
    $GUI_ARG &

LAUNCH_PID=$!
echo "       launch PID: $LAUNCH_PID"

# 等待仿真启动
echo "       等待 PX4 SITL 初始化 (约10秒)..."
sleep 10

# ---- 5. 验证连接 ----
echo ""
echo "[5/5] 验证 MAVROS 连接..."

for i in $(seq 1 30); do
    if rostopic echo /mavros/state --noarr -n 1 2>/dev/null | grep -q "connected: True"; then
        echo ""
        echo "=========================================="
        echo "  ✅ 启动成功!"
        echo "=========================================="
        echo "  PX4 SITL: 运行中"
        echo "  Gazebo:   $([ "$HEADLESS" = true ] && echo "无头模式" || echo "GUI模式")"
        echo "  MAVROS:   已连接"
        echo ""
        echo "  控制方法:"
        echo "    新终端运行: python3 ~/work_vm/vm_control/drone_control.py"
        echo ""
        echo "  停止仿真:"
        echo "    ./stop_all.sh"
        echo "    或: pkill -f roscore; pkill -f gzserver; pkill -f px4"
        echo "=========================================="

        # 等待 launch 进程
        wait $LAUNCH_PID
        exit 0
    fi
    echo -n "."
    sleep 1
done

echo ""
echo "[WARN] MAVROS 连接超时，但仿真可能仍在初始化中"
echo "       请检查 Gazebo 窗口是否已打开并显示无人机模型"
echo ""
echo "  调试方法:"
echo "    rostopic echo /mavros/state           # 查看连接状态"
echo "    rostopic list                          # 查看话题列表"
echo ""

# 即使超时也继续等待 launch
wait $LAUNCH_PID