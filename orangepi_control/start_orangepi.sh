#!/bin/bash
# ==============================================================================
# 香橙派手势控制系统启动脚本
# ==============================================================================
#
# 功能说明:
#     一键启动香橙派端的手势控制程序
#     自动设置ROS环境变量并运行main.py
#
# 使用方法:
#     ./start_orangepi.sh                    # 默认连接192.168.1.11
#     ./start_orangepi.sh 192.168.1.11       # 指定虚拟机IP
#
# 网络配置:
#     香橙派IP: 192.168.1.12
#     虚拟机IP: 192.168.1.11
#     ROS_MASTER_URI: http://192.168.1.11:11311
#
# 注意:
#     1. 先在虚拟机上运行 ./start_px4_sitl.sh 启动仿真
#     2. 确保香橙派和虚拟机网络连通（ping 192.168.1.11）
#     3. 本脚本会自动检测摄像头设备
# ==============================================================================

set -e  # 遇到错误立即退出

echo "======================================================================"
echo "  香橙派手势控制系统 - 启动脚本"
echo "======================================================================"

# 获取虚拟机IP（参数或默认值）
VM_IP="${1:-192.168.1.11}"
VM_PORT="11311"

echo ""
echo "[INFO] 配置信息:"
echo "  香橙派 IP: 192.168.1.12"
echo "  虚拟机 IP: $VM_IP"
echo "  ROS Master: http://$VM_IP:$VM_PORT"
echo ""

# 设置ROS环境变量
echo "[INFO] 设置ROS环境变量..."
export ROS_MASTER_URI="http://$VM_IP:$VM_PORT"
export ROS_IP="192.168.1.12"

echo "  ROS_MASTER_URI = $ROS_MASTER_URI"
echo "  ROS_IP = $ROS_IP"
echo ""

# 自动检测可用的摄像头设备（逐个测试 /dev/video0 ~ /dev/video4）
echo "[INFO] 检测摄像头设备..."
VIDEO_DEVICE=""
for i in 0 1 2 3 4; do
    DEV="/dev/video$i"
    if [ -e "$DEV" ]; then
        if python3 -c "
import cv2
cap = cv2.VideoCapture('$DEV')
ok = cap.isOpened()
if ok:
    ret, frame = cap.read()
    ok = ret and frame is not None
cap.release()
exit(0 if ok else 1)
" 2>/dev/null; then
            VIDEO_DEVICE="$DEV"
            echo "  [OK] 摄像头: $DEV (可正常读取画面)"
            break
        else
            echo "  [--] $DEV 存在但无法读取画面，跳过"
        fi
    fi
done
if [ -z "$VIDEO_DEVICE" ]; then
    echo "  [ERROR] 未找到可用摄像头设备！"
    echo "  请检查摄像头是否已连接"
    exit 1
fi
echo ""

# 检查虚拟机是否可达
echo "[INFO] 检查网络连接..."
if ping -c 1 -W 2 "$VM_IP" > /dev/null 2>&1; then
    echo "  [OK] 虚拟机 $VM_IP 可达"
else
    echo "  [WARN] 虚拟机 $VM_IP 不可达，请检查网络连接"
    echo "         尝试继续运行..."
fi
echo ""

echo "======================================================================"
echo "  正在启动手势控制程序..."
echo "======================================================================"
echo ""
echo "使用说明:"
echo "  - 食指指向: WASD方向控制"
echo "  - 左手张开: 解锁+悬停（相当于按键1）"
echo "  - 左手握拳: 降落（相当于按键3）"
echo "  - 空格键:   手动解锁"
echo "  - T键:      手动起飞到2米"
echo "  - L键:      手动降落"
echo "  - ESC键:    退出程序"
echo ""
echo "======================================================================"
echo ""

# 运行主程序
python3 main.py --vm-ip "$VM_IP" --vm-port "$VM_PORT" --device "$VIDEO_DEVICE"