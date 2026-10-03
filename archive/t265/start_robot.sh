#!/bin/bash



# --- 配置 ---
# Tmux 会话名称，可以自定义
SESSION_NAME="robot_session"

# ROS2 安装路径，根据您的实际情况修改
ROS2_SETUP_PATH="/opt/ros/humble/setup.bash" # 示例为 humble

# ROS2 工作空间路径，根据您的实际情况修改
ROS2_WS_PATH="~/ros2_ws/install/setup.bash" # 假设您的工作空间在 ~/ros2_ws

# T265 命令
# 将T265配置放在这里，为了清晰，我们把它存为一个变量
T265_CMD="ros2 launch realsense2_camera rs_launch.py     camera_name:=t265     enable_fisheye1:=true     enable_fisheye2:=true     enable_pose:=true     unite_imu_method:=2     publish_odom_tf:=true     tf_publish_rate:=30.0     output:=screen"

# --- 脚本主体 ---

# 检查 tmux 是否已经存在同名会话
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
    echo "创建新的 tmux 会话: $SESSION_NAME"

    # 1. 创建一个新的、分离的（-d）tmux会话
    #    第一个窗格 (0) 默认创建
    tmux new-session -d -s $SESSION_NAME -n "Services"

    # -------------------------------------------------------------
    # 窗格 0: 运行 MicroXRCEAgent
    # -------------------------------------------------------------
    # -t 指定目标会话和窗格
    # "source ... && command" 确保在该窗格的环境是正确的
    # C-m 发送一个回车键来执行命令
    tmux send-keys -t $SESSION_NAME:0.0 "source $ROS2_SETUP_PATH && source $ROS2_WS_PATH && MicroXRCEAgent serial -D /dev/ttyTHS1 -b 921600" C-m
    tmux rename-window -t $SESSION_NAME:0 'Services'


    # -------------------------------------------------------------
    # 窗格 1: 运行 ROS2 Launch 文件
    # -------------------------------------------------------------
    # -v 表示垂直分割上一个窗格，创建一个新的窗格
    tmux split-window -v -t $SESSION_NAME:0.0
    tmux send-keys -t $SESSION_NAME:0.1 "source $ROS2_SETUP_PATH && source $ROS2_WS_PATH && ros2 launch realsense2_camera imu_rtabmap.launch.py" C-m
    # -------------------------------------------------------------
    # 窗格 2: 运行 T265 
    # -------------------------------------------------------------
    # -h 表示水平分割上一个窗格
    tmux split-window -h -t $SESSION_NAME:0.1
    # 注意：T265命令被包装在变量中，以提高可读性
    tmux send-keys -t $SESSION_NAME:0.2 "$T265_CMD" C-m

    # -------------------------------------------------------------
    # 布局和焦点
    # -------------------------------------------------------------
    # 选择一个预设的布局，让三个窗格平铺
    tmux select-layout -t $SESSION_NAME:0 tiled

    # -------------------------------------------------------------
    # 创建第二个窗口: Detection Services
    # -------------------------------------------------------------
    tmux new-window -t $SESSION_NAME:1 -n "Detection"

    # 窗格 0: 运行 USB 摄像头
    tmux send-keys -t $SESSION_NAME:1.0 "source $ROS2_SETUP_PATH && source $ROS2_WS_PATH && ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /home/cfly/yolo_ws/config/params_usb.yaml" C-m

    # 窗格 1: 运行 YOLO 检测
    tmux split-window -v -t $SESSION_NAME:1.0
    tmux send-keys -t $SESSION_NAME:1.1 "source $ROS2_SETUP_PATH && source $ROS2_WS_PATH && source ~/yolo_ws/install/setup.bash && ros2 run yolo_detection yolo_scene" C-m

    # 设置Detection窗口的布局
    tmux select-layout -t $SESSION_NAME:1 tiled

fi

# -------------------------------------------------------------
# 附加到会话
# -------------------------------------------------------------
# 无论会话是新建的还是已存在的，都附加到该会话
echo "附加到 tmux 会话: $SESSION_NAME"
echo "使用 'Ctrl+b' 然后按方向键在窗格间切换。"
echo "使用 'Ctrl+b' 然后按 '0' 切换到 Services 窗口。"
echo "使用 'Ctrl+b' 然后按 '1' 切换到 Detection 窗口。"
echo "使用 'Ctrl+b' 然后按 'd' 可以分离会话（程序将继续在后台运行）。"
echo "使用 'tmux attach -t $SESSION_NAME' 可以重新连接。"
sleep 1
tmux attach-session -t $SESSION_NAME