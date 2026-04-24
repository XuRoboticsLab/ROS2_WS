#!/usr/bin/env zsh
# ROS2 workspace startup script
# Usage: ./start.sh

SESSION="ros_$(date +%H%M%S)"
WS_DIR="$(cd "$(dirname "$0")" && pwd)"
SETUP="source $WS_DIR/code/ros_pkgs/install/setup.zsh"
CONDA_ACTIVATE='eval "$(conda shell.zsh hook)" && conda activate ros-humble'

# Create new session with first window: rosbridge (window 0)
tmux new-session -d -s "$SESSION" -n "rosbridge" -x 220 -y 50
tmux send-keys -t "$SESSION:0" "$CONDA_ACTIVATE && $SETUP && ros2 launch rosbridge_server rosbridge_websocket_launch.xml" Enter
sleep 1

# Window 1: camera 1
tmux new-window -t "$SESSION" -n "camera_1"
tmux send-keys -t "$SESSION:1" "$CONDA_ACTIVATE && $SETUP && ros2 run camera camera_publisher --config $WS_DIR/configs/camera_1.yaml" Enter
sleep 1

# Window 2: camera 2
tmux new-window -t "$SESSION" -n "camera_2"
tmux send-keys -t "$SESSION:2" "$CONDA_ACTIVATE && $SETUP && ros2 run camera camera_publisher --config $WS_DIR/configs/camera_2.yaml" Enter
sleep 1

# Window 3: camera 3
tmux new-window -t "$SESSION" -n "camera_3"
tmux send-keys -t "$SESSION:3" "$CONDA_ACTIVATE && $SETUP && ros2 run camera camera_publisher --config $WS_DIR/configs/camera_3.yaml" Enter
sleep 1

# Window 4: foot pedal
tmux new-window -t "$SESSION" -n "foot_pedal"
tmux send-keys -t "$SESSION:4" "sudo /home/xuroboticslab/miniforge3/envs/panthera/bin/python $WS_DIR/code/foot_pedal/main.py --config $WS_DIR/configs/foot_pedal.yaml" Enter

# Focus first window
tmux select-window -t "$SESSION:0"

echo "Attaching to session '$SESSION'..."
echo "Detach with: Ctrl+b d | Switch windows: Ctrl+b [0-4] or Ctrl+b n/p"
tmux attach-session -t "$SESSION"