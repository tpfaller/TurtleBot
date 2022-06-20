#!/bin/bash

#read -sp "Enter password: " PASSWORD
PASSWORD="aimotion"

COMMANDS=(
  "roscore"
  "sshpass -p $PASSWORD ssh aimotion@192.168.0.200 -t sudo date -s $(date '+%Y-%m-%d\\ %H:%M:%S')"
  "sshpass -p $PASSWORD ssh aimotion@192.168.0.200 -t 'roslaunch turtlebot3_bringup turtlebot3_robot.launch'"
  "sshpass -p $PASSWORD ssh aimotion@192.168.0.200 -t 'roslaunch realsense2_camera rs_camera.launch'"
  "roslaunch rosbridge_server rosbridge_websocket.launch"
  "cd uxd ; npm run dev"
  "python segmentation/server.py"
  "python2 object_detection/scripts/publish_objects.py"
  "python2 launchfiles/lite-team.py"
)

send_command_to_pane() {
  tmux send-keys -t "$1" "$2" C-m
}

spawn_panes() {
  for ((i = 0; i < ${#COMMANDS[@]}; ++i)); do
    if [[ $i == 0 ]]; then
      tmux new-session -dn "$1" "${COMMANDS[$i]}"
      tmux set-option mouse on
      tmux set-option remain-on-exit on
      tmux bind-key r respawn-pane -k
    else
      tmux split-window -d "${COMMANDS[$i]}"
      tmux select-layout tiled
    fi

    if [[ ${COMMANDS[$i]} == *"sudo"* ]]; then
      send_command_to_pane %$i $PASSWORD
    fi
  done
}

spawn_panes "TurtleBot"

tmux attach

exit 0
