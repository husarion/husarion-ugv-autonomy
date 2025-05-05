#!/bin/bash

# Wait for network availability
while ! ping -c 1 -W 1 10.15.20.1 > /dev/null 2>&1; do
  echo "Waiting for network to be available..."
  sleep 1
done

# Proceed with the main command
ros2 launch husarion_ugv_docking docking.launch.py namespace:=lynx camera_image_topic:=/oak/rgb/image_rect camera_info_topic:=/oak/rgb/camera_info docking_server_config_path:=/config/docking_server_nonav.yaml apriltag_config_path:=/config/apriltag.yaml apriltag_size:=0.06 use_sim:=False
