#!/bin/bash
echo "Starting data recorder node..."
cd /home/nvidia/auto_clean_bot
source install/setup.bash
ros2 launch data_recorder data_recorder.launch.py