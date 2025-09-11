#!/bin/bash
echo "Building data_recorder package..."
cd /home/nvidia/auto_clean_bot
colcon build --packages-select data_recorder
echo "data_recorder build completed!"