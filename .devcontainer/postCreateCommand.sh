#!/bin/bash

sudo apt update && \
sudo apt install ros-humble-rviz2 -y && \
cd raspberryPI3/ros2_ws && \
colcon build --packages-select interfaces && \
colcon build --packages-skip interfaces && \
sudo chown -R $(whoami) /home/pi/geicar
