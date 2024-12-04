#!/bin/bash

sudo apt update && \
sudo apt install ros-humble-rviz2 -y && \
pip3 install --upgrade pip && \
pip3 install numpy scipy pandas scikit-learn tensorflow torch ultralytics opencv-python && \
cd raspberryPI3/ros2_ws && \
colcon build --packages-select interfaces && \
colcon build --packages-skip interfaces && \
sudo chown -R $(whoami) /home/pi/geicar