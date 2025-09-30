#!/bin/bash

sudo apt install ros-humble-turtlebot3-gazebo-* ros-humble-turtlebot3-teleop ros-humble-cartographer ros-humble-cartographer-ros ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-navigation2
source /opt/ros/humble/setup.bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2 # pacote de exploração de fronteira, marca lugares não explorados para o robo melhorar o mapa de forma autonoma

sudo apt install python3-colcon-common-extensions
cd ~/turtlebot3_ws
colcon build --symlink-install

