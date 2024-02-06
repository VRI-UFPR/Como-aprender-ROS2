#!/bin/bash

# Exemplo mostra
# - Abre o simulador gazebo no mapa padrao e robo waffle

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
