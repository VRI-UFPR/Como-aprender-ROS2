#!/bin/bash

# Exemplo mostra
# - Abre o simulador gazebo no mapa padrao e robo waffle
# - Abre a pilha de SLAM junto com RVIZ jah com tudo configurado
# - Abre o rqt_graph mostrando a arquitetura de nodos e topicos

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True &
rqt_graph