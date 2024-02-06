#!/bin/bash

# Exemplo mostra
# - Abre o simulador gazebo no mapa padrao e robo waffle
# - Abre a pilha de SLAM junto com RVIZ jah com tudo configurado
# - Abre o turtlebot3_teleop para controlar o robo

# lembre-se que o teleop soh pega os valores do teclado, quando o terminal estiver como primeiro plano
# nao adianta apertar as teclas olhando para o gazebo

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True &
ros2 run turtlebot3_teleop teleop_keyboard