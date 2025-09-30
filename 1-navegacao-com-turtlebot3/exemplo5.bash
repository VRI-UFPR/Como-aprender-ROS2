#!/bin/bash

# Exemplo mostra
# - Abre o Simulador gazebo no mapa padrão e robo burguer
# - Abre a pilha do Slam-toolbox, rviz, navigation, custom explorer
# - E rqt_graph mostrando a arquitetura de nodos e tópicos


source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py & ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True & ros2 launch slam_toolbox online_sync_launch.py & ros2 launch nav2_bringup rviz_launch.py & ros2 run custom_explorer explorer & rqt_graph
