# Tutorial primeiros passos para o SLAM

Este tutorial objetiva apresentar um robo simples usando um SLAM para elaborar o Mapa. Este tutorial não dará base suficiente para fazer o mesmo em um robo diferente. Porém o suficiente para mostrar as ferramentas utilizadas e estrutura básica de um robo com SLAM.

# Video Base para o Tutorial

https://www.youtube.com/watch?v=idQb2pB-h2Q

# Pré-requisitos
- sudo apt install terminator
- sudo apt install ros-humble-turtlebot3-gazebo
- sudo apt install ros-humble-turtlebot3-teleop
- sudo apt install ros-humble-turtlebot3-cartographer

# inicializa o gazebo com o turtlebot3 jah configurado
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# controlar o robo
ros2 run turtlebot3_teleop teleop_keyboard

# gera o grafo do robo
rqt_graph

# constroi o mapa
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
