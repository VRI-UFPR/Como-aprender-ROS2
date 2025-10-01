# Tutorial primeiros passos para o SLAM

Este tutorial objetiva apresentar um robo simples usando um SLAM para elaborar o Mapa. Este tutorial não dará base suficiente para fazer o mesmo em um robo diferente. Porém o suficiente para mostrar as ferramentas utilizadas e estrutura básica de um robo com SLAM.

# Video Base para o Tutorial

https://www.youtube.com/watch?v=idQb2pB-h2Q

## scripts de instalação
nesse repositório há um script de instalação (0-install.sh), que instala todos os pacotes necessários, e um script para adicionar as linhas necessárias no bashrc (1-bashrc.sh). Assumindo que o ros já foi instalado.

# Pré-requisitos
- sudo apt install terminator
- sudo apt install ros-humble-turtlebot3-gazebo
- sudo apt install ros-humble-turtlebot3-teleop
- sudo apt install ros-humble-turtlebot3-cartographer
- sudo apt install ros-humble-slam_toolbox
## Pré-requisitos para navegação autonoma

- sudo apt install ros-humble-nav2-bringup ros-humble-navigation2
- cd ~/turtlebot3_ws/src
- git clone https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2
- cd ~/turtlebot3_ws
- colcon build --symlink-install


# inicializa o gazebo com o turtlebot3 jah configurado
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# controlar o robo
ros2 run turtlebot3_teleop teleop_keyboard

# ou se preferir deixar o robo autonomo

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True & ros2 run custom_explorer explorer

# gera o grafo do robo
rqt_graph

# constroi o mapa
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

ou

ros2 launch slam-toolbox online_sync.launch.py use_sim_time:=True


