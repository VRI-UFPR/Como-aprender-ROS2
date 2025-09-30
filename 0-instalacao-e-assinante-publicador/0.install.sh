#!/bin/bash

sudo apt update && sudo apt install locales
sudo locale-gen pt_BR pt_br.UTF-8
sudo update-locale LC_ALL=pt_BR.UTF-8 Lang=en_US.UTF-8
export LANG=pt_BR.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl 
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade

sudo apt install ros-humble-desktop ros-humble-ros-base ros-dev-tools

echo 'source /opt/ros/humble/setup.bash'

