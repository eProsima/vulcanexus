#!/bin/bash

set -e

if (( $EUID == 0 )); then
    shopt -s expand_aliases
    alias sudo=''
fi

if !(locale | grep -e 'utf8' -e 'UTF-8') >/dev/null 2>&1; then

##LINUX_BINARY_LOCALE
locale  # check for UTF-8

sudo apt update && sudo apt install -y locales
# Any UTF-8 locale will work. Using en_US as an example
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
##!

locale

fi

##LINUX_BINARY_UBUNTU_UNIVERSE
apt-cache policy | grep universe

# This should print something similar to:
#
#  500 http://us.archive.ubuntu.com/ubuntu jammy/universe amd64 Packages
# release v=22.04,o=Ubuntu,a=jammy,n=jammy,l=Ubuntu,c=universe,b=amd64
#
# Otherwise run

sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
##!

##LINUX_BINARY_KEYSTORE
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
##!

##LINUX_BINARY_REPO_SOURCELIST
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
##!


##LINUX_BINARY_VULCA_SOURCES
sudo curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg
##!

##LINUX_BINARY_VULCA_ADD_REPO
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] http://repo.vulcanexus.org/debian $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/vulcanexus.list > /dev/null
##!

##LINUX_BINARY_VULCA_UPDATE
sudo apt update -y
##!

##LINUX_BINARY_VULCA_INSTALL
sudo apt install -y vulcanexus-kilted-desktop
##!
