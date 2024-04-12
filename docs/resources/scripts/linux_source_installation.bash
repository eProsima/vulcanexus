#!/bin/bash

set -e

if (( $EUID == 0 )); then
    shopt -s expand_aliases
    alias sudo=''
fi

if !(locale | grep -e 'utf8' -e 'UTF-8') >/dev/null 2>&1; then

##LINUX_SOURCE_LOCALE
locale  # check for UTF-8

sudo apt update && sudo apt install -y locales
# Any UTF-8 locale will work. Using en_US as an example
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
##!

locale

fi

##LINUX_SOURCE_UBUNTU_UNIVERSE
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

##LINUX_SOURCE_KEYSTORE
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
##!

##LINUX_SOURCE_REPO_SOURCELIST
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
##!

##LINUX_SOURCE_ROS2_DEPS
sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget
##!

##LINUX_SOURCE_CLONE_ROS2_REPOS
mkdir -p ~/vulcanexus_iron/src
cd ~/vulcanexus_iron
wget https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos
vcs import src < ros2.repos
##!

##LINUX_SOURCE_DOWN_ROS2_DEPS
sudo apt upgrade -y
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
##!

##LINUX_SOURCE_CLONE_VULCA
cd ~
cd vulcanexus_iron

# Remove ROS 2 packages overridden by Vulcanexus
rm -rf \
    src/eProsima/foonathan_memory_vendor/ \
    src/ros2/rosidl_typesupport_fastrtps/ \
    src/ros2/rosidl_dynamic_typesupport_fastrtps \
    src/ros2/rmw_fastrtps/ \
    src/ros2/rosidl/rosidl_adapter/ \
    src/ros2/rosidl/rosidl_cli/ \
    src/ros2/rosidl/rosidl_cmake/ \
    src/ros2/rosidl/rosidl_generator_c/ \
    src/ros2/rosidl/rosidl_generator_cpp/ \
    src/ros2/rosidl/rosidl_generator_tests/ \
    src/ros2/rosidl/rosidl_generator_type_description/ \
    src/ros2/rosidl/rosidl_parser/ \
    src/ros2/rosidl/rosidl_pycommon/ \
    src/ros2/rosidl/rosidl_runtime_c/ \
    src/ros2/rosidl/rosidl_runtime_cpp/ \
    src/ros2/rosidl/rosidl_typesupport_interface/ \
    src/ros2/rosidl/rosidl_typesupport_introspection_c/ \
    src/ros2/rosidl/rosidl_typesupport_introspection_cpp/ \
    src/ros2/rosidl/rosidl_typesupport_introspection_tests/

# Get Vulcanexus sources
wget https://raw.githubusercontent.com/eProsima/vulcanexus/iron/vulcanexus.repos
wget https://raw.githubusercontent.com/eProsima/vulcanexus/iron/colcon.meta
vcs import --force src < vulcanexus.repos

# Avoid compilation of some documentation and demo packages
touch src/eProsima/Fast-DDS-QoS-Profiles-Manager/docs/COLCON_IGNORE
touch src/eProsima/Vulcanexus-Base/docs/COLCON_IGNORE
touch src/eProsima/Vulcanexus-Base/code/COLCON_IGNORE
##!

##LINUX_SOURCE_VULCA_DEPS
sudo apt update && sudo apt install -y \
    libasio-dev \
    libdocopt-dev \
    libengine-pkcs11-openssl \
    liblog4cxx-dev \
    liblz4-dev \
    libp11-dev \
    libqt5charts5-dev \
    libssl-dev \
    libtinyxml2-dev \
    libxerces-c-dev \
    libyaml-cpp-dev \
    libzstd-dev \
    openjdk-8-jdk \
    python3-sphinx \
    python3-sphinx-rtd-theme \
    qtbase5-dev \
    qtdeclarative5-dev \
    qtquickcontrols2-5-dev \
    swig
##!

##LINUX_SOURCE_VULCA_COMPILE
cd ~/vulcanexus_iron
colcon build
##!
