# Respati UGV
Respati UGV is an Unmanned Ground Vehicle system using Jetson Nano and ROS Humble.
## Features ##
## Requirements ##
### OS ###
Make sure you have Ubuntu 20.04 installed on Jetson Nano. To install said OS, refer to this [guide](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image).
### ROS Humble ###
#### Set Locale ####
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
#### Install Dependencies ####
```
sudo apt update && sudo apt install gnupg wget
sudo apt install software-properties-common
sudo add-apt-repository universe
```
#### Setup Source ####
First, register this GPG key with `apt`. Two options are listed below, one for `.com` (US CDN) and one for `.cn` (China CDN).
```
wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
```
```
wget -qO - https://isaac.download.nvidia.cn/isaac-ros/repos.key | sudo apt-key add -
```
Now add the repository  to your `apt` sources.
```
echo 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' | sudo tee -a /etc/apt/sources.list
```
```
echo 'deb https://isaac.download.nvidia.cn/isaac-ros/ubuntu/main focal main' | sudo tee -a /etc/apt/sources.list
```
Next, ensure you have ROS 2 apt repository sourced.
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
#### Install ROS 2 Packages ####
```
sudo apt update
```
```
sudo apt upgrade
```
Desktop Install (Recommended): ROS, RViz, demos, tutorials.
```
sudo apt install ros-humble-desktop
```
#### Environment Setup ####
Set up your environment by sourcing the following file.
```
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```
## Usage ##
