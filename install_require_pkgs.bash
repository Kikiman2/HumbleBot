#Setup the mobil robot environment

# Update system
sudo apt update
sudo apt upgrade

# Install ROS2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
sudo apt install python3-colcon-common-extensions
source /opt/ros/humble/setup.bash
cd ~
mkdir ros2_ws
cd ros2_ws
mkdir src
colcon build
source install.setup.bash

# Install camera drivers
sudo apt install ros-humble-image-transport-plugins
sudo apt install ros-humble-rqt-image-view
sudo apt install v4l-utils ros-humble-v4l2-camera

# Install RPLidar drivers
sudo apt install ros-humble-rplidar-ros

# Install Raspberry Pi pin drivers
sudo apt-get install python3-pip
pip3 install gpiozero