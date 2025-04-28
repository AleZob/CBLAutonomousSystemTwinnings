#! /bin/sh
apt-get update
apt-get install -y \
    curl \
    git \
    gnupg2 \
    lsb-release \
    locales \
    x11-apps \
    mesa-utils \
    python3-argcomplete \
    chromium-browser
rm -rf /var/lib/apt/lists/*

locale-gen en_US.UTF-8

LANG=en_US.UTF-8
LC_ALL=en_US.UTF-8

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt-get update
apt-get upgrade -y
apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    ros-dev-tools

$SHELL -c "source /opt/ros/humble/setup.bash"

apt-get install -y \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
apt-get install -y python3-colcon-common-extensions
cd ~/turtlebot3_ws
bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
bash -c "source ~/.bashrc"

ROS_DISTRO=humble
ROS_DOMAIN_ID=0
ROS_PACKAGE_PATH=/opt/ros/$ROS_DISTRO/share
PATH=/opt/ros/$ROS_DISTRO/bin:$PATH
LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib:$LD_LIBRARY_PATH
PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3.8/site-packages:$PYTHONPATH

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
bash -c "source ~/.bashrc"

UNITY_PGP_KEY=/usr/share/keyrings/Unity_Technologies_ApS.pgp
wget -qO - https://hub.unity3d.com/linux/keys/public \
    | gpg --dearmor \
    | tee ${UNITY_PGP_KEY} \
    > /dev/null

echo \
    "deb [signed-by=${UNITY_PGP_KEY}] https://hub.unity3d.com/linux/repos/deb stable main" \
    > /etc/apt/sources.list.d/unityhub.list

apt update
apt install -y unityhub=3.4.1
echo "abi <abi/4.0>, \n include <tunables/global> \n profile unityhub /opt/unityhub/unityhub-bin flags=(unconfined) { \n  userns, \n # Site-specific additions and overrides. See local/README for details. \n include if exists <local/unityhub> \n }\n" > /etc/apparmor.d/unityhub
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc