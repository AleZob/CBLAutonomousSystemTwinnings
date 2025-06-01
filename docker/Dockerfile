#
# Leet's Dockerfile for 2IRR10
# Version 2
#
# Changes from TU/e Dockerfile:
# - Include neofetch, nano, net-tools, iputils-ping and iproute2
# - Automatically fetch ROS-TCP-Endpoint from GitHub and adds it to ROS environment
# - Option to use ZSH with Agnoster instead of default Bash environment
# - Slightly tweaked order of operations to prevent environment issues
# Check the CONFIG section to enable ZSH
#

# Use Ubuntu 22.04 as the base image (compatible with ROS 2 Humble)
FROM ubuntu:22.04

# Set non-interactive mode for apt-get to prevent prompts
ENV DEBIAN_FRONTEND=noninteractive

# CONFIG
# Start a bash shell when running a container (default)
CMD ["/bin/bash"]
# Start a zsh shell when running a container
# CMD ["/bin/zsh"]

# Update package lists and install dependencies
RUN apt-get update && apt-get install -y \
    curl \
	git \
    gnupg2 \
    lsb-release \
    locales \
    x11-apps \
    mesa-utils \
	python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Set up UTF-8 locale
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# Setup Sources
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    ros-humble-desktop \
	ros-humble-ros-base \
	ros-dev-tools

# Install addition packages (Custom)
RUN apt -y install neofetch net-tools iputils-ping iproute2 curl nano zsh
RUN sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
RUN cat ~/.zshrc | grep -v ZSH_THEME > temp \
    && echo 'export ZSH_THEME=agnoster' | cat - temp > temp2 \
    && mv temp2 ~/.zshrc && rm temp

# Environment setup
RUN bash -c "source /opt/ros/humble/setup.bash"

# Install dependent ROS 2 packages for TurtleBot3
RUN apt-get install -y \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
	ros-humble-cartographer-ros \
	ros-humble-navigation2 \
	ros-humble-nav2-bringup
	
# Install TurtleBot3 packages
RUN mkdir -p ~/turtlebot3_ws/src \
	&& cd ~/turtlebot3_ws/src/ \
	&& git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
	&& git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
	&& git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git \
	&& git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
	&& apt-get install -y python3-colcon-common-extensions \
	&& cd ~/turtlebot3_ws \
	&& bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install" \
    && bash -c "source ~/turtlebot3_ws/install/setup.bash"

# Install Unity x ROS2 (Custom)
RUN mkdir -p ~/ros2_ws/src \
     && cd ~/ros2_ws/src/ \
     && git clone -b dev-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \
     && cd ~/ros2_ws \
	&& bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install" \
    && bash -c "source ~/ros2_ws/install/setup.bash"

# Set up ROS 2 environment
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0
ENV ROS_PACKAGE_PATH=/opt/ros/$ROS_DISTRO/share
ENV PATH=/opt/ros/$ROS_DISTRO/bin:$PATH
ENV LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3.8/site-packages:$PYTHONPATH
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.zshrc \
    && echo 'export TURTLEBOT3_MODEL=burger' >> ~/.zshrc \
    && echo 'source /usr/share/gazebo/setup.sh' >> ~/.zshrc \
    && echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc \
    && echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc \
    && echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc \
    && bash -c "source /usr/share/gazebo/setup.sh"

# Source ROS 2 setup script automatically
RUN echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc \
	&& echo "source ~/turtlebot3_ws/install/setup.zsh" >> ~/.zshrc \
	&& echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc \
	&& echo "source ~/ros2_ws/install/setup.zsh" >> ~/.zshrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.zsh" >> ~/.zshrc

# Set console theme to Solarized (if supported by terminal, zsh only) (Custom)
RUN echo 'echo -e "' \
'\e]4;0;#073642\e\\\\' \
'\e]4;1;#dc322f\e\\\\' \
'\e]4;2;#859900\e\\\\' \
'\e]4;3;#b58900\e\\\\' \
'\e]4;4;#268bd2\e\\\\' \
'\e]4;5;#d33682\e\\\\' \
'\e]4;6;#2aa198\e\\\\' \
'\e]4;7;#eee8d5\e\\\\' \
'\e]4;8;#002b36\e\\\\' \
'\e]4;9;#cb4b16\e\\\\' \
'\e]4;10;#586e75\e\\\\' \
'\e]4;11;#657b83\e\\\\' \
'\e]4;12;#839496\e\\\\' \
'\e]4;13;#6c71c4\e\\\\' \
'\e]4;14;#93a1a1\e\\\\' \
'\e]4;15;#fdf6e3\e\\\\"' >> /root/.zshrc
RUN echo 'source /opt/ros/humble/share/ros2cli/environment/ros2-argcomplete.zsh' >> /root/.zshrc

# Set default shell for executing commands
SHELL ["/bin/bash", "-c"]

# Set DISPLAY environment variable for GUI applications in WSL2
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
