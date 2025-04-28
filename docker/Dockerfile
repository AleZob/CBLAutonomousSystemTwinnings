# Use Ubuntu 22.04 as the base image (compatible with ROS 2 Humble)
FROM ubuntu:22.04

# Set non-interactive mode for apt-get to prevent prompts
ENV DEBIAN_FRONTEND=noninteractive

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
    chromium-browser \
    && rm -rf /var/lib/apt/lists/*

# Set up UTF-8 locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

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
    && echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc \
    && bash -c "source ~/.bashrc"

# Set up ROS 2 environment
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0
ENV ROS_PACKAGE_PATH=/opt/ros/$ROS_DISTRO/share
ENV PATH=/opt/ros/$ROS_DISTRO/bin:$PATH
ENV LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3.8/site-packages:$PYTHONPATH
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc \
    && echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc \
    && bash -c "source ~/.bashrc"

# Set up Unity Hub
ENV UNITY_PGP_KEY=/usr/share/keyrings/Unity_Technologies_ApS.pgp
## Downloading PGP key
RUN wget -qO - https://hub.unity3d.com/linux/keys/public \
    | gpg --dearmor \
    | tee ${UNITY_PGP_KEY} \
    > /dev/null
## Adding Unity to Apt sources
RUN echo \
    "deb [signed-by=${UNITY_PGP_KEY}] https://hub.unity3d.com/linux/repos/deb stable main" \
    > /etc/apt/sources.list.d/unityhub.list
RUN apt update
## Installing dependencies
## oops it's redundant
# RUN wget http://security.ubuntu.com/ubuntu/pool/main/o/openssl/libssl3t64_3.4.1-1ubuntu3_amd64.deb \
#     && sudo dpkg -i libssl3t64_3.4.1-1ubuntu3_amd64.deb
## Installing Unity Hub
RUN apt install -y unityhub=3.4.1
## Fixing permissions
RUN echo "abi <abi/4.0>, \n include <tunables/global> \n profile unityhub /opt/unityhub/unityhub-bin flags=(unconfined) { \n  userns, \n # Site-specific additions and overrides. See local/README for details. \n include if exists <local/unityhub> \n }\n" > /etc/apparmor.d/unityhub

# Source ROS 2 setup script automatically
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

# Set DISPLAY environment variable for GUI applications in WSL2
# ENV DISPLAY=:0
# ENV QT_X11_NO_MITSHM=1

# Add a user so non-root processes work
RUN useradd cbl-user          \
    --shell /bin/bash         \
    --groups sudo             \
    --gid root                \
    --create-home             \
    --home-dir /home/cbl-user
## Blank their password 
RUN echo 'cbl-user:assword' | chpasswd
USER cbl-user
WORKDIR /home/user

# Start a bash shell by default
CMD ["/bin/bash"]
