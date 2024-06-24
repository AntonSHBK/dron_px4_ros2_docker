# FROM ros:jazzy
FROM osrf/ros:jazzy-desktop-full

# set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Install dependencies (if any)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Set shell console and install some requirements 
SHELL [ "/bin/bash" , "-c" ]

# Development tools
RUN apt-get update && apt-get install -y \
    zsh \
    vim \
    git \
    tmux \
    tree \
    && rm -rf /var/lib/apt/lists/*

# Install necessary tools and libraries for Gazebo
RUN apt-get update && apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    ros-jazzy-gazebo-ros \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-gazebo-ros-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gazebo-plugins \
    ros-jazzy-control-toolbox \
    ros-jazzy-realtime-tools \
    ros-jazzy-ros-control \
    ros-jazzy-ros-controllers \
    ros-jazzy-xacro \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

# Install MAVROS and MAVLink
RUN apt-get update && apt-get install -y \
    ros-jazzy-mavros \
    ros-jazzy-mavros-msgs \
    ros-jazzy-mavlink \
    && rm -rf /var/lib/apt/lists/*

# Install QGroundControl
RUN apt-get update && apt-get install -y \
    qt5-default \
    libqt5svg5-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
RUN chmod +x QGroundControl.AppImage

# Install PX4 packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-uuv-simulator \
    ros-jazzy-octomap \
    && rm -rf /var/lib/apt/lists/*

# Install ArduPilot packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-ardupilot \
    && rm -rf /var/lib/apt/lists/*

# Install Python 3 and pip
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# install python dependencies
COPY ../requirements.txt /tmp/requirements.txt
RUN pip install --upgrade pip && \
    pip install -r /tmp/requirements.txt && \
    rm -rf /root/.cache/pip

# set work directory
ENV WORKSPACE /workspace/src
RUN mkdir -p $WORKSPACE/src
WORKDIR $WORKSPACE

# copy project
COPY . $WORKSPACE

# Source the ROS 2 setup file for the entry point
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "alias sros='source install/setup.bash'" >> ~/.bashrc

# Set the entry point
CMD ["zsh"]

RUN echo "All done!"