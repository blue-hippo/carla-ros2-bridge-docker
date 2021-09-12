FROM nvidia/cudagl:11.4.1-devel-ubuntu20.04

ARG CARLA_VERSION=0.9.12
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV DEBIAN_FRONTEND noninteractive

# add new sudo user
ENV USERNAME carla
ENV HOME /home/$USERNAME
RUN useradd -m $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    mkdir /etc/sudoers.d && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    # Replace 1000 with your user/group id
    usermod  --uid 1000 $USERNAME && \
    groupmod --gid 1000 $USERNAME
# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -fsn /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -y --no-install-recommends \
    tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt install -y --no-install-recommends \
    sudo \
    vim \
    emacs \
    tmux \
    xsel \
    bash-completion \
    software-properties-common \
    xdg-user-dirs \
    xserver-xorg \
    libvulkan1 \
    pcl-tools \
    build-essential \
    git \
    python3-pip \
    python3-opencv \
    dirmngr \
    gnupg2 && \
    rm -rf /var/lib/apt/lists/*
RUN VULKAN_API_VERSION=`dpkg -s libvulkan1 | grep -oP 'Version: [0-9|\.]+' | grep -oP '[0-9|\.]+'` && \
	mkdir -p /etc/vulkan/icd.d/ && \
	echo \
	"{\
		\"file_format_version\" : \"1.0.0\",\
		\"ICD\": {\
			\"library_path\": \"libGLX_nvidia.so.0\",\
			\"api_version\" : \"${VULKAN_API_VERSION}\"\
		}\
	}" > /etc/vulkan/icd.d/nvidia_icd.json
# Suppress ALSA lib warning
RUN echo "pcm.!default { type plug slave.pcm \"null\" }" > /etc/asound.conf

# Use python3 instead of python2
RUN ln -sfn /usr/bin/python3 /usr/bin/python && \
    ln -sfn /usr/bin/pip3 /usr/bin/pip
RUN pip3 install \
    transforms3d \
    pygame \
    pexpect \
    simple-pid \
    networkx \
    numpy


# Setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO foxy
# Setup keys and sources.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list
# Install ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop && \
    rm -rf /var/lib/apt/lists/*
# Install bootstrap tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*
# Addtional ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-derived-object-msgs \
    ros-${ROS_DISTRO}-ackermann-msgs && \
    rm -rf /var/lib/apt/lists/*
RUN rosdep init

# setup colcon mixin and metadata
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

USER $USERNAME
WORKDIR $HOME
COPY CARLA_${CARLA_VERSION}.tar.gz $HOME
RUN mkdir CARLA_${CARLA_VERSION} && \
    tar xfvz CARLA_${CARLA_VERSION}.tar.gz -C CARLA_${CARLA_VERSION} && \
    rm $HOME/CARLA_${CARLA_VERSION}.tar.gz
RUN echo "export PYTHONPATH=$PYTHONPATH:~/CARLA_${CARLA_VERSION}/PythonAPI/carla/dist/carla-${CARLA_VERSION}-py3.8-linux-x86_64.egg:~/CARLA_${CARLA_VERSION}/PythonAPI/carla" >> ~/.bashrc

SHELL ["/bin/bash", "-c"]
RUN mkdir -p ~/carla-ros-bridge && \
    cd ~/carla-ros-bridge && \
    git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r && \
    colcon build

WORKDIR $HOME/CARLA_${CARLA_VERSION}
