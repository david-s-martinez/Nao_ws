################################################################################
################################################################################
#
#           Stage: base-ros
#
################################################################################
################################################################################


FROM nvidia/cudagl:11.2.2-base-ubuntu16.04 as base-ros


# USE German mirror to speed up things
RUN cp /etc/apt/sources.list /etc/apt/sources.list.old \
    && sed -i -e 's/http:\/\/archive\.ubuntu\.com\/ubuntu\// \
    http:\/\/de.archive\.ubuntu\.com\/ubuntu/' /etc/apt/sources.list
RUN apt-key update && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN apt-get update \
    && apt-get remove -y openssl \
    && apt-get install -y --no-install-recommends \
        git \
        ssh \
        openssl \
        curl \
        wget \
        zsh \
        python-pip \
        mesa-utils \
        build-essential
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


# install bootstrap tools
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    python-catkin-tools \
    tzdata
    

# fix ssl cert for rosdep
RUN apt-get install ca-cacert \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*
################################################################################
################################################################################
#
#           Stage: hrs-ros
#
################################################################################
################################################################################

FROM base-ros as hrs-ros

# Install apt dependencies
ENV DEBIAN_FRONTEND noninteractive
ENV DEBIAN_FRONTEND teletype
RUN apt-get update && \
        yes | apt-get install -y --no-install-recommends \
        'ros-kinetic-nao*' \
    && rm -rf /var/lib/apt/lists/*

# Copy PyNaoQi into image and add to pythonpath
ADD pynaoqi /pynaoqi
ENV PYTHONPATH "${PYTHONPATH}:/pynaoqi"
ENV LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:/pynaoqi"


################################################################################
################################################################################
#
#           Stage: base-ros-user
#
################################################################################
################################################################################

FROM hrs-ros as hrs-ros-user

ARG USERNAME=devel
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
#ADD .bashrc /home/$USERNAME/.bashrc
################################################################################
################################################################################
#
#           Stage: team-ros
#
################################################################################
################################################################################

FROM hrs-ros-user as team-ros

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# fix openssl dir for rosdep and reinit rosdep 
# (why does openssl use /home/hkouki/Downloads/openssl/ssl/ as $OPENSSLDIR ???)
RUN sudo mkdir -p /home/hkouki/Downloads/openssl/ssl \
    && sudo cp -r /etc/ssl/* /home/hkouki/Downloads/openssl/ssl/ \
    && sudo rm /etc/ros/rosdep/sources.list.d/20-default.list \
    && sudo rosdep init \
    && rosdep update


# set team-specific settings in separate layer to speed up builds
ENV NAO_IP "10.152.246.180"
# add ros to .bashrc so that its sourced in each terminal
# RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/$USERNAME/.bashrc

    