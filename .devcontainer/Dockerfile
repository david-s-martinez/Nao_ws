################################################################################
################################################################################
#
#           Stage: base-ros
#
################################################################################
################################################################################


FROM osrf/ros:kinetic-desktop-full as base-ros

# USE German mirror to speed up things
RUN cp /etc/apt/sources.list /etc/apt/sources.list.old \
    && sed -i -e 's/http:\/\/archive\.ubuntu\.com\/ubuntu\// \
    http:\/\/de.archive\.ubuntu\.com\/ubuntu/' /etc/apt/sources.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        ssh \
        wget \
        zsh \
        python-catkin-tools \
        python-pip \
    && rm -rf /var/lib/apt/lists/*


################################################################################
################################################################################
#
#           Stage: hrs-ros
#
################################################################################
################################################################################

FROM base-ros as hrs-ros

# USE German mirror to speed up things
RUN cp /etc/apt/sources.list /etc/apt/sources.list.old \
    && sed -i -e 's/http:\/\/archive\.ubuntu\.com\/ubuntu\// \
    http:\/\/de.archive\.ubuntu\.com\/ubuntu/' /etc/apt/sources.list

# Install apt dependencies
ENV DEBIAN_FRONTEND noninteractive
ENV DEBIAN_FRONTEND teletype
RUN apt update && \
        yes | apt install -y --no-install-recommends \
        "ros-kinetic-nao*" \
    && rm -rf /var/lib/apt/lists/*

# Copy PyNaoQi into image and add to pythonpath
ADD pynaoqi /pynaoqi
ENV PYTHONPATH "${PYTHONPATH}:/pynaoqi"
ENV LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:/pynaoqi"

################################################################################
################################################################################
#
#           Stage: team-ros
#
################################################################################
################################################################################

FROM hrs-ros as team-ros

# set team-specific settings in separate layer to speed up builds
ENV NAO_IP "10.152.246.123"

    