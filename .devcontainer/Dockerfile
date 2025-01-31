FROM osrf/ros:humble-desktop

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# Install some dependencies packages
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    xauth \
    curl wget \
    ros-dev-tools \
    ninja-build \
    libbullet-dev \
    python3-colcon-common-extensions \
    python3-colcon-coveragepy-result \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    # install Fast-RTPS dependencies
    && apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev \
    # install Cyclone DDS dependencies
    libcunit1-dev \
    # Cleanup apt cache
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# install some pip packages needed for testing
RUN python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest

# Force clang-format-19 and friends to the default
ENV PATH="/usr/lib/llvm-19/bin:$PATH"

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# This needs to run in fish
# RUN curl -sL https://git.io/fisher | source && fisher install jorgebucaran/fisher \
#   && fisher install edc/bass

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
