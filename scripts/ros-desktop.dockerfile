# Building off the base image saves a bit of space, but the build time is longer
FROM ros:humble-ros-base

# TODO (anton-matosov): Investigate if its really needed for devcontainer to use non root user
# https://docs.github.com/en/actions/sharing-automations/creating-actions/dockerfile-support-for-github-actions#user
# Docker actions must be run by the default Docker user (root). Do not use the USER instruction in your Dockerfile,
# because you won't be able to access the GITHUB_WORKSPACE directory. For more information, see [Store information]
# (https://docs.github.com/en/actions/learn-github-actions/variables#default-environment-variables)
# in variables and [USER](https://docs.docker.com/engine/reference/builder/#user) reference in the Docker documentation.
ARG USERNAME=rosdev
# UID should be 1001 to match ubuntu-latest in order for file writing to work for @action/checkout
# see https://github.com/actions/checkout/issues/956 for more details
ARG UID=1001
ARG GID=$UID


# Force clang-format-19 and friends to the default
ENV PATH="/usr/lib/llvm-19/bin:$PATH"

# Install ROS packages
COPY ./ros /ros-prep
RUN env CI=1 /ros-prep/ros-2-prep.sh \
  && apt clean \
  && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Add configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

RUN sudo apt-get update \
    && rosdep update \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Setup entrypoint
COPY ./ros/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

