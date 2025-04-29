ARG ROS_DISTRO=jazzy

# FROM ubuntu:24.04 # locale remains at POSIX... figure out why later. use ros image that has proper locale
FROM ros:$ROS_DISTRO-ros-base
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8


# TODO (anton-matosov): Investigate if its really needed for devcontainer to use non root user
# https://docs.github.com/en/actions/sharing-automations/creating-actions/dockerfile-support-for-github-actions#user
# Docker actions must be run by the default Docker user (root). Do not use the USER instruction in your Dockerfile,
# because you won't be able to access the GITHUB_WORKSPACE directory. For more information, see [Store information]
# (https://docs.github.com/en/actions/learn-github-actions/variables#default-environment-variables)
# in variables and [USER](https://docs.docker.com/engine/reference/builder/#user) reference in the Docker documentation.
ARG ROS_USERNAME=rosdev

# UID should be 1001 to match ubuntu-latest in order for file writing to work for @action/checkout
# see https://github.com/actions/checkout/issues/956 for more details
ARG ROS_UID=1001
ARG ROS_GID=$ROS_UID

ARG CLANG_VERSION=20

RUN env | sort

# Install Ansible
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,readonly,source=..,target=/ros-scripts \
    apt-get update && apt-get install -y python3 sudo \
    && /ros-scripts/ansible/setup-ansible.sh

# Install ROS
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,readonly,source=..,target=/ros-scripts \
    cd /ros-scripts/ansible \
    && ansible-playbook playbooks/20_ros_setup.yml \
      -i inventories/localhost.yml \
      -vvv \
      -e "ci_mode=true setup_user=true ros_user_setup_username=$ROS_USERNAME ros_user_setup_uid=$ROS_UID ros_user_setup_gid=$ROS_GID clang_version=$CLANG_VERSION ros_distro=$ROS_DISTRO"

WORKDIR /home/$ROS_USERNAME/ros2_ws
USER $ROS_USERNAME

# Force clang installed by llvm.sh in /usr/lib/llvm-${CLANG_VERSION}/bin to be the default in docker
ENV PATH="/usr/lib/llvm-${CLANG_VERSION}/bin:/home/$ROS_USERNAME/.local/bin:$PATH"
ENV CC=clang
ENV CXX=clang++

# Setup entrypoint
COPY ../deploy/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

