ARG ROS_DISTRO=jazzy

FROM ubuntu:24.04

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

# Force clang-format-20 and friends to the default
ENV CLANG_VERSION=20
ENV PATH="/usr/lib/llvm-${CLANG_VERSION}/bin:/home/rosdev/.local/bin:$PATH"
ENV CC=clang
ENV CXX=clang++

# Install ROS
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,readonly,source=..,target=/ros-scripts \
    apt-get update && apt-get install -y python3 sudo \
    && /ros-scripts/ansible/setup-ansible.sh \
    && ansible-playbook -i /ros-scripts/ansible/inventories/localhost.yml \
       /ros-scripts/ansible/playbooks/20_ros_setup.yml \
       -e "ci_mode=true setup_user=true ros_user_setup_username=$USERNAME ros_user_setup_uid=$UID ros_user_setup_gid=$GID clang_version=$CLANG_VERSION ros_distro=$ROS_DISTRO"

WORKDIR /home/$USERNAME/ros2_ws
USER $USERNAME

# Setup entrypoint
COPY ../deploy/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

