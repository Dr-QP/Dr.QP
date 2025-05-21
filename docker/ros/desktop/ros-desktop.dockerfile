ARG FROM_IMAGE=ghcr.io/dr-qp/ubuntu-ansible:edge

FROM $FROM_IMAGE

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=$ROS_DISTRO
ARG ROS_USERNAME=rosdev

# UID should be 1001 to match ubuntu-latest in order for file writing to work for @action/checkout
# see https://github.com/actions/checkout/issues/956 for more details
ARG ROS_UID=1001
ARG ROS_GID=$ROS_UID

ARG CLANG_VERSION=20

# Install ROS
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,readonly,source=..,target=/ros-scripts \
    apt-get update \
    && cd /ros-scripts/ansible \
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

