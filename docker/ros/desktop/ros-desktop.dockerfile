ARG FROM_IMAGE=ghcr.io/dr-qp/ubuntu-ansible:edge

FROM $FROM_IMAGE

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=$ROS_DISTRO

ARG CLANG_VERSION=20
ARG OVERLAY_WS=/opt/ros/overlay_ws

# Install ROS
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,readonly,source=..,target=/ros-scripts \
    apt-get update \
    && cd /ros-scripts/ansible \
    && ansible-playbook playbooks/20_ros_setup.yml \
      -i inventories/localhost.yml \
      -vvv \
      -e "ci_mode=true clang_version=$CLANG_VERSION ros_distro=$ROS_DISTRO install_xpra=true"

WORKDIR $OVERLAY_WS

# Copy Xpra startup script
COPY ../desktop/start-xpra.sh /start-xpra.sh

# Expose Xpra port
EXPOSE 14500

# Force clang installed by llvm.sh in /usr/lib/llvm-${CLANG_VERSION}/bin to be the default in docker
ENV PATH="/usr/lib/llvm-${CLANG_VERSION}/bin:/root/.local/bin:$PATH"
ENV CC=clang
ENV CXX=clang++

# Setup entrypoint
COPY ../deploy/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

