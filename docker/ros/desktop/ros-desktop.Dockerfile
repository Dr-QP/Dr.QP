# Digest-pinned so a Renovate bump rebuilds jazzy-ros-desktop against a newer base --
# the staleness trigger recorded as F7 in docs/agents/specs/agent-devcontainer-migration/.
# The pin lives under docker/**, inside the `ros` path filter, so merging the bump runs
# that build. Renovate manages it through the un-automerged rule in .github/renovate.json.
ARG FROM_IMAGE=ghcr.io/plume-works/agent-desktop:edge@sha256:8ac96714c15c826bcc2dab7ecf3dd56bab91073cde5450db4888be15dc49697c

FROM $FROM_IMAGE

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=$ROS_DISTRO

ARG CLANG_VERSION=20
ARG OVERLAY_WS=/opt/ros/overlay_ws

# Install ROS.
#
# The base image already provides the generic development environment, so this
# playbook is only the ROS delta -- see docker/ros/ansible/playbooks/20_ros_setup.yml.
# The four install_* extra-vars the previous base needed are gone with it.
#
# `cd` (not WORKDIR) into /ros-scripts/ansible: that path only exists for the
# duration of this RUN's bind mount, so WORKDIR would break later build steps.
# hadolint ignore=DL3003
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,readonly,source=..,target=/ros-scripts \
    apt-get update \
    && cd /ros-scripts/ansible \
    && ansible-playbook playbooks/20_ros_setup.yml \
      -i inventories/localhost.yml \
      -vvv \
      -e "clang_version=$CLANG_VERSION \
           ros_distro=$ROS_DISTRO \
         "

WORKDIR $OVERLAY_WS

# Expose Xpra port. /start-xpra.sh comes from the base image; this workspace no
# longer ships its own copy.
EXPOSE 14500

# Force clang installed by llvm.sh in /usr/lib/llvm-${CLANG_VERSION}/bin to be the default in docker
ENV PATH="/usr/lib/llvm-${CLANG_VERSION}/bin:/root/.local/bin:$PATH"
ENV CC=clang
ENV CXX=clang++

# Setup entrypoint. The base image's /entrypoint.sh is a bare `exec "$@"` and does
# not source ROS, so this workspace keeps its own.
COPY --chmod=755 ../deploy/ros_entrypoint.sh /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
