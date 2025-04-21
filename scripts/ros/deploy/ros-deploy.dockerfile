ARG ROS_DISTRO=jazzy

ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG GIT_SHA=main
ARG GIT_REPO=https://github.com/Dr-QP/Dr.QP.git
ARG BUILD_IMAGE=ghcr.io/dr-qp/$ROS_DISTRO-ros-desktop:main

# multi-stage for caching
FROM ros:$ROS_DISTRO-ros-base AS cacher

# clone overlay source
ARG OVERLAY_WS
ARG GIT_SHA
ARG GIT_REPO

WORKDIR $OVERLAY_WS/src
RUN echo "\
repositories: \n\
  drqp: \n\
    type: git \n\
    url: $GIT_REPO \n\
    version: $GIT_SHA \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos

FROM $BUILD_IMAGE AS builder

ARG OVERLAY_WS

WORKDIR $OVERLAY_WS
COPY --from=cacher $OVERLAY_WS/src/ $OVERLAY_WS

ARG DEPLOY_PACKAGE="drqp_control"
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update \
    && rosdep update \
    && rosdep install --ignore-src -y \
      --from-paths "$OVERLAY_WS/drqp/packages/runtime"

ARG OVERLAY_MIXINS="ninja rel-with-deb-info"
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build \
      --packages-up-to \
        $DEPLOY_PACKAGE \
      --mixin $OVERLAY_MIXINS

# deployment
FROM ros:$ROS_DISTRO-ros-base AS deploy

ARG OVERLAY_WS

WORKDIR $OVERLAY_WS
COPY --from=builder $OVERLAY_WS/install $OVERLAY_WS/install

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update \
    && rosdep update \
    && rosdep install --ignore-src -y \
      --from-paths "$OVERLAY_WS/install" \
      -t exec

ENV OVERLAY_WS=$OVERLAY_WS
ENV ROS_DISTRO=$ROS_DISTRO

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "drqp_control", "bringup.launch.py", "show_rviz:=false"]
