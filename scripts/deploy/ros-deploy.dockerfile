ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG GIT_SHA=main
ARG GIT_REPO=https://github.com/Dr-QP/Dr.QP.git

# multi-stage for caching
FROM ros:humble-ros-base AS cacher

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

FROM ghcr.io/dr-qp/ros-desktop:main AS builder

ARG OVERLAY_WS

WORKDIR $OVERLAY_WS
COPY --from=cacher $OVERLAY_WS/src/drqp $OVERLAY_WS

ARG DEPLOY_PACKAGE="drqp_control"
RUN sudo apt-get update \
    && sudo apt install tree -y \
    && tree $OVERLAY_WS \
    && rosdep update \
    && rosdep install --from-paths "$OVERLAY_WS/packages/" --ignore-src -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        $DEPLOY_PACKAGE \
      --mixin $OVERLAY_MIXINS
