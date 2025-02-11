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
COPY --from=cacher $OVERLAY_WS/src/ $OVERLAY_WS

ARG DEPLOY_PACKAGE="drqp_control"
RUN sudo apt-get update \
    && sudo apt install tree -y \
    && tree $OVERLAY_WS \
    && rosdep update \
    && rosdep install --ignore-src -y \
      --from-paths "$OVERLAY_WS/drqp/packages/drqp_control" \
      --from-paths "$OVERLAY_WS/drqp/packages/drqp_a1_16_driver" \
      --from-paths "$OVERLAY_WS/drqp/packages/drqp_interfaces" \
      --from-paths "$OVERLAY_WS/drqp/packages/drqp_lint_common" \
      --from-paths "$OVERLAY_WS/drqp/packages/drqp_rapidjson" \
      --from-paths "$OVERLAY_WS/drqp/packages/drqp_serial" \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ARG OVERLAY_MIXINS="rel-with-deb-info"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/b8436aa16c0bdbc01081b12caa253cbf16e0fb82/index.yaml && \
    colcon mixin update default && \
    colcon build \
      --packages-up-to \
        $DEPLOY_PACKAGE \
      --mixin $OVERLAY_MIXINS

# deployment
FROM ros:humble-ros-base AS deploy

ARG OVERLAY_WS

WORKDIR $OVERLAY_WS
COPY --from=builder $OVERLAY_WS/install $OVERLAY_WS/install
RUN sudo apt-get update \
    && sudo apt install tree -y \
    && tree $OVERLAY_WS \
    && rosdep update \
    && rosdep install --ignore-src -y \
      --from-paths "$OVERLAY_WS/install"\
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

COPY ./ros_entrypoint.sh /

# run walk node
CMD ["ros2", "run", "drqp_control", "control", "walk", "192.168.0.181"]

# run launch file
# CMD ["ros2", "launch", "drqp_control", "drqp_control.py"]
