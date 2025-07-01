ARG ROS_DISTRO=jazzy

ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG GIT_SHA=main
ARG GIT_REPO=https://github.com/Dr-QP/Dr.QP.git
ARG BUILD_IMAGE=ghcr.io/dr-qp/$ROS_DISTRO-ros-desktop:edge

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

ARG DEPLOY_PACKAGE="drqp_brain"
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
ARG DEPLOY_USER=rosdeploy
ARG DEPLOY_UID=1001
ARG DEPLOY_GID=1001

WORKDIR $OVERLAY_WS
COPY --from=builder $OVERLAY_WS/install $OVERLAY_WS/install

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update \
    && rosdep update \
    && rm -f $OVERLAY_WS/install/COLCON_IGNORE \
    && rosdep install --ignore-src -y \
      --from-paths "$OVERLAY_WS/install" \
      -t exec

ENV OVERLAY_WS=$OVERLAY_WS
ENV ROS_DISTRO=$ROS_DISTRO

RUN groupadd -g $DEPLOY_GID $DEPLOY_USER \
    && useradd -m -u $DEPLOY_UID -g $DEPLOY_GID $DEPLOY_USER \
    && sudo chown -R $DEPLOY_USER:$DEPLOY_USER $OVERLAY_WS
USER $DEPLOY_USER

RUN --mount=type=bind,readonly,source=.,target=/deploy-scripts \
    /deploy-scripts/prod-venv.sh "$OVERLAY_WS/install"

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "drqp_brain", "bringup.launch.py"]
