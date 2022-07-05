FROM dr-qp/ros-base:latest

COPY --chown=$MAMBA_USER:$MAMBA_USER scripts/ros/ros-desktop.yml /tmp/env.yaml
RUN micromamba env update -y -n base -f /tmp/env.yaml && \
    micromamba clean --all --yes

ARG MAMBA_DOCKERFILE_ACTIVATE=1
