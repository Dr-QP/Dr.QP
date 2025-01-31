FROM ghcr.io/dr-qp/ros-base:latest

COPY --chown=$MAMBA_USER:$MAMBA_USER ./ros-desktop.yml /tmp/env.yaml
RUN --mount=type=cache,target=/opt/conda/pkgs micromamba install -y -n base -f /tmp/env.yaml

ARG MAMBA_DOCKERFILE_ACTIVATE=1
