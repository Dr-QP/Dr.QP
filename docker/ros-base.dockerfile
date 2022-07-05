FROM mambaorg/micromamba:0.24-jammy 
# ubuntu:jammy - 22.04

COPY --chown=$MAMBA_USER:$MAMBA_USER scripts/ros/ros-base.yml /tmp/env.yaml
RUN micromamba install -y -n base -f /tmp/env.yaml && \
    micromamba install -y -n base mamba && \
    micromamba clean --all --yes

ARG MAMBA_DOCKERFILE_ACTIVATE=1
