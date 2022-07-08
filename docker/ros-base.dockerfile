FROM mambaorg/micromamba:0.24-jammy 
# ubuntu:jammy - 22.04

USER root

RUN cp /usr/local/bin/_activate_current_env.sh /etc/profile.d/100-activate_current_env.sh

USER $MAMBA_USER

COPY --chown=$MAMBA_USER:$MAMBA_USER scripts/ros/ros-base.yml /tmp/env.yaml
RUN --mount=type=cache,target=/opt/conda/pkgs micromamba install -y -n base -f /tmp/env.yaml && \
    micromamba install -y -n base mamba

ARG MAMBA_DOCKERFILE_ACTIVATE=1
