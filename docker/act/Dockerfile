ARG DISTRO_VERSION=24.04

FROM ghcr.io/catthehacker/ubuntu:act-${DISTRO_VERSION}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update \
    && sudo apt-get -yq install --no-install-recommends --no-install-suggests moby-engine iptables
