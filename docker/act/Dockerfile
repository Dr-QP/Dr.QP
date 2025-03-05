ARG VERSION=24.04

FROM ghcr.io/catthehacker/ubuntu:act-${VERSION}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update \
    sudo apt-install moby-engine iptables
