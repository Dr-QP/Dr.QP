FROM ubuntu:24.04

# Install Ansible
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,readonly,source=..,target=/scripts \
    apt-get update && apt-get install -y python3 sudo \
    && /scripts/setup-ansible.sh
