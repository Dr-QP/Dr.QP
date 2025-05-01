FROM ubuntu:24.04

# Install Ansible
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,readonly,source=..,target=/scripts \
    apt-get update && apt-get install -y python3 sudo locales \
    && /scripts/setup-ansible.sh

# Set the locale en_US.UTF-8
# Docker runs a non login shell that does not read bashrc and does NOT initialize locale
# from /etc/default/locale requiring to set it manually via ENV instruction
# https://web.archive.org/web/20230323021946/http://jaredmarkell.com/docker-and-locales/
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
