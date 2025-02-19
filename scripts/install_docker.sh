#!/usr/bin/env bash

sudo apt update

# Uninstall old versions if any
set +e
sudo apt-get purge --auto-remove docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc
set -e

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl -y
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y

sudo mkdir -p /etc/docker/
echo '{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  }
}' | sudo tee /etc/docker/daemon.json > /dev/null

echo "Enabling docker service"
sudo systemctl enable docker.service
sudo systemctl enable containerd.service

# Restart to pick up the config changes if service was already running
sudo systemctl restart docker

echo To stop this behavior, use disable instead.
echo sudo systemctl disable docker.service
echo sudo systemctl disable containerd.service

# Allow docker without root
sudo groupadd docker || true # Ignore if group already exists
sudo usermod -aG docker $USER


echo Run 'newgrp docker' to apply the group changes to the current shell
echo
echo Run 'docker run hello-world' to test the installation
echo Run 'docker-compose --version' to test the installation
echo Run 'docker buildx --help' to test the installation

