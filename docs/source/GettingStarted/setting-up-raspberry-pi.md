# Setting up Raspberry Pi for the Robot

## Install Ubuntu 24.04

Ubuntu 24.04 is the only LTS currently supported by Raspberry Pi 5

## Initial setup

Setup ssh agent based authentication to avoid typing password every time

As well as configure Raspberry Pi. See playbooks for details.

```bash
cd docker/ros/ansible
ansible-playbook playbooks/1_pam_ssh_agent_auth.yml --ask-become-pass
ansible-playbook playbooks/5_raspberry_pi_setup.yml
```

## Install ROS and docker

```bash
cd docker/ros/ansible
ansible-playbook playbooks/10_install_docker.yml
ansible-playbook playbooks/20_ros_setup.yml
```

## Installing deployment service

Production docker container is deployed using Ansible.
Run the following command on your dev host to install docker and setup autorun service

```bash
cd docker/ros/ansible
ansible-playbook playbooks/100_startup_service.yml
```
