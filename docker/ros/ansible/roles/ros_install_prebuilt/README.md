# ROS Install Prebuilt Role

This Ansible role installs ROS 2 from prebuilt packages.

## Example Usage

```yaml
- name: Install ROS 2 from packages
  hosts: all
  become: true
  roles:
    - {
        role: ros_install_prebuilt,
        tags: ['ros_install', 'ros_install_prebuilt'],
      }
```

## Notes

This role is used when `source_install` is set to `false` (the default). It installs the ROS 2 desktop metapackage, which includes the core ROS 2 packages, rqt, rviz, and various demos.

The ROS 2 distribution to install is determined by the `ros_distro` variable, which defaults to `jazzy` (ROS 2 Jazzy Jalisco). The exact version of `ros-<distro>-desktop` comes from `vars/apt_pins_<suite>_<arch>.yml`; see [Pinned apt versions](../../../../../docs/source/GettingStarted/ansible.md#pinned-apt-versions).

This role used to run a blanket `apt upgrade` before installing. It has been removed, and no role replaces it: every package `playbooks/20_ros_setup.yml` installs is pinned, so an upgrade could only move something off a version this repository records. Upgrading inherited packages belongs to the base image prep phase, which lives in agentdev; this repository consumes the result through the agent-desktop digest pinned in `docker/ros/desktop/ros-desktop.Dockerfile`.
