# ROS Install Prebuilt Role

This Ansible role installs ROS 2 from prebuilt packages, upgrading existing packages first.

## Example Usage

```yaml
- name: Install ROS 2 from packages
  hosts: all
  become: true
  roles:
    - { role: ros_install_prebuilt, tags: ["ros_install", "ros_install_prebuilt"] }
```

## Notes

This role is used when `source_install` is set to `false` (the default). It installs the ROS 2 desktop metapackage, which includes the core ROS 2 packages, rqt, rviz, and various demos.

The ROS 2 distribution to install is determined by the `ros_distro` variable, which defaults to `jazzy` (ROS 2 Jazzy Jalisco).
