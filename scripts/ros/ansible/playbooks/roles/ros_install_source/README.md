# ROS Install Source Role

This Ansible role installs ROS 2 from source by setting up a workspace, importing repositories, and building with colcon.

## Example Usage

```yaml
- name: Install ROS 2 from source
  hosts: all
  become: true
  roles:
    - { role: ros_install_source, tags: ["ros_install", "ros_install_source"] }
```

## Notes

This role is used when `source_install` is set to `true`. It builds ROS 2 from source in the user's home directory at `~/ros2_<ros_distro>`.

Building from source provides the latest features and bug fixes, but takes longer to install and may be less stable than the prebuilt packages. It's recommended for development and testing of new ROS 2 features.
