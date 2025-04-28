# ROS User Setup Role

This Ansible role creates and configures a ROS user account with the necessary environment setup.

## Role Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `ros_username` | Username for the ROS user | `rosdev` |
| `ros_uid` | User ID for the ROS user | `1001` |
| `ros_gid` | Group ID for the ROS user | `{{ ros_uid }}` |
| `ros_workspace_dir` | Path to the ROS workspace directory | `{{ user_home }}/ros2_ws` |
| `ros_user_setup_clang_version` | Clang version to use | `{{ clang_version \| default(20) }}` |

## Tasks

This role performs the following tasks:

1. Creates a user group for the ROS user
2. Creates the ROS user with the specified UID and GID
3. Adds the ROS user to sudoers with NOPASSWD privileges
4. Creates the ROS workspace directory
5. Configures the user's `.bashrc` with:
   - ROS setup source commands
   - Workspace setup source commands
   - Clang environment variables

## Example Usage

```yaml
- name: Setup ROS user
  hosts: all
  become: true
  roles:
    - { role: ros_user_setup, tags: ["ros_user_setup"] }
```

## Notes

This role was created to replace the user setup portion of the `scripts/ros/desktop/ros-desktop.dockerfile` file, providing the same functionality in an Ansible-managed way.
