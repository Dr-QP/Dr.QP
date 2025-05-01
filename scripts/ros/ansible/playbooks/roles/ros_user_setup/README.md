# ROS User Setup Role

This Ansible role creates and configures a ROS user account with the necessary environment setup.

## Role Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `ros_user_setup_username` | Username for the ROS user | `rosdev` |
| `ros_user_setup_uid` | User ID for the ROS user | `1001` |
| `ros_user_setup_gid` | Group ID for the ROS user | `{{ ros_user_setup_uid }}` |
| `ros_user_setup_workspace_dir` | Path to the ROS workspace directory | `{{ user_home }}/ros2_ws` |

## Example Usage

```yaml
- name: Setup ROS user
  hosts: all
  become: true
  roles:
    - { role: ros_user_setup, tags: ["ros_user_setup"] }
```
