# Clang Role

This Ansible role installs and configures Clang/LLVM for C++ development.

## Role Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `clang_version` | Clang/LLVM version to install | `20` |

## Example Usage

```yaml
- name: Install Clang
  hosts: all
  become: true
  roles:
    - { role: clang, tags: ["clang"] }
```

## Notes

The role sets up Clang to be available in the PATH for both fish and bash shells. When using this role with Docker, you may need to add the following line to your Dockerfile:

```dockerfile
ENV PATH=/usr/lib/llvm-{{ clang_version }}/bin:$PATH
```
