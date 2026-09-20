# Clang Role

This Ansible role installs and configures Clang/LLVM for C++ development.

## Role Variables

| Variable        | Description                   | Default |
| --------------- | ----------------------------- | ------- |
| `clang_version` | Clang/LLVM version to install | `20`    |

## Example Usage

```yaml
- name: Install Clang
  hosts: all
  become: true
  roles:
    - { role: clang, tags: ['clang'] }
```

## Notes

Clang itself comes from `apt.llvm.org` through `llvm.sh`, which installs whatever the LLVM project currently publishes for `clang_version`; that stream is not version-addressable, so the major version above is the only pin available for it. The packages the role installs through apt are pinned in `vars/apt_pins_<suite>_<arch>.yml` — see [Pinned apt versions](../../../../../docs/source/GettingStarted/ansible.md#pinned-apt-versions).

The role sets up Clang to be available in the PATH for both fish and bash shells. When using this role with Docker, you may need to add the following line to your Dockerfile:

```dockerfile
ENV PATH=/usr/lib/llvm-{{ clang_version }}/bin:$PATH
```
