# UTC Timezone Role

This Ansible role configures the system timezone to UTC, ensuring consistent time settings across all systems.

## Example Usage

```yaml
- name: Configure UTC timezone
  hosts: all
  become: true
  roles:
    - { role: utc_timezone, tags: ["utc_timezone"] }
```
