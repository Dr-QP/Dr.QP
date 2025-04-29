# UTC Timezone Role

This Ansible role configures the system timezone to UTC.

## Role Tasks

This role performs the following tasks:

1. Checks the current timezone
2. Sets the timezone to UTC by updating `/etc/timezone`
3. Updates the `/etc/localtime` symlink to point to the UTC timezone
4. Installs the `tzdata` package

## Example Usage

```yaml
- name: Configure UTC timezone
  hosts: all
  become: true
  roles:
    - { role: utc_timezone, tags: ["utc_timezone"] }
```
