# Extra Facts Role

This Ansible role gathers additional system facts needed by other roles, including architecture normalization and user variables.

## Example Usage

```yaml
- name: Gather extra facts
  hosts: all
  roles:
    - { role: extra_facts, tags: ["extra_facts"] }
```

## Notes

This role is typically included with the `always` tag to ensure that the necessary facts are available to all other roles. It's a prerequisite for most other roles in the playbook.
