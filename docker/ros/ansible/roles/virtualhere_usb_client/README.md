# VirtualHere USB Client

Installs the VirtualHere USB client binary and runs it as a systemd service.

The role follows VirtualHere's Linux service guidance: the console client runs
with `-n` to start in daemon mode.

## Variables

| Variable                                | Default                                       | Description                                               |
| --------------------------------------- | --------------------------------------------- | --------------------------------------------------------- |
| `virtualhere_usb_client_service_name`   | `virtualhereclient`                           | Systemd service name.                                     |
| `virtualhere_usb_client_install_path`   | `/usr/sbin/vhclient`                          | Destination for the installed client binary.              |
| `virtualhere_usb_client_download_url`   | Selected from `ansible_facts['architecture']` | Override URL for custom architectures or pinned binaries. |
| `virtualhere_usb_client_extra_args`     | `""`                                          | Additional arguments passed after `-n`.                   |
| `virtualhere_usb_client_manage_service` | `true` when systemd is available              | Start, enable, and restart the systemd service.           |

Supported architecture defaults:

- `aarch64` / `arm64`: `https://www.virtualhere.com/sites/default/files/usbclient/vhclientarm64`
- `x86_64` / `amd64`: `https://www.virtualhere.com/sites/default/files/usbclient/vhclientx86_64`

## Example

```yaml
- hosts: robots
  become: true
  roles:
    - virtualhere_usb_client
```

Control the daemon with the installed binary:

```bash
/usr/sbin/vhclient -t "LIST"
```
