# ROS 2 Setup Ansible Playbooks

This directory contains Ansible playbooks to set up a ROS 2 development environment.

## Structure

- `ros_setup.yml`: Main playbook that orchestrates all roles
- `group_vars/all.yml`: Configuration variables
- `inventory.ini`: Ansible inventory file
- `roles/`: Directory containing all the individual roles
  - `basic_prereqs`: Installs basic prerequisites
  - `locale_setup`: Sets up UTF-8 locale
  - `ros_repo`: Configures ROS 2 repository
  - `cmake`: Installs CMake
  - `dev_tools`: Installs development tools
  - `colcon_setup`: Sets up colcon mixin
  - `ros_install_prebuilt`: Installs ROS 2 from prebuilt packages (used when `source_install=false`)
  - `ros_install_source`: Installs ROS 2 from source (used when `source_install=true`)
  - `ros_dependencies`: Installs ROS dependencies
  - `nodejs`: Installs Node.js and NPM
  - `clang`: Installs Clang

## Configuration

Edit `group_vars/all.yml` to configure:

- `ros_distro`: ROS 2 distribution (default: jazzy)
- `source_install`: Whether to install ROS 2 from source (default: false)
- `clang_version`: Clang version to install (default: 20)
- `ci_mode`: Whether running in CI environment (default: false)

## Usage

### Install ROS 2 from packages (default)

```bash
ansible-playbook ros_setup.yml
```

### Install ROS 2 from source

```bash
ansible-playbook ros_setup.yml -e "source_install=true"
```

### Run specific roles only

```bash
ansible-playbook ros_setup.yml --tags "basic_prereqs,ros_repo"
```

### Run only the ROS installation part

For prebuilt packages:

```bash
ansible-playbook ros_setup.yml --tags "ros_install_prebuilt"
```

For source installation:

```bash
ansible-playbook ros_setup.yml --tags "ros_install_source" -e "source_install=true"
```

## Requirements

- Ubuntu 22.04 or newer
- Ansible 2.9 or newer
- Required Ansible collections:

  ```bash
  ansible-galaxy collection install community.general
  ```

## Notes

After installation, you may need to add Clang to your PATH:

For fish (in ~/.config/fish/config.fish):

```fish
set -gx PATH /usr/lib/llvm-20/bin $PATH
```

For bash (in ~/.bashrc):

```bash
export PATH="/usr/lib/llvm-20/bin:$PATH"
```

For dockerfile:

```dockerfile
ENV PATH="/usr/lib/llvm-20/bin:$PATH"
```
