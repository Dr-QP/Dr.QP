# Ansible Playbooks for ROS 2 Setup

```{admonition} Summary
The project includes Ansible playbooks to set up a ROS 2 development environment. These playbooks replace the shell scripts previously used for installation.
```

## Directory Structure

The Ansible playbooks are located in the `ansible` directory:

```text
ansible/
├── group_vars/
│   └── all.yml                  # Configuration variables
├── inventories/                 # Inventory files for different environments
│   ├── docker-poc.yml           # Docker-based inventory for testing
│   └── real-robots.yml          # Real robot inventory
├── playbooks/                   # Playbooks for different tasks
│   ├── 1_pam_ssh_agent_auth.yml # SSH agent authentication setup
│   ├── 10_install_docker.yaml   # Docker installation
│   ├── 20_ros_setup.yml         # Main ROS 2 setup playbook
│   ├── 100_startup_service.yaml # Robot startup service
│   └── 200_pair_controller.yml  # Controller pairing
├── roles/                       # Individual roles for each component
│   ├── basic_prereqs/           # Basic prerequisites
│   ├── locale_setup/            # UTF-8 locale setup
│   ├── ros_repo/                # ROS 2 repository setup
│   ├── cmake/                   # CMake installation
│   ├── dev_tools/               # Development tools
│   ├── colcon_setup/            # Colcon mixin setup
│   ├── ros_install_prebuilt/    # ROS 2 installation from packages
│   ├── ros_install_source/      # ROS 2 installation from source
│   ├── ros_dependencies/        # ROS dependencies (uses known packages)
│   ├── nodejs/                  # Node.js and NPM
│   └── clang/                   # Clang installation
├── ansible.cfg                  # Ansible configuration
├── inventory.ini                # Default inventory file
├── README.md                    # Documentation
└── setup-ros.sh                 # Wrapper script
```

## Configuration

Edit `ansible/group_vars/all.yml` to configure:

- `ros_distro`: ROS 2 distribution (default: jazzy)
- `source_install`: Whether to install ROS 2 from source (default: false)
- `clang_version`: Clang version to install (default: 20)
- `ci_mode`: Whether running in CI environment (default: false)

## Usage

### Quick Start

To set up a ROS 2 development environment:

```bash
cd ansible
./setup-ros.sh
```

### Install ROS 2 from packages (default)

```bash
ansible-playbook playbooks/20_ros_setup.yml
```

### Install ROS 2 from source

```bash
ansible-playbook playbooks/20_ros_setup.yml -e "source_install=true"
```

### Run specific roles only

```bash
ansible-playbook playbooks/20_ros_setup.yml --tags "basic_prereqs,ros_repo"
```

### Run only the ROS installation part

For prebuilt packages:

```bash
ansible-playbook playbooks/20_ros_setup.yml --tags "ros_install_prebuilt"
```

For source installation:

```bash
ansible-playbook playbooks/20_ros_setup.yml --tags "ros_install_source" -e "source_install=true"
```

### Using specific inventory

```bash
ansible-playbook -i inventories/real-robots.yml playbooks/20_ros_setup.yml
```

For Docker-based testing:

```bash
ansible-playbook -i inventories/docker-poc.yml playbooks/20_ros_setup.yml
```

## Generating ROS Dependencies

The project includes a script to generate ROS dependencies based on your project:

```bash
./scripts/ros-dep-gen-ansible.sh
```

This script:

1. Analyzes the project's ROS dependencies using `rosdep`
2. Generates a variables file in `ansible/roles/ros_dependencies/vars/known_ros_dependencies.yml`
3. The generated variables are automatically used by the `ros_dependencies` role

To run only the ROS dependencies role:

```bash
ansible-playbook playbooks/20_ros_setup.yml --tags "ros_dependencies"
```

## Requirements

- Ubuntu 22.04 or newer
- Ansible 2.9 or newer
- Required Ansible collections:

  ```bash
  ansible-galaxy collection install community.general
  ```

## Post-Installation Notes

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

## Migration from Shell Scripts

The Ansible playbooks replace the following shell scripts:

- `scripts/ros/ros-2-prep.sh` → `ansible/playbooks/20_ros_setup.yml`
- `scripts/ros/ros-2-src-build.sh` → `ansible/roles/ros_install_source`
- `scripts/ros/__gen_install_ros_dependencies.sh` → `ansible/roles/ros_dependencies` with known packages

```{admonition} Benefits
:class: note

The Ansible playbooks provide several advantages:

- Better organization with roles for each component
- Idempotency (can be run multiple times without issues)
- Selective execution with tags
- Better error handling
- Configurable variables
- Better documentation
```
