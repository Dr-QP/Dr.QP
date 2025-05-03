# Ansible Playbooks for ROS 2 Setup

The project has been migrated from shell scripts to Ansible playbooks for all the aspects of robot and dev machine setup.

## Directory Structure

The Ansible playbooks are located in the `ansible` directory:

```bash
ansible/
├── ansible-virtual.cfg          # Configuration for virtual bots
├── ansible.cfg                  # Ansible configuration
├── inventories/                 # Inventory files for different environments
│   ├── localhost.yml            # Local machine inventory
│   ├── real-robots.yml          # Real robot inventory
│   └── virtual-bots.yml         # Docker-based virtual bots inventory
├── playbooks/                   # Playbooks for different tasks
│   ├── 0_start_virtual_bots.yml # Start Docker containers for virtual bots
│   ├── 1_pam_ssh_agent_auth.yml # SSH agent authentication setup
│   ├── 10_install_docker.yml    # Docker installation
│   ├── 20_ros_setup.yml         # Main ROS 2 setup playbook
│   ├── 100_startup_service.yml  # Robot startup service
│   ├── 200_pair_controller.yml  # Controller pairing
│   ├── 9999_stop_virtual_bots.yml # Stop Docker containers for virtual bots
│   ├── group_vars/              # Playbook-specific variables
│   │   └── all.yml              # Configuration variables
│   ├── roles/                   # Individual roles for each component
│   │   ├── basic_prereqs/       # Basic prerequisites
│   │   ├── clang/               # Clang installation
│   │   ├── cmake/               # CMake installation
│   │   ├── colcon_setup/        # Colcon mixin setup
│   │   ├── dev_tools/           # Development tools
│   │   ├── extra_facts/         # System facts gathering
│   │   ├── locale_setup/        # UTF-8 locale setup
│   │   ├── utc_timezone/        # UTC timezone setup
│   │   ├── nodejs/              # Node.js and NPM
│   │   ├── ros_dependencies/    # ROS dependencies (uses known packages)
│   │   ├── ros_install_prebuilt/ # ROS 2 installation from packages
│   │   ├── ros_install_source/  # ROS 2 installation from source
│   │   └── ros_user_setup/      # ROS user creation and setup
│   └── test_ros_dependencies.yml # Test playbook for ROS dependencies
└── README.md                    # Points back to this documentation
```

## Configuration

Edit `ansible/playbooks/group_vars/all.yml` to configure:

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
ansible-playbook playbooks/20_ros_setup.yml --tags "basic_prereqs,cmake"
```

Available role tags:

- `extra_facts`: System facts gathering (always runs)
- `basic_prereqs`: Basic prerequisites
- `locale_setup`: UTF-8 locale setup
- `utc_timezone`: UTC timezone setup
- `cmake_kitware`: CMake installation from Kitware repository
- `cmake_system`: CMake installation from system packages
- `dev_tools`: Development tools
- `colcon_setup`: Colcon mixin setup
- `fish_setup`: Fish shell configuration for ROS 2
- `ros_install_prebuilt`: ROS 2 installation from packages
- `ros_install_source`: ROS 2 installation from source
- `ros_dependencies`: ROS dependencies
- `nodejs`: Node.js and NPM
- `clang`: Clang installation
- `ros_user_setup`: ROS user creation and setup

### Run only the ROS installation part

For prebuilt packages:

```bash
ansible-playbook playbooks/20_ros_setup.yml --tags "ros_install_prebuilt"
```

For source installation:

```bash
ansible-playbook playbooks/20_ros_setup.yml --tags "ros_install_source" -e "source_install=true"
```

You can also use the combined tag:

```bash
ansible-playbook playbooks/20_ros_setup.yml --tags "ros_install"
```

This will run either `ros_install_prebuilt` or `ros_install_source` based on the `source_install` variable.

### Using specific inventory

For real robots:

```bash
ansible-playbook -i inventories/real-robots.yml playbooks/20_ros_setup.yml
```

For local machine:

```bash
ansible-playbook -i inventories/localhost.yml playbooks/20_ros_setup.yml
```

For Docker-based virtual bots:

```bash
# First start the virtual bots
env ANSIBLE_CONFIG=ansible-virtual.cfg ansible-playbook playbooks/0_start_virtual_bots.yml

# Then run the setup playbook
env ANSIBLE_CONFIG=ansible-virtual.cfg ansible-playbook playbooks/20_ros_setup.yml

# When finished, stop the virtual bots
env ANSIBLE_CONFIG=ansible-virtual.cfg ansible-playbook playbooks/9999_stop_virtual_bots.yml
```

For faster iteration, playbooks can be chained:

```bash
env ANSIBLE_CONFIG=ansible-virtual.cfg ansible-playbook \
    playbooks/9999_stop_virtual_bots.yml \
    playbooks/0_start_virtual_bots.yml \
    playbooks/20_ros_setup.yml -vvv
```


## Generating ROS Dependencies

The project includes a script to generate ROS dependencies based on your project:

```bash
./scripts/ros-dep-gen-ansible.sh
```

This script:

1. Analyzes the project's ROS dependencies using `rosdep`
2. Generates a variables file in `ansible/playbooks/roles/ros_dependencies/vars/known_ros_dependencies.yml`
3. The generated variables (`ros_dependencies_known_packages`) are automatically used by the `ros_dependencies` role

To run only the ROS dependencies role:

```bash
ansible-playbook playbooks/20_ros_setup.yml --tags "ros_dependencies"
```

Alternatively, you can use the dedicated test playbook:

```bash
ansible-playbook playbooks/test_ros_dependencies.yml
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

## Docker Integration

The project includes Ansible playbooks for Docker integration:

### Installing Docker

To install Docker on robots:

```bash
ansible-playbook -i inventories/real-robots.yml playbooks/10_install_docker.yml
```

This playbook:

- Installs Docker Engine and related tools
- Configures Docker daemon with proper logging settings
- Adds the user to the Docker group for non-root access

### Robot Startup Service

The project includes a playbook to set up a systemd service that runs ROS nodes in Docker containers:

```bash
ansible-playbook -i inventories/real-robots.yml playbooks/100_startup_service.yml
```

This playbook:

- Pulls the latest Docker image from GitHub Container Registry
- Creates two systemd services:
  - `drqp-control-service`: Runs the main control nodes
  - `drqp-joystick-service`: Runs the joystick node with hot-plugging support

### SSH Agent Authentication

To enable passwordless sudo using SSH agent authentication:

```bash
ansible-playbook -i inventories/real-robots.yml playbooks/1_pam_ssh_agent_auth.yml
```

This playbook:

- Installs `libpam-ssh-agent-auth` package
- Configures PAM to allow sudo authentication via SSH agent
- Adds your SSH public key to the authorized keys file

### Controller Pairing

To pair a DualSense wireless controller with a robot:

```bash
ansible-playbook -i inventories/real-robots.yml playbooks/200_pair_controller.yml -e "controller_name=DualSense"
```

or

```bash
ansible-playbook -i inventories/real-robots.yml playbooks/200_pair_controller.yml -e "controller_mac=XX:XX:XX:XX:XX:XX"
```

## Migration from Shell Scripts

The Ansible playbooks replace the following shell scripts:

- `scripts/ros/ros-2-prep.sh` → `ansible/playbooks/20_ros_setup.yml`
- `scripts/ros/ros-2-src-build.sh` → `ansible/playbooks/roles/ros_install_source`
- `scripts/ros/__gen_install_ros_dependencies.sh` → `ansible/playbooks/roles/ros_dependencies` with known packages
- `scripts/install_docker.sh` → `ansible/playbooks/10_install_docker.yml`
- `scripts/ros/desktop/ros-desktop.dockerfile` (user setup) → `ansible/playbooks/roles/ros_user_setup`
- `scripts/ros/fish/setup.fish` → `ansible/playbooks/roles/fish_setup`
- `scripts/ros/colcon-mixin.sh` → `ansible/playbooks/roles/colcon_setup`
- `scripts/ros/rosdep-update.sh` → `ansible/playbooks/roles/colcon_setup`

```{admonition} Benefits
:class: note

The Ansible playbooks provide several advantages:

- Better organization with roles for each component
- Idempotency (can be run multiple times without issues)
- Selective execution with tags
- Better error handling
- Configurable variables
- Better documentation
- Docker integration for containerized deployment
- Support for virtual bots for testing
- Improved SSH authentication
- Controller pairing automation
```
