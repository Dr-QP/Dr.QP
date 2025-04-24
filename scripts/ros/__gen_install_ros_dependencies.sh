#!/usr/bin/env bash

packages=(libboost-all-dev
python3-pytest
ros-jazzy-ament-cmake
ros-jazzy-ament-cmake-clang-format
ros-jazzy-ament-cmake-copyright
ros-jazzy-ament-cmake-core
ros-jazzy-ament-cmake-cppcheck
ros-jazzy-ament-cmake-cpplint
ros-jazzy-ament-cmake-export-dependencies
ros-jazzy-ament-cmake-flake8
ros-jazzy-ament-cmake-lint-cmake
ros-jazzy-ament-cmake-pep257
ros-jazzy-ament-cmake-xmllint
ros-jazzy-ament-copyright
ros-jazzy-ament-flake8
ros-jazzy-ament-lint-auto
ros-jazzy-ament-pep257
ros-jazzy-catch-ros2
ros-jazzy-gz-ros2-control
ros-jazzy-joint-state-publisher
ros-jazzy-joint-state-publisher-gui
ros-jazzy-joy
ros-jazzy-launch
ros-jazzy-launch-ros
ros-jazzy-rclcpp
ros-jazzy-rclpy
ros-jazzy-robot-state-publisher
ros-jazzy-ros-gz-sim
ros-jazzy-ros2-control
ros-jazzy-ros2-controllers
ros-jazzy-ros2launch
ros-jazzy-rosidl-default-generators
ros-jazzy-rosidl-default-runtime
ros-jazzy-rviz2
ros-jazzy-sensor-msgs
ros-jazzy-std-msgs
ros-jazzy-xacro)

available_packages=()

# Check which packages are available
for pkg in "${packages[@]}"; do
    # if apt-cache show "$pkg" >/dev/null 2>&1; then
    if apt-cache policy "$pkg" | grep -q "Candidate:"; then
        available_packages+=("$pkg")
    else
        echo "Skipping $pkg (not found in repo)"
    fi
done

# Install available packages in a single command
if [ ${#available_packages[@]} -gt 0 ]; then
    echo "Installing: ${available_packages[*]}"
    sudo apt-get install -y "${available_packages[@]}"
else
    echo "No packages available for installation."
fi

