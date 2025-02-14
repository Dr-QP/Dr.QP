#!/usr/bin/env bash

packages=(libboost-all-dev ros-humble-ament-cmake ros-humble-ament-cmake-clang-format ros-humble-ament-cmake-copyright ros-humble-ament-cmake-core ros-humble-ament-cmake-cppcheck ros-humble-ament-cmake-cpplint ros-humble-ament-cmake-export-dependencies ros-humble-ament-cmake-flake8 ros-humble-ament-cmake-lint-cmake ros-humble-ament-cmake-pep257 ros-humble-ament-cmake-xmllint ros-humble-ament-lint-auto ros-humble-catch-ros2 ros-humble-gazebo-ros ros-humble-gazebo-ros2-control ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-rclcpp ros-humble-robot-state-publisher ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ros2launch ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime ros-humble-rviz2 ros-humble-std-msgs ros-humble-xacro)

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

