#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

if [[ -z $ROS_DISTRO ]]; then
  echo set ROS_DISTRO to required distribution or source setup file
  exit 1
fi

source /opt/ros/"$ROS_DISTRO"/setup.bash

keys=$(rosdep keys --from-paths "$sources_dir")
resolved_list=$(rosdep resolve $keys 2>/dev/null)

# Array to store package names
packages=()

# Convert the variable into a line-by-line stream
found_apt=false
while IFS= read -r line; do
    # If the line is "#apt", mark that the next line is a package name
    if [[ "$line" == "#apt" ]]; then
        found_apt=true
        continue
    fi

    # If we previously saw "#apt", this line is the package name
    if $found_apt; then
        packages+=("$line")
        found_apt=false  # Reset flag
    fi
done <<< "$resolved_list"

# Check if there are packages to install
if [[ ${#packages[@]} -eq 0 ]]; then
    echo "No packages found for installation."
    exit 1
fi

sorted_packages=($(for element in "${packages[@]}"; do echo "$element"; done | sort))

# Generate the installation script
output_script="$script_dir/install_dependencies.sh"
cat <<EOF > "$output_script"
#!/usr/bin/env bash

echo "Installing dependencies: ${sorted_packages[*]}"
sudo apt-get install -y ${sorted_packages[@]}

EOF

# Make the new script executable
chmod +x "$output_script"
