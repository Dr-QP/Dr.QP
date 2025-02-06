#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

# to lower is needed for Boost as it's name for CMake is
keys=$(rosdep keys --from-paths "$sources_dir" | tr '[:upper:]' '[:lower:]')
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

# Generate the installation script
output_script="$script_dir/install_dependencies.sh"
cat <<EOF > "$output_script"
#!/bin/bash

echo "Installing dependencies: ${packages[*]}"
sudo apt-get install -y -q --no-install-recommends "${packages[@]}"
EOF

# Make the new script executable
chmod +x "$output_script"
