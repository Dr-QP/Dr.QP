#!/usr/bin/env fish

# Install fisher (package manager for fish)
curl -sL https://git.io/fisher | source && fisher install jorgebucaran/fisher

# Install bass
fisher install edc/bass

set script_dir (status dirname)
echo Installing \"$script_dir/ros.fish\" to \"$HOME/.config/fish/conf.d/ros.fish\"
cp -f "$script_dir/ros.fish" "$HOME/.config/fish/conf.d/ros.fish"
