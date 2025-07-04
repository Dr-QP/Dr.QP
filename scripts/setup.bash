
script_dir=$(dirname "${BASH_SOURCE[0]}")
source "$script_dir/__utils.sh"

source "/opt/ros/$ROS_DISTRO/setup.bash"

if [[ -f "$root_dir/install/local_setup.bash" ]]; then
  source "$root_dir/install/local_setup.bash"
fi

if [[ -f "$root_dir/.venv-prod/bin/activate" ]]; then
  source "$root_dir/.venv-prod/bin/activate"
fi
