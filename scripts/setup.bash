
script_dir=$(dirname "${BASH_SOURCE[0]}")
source "$script_dir/__utils.sh"

source "/opt/ros/$ROS_DISTRO/setup.bash"

if [[ -f "$root_dir/install/local_setup.bash" ]]; then
  source "$root_dir/install/local_setup.bash"
fi

if [[ ! -f "$root_dir/.venv-prod/bin/activate" ]]; then
  $root_dir/docker/ros/deploy/prod-venv.sh "$root_dir/install"
  $root_dir/docker/ros/deploy/prod-venv.sh "$root_dir/build"
fi

source "$root_dir/.venv-prod/bin/activate"
