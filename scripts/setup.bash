script_file="${BASH_SOURCE[0]}"
script_dir=$(dirname "$script_file")
source "$script_dir/__utils.sh"

# ROS2 setup scripts may unset variables, so disable 'set -u' temporarily to avoid errors.
nounset_was_set=0
if [[ -o nounset ]]; then
  nounset_was_set=1
  set +u
fi

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [[ -f "$root_dir/install/local_setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$root_dir/install/local_setup.bash"
fi

# Restore 'set -u' if it was previously enabled.
if [[ $nounset_was_set -eq 1 ]]; then
  set -u
fi
