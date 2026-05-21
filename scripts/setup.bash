script_file="${BASH_SOURCE[0]}"
script_dir=$(dirname "$script_file")
source "$script_dir/__utils.sh"

# ROS2 setup scripts may unset variables, so disable 'set -u' temporarily to avoid errors.
restore_nounset=0
if [[ -o nounset ]]; then
  restore_nounset=1
  set +u
fi

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [[ -f "$root_dir/install/local_setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$root_dir/install/local_setup.bash"
fi

# Restore 'set -u' if it was previously enabled.
if [[ $restore_nounset -eq 1 ]]; then
  set -u
fi
