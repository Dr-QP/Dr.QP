script_file="${BASH_SOURCE[0]}"
script_dir=$(dirname "$script_file")
source "$script_dir/__utils.sh"

while [[ $# -gt 0 ]]; do
  case $1 in
    --help)
      echo "Usage: source $script_file"
      return
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Usage: source $script_file" >&2
      return 1
      ;;
  esac
done

# ROS2 setup scripts may unset variables, so disable 'set -u' temporarily to avoid errors.
no_unset=1
[[ -o nounset ]] && no_unset=0
[[ $no_unset -eq 0 ]] && set +u

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [[ -f "$root_dir/install/local_setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$root_dir/install/local_setup.bash"
fi

# Restore 'set -u' if it was previously enabled.
[[ $no_unset -eq 0 ]] && set -u

true
