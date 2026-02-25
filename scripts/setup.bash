script_file="${BASH_SOURCE[0]}"
script_dir=$(dirname "$script_file")
source "$script_dir/__utils.sh"

(cd "$root_dir" && "$root_dir/docker/ros/deploy/prod-venv-create.sh")

while [[ $# -gt 0 ]]; do
  case $1 in
    --update-venv)
      "$root_dir/docker/ros/deploy/prod-venv.sh" "$root_dir/build"
      "$root_dir/docker/ros/deploy/prod-venv.sh" "$root_dir/install"

      shift
      ;;
    --help)
      echo "Usage: source $script_file [--update-venv]"
      echo
      echo "  --update-venv: Update the virtual environment"
      return
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

source "$root_dir/docker/ros/deploy/prod-venv.sh"
