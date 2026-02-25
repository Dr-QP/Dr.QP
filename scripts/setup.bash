script_file="${BASH_SOURCE[0]}"
script_dir=$(dirname "$script_file")
source "$script_dir/__utils.sh"

$(cd "$root_dir" && "$root_dir/docker/ros/deploy/prod-venv-create.sh")

while [[ $# -gt 0 ]]; do
  case $1 in
    --update-venv)
      $root_dir/docker/ros/deploy/prod-venv.sh $root_dir/build
      $root_dir/docker/ros/deploy/prod-venv.sh $root_dir/install

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

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-$root_dir/.venv-prod/bin/python3}"
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [[ -f "$root_dir/install/local_setup.bash" ]]; then
  source "$root_dir/install/local_setup.bash"
fi

source $root_dir/docker/ros/deploy/prod-venv.sh
