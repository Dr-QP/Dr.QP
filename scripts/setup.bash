script_file="${BASH_SOURCE[0]}"
script_dir=$(dirname "$script_file")
source "$script_dir/__utils.sh"

# ROS2 setup scripts may unset variables, so disable 'set -u' temporarily to avoid errors.
nounset_was_set=0
if [[ -o nounset ]]; then
  nounset_was_set=1
  set +u
fi

if [[ -z "${ROS_DISTRO:-}" || ! -f "/opt/ros/${ROS_DISTRO:-}/setup.bash" ]]; then
  {
    echo "ERROR: ROS 2 is not installed on this host (ROS_DISTRO='${ROS_DISTRO:-}', /opt/ros/${ROS_DISTRO:-<unset>}/setup.bash missing)."
    echo "Do NOT give up and do NOT retry this command locally — it cannot succeed here."
    echo "Escalate to a containerized ROS 2 environment instead:"
    echo "  - Docker daemon available: use the 'microvm-sandbox' skill (devcontainer exec)."
    echo "  - No Docker daemon (e.g. Codex Tasks): use the 'remote-codespace-session' skill (GitHub Codespace over SSH)."
  } >&2
  # shellcheck disable=SC2317  # exit is reachable when the script is run directly, not sourced
  return 1 2>/dev/null || exit 1
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
