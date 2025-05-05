script_dir=$(dirname "${BASH_SOURCE[0]}")
export root_dir=$(dirname $script_dir)
export sources_dir="$root_dir/packages"

isCI()
{
  test ! -z "$CI"
}

export ROS_DISTRO=jazzy
