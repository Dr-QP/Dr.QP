export script_dir=$(dirname $0)
export root_dir=$(dirname $(dirname $script_dir))
export sources_dir="$root_dir/packages"

isCI()
{
  test ! -z "$CI"
}

export ROS_DISTRO=humble