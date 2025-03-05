
function register_argcomplete
  #Produced by: register-python-argcomplete --shell fish ros2
  function __fish_complete
      set -l cmd $argv[1]
      set -x _ARGCOMPLETE 1
      set -x _ARGCOMPLETE_DFS \t
      set -x _ARGCOMPLETE_IFS \n
      set -x _ARGCOMPLETE_SUPPRESS_SPACE 1
      set -x _ARGCOMPLETE_SHELL fish
      set -x COMP_LINE (commandline -p)
      set -x COMP_POINT (string length (commandline -cp))
      set -x COMP_TYPE
      if set -q _ARC_DEBUG
          $cmd 8>&1 9>&2 1>&9 2>&1
      else
          $cmd 8>&1 9>&2 1>/dev/null 2>&1
      end
  end
  complete --command ros2 -f -a '(__fish_complete ros2)'
  complete --command colcon -f -a '(__fish_complete colcon)'
  complete --command rosidl -f -a '(__fish_complete rosidl)'
  complete --command ament_index -f -a '(__fish_complete ament_index)'
end

function ros2_activate
  bass source /opt/ros/jazzy/setup.bash
  register_argcomplete
end

function ros2_ws
  bass source ./install/setup.bash
  register_argcomplete
end
