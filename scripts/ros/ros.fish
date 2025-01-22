
function register_argcomplete
    #Produced by: register-python-argcomplete --shell fish ros2
    function __fish_ros2_complete
        set -x _ARGCOMPLETE 1
        set -x _ARGCOMPLETE_DFS \t
        set -x _ARGCOMPLETE_IFS \n
        set -x _ARGCOMPLETE_SUPPRESS_SPACE 1
        set -x _ARGCOMPLETE_SHELL fish
        set -x COMP_LINE (commandline -p)
        set -x COMP_POINT (string length (commandline -cp))
        set -x COMP_TYPE
        if set -q _ARC_DEBUG
            ros2 8>&1 9>&2 1>&9 2>&1
        else
            ros2 8>&1 9>&2 1>/dev/null 2>&1
        end
    end
    complete --command ros2 -f -a '(__fish_ros2_complete)'
end

function ros2_activate
    bass source /opt/ros/humble/setup.bash
    register_argcomplete
end

function ros2_ws
    bass source ./install/setup.bash
    register_argcomplete
end
