
function ros2_activate
    bass source /opt/ros/humble/setup.bash
    register-python-argcomplete --shell fish ros2 | source
end

function ros2_ws
    bass source ./install/setup.bash
    register-python-argcomplete --shell fish ros2 | source
end
