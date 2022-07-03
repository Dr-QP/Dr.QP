
set -gx MAMBA_NO_BANNER 1

function ros2_activate
    conda activate ros_env
    bass source $CONDA_PREFIX/setup.bash
end

function ros2_ws
    ros2_activate
    bass source setup.bash
end