Installing ROS
####################

Development machine
--------------------

For the development machine use Ubuntu 22.04 and install ros using [ros-2-prep.sh](ros-2-prep.sh). At the end script will print additional installation instructions.

Bash
-------------------------

Update your ``~/.bashrc`` with the following snippet that adds couple alises for easy activation of ros1 and 2 globally or in workspace

.. code-block:: bash

  #################################################################

  alias ros2_activate="source /opt/ros/jazzy/setup.bash"
  alias ros2_ws="source ./install/setup.bash"

Fish
-------------------------

For ``fish`` shell users run setup script

.. code-block:: bash

  ./scripts/ros/fish/setup.fish

