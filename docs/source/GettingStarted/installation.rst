Dev machine setup
===================================


There are several ways to setup your dev environment, but in either case you would need source code from this repo

.. code-block:: bash

  git clone https://github.com/Dr-QP/Dr.QP.git
  cd Dr.QP


Devcontainer (any desktop OS)
-----------------------------------

1. Open `Dr.QP.code-workspace` in VSCode
2. Install recommended VSCode extensions
3. Install docker (either `Docker Desktop <https://www.docker.com/products/docker-desktop/>`_ for macOS and Windows) or native docker installtion for linux `scripts/install_docker.sh`
4. 4. Run `Dev Containers: Open Workspace in Container...` via command pallet `F1` and select [Dr.QP.code-workspace](Dr.QP.code-workspace) workspace
5. Choose devcontainer to open:
   - (recommended, fast) `prebuilt` to use devcontainer based on `ghcr.io/dr-qp/${{ env.ROS_DISTRO }}-ros-desktop:edge` image built from `main` branch
   - `source` to build devcontainer from source in the current branch. This setup is useful if you will need to make changes to the dockerfile or installation scripts

Raw Dev machine (linux only)
-----------------------------------

While ROS 2 is supported on many OS, the quality of support is still low and many non core packages have bugs.

Linux is the only platform that has been tested for this project.

1. Follow :doc:`installation-ros` to install ROS2
2. Open [Dr.QP.code-workspace](Dr.QP.code-workspace) in VSCode
3. Install recommended VSCode extensions
4. Install ROS dependencies

.. code-block:: bash

  ros2_activate # Source ROS environment
  ./scripts/ros/ros-dep.sh # Install all ROS dependencies


.. toctree::

   installation-ros
