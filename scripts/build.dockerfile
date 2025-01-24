# FROM osrf/ros:humble-desktop
# There is no point of reusing desktop image as all the packages will be upgraded to the latest version leading to a lot of unused layers from original image
FROM ros:humble-ros-base

# Install ROS packages
COPY ./ros /ros-prep
RUN /ros-prep/ros-2-prep.sh

# From base image
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
