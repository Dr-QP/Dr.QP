# Building 357.7s
# Resulting "Size": 5516300343,
FROM osrf/ros:humble-desktop

# Building off the base image saves a bit of space, but the build time is longer
# Building 478.3s
# Resulting "Size": 5309090371,
# FROM ros:humble-ros-base

# Force clang-format-19 and friends to the default
ENV PATH="/usr/lib/llvm-19/bin:$PATH"

# Install ROS packages
COPY ./ros /ros-prep
RUN /ros-prep/ros-2-prep.sh

# From base image
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
