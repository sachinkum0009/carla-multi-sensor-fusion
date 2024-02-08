FROM ros:humble

# change to bash
SHELL ["/bin/bash", "-c"]

WORKDIR /zenoh

# Copy the source code to the src directory
COPY . /zenoh/

# Install any dependencies required to build the code
RUN apt update
RUN apt install ros-humble-rmw-cyclonedds-cpp ros-humble-demo-nodes-cpp -y
RUN source /opt/ros/humble/setup.bash

EXPOSE 7447

CMD ["/bin/bash", "/zenoh/entrypoint.sh"]
