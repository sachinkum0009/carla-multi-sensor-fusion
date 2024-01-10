FROM ros:humble

# change to bash
SHELL ["/bin/bash", "-c"]

# Set the working directory to /colcon_ws
WORKDIR /colcon_ws

# Create the src directory
RUN mkdir src

# Copy the source code to the src directory
COPY . /colcon_ws/src

# Install any dependencies required to build the code
RUN apt update
RUN source /opt/ros/humble/setup.bash
RUN rosdep install --from-paths src -r -y
RUN apt install ros-humble-ament-cmake
RUN source /opt/ros/humble/setup.bash; colcon build --packages-skip yolop multi_sensor_fusion

# Set the default command to run the launch file
# CMD ["source", "install" "ros2", "launch", "sensor_fusion_perception", "opencv_publisher.launch"]
# CMD source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch sensor_fusion_perception opencv_publisher.launch.py
CMD ["/bin/bash", "/colcon_ws/src/entrypoint.sh"]
