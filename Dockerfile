# Use the official ROS2 base image
FROM osrf/ros:foxy-desktop

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Create a workspace
RUN mkdir -p /root/ros2_ws/src

# Set the working directory
WORKDIR /root/ros2_ws

# Copy the source code to the workspace
COPY src /root/ros2_ws/src

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN colcon build

# Source the setup script
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /root/ros2_ws/install/setup.bash && bash"]