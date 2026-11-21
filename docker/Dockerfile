# Shared development environment for isolated ROS 2 builds (AMD64/ARM64).
# Hardware drivers are not started, and no device or network access is required.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-jazzy-demo-nodes-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
CMD ["bash"]
