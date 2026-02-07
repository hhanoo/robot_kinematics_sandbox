# ROS 2 Humble Desktop base -------------------------------------------------
# https://hub.docker.com/_/ros/tags
FROM hhanoo/ros:humble-desktop

# Locale and Font Settings --------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    fonts-noto-color-emoji \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# Build Dependencies --------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Control Dependencies -------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# URDF / Visualization tools -----------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*


# Tool Dependencies ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libmodbus-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Development Tools ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    bear \
    clang-format \
    clangd \
    && rm -rf /var/lib/apt/lists/*

# Python base ---------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    python3-vcstool \
    python3-rosdep \
    ca-certificates \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install ROS2-compatible setuptools/packaging --------------
RUN python3 -m pip install --no-cache-dir --upgrade \
    pip \
    setuptools==58.2.0 \
    packaging==21.3 \
    wheel

# Python Dependencies (system-wide) -----------------------------------------
# Math / Linear algebra -----------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# USB device access groups --------------------------------------------------
RUN groupadd -f plugdev && \
    groupadd -f dialout && \
    usermod -aG plugdev root && \
    usermod -aG dialout root && \
    usermod -aG video root

# Setup workspace -----------------------------------------------------------
WORKDIR /ros2_ws

# ROS auto source (no venv) -------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

# Default command -----------------------------------------------------------
CMD ["bash"]