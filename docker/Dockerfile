# =========================================================================
# Base Image
# =========================================================================
# - https://hub.docker.com/r/hhanoo/ros/tags
FROM hhanoo/ros:humble-desktop

# - Build dependencies
ARG DEBIAN_FRONTEND=noninteractive

# =========================================================================
# Locale and Font
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    fonts-noto-color-emoji \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# =========================================================================
# Essential Packages
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# ROS2
# =========================================================================
# - ROS2 setup
ARG ROS_VERSION=humble
ENV ROS_DISTRO=${ROS_VERSION}

# - URDF / Visualization tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Python Dependencies
# =========================================================================
# - Kinematics core (numpy) + unit tests (pytest)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-numpy \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# - ROS2-compatible setuptools/packaging for ament_python builds
RUN python3 -m pip install --no-cache-dir --upgrade \
    pip \
    setuptools==58.2.0 \
    packaging==21.3 \
    wheel

# =========================================================================
# Development Tools
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    bear \
    clang-format \
    clangd \
    gdb \
    gdbserver \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Workspace Setup
# =========================================================================
# - Workspace
WORKDIR /ros2_ws

# - Commands
COPY ./commands.sh /etc/profile.d/commands.sh
RUN chmod 644 /etc/profile.d/commands.sh && \
    echo 'source /etc/profile.d/commands.sh' >> /etc/bash.bashrc

# - Entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# - Default command
CMD ["bash"]
