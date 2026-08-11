from setuptools import find_packages, setup

package_name = "robot_control"

setup(
    # =========================================================
    # Package metadata
    # =========================================================
    name=package_name,
    version="0.1.0",
    # =========================================================
    # Package discovery
    # Automatically find all Python packages in the directory
    # =========================================================
    packages=find_packages(exclude=["test"]),
    # =========================================================
    # Data files to install
    # These files are required for ROS2 package discovery
    # =========================================================
    data_files=[
        # Register package with ROS2 index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Install package.xml for dependency management
        ("share/" + package_name, ["package.xml"]),
        # Install launch files
        ("share/" + package_name + "/launch", ["launch/control.launch.py"]),
        # Install RViz configuration
        ("share/" + package_name + "/rviz", ["rviz/control.rviz"]),
    ],
    # =========================================================
    # Dependencies
    # =========================================================
    install_requires=["setuptools"],
    # =========================================================
    # Package information
    # =========================================================
    zip_safe=True,
    maintainer="hhanoo",
    maintainer_email="woo980711@gmail.com",
    description="Runtime motion control: motion server, goal marker, keyboard teleop",
    license="Apache-2.0",
    # =========================================================
    # Extra dependencies (for testing, etc.)
    # =========================================================
    extras_require={},
    # =========================================================
    # Entry points (Executable scripts)
    # Maps command names to Python functions
    # Format: 'command_name = package.module:function'
    # =========================================================
    entry_points={
        "console_scripts": [
            # ros2 run robot_control motion_server
            "motion_server = robot_control.motion_server:main",
            # ros2 run robot_control marker_server
            "marker_server = robot_control.marker_server:main",
            # ros2 run robot_control teleop_keyboard
            "teleop_keyboard = robot_control.teleop_keyboard:main",
        ],
    },
)
