from setuptools import find_packages, setup

package_name = "robot_kinematics"

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
    description="DH-based FK / Jacobian / DLS IK core (pure numpy, ROS-independent)",
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
        "console_scripts": [],
    },
)
