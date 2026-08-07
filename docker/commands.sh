# ===== ROS env =====
[ -f /opt/ros/humble/setup.bash ]  && source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
[ -f /ros2_ws/docker/config.sh ]   && source /ros2_ws/docker/config.sh

# ===== Common helpers =====
source-ros-ws() {
    [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
}

source-config() {
    [ -f /ros2_ws/docker/config.sh ] && source /ros2_ws/docker/config.sh
}

# ===== Build =====
build() {
    cd /ros2_ws || return 1
    colcon build \
        --symlink-install \
        --cmake-args \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            -DCMAKE_CXX_STANDARD=17 \
            -DCMAKE_CXX_STANDARD_REQUIRED=ON \
            -DCMAKE_BUILD_TYPE=Release "$@"
    source-ros-ws
    source-config
}

# ===== Tests (pure python, no ROS runtime needed) =====
test-kinematics() {
    cd /ros2_ws/src/robot_kinematics && python3 -m pytest "$@"
}

test-trajectory() {
    cd /ros2_ws/src/robot_trajectory && python3 -m pytest "$@"
}

# ===== Launchers =====
run-view() {
    source-ros-ws
    ros2 launch robot_description view.launch.py "$@"
}

run-demo() {
    source-ros-ws
    ros2 launch robot_bringup demo.launch.py "$@"
}

# ===== Help =====
cmd-help() {
    printf "\n[robot_kinematics_sandbox] Commands:\n\n"

    printf "  Build:\n"
    printf "    %-18s - %s\n" "build"            "colcon build --symlink-install + source overlay"
    printf "\n"

    printf "  Tests (pytest, no ROS runtime needed):\n"
    printf "    %-18s - %s\n" "test-kinematics"  "FK / Jacobian / IK unit tests"
    printf "    %-18s - %s\n" "test-trajectory"  "Trajectory generation unit tests"
    printf "\n"

    printf "  Launchers:\n"
    printf "    %-18s - %s\n" "run-view"         "UR10e model viewer (RViz + joint_state_publisher_gui)"
    printf "    %-18s - %s\n" "run-demo"         "FK/IK/trajectory demo sequence (RViz)"
    printf "\n"

    printf "  Config / Help:\n"
    printf "    %-18s - %s\n" "source-config"    "Reload /ros2_ws/docker/config.sh"
    printf "    %-18s - %s\n" "cmd-help"         "Show this help"
    printf "\n"
}

# ===== Show help on interactive shell =====
case $- in
    *i*) cmd-help ;;
esac
