cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

# =========================================================
# Dependencies
# =========================================================
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# =========================================================
# Interface generation
# =========================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveJ.srv"
  "srv/MoveL.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
