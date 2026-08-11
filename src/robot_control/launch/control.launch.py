from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ===============================
    # 1. Arguments
    # ===============================
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz alongside the motion server",
    )

    # ===============================
    # 2. Robot description
    # ===============================
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "urdf", "ur10e.urdf.xacro"]
    )
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # ===============================
    # 3. Motion server
    # ===============================
    motion_server = Node(
        package="robot_control",
        executable="motion_server",
        parameters=[{"rate": 50.0}],
        output="screen",
    )

    # ===============================
    # 4. Goal marker
    # ===============================
    marker_server = Node(
        package="robot_control",
        executable="marker_server",
        output="screen",
    )

    # ===============================
    # 5. RViz
    # ===============================
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("robot_control"), "rviz", "control.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [use_rviz, robot_state_publisher, motion_server, marker_server, rviz]
    )
