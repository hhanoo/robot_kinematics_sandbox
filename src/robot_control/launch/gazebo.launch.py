"""
Gazebo bring-up
===============
Order matters here: the model must exist inside Gazebo before the
controllers can claim its interfaces, so each stage waits for the
previous process to exit.

    gz sim  ->  spawn model  ->  joint_state_broadcaster
                                 -> joint_position_controller
                                 -> motion_server
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
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
        description="Launch RViz alongside Gazebo",
    )
    world = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="Gazebo world file",
    )
    headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run the Gazebo server without its GUI",
    )

    # ===============================
    # 2. Robot description
    # ===============================
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "urdf", "ur10e.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("robot_control"), "config", "controllers.yaml"]
    )
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " sim_gazebo:=true",
                " simulation_controllers:=",
                controllers_file,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # ===============================
    # 3. Gazebo
    # ===============================
    # This wrapper exports Gazebo's search paths
    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    # -r starts the world unpaused, -s drops the GUI
    server_only = PythonExpression(
        ["'-s ' if '", LaunchConfiguration("headless"), "' == 'true' else ''"]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            "gz_args": [server_only, "-r -v 1 ", LaunchConfiguration("world")],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ===============================
    # 4. Spawn the model from /robot_description
    # ===============================
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "ur10e", "-z", "0.0"],
        output="screen",
    )

    # ===============================
    # 5. Clock bridge
    # ===============================
    # Without /clock a use_sim_time node keeps its timers frozen
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # ===============================
    # 6. Controllers (after the model exists)
    # ===============================
    broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_controller"],
        output="screen",
    )

    # ===============================
    # 7. Motion server on the Gazebo backend
    # ===============================
    motion_server = Node(
        package="robot_control",
        executable="motion_server",
        parameters=[{"rate": 50.0, "backend": "gazebo", "use_sim_time": True}],
        output="screen",
    )
    marker_server = Node(
        package="robot_control",
        executable="marker_server",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ===============================
    # 8. RViz
    # ===============================
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("robot_control"), "rviz", "control.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    # ===============================
    # 9. Staging
    # ===============================
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[broadcaster])
    )
    after_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=broadcaster, on_exit=[position_controller])
    )
    after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=position_controller,
            on_exit=[motion_server, marker_server, rviz],
        )
    )

    return LaunchDescription(
        [
            use_rviz,
            world,
            headless,
            gazebo,
            robot_state_publisher,
            clock_bridge,
            spawn,
            after_spawn,
            after_broadcaster,
            after_controller,
        ]
    )
