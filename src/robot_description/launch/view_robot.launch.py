from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # xacro path
    xacro_file = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'ur10e.urdf.xacro'
    ])

    # rviz config path
    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'rviz',
        'view_robot.rviz'
    ])

    # robot_description parameter
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
