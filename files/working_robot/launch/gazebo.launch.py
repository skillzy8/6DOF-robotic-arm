import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "mypackage2"

    pkg_share = get_package_share_directory(package_name)
    gz_resource_path = os.path.dirname(pkg_share)

    urdf_file = os.path.join(
        pkg_share,
        "urdf",
        "marco1.urdf"
    )

    config_file = os.path.join(
        pkg_share,
        "config",
        "controllers.yaml"
    )

    with open(urdf_file, "r") as infp:
        robot_description = infp.read()

    robot_description = robot_description.replace(
        "$(find mypackage2)/config/controllers.yaml",
        config_file
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf"
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.2"
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen"
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gz_resource_path
        ),

        gazebo,
        robot_state_publisher,

        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),

        TimerAction(
            period=6.0,
            actions=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner
            ]
        )
    ])
    
