import os
from launch import LaunchDescription
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('working_robot').find('working_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'marco1.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    rviz_file = os.path.join(pkg_share, 'launch', 'display.rviz')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(robot_desc, value_type=str)}],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            output='screen'
        )
    ])
