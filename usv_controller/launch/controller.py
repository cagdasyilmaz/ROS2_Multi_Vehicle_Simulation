from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='usv_controller',
            executable='usv_01_controller',
            output='screen'
        ),
        Node(
            package='usv_controller',
            executable='usv_02_controller',
            output='screen'
        ),
        Node(
            package='usv_controller',
            executable='usv_03_controller',
            output='screen'
        ),
        Node(
            package='usv_controller',
            executable='usv_04_controller',
            output='screen'
        ),
        Node(
            package='usv_controller',
            executable='usv_05_controller',
            output='screen'
        ),
        Node(
            package='usv_controller',
            executable='usv_06_controller',
            output='screen'
        )
    ])