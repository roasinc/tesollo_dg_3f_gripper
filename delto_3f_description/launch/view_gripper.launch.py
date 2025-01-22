import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    rviz_config_file = os.path.join(get_package_share_directory('delto_3f_description'), 'rviz', 'view_gripper.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        on_exit=Shutdown()
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        get_package_share_directory('delto_3f_description'),
                        'urdf/robot.urdf.xacro',
                    ]),
                ]),
        }]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen'
    )

    ld.add_action(rviz_node)
    ld.add_action(rsp_node)
    ld.add_action(jsp_gui_node)

    return ld