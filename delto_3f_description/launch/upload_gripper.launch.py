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

    namespace = DeclareLaunchArgument("namespace", default_value="")

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='gripper_robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
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
            'frame_prefix': [LaunchConfiguration('namespace'), '/'],
        }],
        remappings=[
            ('joint_states', 'gripper_joint_states'),
        ]
    )

    ld.add_action(namespace)
    ld.add_action(rsp_node)
    return ld