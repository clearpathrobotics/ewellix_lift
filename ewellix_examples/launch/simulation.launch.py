# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    controller_file = LaunchConfiguration('controller_file')
    arg_controller_file = DeclareLaunchArgument(
        'controller_file',
        choices=['jtc.yaml', 'jpc.yaml'],
        default_value='jpc.yaml',
        description='Controller Type, Joint Traj. (jtc) or Position (jpc)'
    )

    launch_rviz = LaunchConfiguration('rviz')
    arg_launch_rviz = DeclareLaunchArgument(
        'rviz',
        choices=['true', 'false'],
        default_value='true',
        description='If true, launch rviz.'
    )

    launch_moveit = LaunchConfiguration('moveit')
    arg_launch_moveit = DeclareLaunchArgument(
        'moveit',
        choices=['true', 'false'],
        default_value='true',
        description='If true, launch MoveIt MoveGroup'
    )

    # Determine all ros packages that are sourced
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]

    # Set ignition resource path to include all sourced ros packages
    gz_sim_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[':' + ':'.join(packages_paths)])

    # Directories
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_description = FindPackageShare('ewellix_description')

    # Paths
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    description_launch = PathJoinSubstitution([pkg_description, 'launch', 'description.launch.py'])
    control_config_file = PathJoinSubstitution([
        pkg_description,
        'config',
        'control',
        controller_file
    ])

    # Gazebo
    ros_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', ['empty.sdf',
                         ' -r',
                         ' -v 4',
                         ])
        ]
    )

    # Spawn Robot
    group_action_spawn_robot = GroupAction([
        # Description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_launch]),
            launch_arguments=[
                ('use_sim_time', 'true'),
                ('control_config', control_config_file)
            ],
        ),

        # Create
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'lift',
                       '-x', '0.0',
                       '-y', '0.0',
                       '-z', '0.0',
                       '-Y', '0.0',
                       '-topic', 'robot_description'],
            output='screen'
        ),
    ])

    # Controllers
    group_action_controllers = GroupAction([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[control_config_file],
            output={
                'stdout': 'screen'
            },
            remappings=[
                ('~/robot_description', 'robot_description'),
            ]
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster', '--controller-manager-timeout', '60'
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'lift_joint_trajectory_controller', '--controller-manager-timeout', '60'
            ],
            output='screen',
            condition=LaunchConfigurationEquals(
                'controller_file', 'jtc.yaml'
            )
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'lift_position_controller', '--controller-manager-timeout', '60'
            ],
            output='screen',
            condition=LaunchConfigurationEquals(
                'controller_file', 'jpc.yaml'
            )
        ),
    ])

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                          '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
                        ])

    # Launch RViz
    config_rviz = PathJoinSubstitution([
                     FindPackageShare('ewellix_examples'),
                     'config',
                     'moveit.rviz'])
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     arguments=['-d', config_rviz],
                     parameters=[{'use_sim_time': True}],
                     output='screen',
                     condition=IfCondition(launch_rviz))

    # Launch MoveIt
    moveit_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ewellix_moveit_config'),
                    'launch',
                    'move_group_sim.launch.py'
                ])
            ]),
            condition=IfCondition(launch_moveit))

    ld = LaunchDescription()
    ld.add_action(arg_controller_file)
    ld.add_action(arg_launch_moveit)
    ld.add_action(arg_launch_rviz)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(ros_gz_sim)
    ld.add_action(group_action_spawn_robot)
    ld.add_action(group_action_controllers)
    ld.add_action(clock_bridge)
    ld.add_action(rviz_node)
    ld.add_action(moveit_node)
    return ld
