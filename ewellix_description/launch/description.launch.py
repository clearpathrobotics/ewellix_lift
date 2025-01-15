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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable,
                                  PathJoinSubstitution, LaunchConfiguration)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('ewellix_description')
    # Launch Configurations
    robot_description_command = LaunchConfiguration('robot_description_command')

    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices=['true', 'false'],
        default_value='false',
        description='Use simulation time'
    )

    control_config = LaunchConfiguration('control_config')
    arg_control_config = DeclareLaunchArgument(
        'control_config',
        default_value=PathJoinSubstitution([
            package,
            'config',
            'control',
            'jtc.yaml'
        ]),
        description='ROS 2 controller file to pass to Gazebo.'
    )

    lift_parameters = LaunchConfiguration('lift_parameters')
    arg_lift_parameters = DeclareLaunchArgument(
        'lift_parameters',
        default_value=PathJoinSubstitution([
            package,
            'config',
            'tlt_x25.yaml'
        ]),
        description='Lift description configuration file.'
    )

    # Paths
    urdf = PathJoinSubstitution([
        package,
        'urdf',
        'ewellix_lift.urdf.xacro'
    ])

    # Get URDF via xacro
    arg_robot_description_command = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            urdf,
            ' ',
            'sim_ignition:=',
            use_sim_time,
            ' ',
            'use_fake_hardware:=',
            'false',
            ' ',
            'generate_ros2_control_tag:=',
            'true',
            ' ',
            'gazebo_controllers:=',
            control_config,
            ' ',
            'parameters_file:=',
            lift_parameters
        ]
    )

    robot_description_content = ParameterValue(
        Command(robot_description_command),
        value_type=str
    )

    # Lift State Publisher
    lift_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_lift_parameters)
    ld.add_action(arg_control_config)
    ld.add_action(arg_robot_description_command)
    # Nodes
    ld.add_action(lift_state_publisher)
    return ld
