# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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

#
# Author: Denis Stogl

import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")

    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")



    #Get information from terminal 

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur5",
        )
    )

    # General arguments description_beerpong
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="description_beerpong", #package_name from URDF
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="full_robot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="beerpong_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='ur_',
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP to connect the robot",
            default_value='"192.168.1.103"',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            description="Use fake hardware",
            default_value="false",

        )
    )


    # Setup URDF Model // Send information to nodes
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ), # defines collisions between links etc.
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    #For the MoveGroupNode -> Services as MoveToPose, Set Velocity etc.
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    ##################################################################################################
    ##                                  launch driver from ur package                               ##
    ##################################################################################################

    driver_package = "beerpong_robot_driver"
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(driver_package), 'launch']), "/ur_control.launch.py"]),
            launch_arguments={
                "ur_type": ur_type,
                "robot_ip": robot_ip, 
                "description_package": description_package,
                "description_file": description_file,
                "tf_prefix": tf_prefix,
                "use_fake_hardware": use_fake_hardware,
                "launch_rviz": "false",
            }.items(),
    )



    ##################################################################################################
    ##                         add our custom node (moveit_wrapper)                                 ##
    ##################################################################################################
    planning_group_name = "ur_manipulator"         ##HARDCODED for neobotix
    planning_group = {"planning_group": planning_group_name}

    moveit_wrapper_node = Node(
        package="moveit_wrapper",
        executable="moveit_wrapper_node",
        output="screen",
        parameters=[robot_description, robot_description_semantic, robot_description_kinematics, planning_group],
    )


    ##################################################################################################
    ##                                           launch moveit                                      ##
    ##################################################################################################
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(moveit_config_package), 'launch']), "/ur_moveit.launch.py"]),
            launch_arguments={
                "ur_type": ur_type,
                "description_package": description_package,
                "description_file": description_file,
                "moveit_config_package": moveit_config_package,
                "moveit_config_file": moveit_config_file,
                "tf_prefix": tf_prefix,
                "use_fake_hardware": use_fake_hardware,
                "launch_rviz": launch_rviz,   
            }.items(),
    )
    nodes_to_start = [moveit_launch, moveit_wrapper_node, driver_launch]  


    return LaunchDescription(declared_arguments + nodes_to_start)