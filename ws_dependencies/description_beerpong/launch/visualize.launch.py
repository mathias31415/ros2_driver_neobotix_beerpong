from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_package = "description_beerpong"

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='ur_',
            description="tf_prefix for the links and joints in the robot cell",
        )
    )

    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_description_content = Command(    # enthält ganze Kinematik des Roboters
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "full_robot.urdf.xacro"]),
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "rviz_config_20240430.rviz"]) # define path to rviz-config file


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    #run joint_state_publisher node, because we don't need to bringup the robot model (without no transforms !) --> only for visualizing porposes     
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    nodes_to_start = [
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
