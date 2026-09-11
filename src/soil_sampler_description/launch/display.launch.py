from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable

def generate_launch_description():

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the implement"
    )

    namespace = LaunchConfiguration("namespace")

    xacro_file = PathJoinSubstitution([
        FindPackageShare("soil_sampler_description"),
        "urdf",
        "soil_sampler.urdf.xacro"
    ])

    soil_sampler_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
        ])
    }

    rviz_config = PathJoinSubstitution([
        FindPackageShare("soil_sampler_description"),
        "rviz",
        "soil_sampler.rviz"
    ])

    return LaunchDescription([

    declare_namespace,

    Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[soil_sampler_description],
        output="screen"
    ),

    Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace=namespace,
        output="screen"
    ),

    Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

])

