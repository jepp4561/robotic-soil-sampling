import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    controller_manager = LaunchConfiguration("controller_manager").perform(context)
    tool_namespace = LaunchConfiguration("tool_namespace").perform(context)

    velocity_controller_fqn = f"/{robot_namespace}/soil_sampler_velocity_controller"
    joint_states_topic = f"/{robot_namespace}/platform/joint_states"

    configure_script = f"""
    set -e

    until ros2 param set {controller_manager} soil_sampler_velocity_controller.type velocity_controllers/JointGroupVelocityController > /dev/null 2>&1; do
      sleep 1
    done

    until ros2 run controller_manager spawner soil_sampler_velocity_controller --controller-manager {controller_manager} --load-only > /dev/null 2>&1; do
      sleep 1
    done

    until ros2 param set {velocity_controller_fqn} joints "[soil_sampler_slider_1]" > /dev/null 2>&1; do
      sleep 1
    done

    until ros2 param set {velocity_controller_fqn} interface_name velocity > /dev/null 2>&1; do
      sleep 1
    done

    until ros2 run controller_manager spawner soil_sampler_velocity_controller --controller-manager {controller_manager} > /dev/null 2>&1; do
      sleep 1
    done
    """

    configure_soil_sampler_controller = ExecuteProcess(
        cmd=["bash", "-c", configure_script],
        output="screen",
    )

    hardware_simulator = Node(
        package="soil_sampler_control",
        executable="hardware_simulator",
        namespace=tool_namespace,
        name="hardware_simulator",
        parameters=[
            {
                "joint_name": "soil_sampler_slider_1",
                "lower_limit": 0.0,
                "upper_limit": 0.25,
                "velocity": 0.01,
            }
        ],
        remappings=[
            ("joint_states", joint_states_topic),
            ("velocity_command", f"{velocity_controller_fqn}/commands"),
        ],
    )

    return [configure_soil_sampler_controller, hardware_simulator]


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory("soil_sampler_bringup"),
        "config",
        "soil_sampler.yaml",
    )

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="husky",
    )

    declare_controller_manager = DeclareLaunchArgument(
        "controller_manager",
        default_value="/husky/controller_manager",
    )

    declare_tool_namespace = DeclareLaunchArgument(
        "tool_namespace",
        default_value="soil_sampler",
    )

    return LaunchDescription([
        declare_robot_namespace,
        declare_controller_manager,
        declare_tool_namespace,
        OpaqueFunction(function=launch_setup),

        Node(
            package="soil_sampler_control",
            executable="soil_sampler_node",
            name="soil_sampler",
            namespace="soil_sampler",
            output="screen",
            parameters=[config_file],
        ),
    ])